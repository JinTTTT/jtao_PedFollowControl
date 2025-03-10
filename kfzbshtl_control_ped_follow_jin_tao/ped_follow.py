import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import time
from enum import Enum
from kfzbshtl_control_ped_follow_jin_tao.helper_functions import steering_control, prepare_steering_debug_msgs, distance_control
from kfzbshtl_msgs.msg import StampedControlGuardInfo  # Import for StampedControlGuardInfo

class MissionState(Enum):
    STOP = 0
    IDLE = 1
    RUNNING = 2

class PedFollow(Node):
    def __init__(self):
        """
        Initialize a node to control a bus continuously following a pedestrian at a constant distance.
        """
        super().__init__('ped_follow')

        self.declare_and_initialize_parameters()
        self.initialize_state_variables()
        self.setup_publishers_and_subscribers()
        self.display_parameters()

        # Immediately check and output current state and unmet conditions
        self.update_mission_state(initial_check=True)
        
        self.timer = self.create_timer(self.pub_time, self.timer_callback)

    def declare_and_initialize_parameters(self):
        self.declare_parameter('track_distance', 10.0)     # m
        self.declare_parameter('safe_distance', 8.0)       # m
        self.declare_parameter('v_max', 2.0)               # m/s
        self.declare_parameter('acc_max_forward', 0.5)     # m/s^2
        self.declare_parameter('acc_max_emergency', 3.0)   # m/s^2 
        self.declare_parameter('steering_range_max', 24 * math.pi / 180)  # rad
        self.declare_parameter('steering_change_max', np.deg2rad(11.4))   # rad/s

        self.track_distance = self.get_parameter('track_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.v_max = self.get_parameter('v_max').value
        self.acc_max_forward = self.get_parameter('acc_max_forward').value
        self.acc_max_emergency = self.get_parameter('acc_max_emergency').value
        self.steering_range_max = self.get_parameter('steering_range_max').value
        self.steering_change_max = self.get_parameter('steering_change_max').value

    def initialize_state_variables(self):
        # Pedestrian state
        self.dist_to_ped_now = None  # Distance to the pedestrian (m)
        self.rel_ped_x = 0.0         # Pedestrian relative X position (m)
        self.rel_ped_y = 0.0         # Pedestrian relative Y position (m)

        self.last_ped_data_time = None

        # Vehicle state
        self.vehicle_v_actual = 0.0  # Actual vehicle velocity (m/s)

        # Control state
        self.target_vel_dist_control_last = 0.0  # Last desired velocity from distance control (m/s)
        self.desired_vel_latest = 0.0            # Latest desired velocity (m/s)
        self.desired_steering_latest = 0.0       # Latest desired steering angle (rad)

        # Error variables for PID control
        self.dist_error_sum = 0.0

        # Flags and timers
        self.guard_mode_auto_active = False
        self.get_ped_pose = False
        self.get_can_msg = False
        self.pub_time = 0.05  # 20 Hz control loop frequency (s)
        self.latest_cmd_update_time = time.time()
        self.last_can_msg_time = time.time()
        self.last_log_time = time.time()
        self.log_interval = 5.0  # Status logging interval (s)

        # Mission state
        self.mission_state = MissionState.STOP
        self.last_conditions = None  # Used to track condition changes

        # Gesture variables
        self.filtered_gesture_id = 0          # Initialize current gesture ID
        self.prev_filtered_gesture_id = 0     # Initialize previous gesture ID
        self.gesture_processed = False        # Flag to track if gesture has been processed

    def setup_publishers_and_subscribers(self):
        # Subscribers
        self.create_subscription(PoseStamped, '/filtered_obj', self.ped_rel_pos_callback, 1) 
        self.create_subscription(Float32, 'vehicle_v_actual', self.vehicle_vel_callback, 1)
        self.create_subscription(Float32, 'distance_to_ped', self.distance_to_ped_callback, 1)
        self.create_subscription(Float32, 'ped_relative_speed', self.relative_vel_callback, 1)
        self.create_subscription(Int32, 'filtered_gesture_id', self.filtered_gesture_callback, 1)
        self.create_subscription(StampedControlGuardInfo, '/act/control_guard_info', self.guard_info_callback, 1)  
        self.create_subscription(PoseStamped, '/pose/ped_global_pose', self.ped_global_pose_callback, 1)  
        self.create_subscription(PoseStamped, '/pose/bus_global_pose', self.vehicle_global_pose_callback, 1)  

        # Publishers
        self.desired_vel_publisher = self.create_publisher(Float32, 'desired_velocity', 1)
        self.desired_steering_publisher = self.create_publisher(Float32, 'desired_steering', 1)
        self.track_distance_publisher = self.create_publisher(Float32, '/track_distance', 1)
        self.steering_debug_pub_heading = self.create_publisher(Float32, '/heading_part', 1)
        self.steering_debug_pub_cross = self.create_publisher(Float32, '/cross_part', 1)
        self.steering_debug_pub_target = self.create_publisher(Float32, '/target_steering', 1)
        self.long_debug_required_distance = self.create_publisher(Float32, '/required_distance', 1)
        self.long_debug_proportional_contribution = self.create_publisher(Float32, '/proportional_contribution', 1)
        self.long_debug_integral_contribution = self.create_publisher(Float32, '/integral_contribution', 1)
        self.long_debug_derivative_contribution = self.create_publisher(Float32, '/derivative_contribution', 1)
        self.long_debug_d_stop = self.create_publisher(Float32, '/d_stop', 1)
        self.long_debug_d_headway = self.create_publisher(Float32, '/d_headway', 1)

    def display_parameters(self):
        self.get_logger().info('------------------------------------------')
        self.get_logger().info('|        Parameters showed below         |')
        self.get_logger().info('|----------------------------------------|')
        self.get_logger().info('| Parameter    | Value  | Unit           |')
        self.get_logger().info('|--------------|--------|----------------|')
        self.get_logger().info(f'| Track dist   | {self.track_distance:5.2f}  | m              |')
        self.get_logger().info(f'| Safe dist    | {self.safe_distance:5.2f}  | m              |')
        self.get_logger().info(f'| Max velocity | {self.v_max:5.2f}  | m/s            |')
        self.get_logger().info('|----------------------------------------|')
        self.get_logger().info('| Tip: safe dist should be < track dist! |')
        self.get_logger().info('------------------------------------------')

    def update_mission_state(self, initial_check=False):
        can_ready = self.check_if_can_msg_ready()
        ped_ready = self.check_if_ped_pose_ready()
        auto_mode_active = self.guard_mode_auto_active
        safe_distance = self.check_if_out_safe_distance()

        current_conditions = (can_ready, ped_ready, auto_mode_active, safe_distance)
        all_conditions_met = all(current_conditions)

        previous_state = self.mission_state

        # Use filtered_gesture_id to determine gesture_signal
        gesture_signal = self.get_gesture_signal()

        state_changed = False

        if self.mission_state == MissionState.STOP:
            if all_conditions_met:
                self.mission_state = MissionState.IDLE
                state_changed = True
            elif initial_check or self.last_conditions != current_conditions:
                state_changed = True  # Log unmet conditions at initial check or when conditions change
        elif self.mission_state == MissionState.IDLE:
            if not all_conditions_met:
                self.mission_state = MissionState.STOP
                state_changed = True
            elif gesture_signal == 'toggle':
                self.mission_state = MissionState.RUNNING
                state_changed = True
        elif self.mission_state == MissionState.RUNNING:
            if not all_conditions_met:
                self.mission_state = MissionState.STOP
                state_changed = True
            elif gesture_signal == 'toggle':
                self.mission_state = MissionState.STOP
                state_changed = True

        if state_changed:
            self.log_state_change(
                previous_state,
                self.mission_state,
                conditions_met=all_conditions_met,
                gesture_signal=gesture_signal,
                conditions=current_conditions
            )

        # Update last_conditions
        self.last_conditions = current_conditions

    def get_gesture_signal(self):
        """
        Determine the gesture signal based on the filtered_gesture_id.
        """
        if self.prev_filtered_gesture_id == 0 and self.filtered_gesture_id == 1 and not self.gesture_processed:
            self.gesture_processed = True  # Mark the gesture as processed
            return 'toggle'
        elif self.filtered_gesture_id == 0 and self.prev_filtered_gesture_id != 0:
            self.gesture_processed = False  # Reset when gesture returns to 0
        return None

    def filtered_gesture_callback(self, msg):
        self.prev_filtered_gesture_id = self.filtered_gesture_id  # Save previous gesture ID
        self.filtered_gesture_id = msg.data  # Update current gesture ID

    def guard_info_callback(self, msg):
        self.guard_mode_auto_active = msg.automatic_mode_active
    
    def ped_global_pose_callback(self, msg):
        self.global_ped_x = msg.pose.position.x
        self.global_ped_y = msg.pose.position.y
    
    def vehicle_global_pose_callback(self, msg):
        self.vehicle_x = msg.pose.position.x
        self.vehicle_y = msg.pose.position.y

    def log_state_change(self, previous_state, current_state, conditions_met=True, gesture_signal=None, conditions=None):
        self.get_logger().warn("  -")
        self.get_logger().warn("=" * 80)
        if current_state == MissionState.IDLE:
            self.get_logger().warn('Mission mode is "Idle": waiting for gesture command.')
        elif current_state == MissionState.RUNNING:
            if gesture_signal == 'toggle':
                self.get_logger().warn('Mission mode is "Running": received toggle gesture.')
        elif current_state == MissionState.STOP:
            if not conditions_met:
                self.get_logger().warn('Mission mode is "Stop": conditions not all met.')
                self.log_unmet_conditions(conditions)
            elif gesture_signal == 'toggle':
                self.get_logger().warn('Mission mode is "Stop": received toggle gesture.')
            else:
                self.get_logger().warn('Mission mode is "Stop".')
        self.get_logger().warn("=" * 80)

    def log_unmet_conditions(self, conditions):
        can_ready, ped_ready, auto_mode_active, safe_distance = conditions
        unmet_conditions = []
        if not can_ready:
            unmet_conditions.append("Missing CAN message")
        if not ped_ready:
            unmet_conditions.append("Missing pedestrian position data")
        if not auto_mode_active:
            unmet_conditions.append("Vehicle not in automatic mode (press Deadman + A)")
        if not safe_distance:
            unmet_conditions.append("Not within safe distance")

        if unmet_conditions:
            self.get_logger().warn("Unmet conditions:")
            for condition in unmet_conditions:
                self.get_logger().warn(f"  - {condition}")

    def ped_rel_pos_callback(self, msg):
        self.get_ped_pose = True

        # Get current time and update last ped data time
        current_time = time.time()
        self.last_ped_data_time = current_time

        # Update pedestrian relative position
        self.rel_ped_x = msg.pose.position.x
        self.rel_ped_y = msg.pose.position.y

    def vehicle_vel_callback(self, msg):
        self.get_can_msg = True
        current_time = time.time()
        self.last_can_msg_time = current_time
        self.vehicle_v_actual = msg.data

    def distance_to_ped_callback(self, msg):
        self.dist_to_ped_now = msg.data

    def relative_vel_callback(self, msg):
        self.relative_vel = msg.data

    def check_if_can_msg_ready(self):
        time_now = time.time()
        if not self.get_can_msg or time_now - self.last_can_msg_time > 1.0:
            return False
        else:
            return True

    def check_if_ped_pose_ready(self):
        time_now = time.time()
        if not self.get_ped_pose or time_now - self.last_ped_data_time > 1.0:
            # Reset variables related to pedestrian data
            self.dist_error_sum = 0.0
            self.relative_vel = 0.0
            self.dist_to_ped_now = None  # Reset distance to ped
            self.get_ped_pose = False
            return False
        else:
            return True

    def check_if_out_safe_distance(self):
        SAFE_DISTANCE_THRESHOLD = 0.5  # m
        safe_distance = self.safe_distance + SAFE_DISTANCE_THRESHOLD
        if self.dist_to_ped_now is None:
            return False
        return self.dist_to_ped_now >= safe_distance

    def lateral_control(self):
        """
        High level control: Stanley Controller using relative pedestrian positions
        """
        self.desired_steering_latest, heading_error, cross_error, steering_angle = steering_control(
            self.rel_ped_y, self.global_ped_x, self.global_ped_y, self.vehicle_x, self.vehicle_y, self.vehicle_v_actual, self.desired_steering_latest, 
            self.cmd_dt, self.steering_change_max, self.steering_range_max
        )

        # Prepare and publish debug information
        head_msg, cross_msg, target_steering_msg = prepare_steering_debug_msgs(
            heading_error, cross_error, steering_angle
        )
        self.steering_debug_pub_heading.publish(head_msg)
        self.steering_debug_pub_cross.publish(cross_msg)
        self.steering_debug_pub_target.publish(target_steering_msg)

        if not self.guard_mode_auto_active:
            self.desired_steering_latest = 0.0

    def longitudinal_control(self):
        """
        High level control: eliminate the distance error between track distance and actual distance
        """
        target_vel_high_level, required_distance, proportional_contribution, integral_contribution, derivative_contribution, d_stop, d_headway, self.dist_error_sum = distance_control(
            self.dist_to_ped_now,
            self.track_distance,
            self.relative_vel,
            self.cmd_dt,
            self.v_max,
            self.acc_max_forward,
            self.target_vel_dist_control_last,
            self.desired_vel_latest,  # self.vehicle_v_actual,
            self.dist_error_sum
        )

        self.target_vel_dist_control_last = target_vel_high_level
        self.desired_vel_latest = target_vel_high_level  # Set desired velocity for publishing

        # Publish required distance, proportional and derivative contribution
        required_distance_msg = Float32()
        required_distance_msg.data = required_distance
        self.long_debug_required_distance.publish(required_distance_msg)

        proportional_contribution_msg = Float32()
        proportional_contribution_msg.data = proportional_contribution
        self.long_debug_proportional_contribution.publish(proportional_contribution_msg)

        integral_contribution_msg = Float32()
        integral_contribution_msg.data = integral_contribution
        self.long_debug_integral_contribution.publish(integral_contribution_msg)

        derivative_contribution_msg = Float32()
        derivative_contribution_msg.data = derivative_contribution
        self.long_debug_derivative_contribution.publish(derivative_contribution_msg)

        d_stop_msg = Float32()
        d_stop_msg.data = d_stop
        self.long_debug_d_stop.publish(d_stop_msg)

        d_headway_msg = Float32()
        d_headway_msg.data = d_headway
        self.long_debug_d_headway.publish(d_headway_msg)

        if not self.guard_mode_auto_active:
            self.desired_vel_latest = 0.0

    def publish_desired_commands(self):
        """
        Publish desired velocity and steering to respective topics.
        """
        desired_vel_msg = Float32()
        desired_vel_msg.data = self.desired_vel_latest
        self.desired_vel_publisher.publish(desired_vel_msg)

        desired_steering_msg = Float32()
        desired_steering_msg.data = self.desired_steering_latest
        self.desired_steering_publisher.publish(desired_steering_msg)

        self.latest_cmd_update_time = time.time()

    def stop_vehicle(self):
        self.desired_vel_latest = 0.0
        self.desired_steering_latest = 0.0

    def reset_pid_controllers(self):
        self.dist_error_sum = 0.0

    def timer_callback(self):
        self.cmd_dt = time.time() - self.latest_cmd_update_time

        self.update_mission_state()  

        # Publish track distance
        track_distance_msg = Float32()
        track_distance_msg.data = self.track_distance
        self.track_distance_publisher.publish(track_distance_msg)

        if self.mission_state == MissionState.RUNNING:
            self.lateral_control()
            self.longitudinal_control()
            self.publish_desired_commands()
        else:
            self.reset_pid_controllers()
            self.stop_vehicle()
            self.publish_desired_commands()

def main(args=None):
    rclpy.init(args=args)
    ped_follow_node = PedFollow()
    rclpy.spin(ped_follow_node)
    ped_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

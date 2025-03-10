import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import numpy as np
import time
from kfzbshtl_msgs.msg import StampedControlGuardCmdLongitudinal, StampedControlGuardCmdLateral, StampedControlGuardInfo, PidDebugInfo
from kfzbshtl_control_ped_follow_jin_tao.helper_functions import velocity_control

class LowLevelController(Node):
    def __init__(self):
        super().__init__('low_level_controller')

        self.declare_and_initialize_parameters()
        self.initialize_state_variables()
        self.setup_publishers_and_subscribers()

        self.timer = self.create_timer(self.pub_time, self.timer_callback)

    def declare_and_initialize_parameters(self):
        self.declare_parameter('v_max', 2.0)               # m/s
        self.declare_parameter('acc_max_forward', 0.5)     # m/s^2
        self.declare_parameter('acc_max_emergency', 3.0)   # m/s^2 
        self.declare_parameter('steering_range_max', 24 * math.pi / 180)  # rad
        self.declare_parameter('steering_change_max', np.deg2rad(11.4))   # rad/s

        self.v_max = self.get_parameter('v_max').value
        self.acc_max_forward = self.get_parameter('acc_max_forward').value
        self.acc_max_emergency = self.get_parameter('acc_max_emergency').value
        self.steering_range_max = self.get_parameter('steering_range_max').value
        self.steering_change_max = self.get_parameter('steering_change_max').value

    def initialize_state_variables(self):
        # Desired commands
        self.desired_vel_latest = 0.0
        self.desired_steering_latest = 0.0

        # Actual vehicle state
        self.vehicle_v_actual = 0.0  # Actual vehicle velocity (m/s)

        # Control variables
        self.vel_error_sum = 0.0
        self.vel_error_last = 0.0

        # Flags and timers
        self.guard_mode_auto_active = False
        self.pub_time = 0.05  # 20 Hz control loop frequency (s)
        self.latest_cmd_update_time = time.time()

    def setup_publishers_and_subscribers(self):
        # Subscribers
        self.create_subscription(Float32, 'desired_velocity', self.desired_velocity_callback, 1)
        self.create_subscription(Float32, 'desired_steering', self.desired_steering_callback, 1)
        self.create_subscription(Float32, 'vehicle_v_actual', self.vehicle_vel_callback, 1)
        self.create_subscription(StampedControlGuardInfo, '/act/control_guard_info', self.guard_info_callback, 1)

        # Publishers
        self.dd_vel_publisher = self.create_publisher(StampedControlGuardCmdLongitudinal, '/plan/control_cmd_long', 1)
        self.dd_steering_ang_publisher = self.create_publisher(StampedControlGuardCmdLateral, '/plan/control_cmd_lat', 1)
        self.pid_debug_vel_control = self.create_publisher(PidDebugInfo, 'pid_debug/vel_control', 1)

    def desired_velocity_callback(self, msg):
        self.desired_vel_latest = msg.data

    def desired_steering_callback(self, msg):
        self.desired_steering_latest = msg.data

    def vehicle_vel_callback(self, msg):
        self.vehicle_v_actual = msg.data

    def guard_info_callback(self, msg):
        self.guard_mode_auto_active = msg.automatic_mode_active

    def low_level_velocity_control(self):
        # Velocity control
        target_vel_low_level, self.vel_error_sum, self.vel_error_last, pid_debug_vel_control = velocity_control(
            self.desired_vel_latest, self.vehicle_v_actual, self.vel_error_sum, self.vel_error_last,
            self.cmd_dt, self.v_max, self.acc_max_forward, self.desired_vel_latest)
        # Publish velocity control debug information
        self.pid_debug_vel_control.publish(pid_debug_vel_control)
        self.desired_vel_latest = target_vel_low_level

        if not self.guard_mode_auto_active:
            self.desired_vel_latest = 0.0

    def publish_control_commands(self):
        if abs(self.desired_vel_latest) < 0.01:
            self.desired_vel_latest = 0.0
            cmd_longitudinal_to_guard = 0.0
        else:
            cmd_longitudinal_to_guard = self.desired_vel_latest

        if abs(self.desired_steering_latest) < 0.001:
            self.desired_steering_latest = 0.0
            cmd_lateral_to_guard = 0.0
        else:
            cmd_lateral_to_guard = self.desired_steering_latest

        vel_msg = StampedControlGuardCmdLongitudinal()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.vel = cmd_longitudinal_to_guard
        vel_msg.mode_req = 4  # [2:"Standby", 4:"Use"]
        self.dd_vel_publisher.publish(vel_msg)

        steering_msg = StampedControlGuardCmdLateral()
        steering_msg.header.stamp = self.get_clock().now().to_msg()
        steering_msg.steering_angle_front = cmd_lateral_to_guard * 180 / math.pi  # in deg
        steering_msg.steering_angle_rear = 0.0
        self.dd_steering_ang_publisher.publish(steering_msg)

        self.latest_cmd_update_time = time.time()

    def timer_callback(self):
        self.cmd_dt = time.time() - self.latest_cmd_update_time

        self.low_level_velocity_control()
        self.publish_control_commands()

def main(args=None):
    rclpy.init(args=args)
    low_level_controller_node = LowLevelController()
    rclpy.spin(low_level_controller_node)
    low_level_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

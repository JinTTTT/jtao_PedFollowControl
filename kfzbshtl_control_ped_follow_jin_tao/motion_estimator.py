import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import math
from collections import deque

from kfzbshtl_msgs.msg import DecodedCanMsg
from std_msgs.msg import Float32

import tf2_ros


class MotionEstimator(Node):
    """
    MotionEstimator Node: Estimates the motion of the vehicle and pedestrian.

    - Receives vehicle speed and steering angle CAN bus message:
        - publishes the vehicle's steering angle and speed.
        - estimates the vehicle's pose
    - Receives pedestrian relative position data:
        - publishes pedestrian's relative distance and calculates
        - publishes estimated pedestrian's relative movement speed
    - With the pedestrian's relative position and vehicle's position:
        - publishes calculated pedestrian's global coordinates
        - publishes calculated historical path information of the pedestrian and the vehicle
        - publishes pedestrian's relative speed and global speed
    """

    def __init__(self):
        super().__init__('motion_estimator')

        # Parameter initialization
        self.initialize_parameters()  # Renamed method
        self.initialize_variables()

        # Initialize subscribers and publishers
        self.init_subscribers()
        self.init_publishers()

        # Create a timer
        self.timer = self.create_timer(self.pub_time_diff, self.timer_callback)

    def initialize_parameters(self):
        """Declare and retrieve parameters."""
        self.declare_parameter('wheelbase', 2.9)
        self.declare_parameter('wheelradius', 0.34)
        self.declare_parameter('steering_factor', 375.9 / 24.3 / 1.014492675339367)

        self.L = self.get_parameter('wheelbase').value
        self.r_wheels = self.get_parameter('wheelradius').value
        self.steering_factor = self.get_parameter('steering_factor').value

    def initialize_variables(self):
        """Initialize variables."""
        # Vehicle state
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_v_actual = 0.0
        self.vehicle_steering_actual = 0.0

        # Pedestrian state
        self.global_ped_x = 0.0
        self.global_ped_y = 0.0
        self.ped_global_speed = 0.0  # New variable for pedestrian's global speed

        # Timestamps
        self.last_ped_data_time = self.get_clock().now()
        self.last_vel_data_time = self.get_clock().now()
        self.last_steering_data_time = self.get_clock().now()
        self.last_pub_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.pub_time_diff = 0.05  # Publish frequency

        # Flags
        self.received_vehicle_vel = False
        self.received_vehicle_steering = False
        self.received_ped_data = False

        # Filter parameters
        self.position_alpha = 0.2  # Position filter parameter
        self.speed_window_size = 40  # Relative speed filter window size
        self.relative_speed_buffer = deque(maxlen=self.speed_window_size)
        self.filtered_ped_rel_x = None
        self.filtered_ped_rel_y = None
        self.filtered_relative_speed = 0.0
        self.prev_distance = None
        self.max_acceleration = 0.5  # Maximum acceleration

        # Path initialization
        self.vehicle_path = Path()
        self.vehicle_path.header.frame_id = 'world'
        self.pedestrian_path = Path()
        self.pedestrian_path.header.frame_id = 'world'

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def init_subscribers(self):
        """Initialize subscribers."""
        self.create_subscription(DecodedCanMsg, '/can/from_can_bus/msgid451', self.vehicle_vel_callback, 1)
        self.create_subscription(DecodedCanMsg, '/can/from_can_bus/msgid1281', self.vehicle_steering_callback, 1)
        self.create_subscription(PoseStamped, '/filtered_obj', self.ped_object_callback, 1)

    def init_publishers(self):
        """Initialize publishers."""
        self.ped_global_pos_pub = self.create_publisher(PoseStamped, '/pose/ped_global_pose', 1)
        self.vehicle_global_pos_pub = self.create_publisher(PoseStamped, '/pose/bus_global_pose', 1)
        self.vehicle_vel_pub = self.create_publisher(Float32, 'vehicle_v_actual', 1)
        self.vehicle_steering_pub = self.create_publisher(Float32, 'vehicle_steering_actual', 1)
        self.relative_speed_pub = self.create_publisher(Float32, 'ped_relative_speed', 1)
        self.distance_pub = self.create_publisher(Float32, '/distance_to_ped', 1)
        self.vehicle_path_pub = self.create_publisher(Path, '/path/bus_path', 1)
        self.pedestrian_path_pub = self.create_publisher(Path, '/path/ped_path', 1)
        self.ped_global_speed_pub = self.create_publisher(Float32, '/ped_global_speed', 1)  # New publisher

    def vehicle_vel_callback(self, msg):
        """Vehicle speed callback."""
        self.received_vehicle_vel = True
        self.last_vel_data_time = self.get_clock().now()

        # Calculate the actual vehicle speed
        avg_wheel_speed = sum(msg.sgnl_val[:4]) / 4
        vehicle_speed_kmh = (avg_wheel_speed * 2 * math.pi * self.r_wheels * 60) / 1000
        self.vehicle_v_actual = vehicle_speed_kmh / 3.6
        if self.vehicle_v_actual < 0.01:
            self.vehicle_v_actual = 0.0

        self.publish_vehicle_speed()

    def vehicle_steering_callback(self, msg):
        """Vehicle steering angle callback."""
        self.received_vehicle_steering = True
        self.last_steering_data_time = self.get_clock().now()

        steering_angle_deg = msg.sgnl_val[0] / self.steering_factor
        self.vehicle_steering_actual = math.radians(steering_angle_deg)
        if abs(self.vehicle_steering_actual) < 0.0001:
            self.vehicle_steering_actual = 0.0

        self.publish_vehicle_steering()

    def ped_object_callback(self, msg):
        """Pedestrian relative position callback."""
        self.received_ped_data = True
        self.last_ped_data_time = self.get_clock().now()

        # Retrieve pedestrian relative position from 'filtered_obj' topic
        ped_rel_x = msg.pose.position.x - 0.5  # Adjusted for coordinate frame offset
        ped_rel_y = msg.pose.position.y

        # Calculate and publish distance to pedestrian
        self.calculate_and_publish_distance(ped_rel_x, ped_rel_y)

        # Apply position filter (for speed calculation)
        self.apply_position_filter(ped_rel_x, ped_rel_y)

        # Calculate and publish relative speed
        self.calculate_and_publish_relative_speed()

        # Calculate and publish pedestrian global position
        self.calculate_and_publish_ped_global_position(ped_rel_x, ped_rel_y)

        # Calculate and publish pedestrian global speed
        self.calculate_and_publish_ped_global_speed()

    def apply_position_filter(self, ped_rel_x, ped_rel_y):
        """Apply position filter for speed calculation."""
        if self.filtered_ped_rel_x is None:
            self.filtered_ped_rel_x = ped_rel_x
            self.filtered_ped_rel_y = ped_rel_y
        else:
            alpha = self.position_alpha
            self.filtered_ped_rel_x = alpha * ped_rel_x + (1 - alpha) * self.filtered_ped_rel_x
            self.filtered_ped_rel_y = alpha * ped_rel_y + (1 - alpha) * self.filtered_ped_rel_y

    def calculate_and_publish_relative_speed(self):
        """Calculate and publish relative speed."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        filtered_ped_rel_x = self.filtered_ped_rel_x
        filtered_ped_rel_y = self.filtered_ped_rel_y
        current_distance = math.hypot(filtered_ped_rel_x, filtered_ped_rel_y)

        if self.prev_distance is None:
            self.prev_distance = current_distance
            self.filtered_relative_speed = 0.0
        else:
            if dt > 0:
                delta_distance = current_distance - self.prev_distance
                max_delta_distance = max(self.max_acceleration * dt ** 2, 0.05)
                delta_distance = max(min(delta_distance, max_delta_distance), -max_delta_distance)
                relative_speed = delta_distance / dt
                self.relative_speed_buffer.append(relative_speed)
                self.filtered_relative_speed = sum(self.relative_speed_buffer) / len(self.relative_speed_buffer)
            else:
                self.filtered_relative_speed = 0.0
            self.prev_distance = current_distance

        # Dead zone handling
        if abs(self.filtered_relative_speed) < 0.1:
            self.filtered_relative_speed = 0.0

        # Publish relative speed
        relative_speed_msg = Float32()
        relative_speed_msg.data = self.filtered_relative_speed
        self.relative_speed_pub.publish(relative_speed_msg)

    def calculate_and_publish_distance(self, ped_rel_x, ped_rel_y):
        """Calculate and publish the distance."""
        distance = math.hypot(ped_rel_x, ped_rel_y)
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

    def calculate_and_publish_ped_global_position(self, ped_rel_x, ped_rel_y):
        """Calculate and publish pedestrian global position."""
        cos_yaw = math.cos(self.vehicle_yaw)
        sin_yaw = math.sin(self.vehicle_yaw)

        global_ped_x = self.vehicle_x + ped_rel_x * cos_yaw - ped_rel_y * sin_yaw
        global_ped_y = self.vehicle_y + ped_rel_x * sin_yaw + ped_rel_y * cos_yaw

        self.global_ped_x = global_ped_x
        self.global_ped_y = global_ped_y

        ped_global_pose = PoseStamped()
        ped_global_pose.header.stamp = self.get_clock().now().to_msg()
        ped_global_pose.header.frame_id = 'world'
        ped_global_pose.pose.position.x = global_ped_x
        ped_global_pose.pose.position.y = global_ped_y
        self.ped_global_pos_pub.publish(ped_global_pose)

        # Update and publish pedestrian path
        self.pedestrian_path.header.stamp = self.get_clock().now().to_msg()
        self.pedestrian_path.poses.append(ped_global_pose)
        self.pedestrian_path_pub.publish(self.pedestrian_path)

    def calculate_and_publish_ped_global_speed(self):
        """Calculate and publish pedestrian global speed."""
        # The pedestrian's global speed is the vehicle's speed plus the relative speed
        ped_global_speed = self.vehicle_v_actual + self.filtered_relative_speed
        self.ped_global_speed = ped_global_speed

        # Publish pedestrian global speed
        ped_global_speed_msg = Float32()
        ped_global_speed_msg.data = ped_global_speed
        self.ped_global_speed_pub.publish(ped_global_speed_msg)

    def publish_vehicle_speed(self):
        """Publish vehicle speed."""
        vehicle_speed_msg = Float32()
        vehicle_speed_msg.data = self.vehicle_v_actual
        self.vehicle_vel_pub.publish(vehicle_speed_msg)

    def publish_vehicle_steering(self):
        """Publish vehicle steering angle."""
        vehicle_steering_msg = Float32()
        vehicle_steering_msg.data = math.degrees(self.vehicle_steering_actual)
        self.vehicle_steering_pub.publish(vehicle_steering_msg)

    def update_and_publish_vehicle_position(self):
        """Update and publish vehicle global position."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_pub_time).nanoseconds / 1e9
        self.last_pub_time = current_time

        # Update vehicle position and yaw
        self.vehicle_x += self.vehicle_v_actual * math.cos(self.vehicle_yaw) * dt
        self.vehicle_y += self.vehicle_v_actual * math.sin(self.vehicle_yaw) * dt
        self.vehicle_yaw += (self.vehicle_v_actual / self.L) * math.tan(self.vehicle_steering_actual) * dt

        # Publish vehicle global position
        vehicle_global_pose = PoseStamped()
        vehicle_global_pose.header.stamp = current_time.to_msg()
        vehicle_global_pose.header.frame_id = 'world'
        vehicle_global_pose.pose.position.x = self.vehicle_x
        vehicle_global_pose.pose.position.y = self.vehicle_y
        vehicle_global_pose.pose.orientation.z = math.sin(self.vehicle_yaw / 2.0)
        vehicle_global_pose.pose.orientation.w = math.cos(self.vehicle_yaw / 2.0)
        self.vehicle_global_pos_pub.publish(vehicle_global_pose)

        # Update and publish vehicle path
        self.vehicle_path.header.stamp = current_time.to_msg()
        self.vehicle_path.poses.append(vehicle_global_pose)
        self.vehicle_path_pub.publish(self.vehicle_path)

        # Broadcast vehicle TF
        self.broadcast_vehicle_tf(current_time)

    def broadcast_vehicle_tf(self, current_time):
        """Broadcast vehicle coordinate transform (TF)."""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'vehicle'
        t.transform.translation.x = self.vehicle_x
        t.transform.translation.y = self.vehicle_y
        t.transform.rotation.z = math.sin(self.vehicle_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.vehicle_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def check_vehicle_data_ready(self):
        """Check if vehicle data is received in time."""
        time_now = self.get_clock().now()
        vel_duration = (time_now - self.last_vel_data_time).nanoseconds / 1e9
        steering_duration = (time_now - self.last_steering_data_time).nanoseconds / 1e9

        self.received_vehicle_vel = vel_duration <= 1.0
        self.received_vehicle_steering = steering_duration <= 1.0

        return self.received_vehicle_vel and self.received_vehicle_steering

    def check_pedestrian_data_ready(self):
        """Check if pedestrian data is received in time."""
        time_now = self.get_clock().now()
        ped_duration = (time_now - self.last_ped_data_time).nanoseconds / 1e9
        self.received_ped_data = ped_duration <= 1.0

        if not self.received_ped_data:
            # Reset filters
            self.filtered_ped_rel_x = None
            self.filtered_ped_rel_y = None
            self.filtered_relative_speed = 0.0
            self.prev_distance = None

        return self.received_ped_data

    def timer_callback(self):
        """Timer callback function."""
        if self.check_vehicle_data_ready():
            self.update_and_publish_vehicle_position()

        self.check_pedestrian_data_ready()

        # Additional processing can be added here


def main(args=None):
    rclpy.init(args=args)
    node = MotionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

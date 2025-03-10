import rclpy
from rclpy.node import Node
from udp_msgs.msg import CustomMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, Bool, Int32
from tf2_ros import TransformBroadcaster
import tf2_ros
import math
import random

class PedestrianPositionPub(Node):
    def __init__(self):
        super().__init__('pedestrian_pos_pub_node')

        # Global pedestrian position and velocity
        self.global_pedestrian_x = 0.0
        self.global_pedestrian_y = 0.0
        self.global_pedestrian_velocity = 0.0  # Desired speed from external input
        self.current_velocity = 0.0  # Current speed of the pedestrian
        self.desired_velocity = 0.0  # Target speed to accelerate/decelerate to
        self.max_acceleration = 0.6  # Maximum acceleration (m/s^2)
        self.dt = 0.05
        self.is_moving = False
        self.is_visible = True

        # Noise parameters for X and Y coordinates
        self.noise_mean_x = 0.0  # Mean of noise for X coordinate
        self.noise_std_x = 0.0759  # Standard deviation of noise for X coordinate
        self.noise_mean_y = 0.0  # Mean of noise for Y coordinate
        self.noise_std_y = 0.1168  # Standard deviation of noise for Y coordinate

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.ped_rel_pose_publisher = self.create_publisher(CustomMessage, 'udp_topic', 1)
        self.ped_input_subscription = self.create_subscription(
            PoseStamped,
            'pedestrian_input',
            self.pedestrian_input_callback,
            10
        )
        self.velocity_subscription = self.create_subscription(
            Float32,
            'pedestrian_velocity',
            self.velocity_callback,
            10
        )
        self.movement_subscription = self.create_subscription(
            Bool,
            'pedestrian_movement',
            self.movement_callback,
            10
        )
        self.visibility_subscription = self.create_subscription(
            Bool,
            'pedestrian_visibility',
            self.visibility_callback,
            10
        )
        # Subscriber for fake_gesture_id
        self.gesture_id_subscription = self.create_subscription(
            Int32,
            'fake_gesture_id',
            self.gesture_id_callback,
            10
        )
        # Variable to store the current gesture ID
        self.current_gesture_id = 0
        
        # Timer for periodic position updates
        self.timer = self.create_timer(self.dt, self.publish_pedestrian_position)
    
    def pedestrian_input_callback(self, msg):
        self.global_pedestrian_x = msg.pose.position.x
        self.global_pedestrian_y = msg.pose.position.y
        self.get_logger().info(f'Received global pedestrian position: x = {self.global_pedestrian_x}, y = {self.global_pedestrian_y}')

    def velocity_callback(self, msg):
        self.global_pedestrian_velocity = msg.data
        if self.is_moving:
            self.desired_velocity = self.global_pedestrian_velocity
        self.get_logger().info(f'Received pedestrian velocity: {self.global_pedestrian_velocity}')

    def movement_callback(self, msg):
        self.is_moving = msg.data
        if self.is_moving:
            self.desired_velocity = self.global_pedestrian_velocity
        else:
            self.desired_velocity = 0.0
        self.get_logger().info(f'Pedestrian movement {"started" if self.is_moving else "suspended"}')

    def visibility_callback(self, msg):
        self.is_visible = msg.data
        self.get_logger().info(f'Pedestrian {"visible" if self.is_visible else "invisible"}')
    
    def gesture_id_callback(self, msg):
        self.current_gesture_id = msg.data
        self.get_logger().info(f'Received gesture ID: {self.current_gesture_id}')

    def publish_pedestrian_position(self):
        if self.is_visible:
            # Update current velocity towards desired velocity with acceleration limit
            delta_v = self.desired_velocity - self.current_velocity
            max_delta_v = self.max_acceleration * self.dt
            if delta_v > max_delta_v:
                delta_v = max_delta_v
            elif delta_v < -max_delta_v:
                delta_v = -max_delta_v
            self.current_velocity += delta_v

            # Update position based on current velocity
            self.global_pedestrian_x += self.current_velocity * self.dt

            self.publish_global_position()
            self.publish_relative_position()
        else:
            self.publish_invisible_position()

    def publish_global_position(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "fake_global_pedestrian"
        t.transform.translation.x = self.global_pedestrian_x
        t.transform.translation.y = self.global_pedestrian_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_relative_position(self):
        from_frame = 'vehicle'
        to_frame = 'fake_global_pedestrian'

        try:
            t = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #self.get_logger().warn('Transform not available.')
            return

        ped_rel_x = t.transform.translation.x - 0.3
        ped_rel_y = t.transform.translation.y

        # Add Gaussian noise
        noise_x = random.gauss(self.noise_mean_x, self.noise_std_x)
        noise_y = random.gauss(self.noise_mean_y, self.noise_std_y)

        ped_rel_x += noise_x
        ped_rel_y += noise_y

        pedestrian_position = CustomMessage()
        pedestrian_position.msg_type = 1
        pedestrian_position.size = 400
        pedestrian_position.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        pedestrian_position.joint_id = [0] * 24
        pedestrian_position.tx = [0.0] * 24
        pedestrian_position.ty = [0.0] * 24
        pedestrian_position.tz = [0.0] * 24
                
        pedestrian_position.tx[0] = ped_rel_x
        pedestrian_position.ty[0] = ped_rel_y
        pedestrian_position.gesture_id = self.current_gesture_id  # Use the received gesture ID

        self.ped_rel_pose_publisher.publish(pedestrian_position)

    def publish_invisible_position(self):
        pedestrian_position = CustomMessage()
        pedestrian_position.msg_type = 1
        pedestrian_position.size = 400
        pedestrian_position.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        pedestrian_position.joint_id = [0] * 24
        pedestrian_position.tx = [0.0] * 24
        pedestrian_position.ty = [0.0] * 24
        pedestrian_position.tz = [0.0] * 24
                
        pedestrian_position.tx[0] = 0.0
        pedestrian_position.ty[0] = 0.0
        pedestrian_position.gesture_id = self.current_gesture_id  # Use the received gesture ID even when invisible

        self.ped_rel_pose_publisher.publish(pedestrian_position)
        
def main(args=None):
    rclpy.init(args=args)
    pedestrian_position_pub_node = PedestrianPositionPub()
    rclpy.spin(pedestrian_position_pub_node)
    pedestrian_position_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

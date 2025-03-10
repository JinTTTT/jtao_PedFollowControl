import rclpy
from rclpy.node import Node
from udp_msgs.msg import CustomMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import time

class TargetSelector(Node):
    """
    Use this node to select the nearest object from the received UDP data and publish its position and gesture ID.
    """
    def __init__(self):
        super().__init__('get_nearest_obj')

        # Subscriptions and Publishers
        self.subscription = self.create_subscription(CustomMessage, '/udp_topic', self.udp_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/filtered_obj', 10)
        self.raw_publisher_ = self.create_publisher(PoseStamped, '/raw_obj', 10)
        self.gesture_publisher = self.create_publisher(Int32, 'gesture_id', 10)
        
        # Time window for grouping detections
        self.time_window = 0.05  # 50 ms
        self.last_time = self.get_clock().now()
        self.objects_in_window = []
        self.last_received_time = self.get_clock().now()

        # Pre-allocated PoseStamped messages
        self.raw_msg = PoseStamped()
        self.filtered_msg = PoseStamped()
        self.raw_msg.header.frame_id = 'world'
        self.filtered_msg.header.frame_id = 'world'

        # Flags and timers
        self.received_udp_topic = False
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz status check

        # Logging flags
        self.log_no_udp = True
        self.log_udp_ready = True

    def udp_callback(self, msg):
        current_time = self.get_clock().now()
        
        if not msg.tx[0] or msg.tx[0] < 0:
            return

        time_diff = (current_time - self.last_received_time).nanoseconds / 1e9
        if time_diff > 1.0:
            self.objects_in_window.clear()

        self.last_received_time = current_time

        self.received_udp_topic = True
        
        # Update and publish the raw object position
        self.raw_msg.header.stamp = current_time.to_msg()
        self.raw_msg.pose.position.x = msg.tx[0]
        self.raw_msg.pose.position.y = msg.ty[0]
        self.raw_msg.pose.position.z = 0.0
        self.raw_publisher_.publish(self.raw_msg)

        time_diff = (current_time - self.last_time).nanoseconds / 1e9

        if time_diff < self.time_window:
            self.objects_in_window.append([msg.tx[0], msg.ty[0], msg.gesture_id])
        else:
            if self.objects_in_window:
                filtered_obj, gesture_id = self.select_target_obj(self.objects_in_window)
                self.publish_pose(filtered_obj)
                self.publish_gesture_id(gesture_id)
            self.objects_in_window = [[msg.tx[0], msg.ty[0], msg.gesture_id]]
            self.last_time = current_time

    def select_target_obj(self, objects):
        # Select the closest object and return its position and gesture ID
        min_distance = float('inf')
        target_obj = None
        target_gesture_id = 0

        for obj in objects:
            x, y, gesture_id = obj
            distance = x**2 + y**2
            if distance < min_distance:
                min_distance = distance
                target_obj = [x, y]
                target_gesture_id = gesture_id

        return target_obj, target_gesture_id

    def publish_pose(self, filtered_obj):
        self.filtered_msg.header.stamp = self.get_clock().now().to_msg()
        self.filtered_msg.pose.position.x = filtered_obj[0]
        self.filtered_msg.pose.position.y = filtered_obj[1]
        self.filtered_msg.pose.position.z = 0.0
        self.filtered_msg.pose.orientation.w = 1.0
        self.publisher_.publish(self.filtered_msg)

    def publish_gesture_id(self, gesture_id):
        gesture_msg = Int32()
        gesture_msg.data = gesture_id
        self.gesture_publisher.publish(gesture_msg)

    def timer_callback(self):
        current_time = self.get_clock().now()
        if not self.received_udp_topic or (current_time - self.last_time).nanoseconds / 1e9 > 1.0:
            if self.log_no_udp:
                #self.get_logger().warn('TargetSelector not ready: No UDP data received!')
                self.log_no_udp = False
            self.log_udp_ready = True
        else:
            if self.log_udp_ready:
                #self.get_logger().info('TargetSelector is ready and receiving UDP data.')
                self.log_udp_ready = False
            self.log_no_udp = True

def main(args=None):
    rclpy.init(args=args)
    target_selector = TargetSelector()
    rclpy.spin(target_selector)
    target_selector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

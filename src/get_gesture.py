import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import time
from collections import deque

class GestureFilter(Node):
    def __init__(self):
        super().__init__('get_gesture')

        # Subscribers and Publishers
        self.gesture_subscriber = self.create_subscription(Int32, 'gesture_id', self.gesture_id_callback, 10)
        self.filtered_gesture_publisher = self.create_publisher(Int32, 'filtered_gesture_id', 10)
        self.certainty_publisher = self.create_publisher(Float32, 'gesture_certainty', 10)

        # Gesture recognition variables
        self.gesture_ids = deque()
        self.gesture_window_time = 1.0  # 1 second window, approximately 20 gesture IDs
        self.last_log_time = 0
        self.last_filtered_gesture_id = 0  # Initialize last filtered gesture ID

        # Timer to check and publish filtered gesture
        self.timer = self.create_timer(0.1, self.timer_callback)

    def gesture_id_callback(self, msg):
        current_time = time.time()
        self.gesture_ids.append((current_time, msg.data))
        # Remove old gesture_ids outside the window
        while self.gesture_ids and current_time - self.gesture_ids[0][0] > self.gesture_window_time:
            self.gesture_ids.popleft()

    def check_gesture_signal(self):
        if not self.gesture_ids:
            return self.last_filtered_gesture_id, 0.0  # No data, return last gesture ID and 0% certainty

        current_time = time.time()
        # Only consider gestures within the last second
        gestures_in_window = [gid for t, gid in self.gesture_ids if current_time - t <= self.gesture_window_time]
        total_gestures = len(gestures_in_window)
        if total_gestures == 0:
            return self.last_filtered_gesture_id, 0.0

        gesture_counts = {}
        for gid in gestures_in_window:
            if gid not in gesture_counts:
                gesture_counts[gid] = 1
            else:
                gesture_counts[gid] += 1

        # Find the gesture ID with the highest count
        most_common_gesture = max(gesture_counts, key=gesture_counts.get)
        max_count = gesture_counts[most_common_gesture]
        gesture_ratio = max_count / total_gestures  # Certainty ratio

        # Log the gesture ID and certainty ratio every 1 seconds
        if current_time - self.last_log_time >= 1.0:
            #self.get_logger().info(f'Gesture ID {most_common_gesture}: {max_count}/{total_gestures} ({gesture_ratio*100:.1f}%)')
            self.last_log_time = current_time

        # If the most common gesture is detected more than 70% of the time, consider it as the filtered gesture
        if gesture_ratio >= 0.5:
            self.last_filtered_gesture_id = most_common_gesture
            return most_common_gesture, gesture_ratio
        else:
            # Keep the last filtered gesture ID
            return self.last_filtered_gesture_id, gesture_ratio

    def timer_callback(self):
        filtered_gesture_id, gesture_ratio = self.check_gesture_signal()
        #filtered_gesture_id = 2  # "start" gesture

        # Publish the filtered gesture ID
        gesture_msg = Int32()
        gesture_msg.data = filtered_gesture_id
        #gesture_msg.data = 2 # "start" gesture
        self.filtered_gesture_publisher.publish(gesture_msg)

        # Publish the certainty percentage
        certainty_msg = Float32()
        certainty_msg.data = gesture_ratio * 100  # Convert to percentage
        self.certainty_publisher.publish(certainty_msg)

    def destroy_node(self):
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    gesture_filter_node = GestureFilter()
    rclpy.spin(gesture_filter_node)
    gesture_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

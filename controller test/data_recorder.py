import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from kfzbshtl_msgs.msg import StampedControlGuardCmdLongitudinal
import csv
from collections import deque

class DataRecorder(Node):

    def __init__(self):
        super().__init__('data_recorder_node')
        self.actual_vel_sub = self.create_subscription(Float32, '/vehicle_v_actual', self.actual_vel_callback, 10)
        self.target_vel_sub = self.create_subscription(StampedControlGuardCmdLongitudinal, '/plan/control_cmd_long', self.target_vel_callback, 10)
        
        self.actual_data = deque()
        self.target_data = deque()
        self.data = []

    def actual_vel_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.actual_data.append((timestamp, msg.data))
        self.match_and_record()

    def target_vel_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.target_data.append((timestamp, msg.vel))
        self.match_and_record()

    def match_and_record(self):
        while self.actual_data and self.target_data:
            actual_time, actual_value = self.actual_data[0]
            target_time, target_value = self.target_data[0]

            if abs(actual_time - target_time) < 0.01:  # 时间戳对齐的误差阈值
                self.data.append({'timestamp': actual_time, 'type': 'actual', 'value': actual_value})
                self.data.append({'timestamp': target_time, 'type': 'target', 'value': target_value})
                self.actual_data.popleft()
                self.target_data.popleft()
            elif actual_time < target_time:
                self.actual_data.popleft()
            else:
                self.target_data.popleft()

    def save_data(self):
        self.get_logger().info(f'Saving {len(self.data)} records to ros2_bag_data.csv')
        with open('ros2_bag_data.csv', mode='w') as file:
            writer = csv.DictWriter(file, fieldnames=['timestamp', 'type', 'value'])
            writer.writeheader()
            for row in self.data:
                writer.writerow(row)
        self.get_logger().info('Data saved successfully.')

def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Keyboard Interrupt detected! Stopping the node.')
    finally:
        recorder.save_data()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

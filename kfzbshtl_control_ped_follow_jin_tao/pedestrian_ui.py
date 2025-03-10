import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PedestrianUI(Node):
    def __init__(self):
        super().__init__('pedestrian_ui_node')
        self.publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        self.window = tk.Tk()
        self.window.title("Pedestrian Position Input")
        
        tk.Label(self.window, text="X Coordinate:").grid(row=0, column=0)
        self.x_entry = tk.Entry(self.window)
        self.x_entry.grid(row=0, column=1)
        
        tk.Label(self.window, text="Y Coordinate:").grid(row=1, column=0)
        self.y_entry = tk.Entry(self.window)
        self.y_entry.grid(row=1, column=1)
        
        tk.Button(self.window, text="Submit", command=self.submit_coordinates).grid(row=2, column=0, columnspan=2)

    def submit_coordinates(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.pose.position.x = x
            msg.pose.position.y = y
            
            self.publisher.publish(msg)
            print(f"Published coordinates: x={x}, y={y}")
        except ValueError:
            print("Please enter valid numbers for coordinates.")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.window.update()

def main():
    rclpy.init()
    ui = PedestrianUI()
    ui.run()
    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool, Int32
import tkinter as tk
from tkinter import messagebox
import threading
import time

class InputUI(Node):
    def __init__(self):
        super().__init__('input_fake_ped_ui')
        self.pose_publisher = self.create_publisher(PoseStamped, 'pedestrian_input', 10)
        self.velocity_publisher = self.create_publisher(Float32, 'pedestrian_velocity', 10)
        self.movement_publisher = self.create_publisher(Bool, 'pedestrian_movement', 10)
        self.visibility_publisher = self.create_publisher(Bool, 'pedestrian_visibility', 10)
        self.gesture_publisher = self.create_publisher(Int32, 'fake_gesture_id', 10)
        
        self.window = tk.Tk()
        self.window.title("Pedestrian Control UI")
        self.window.geometry("450x350")
        self.window.resizable(False, False)
        
        # Existing UI elements for pedestrian control
        control_frame = tk.Frame(self.window)
        control_frame.pack(pady=10)
        
        position_frame = tk.Frame(control_frame)
        position_frame.pack()
        
        tk.Label(position_frame, text="X:").grid(row=0, column=0, padx=5, pady=5)
        self.x_entry = tk.Entry(position_frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)
        
        tk.Label(position_frame, text="Y:").grid(row=0, column=2, padx=5, pady=5)
        self.y_entry = tk.Entry(position_frame, width=10)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)
        
        tk.Label(position_frame, text="V(m/s):").grid(row=0, column=4, padx=5, pady=5)
        self.velocity_entry = tk.Entry(position_frame, width=10)
        self.velocity_entry.grid(row=0, column=5, padx=5, pady=5)
        
        submit_button = tk.Button(control_frame, text="Submit", command=self.submit_coordinates)
        submit_button.pack(pady=5)
        
        movement_frame = tk.Frame(control_frame)
        movement_frame.pack(pady=5)
        
        self.movement_button = tk.Button(movement_frame, text="Start/Stop Moving", width=15, command=self.toggle_movement)
        self.movement_button.grid(row=0, column=0, padx=5)
        
        visibility_button = tk.Button(movement_frame, text="Ped Disappear/Appear", width=15, command=self.toggle_visibility)
        visibility_button.grid(row=0, column=1, padx=5)
        
        # New UI elements for gesture input
        gesture_frame = tk.LabelFrame(self.window, text="Gesture Control", padx=10, pady=10)
        gesture_frame.pack(pady=10)
        
        self.create_gesture_button(gesture_frame, "Stop", 1, 0, 0)
        self.create_gesture_button(gesture_frame, "Start", 2, 0, 1)
        self.create_gesture_button(gesture_frame, "Left", 3, 1, 0)
        self.create_gesture_button(gesture_frame, "Right", 4, 1, 1)
        self.create_gesture_button(gesture_frame, "Slower", 5, 2, 0)
        self.create_gesture_button(gesture_frame, "Faster", 6, 2, 1)
        
        self.is_moving = False
        self.is_visible = True
        
        # Gesture publishing variables
        self.current_gesture_id = 0
        self.gesture_publishing = False

    def create_gesture_button(self, parent, text, gesture_id, row, column):
        button = tk.Button(parent, text=text, width=12,
                           command=lambda: self.start_gesture(gesture_id))
        button.grid(row=row, column=column, padx=5, pady=5)
        
    def submit_coordinates(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            velocity = float(self.velocity_entry.get())
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            
            velocity_msg = Float32()
            velocity_msg.data = velocity
            
            self.pose_publisher.publish(pose_msg)
            self.velocity_publisher.publish(velocity_msg)
            
            self.get_logger().info(f'Published pedestrian position: x = {x}, y = {y}, velocity = {velocity}')
            
            self.x_entry.delete(0, tk.END)
            self.y_entry.delete(0, tk.END)
            self.velocity_entry.delete(0, tk.END)
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values.")

    def toggle_movement(self):
        self.is_moving = not self.is_moving
        movement_msg = Bool()
        movement_msg.data = self.is_moving
        self.movement_publisher.publish(movement_msg)
        
        self.get_logger().info(f'Pedestrian movement {"started" if self.is_moving else "suspended"}')

    def toggle_visibility(self):
        self.is_visible = not self.is_visible
        visibility_msg = Bool()
        visibility_msg.data = self.is_visible
        self.visibility_publisher.publish(visibility_msg)
        self.get_logger().info(f'Pedestrian {"visible" if self.is_visible else "invisible"}')

    def start_gesture(self, gesture_id):
        if not self.gesture_publishing:
            self.current_gesture_id = gesture_id
            self.gesture_publishing = True
            threading.Thread(target=self.publish_gesture_for_duration, daemon=True).start()
            self.get_logger().info(f'Started gesture ID: {gesture_id}')
        else:
            messagebox.showinfo("Info", "A gesture is already being published.")

    def publish_gesture_for_duration(self):
        duration = 3  # Duration in seconds
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        while time.time() - start_time < duration and rclpy.ok():
            self.publish_gesture(self.current_gesture_id)
            rate.sleep()
        # After duration, reset gesture ID to 0
        self.publish_gesture(0)
        self.gesture_publishing = False
        self.get_logger().info(f'Gesture ID {self.current_gesture_id} ended')

    def publish_gesture(self, gesture_id):
        gesture_msg = Int32()
        gesture_msg.data = gesture_id
        self.gesture_publisher.publish(gesture_msg)
        # Optionally, update UI or log the gesture ID
        # self.get_logger().info(f'Published gesture ID: {gesture_id}')

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.window.update()

    def destroy_node(self):
        self.window.destroy()
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    input_ui = InputUI()
    try:
        input_ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        input_ui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

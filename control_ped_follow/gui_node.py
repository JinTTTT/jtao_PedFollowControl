import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import numpy as np
from collections import namedtuple

Pedestrian = namedtuple('Pedestrian', ['x', 'y'])
Vehicle = namedtuple('Vehicle', ['x', 'y', 'yaw'])

class GridMapUI(Node):
    VEHICLE_LENGTH = 2.9
    VEHICLE_WIDTH = 2.0
    UPDATE_INTERVAL = 100  # milliseconds
    VIEW_SIZE = 40  # Total view size in meters

    def __init__(self):
        super().__init__('grid_map_ui')
        self.create_subscriptions()

        # Declare and get parameters
        self.declare_parameter('track_distance', 10.0)
        self.declare_parameter('safe_distance', 6.0)
        self.declare_parameter('v_max', 2.0)

        self.TRACK_DISTANCE = self.get_parameter('track_distance').value
        self.SAFE_DISTANCE = self.get_parameter('safe_distance').value
        self.V_MAX = self.get_parameter('v_max').value

        self.pedestrian = None
        self.vehicle = None
        self.required_distance = None
        self.dist_to_ped_now = None
        self.vehicle_v_actual = None
        self.vehicle_steering_actual = None
        self.distance_rate = None  # Variable to store pedestrian relative speed
        self.gesture_id = None
        self.gesture_certainty = None

        self.setup_gui()
        self.setup_animation()

    def create_subscriptions(self):
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.pedestrian_sub = self.create_subscription(PoseStamped, 'pose/ped_global_pose', self.pedestrian_callback, qos_profile)
        self.vehicle_sub = self.create_subscription(PoseStamped, 'pose/bus_global_pose', self.vehicle_callback, qos_profile)
        self.dist_ped_sub = self.create_subscription(Float32, '/distance_to_ped', self.dist_ped_callback, 10)
        self.vehicle_v_sub = self.create_subscription(Float32, 'vehicle_v_actual', self.vehicle_v_callback, 10)
        self.vehicle_steering_sub = self.create_subscription(Float32, 'vehicle_steering_actual', self.vehicle_steering_callback, 10)
        self.distance_rate_sub = self.create_subscription(Float32, 'ped_relative_speed', self.distance_rate_callback, 10)
        self.required_distance_sub = self.create_subscription(Float32, 'required_distance', self.required_distance_callback, 10)
        # Subscriptions for gesture data
        self.gesture_id_sub = self.create_subscription(Int32, 'filtered_gesture_id', self.gesture_id_callback, 10)
        self.gesture_certainty_sub = self.create_subscription(Float32, 'gesture_certainty', self.gesture_certainty_callback, 10)

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Grid Map UI")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.fig, (self.ax, self.ax_info) = plt.subplots(1, 2, figsize=(10, 6), gridspec_kw={'width_ratios': [3, 1]})
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        # Initialize graphical elements
        self.vehicle_dot, = self.ax.plot([], [], 'ro', markersize=5)
        self.vehicle_shape, = self.ax.plot([], [], 'r-')
        self.pedestrian_dot, = self.ax.plot([], [], 'bo', markersize=10)
        self.vehicle_text = self.ax.text(0, 0, '', fontsize=9, ha='left', va='top')
        self.pedestrian_text = self.ax.text(0, 0, '', fontsize=9, ha='right', va='bottom')

        self.ax.set_frame_on(True)
        self.setup_info_panel()

        # Create background
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)

    def setup_info_panel(self):
        self.ax_info.axis('off')

        # Adjusted spacing between panels
        panel_spacing = 0.05  # Adjust this value for spacing between panels

        # Set Parameters
        y_start = 0.95
        self.ax_info.text(0.05, y_start, "Const Parameters:", fontsize=11, fontweight='bold')
        self.info_texts = {
            'safe_distance': self.ax_info.text(0.1, y_start - 0.05, f"Safe Distance: {self.SAFE_DISTANCE} m", fontsize=10),
            'track_distance': self.ax_info.text(0.1, y_start - 0.10, f"Track Distance: {self.TRACK_DISTANCE} m", fontsize=10),
            'v_max': self.ax_info.text(0.1, y_start - 0.15, f"Max Velocity: {self.V_MAX:.2f} m/s", fontsize=10),
        }

        # Increase spacing after Const Parameters
        y_start -= 0.25  # Increased spacing after Const Parameters

        # Pedestrian Info
        self.ax_info.text(0.05, y_start, "Info Distance:", fontsize=11, fontweight='bold')
        self.info_texts.update({
            'required_distance': self.ax_info.text(0.1, y_start - 0.05, "Required Distance: N/A", fontsize=10),
            'actual_distance': self.ax_info.text(0.1, y_start - 0.10, "Actual Distance: N/A", fontsize=10),
            'distance_rate': self.ax_info.text(0.1, y_start - 0.15, "Distance Rate: N/A", fontsize=10)
        })

        # Increase spacing after Pedestrian Info
        y_start -= 0.25  # Increased spacing between panels

        # Driving Commands
        self.ax_info.text(0.05, y_start, "Driving Commands:", fontsize=11, fontweight='bold')
        self.info_texts.update({
            'steering_angle': self.ax_info.text(0.1, y_start - 0.05, "Steering Angle: N/A", fontsize=10),
            'velocity': self.ax_info.text(0.1, y_start - 0.10, "Velocity: N/A", fontsize=10)
        })

        # Increase spacing after Driving Commands
        y_start -= 0.20  # Increased spacing between panels

        # Gesture Info
        self.ax_info.text(0.05, y_start, "Gesture:", fontsize=11, fontweight='bold')
        self.info_texts.update({
            'gesture_id': self.ax_info.text(0.1, y_start - 0.05, "Gesture ID: N/A", fontsize=10),
            'gesture_certainty': self.ax_info.text(0.1, y_start - 0.10, "Gesture Certainty: N/A", fontsize=10)
        })

    def setup_animation(self):
        self.ani = self.fig.canvas.new_timer(interval=self.UPDATE_INTERVAL)
        self.ani.add_callback(self.update_plot)
        self.ani.start()

    def vehicle_v_callback(self, msg):
        self.vehicle_v_actual = msg.data

    def vehicle_steering_callback(self, msg):
        self.vehicle_steering_actual = msg.data

    def pedestrian_callback(self, msg):
        self.pedestrian = Pedestrian(msg.pose.position.x, msg.pose.position.y)

    def vehicle_callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.vehicle = Vehicle(x, y, yaw)
    
    def required_distance_callback(self, msg):
        self.required_distance = msg.data

    def dist_ped_callback(self, msg):
        self.dist_to_ped_now = msg.data
    
    def distance_rate_callback(self, msg):
        self.distance_rate = msg.data

    def gesture_id_callback(self, msg):
        self.gesture_id = msg.data

    def gesture_certainty_callback(self, msg):
        self.gesture_certainty = msg.data

    def update_plot(self):
        try:
            # Restore background
            self.fig.canvas.restore_region(self.background)

            if self.vehicle:
                self.setup_axes()
                self.update_vehicle()
            if self.pedestrian:
                self.update_pedestrian()

            # Redraw dynamic elements
            self.ax.draw_artist(self.vehicle_dot)
            self.ax.draw_artist(self.vehicle_shape)
            self.ax.draw_artist(self.pedestrian_dot)
            self.ax.draw_artist(self.vehicle_text)
            self.ax.draw_artist(self.pedestrian_text)

            # Update canvas
            self.fig.canvas.blit(self.ax.bbox)
            self.fig.canvas.flush_events()

        except Exception as e:
            self.get_logger().error(f"Error in update_plot: {e}")

    def update_vehicle(self):
        self.vehicle_dot.set_data(self.vehicle.y, self.vehicle.x)
        corners = self.get_vehicle_corners()
        self.vehicle_shape.set_data(corners[:, 1], corners[:, 0])
        yaw_degrees = math.degrees(self.vehicle.yaw)
        self.vehicle_text.set_position((self.vehicle.y, self.vehicle.x))
        self.vehicle_text.set_text(f'Vehicle ({self.vehicle.x:.2f}, {self.vehicle.y:.2f}, {yaw_degrees:.2f}°)')

    def update_pedestrian(self):
        self.pedestrian_dot.set_data(self.pedestrian.y, self.pedestrian.x)
        self.pedestrian_text.set_position((self.pedestrian.y, self.pedestrian.x))
        self.pedestrian_text.set_text(f'Ped ({self.pedestrian.x:.2f}, {self.pedestrian.y:.2f})')

    def update_info(self):
        if self.required_distance is not None:
            self.info_texts['required_distance'].set_text(f"Required Distance: {self.required_distance:.2f} m")
        if self.dist_to_ped_now is not None:
            self.info_texts['actual_distance'].set_text(f"Actual Distance: {self.dist_to_ped_now:.2f} m")
        if self.distance_rate is not None:
            self.info_texts['distance_rate'].set_text(f"Distance Rate: {self.distance_rate:.2f} m/s")
        if self.vehicle_steering_actual is not None:
            self.info_texts['steering_angle'].set_text(f"Steering Angle: {self.vehicle_steering_actual:.2f} deg")
        if self.vehicle_v_actual is not None:
            self.info_texts['velocity'].set_text(f"Velocity: {self.vehicle_v_actual:.2f} m/s")
        # Update Gesture Information
        if self.gesture_id is not None:
            gesture_text = self.get_gesture_text(self.gesture_id)
            self.info_texts['gesture_id'].set_text(f"Gesture ID: {gesture_text}")
        if self.gesture_certainty is not None:
            certainty_percent = self.gesture_certainty  # Assuming certainty is between 0 and 1
            self.info_texts['gesture_certainty'].set_text(f"Gesture Certainty: {certainty_percent:.2f}%")

        self.ax_info.figure.canvas.draw()

    def get_gesture_text(self, gesture_id):
        if gesture_id == 1:
            return "Stop"
        elif gesture_id == 0:
            return "None"
        else:
            return "Start"

    def setup_axes(self):
        forward_offset = 10  # 20 meters ahead of the vehicle

        # Calculate the new center point
        center_x = self.vehicle.x + forward_offset * math.cos(self.vehicle.yaw)
        center_y = self.vehicle.y + forward_offset * math.sin(self.vehicle.yaw)

        self.ax.set_xlim(center_y + 17.5, center_y - 17.5)
        self.ax.set_ylim(center_x - 15, center_x + 15)
        self.ax.grid(True)
        self.ax.set_xlabel('Y')
        self.ax.set_ylabel('X')
        self.ax.set_aspect('equal', 'box')
        self.ax.set_title("Grid Map", fontsize=14)


    def get_vehicle_corners(self):
        corners = np.array([
            [0, -self.VEHICLE_WIDTH/2],
            [-self.VEHICLE_LENGTH, -self.VEHICLE_WIDTH/2],
            [-self.VEHICLE_LENGTH, self.VEHICLE_WIDTH/2],
            [0, self.VEHICLE_WIDTH/2],
            [0, -self.VEHICLE_WIDTH/2]  # Close the rectangle
        ])
        rotation_matrix = np.array([
            [np.cos(self.vehicle.yaw), -np.sin(self.vehicle.yaw)],
            [np.sin(self.vehicle.yaw), np.cos(self.vehicle.yaw)]
        ])
        rotated_corners = np.dot(corners, rotation_matrix.T)
        return rotated_corners + np.array([self.vehicle.x, self.vehicle.y])

    def on_closing(self):
        self.destroy_node()
        self.root.quit()
        self.root.destroy()

    def update(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.update_info()
        self.root.after(10, self.update)  # More frequent processing of ROS messages

    def run(self):
        self.update()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    ui_node = GridMapUI()

    try:
        ui_node.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    finally:
        ui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

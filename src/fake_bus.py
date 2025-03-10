import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

from kfzbshtl_msgs.msg import DecodedCanMsg
from kfzbshtl_msgs.msg import StampedControlGuardCmdLongitudinal, StampedControlGuardCmdLateral

import math
import numpy as np


class FakeBus(Node):

    """
    This code is used to simulate the behavior of the bus: velocity response and steering response.
    """

    def __init__(self):
        super().__init__('fake_bus_node')

        # Vehicle parameters
        self.mode = 1  # 0: no pt2, 1: with pt2
        self.magic_navya_steering_factor_ = 375.9 / 24.3 / 1.014492675339367  # 375.9/24.3 is initial guess, 1.01.. is correction factor from axes measurements
        self.L = 2.9  # Wheelbase [m]
        self.r_wheels_ = 0.34  # Tire radius [m]

        self.vehicle_x = 0.0  # Vehicle x position [m]
        self.vehicle_y = 0.0  # Vehicle y position [m]
        self.vehicle_yaw = 0.0  # Vehicle yaw angle [rad]
        self.pub_dt = 0.05  # Time step [s]

        self.vehicle_v_actual = 0.0  # Actual vehicle velocity [m/s]
        self.vehicle_steering_actual = 0.0  # Actual vehicle steering angle [rad]

        self.vel_past1 = 0.0  # Last time's velocity value
        self.vel_past2 = 0.0  # Last last time's velocity value
        self.steering_past1 = 0.0  # Last time's steering value
        self.steering_past2 = 0.0  # Last last time's steering value

        self.can_vel_steering_subscription = self.create_subscription(DecodedCanMsg, '/can/to_can_bus', self.can_vel_steering_callback, 10)
        
        self.actual_vel_publisher = self.create_publisher(DecodedCanMsg, '/can/from_can_bus/msgid451', 10)
        self.actual_steering_publisher = self.create_publisher(DecodedCanMsg, '/can/from_can_bus/msgid1281', 10)
        self.can_mode_publisher = self.create_publisher(DecodedCanMsg, '/can/from_can_bus/msgid1155', 10)
        
        self.timer = self.create_timer(self.pub_dt, self.publish_vehicle_can)  # Adjust the rate as needed

    def can_vel_steering_callback(self, msg):

        # pt2 filter
        # G(s) = 1 / (as^2 + bs + 1)
        
        # VELOCITY
        if msg.msg_id == 385:
            can_vel_input = msg.sgnl_val[0]
            can_request_mode = msg.sgnl_val[16]  # 2 standby, 4 use

            #self.get_logger().info("Received velocity: %f m/s" % can_vel_input)
            can_sgnl_request_mode = DecodedCanMsg()
            can_sgnl_request_mode.msg_id = 1155
            can_sgnl_request_mode.msg_name = "UCVE_M_ModeStates"
            can_sgnl_request_mode.msg_numel = 1
            can_sgnl_request_mode.sgnl_name = ["VehicleMode"]
            can_sgnl_request_mode.sgnl_val = [can_request_mode]
            self.can_mode_publisher.publish(can_sgnl_request_mode)
            
            pt2_vel_a = 0.3969
            pt2_vel_b = 0.2394

            T_s = 0.05

            # Difference equation parameters
            A_pt2_vel = pt2_vel_a
            B_pt2_vel = -(pt2_vel_b * T_s - 2 * pt2_vel_a)
            C_pt2_vel = -(pt2_vel_a - pt2_vel_b * T_s + T_s**2)
            D_pt2_vel = T_s**2

            actual_velocity = (D_pt2_vel * can_vel_input + B_pt2_vel * self.vel_past1 + C_pt2_vel * self.vel_past2) / A_pt2_vel
            self.vel_past2 = self.vel_past1
            self.vel_past1 = actual_velocity

            if self.mode == 0:
                self.vehicle_v_actual = can_vel_input
            else:
                self.vehicle_v_actual = actual_velocity  # Use the filtered value

        # STEERING
        if msg.msg_id == 1280:
            
            can_steering_input = msg.sgnl_val[5] 
            pt2_steering_a = 0.0182
            pt2_steering_b = 0.1796

            T_s = 0.05

            # Difference equation parameters
            A_pt2_steering = pt2_steering_a
            B_pt2_steering = -(pt2_steering_b * T_s - 2 * pt2_steering_a)
            C_pt2_steering = -(pt2_steering_a - pt2_steering_b * T_s + T_s**2)
            D_pt2_steering = T_s**2

            actual_steering = (D_pt2_steering * can_steering_input + B_pt2_steering * self.steering_past1 + C_pt2_steering * self.steering_past2) / A_pt2_steering
            self.steering_past2 = self.steering_past1
            self.steering_past1 = actual_steering

            if self.mode == 0:
                self.vehicle_steering_actual = can_steering_input
            else:
                self.vehicle_steering_actual = actual_steering  # Use the filtered value


    def publish_vehicle_can(self):
        # Publish steering
        can_sgnl_steering = DecodedCanMsg()
       
        sgnl_val = self.vehicle_steering_actual
        
        can_sgnl_steering.sgnl_name = ["steering_angle_front", "steering_angle_rear"]
        can_sgnl_steering.sgnl_val = [0.0] * 2

        can_sgnl_steering.sgnl_val[0] = sgnl_val
        #self.get_logger().info("Publishing navya steering: %f deg" % (sgnl_val))
        can_sgnl_steering.sgnl_val[1] = 0.0

        can_sgnl_steering.header.stamp = self.get_clock().now().to_msg()
        can_sgnl_steering.header.frame_id = "base_link"
        can_sgnl_steering.msg_id = 1281
        can_sgnl_steering.msg_name = "SteeringAngles"
        can_sgnl_steering.msg_numel = 2

        self.actual_steering_publisher.publish(can_sgnl_steering)

        # Publish velocity
        can_sgnl_velocity = DecodedCanMsg()

        vel_kmh = self.vehicle_v_actual * 3.6
        speed_wheel = 1000 * vel_kmh / (2 * math.pi * self.r_wheels_ * 60)

        can_sgnl_velocity.sgnl_name = ["wheel_speed_fl", "wheel_speed_fr", "wheel_speed_rl", "wheel_speed_rr"]
        can_sgnl_velocity.sgnl_val = [0.0] * 4

        can_sgnl_velocity.sgnl_val[0] = speed_wheel
        can_sgnl_velocity.sgnl_val[1] = speed_wheel
        can_sgnl_velocity.sgnl_val[2] = speed_wheel
        can_sgnl_velocity.sgnl_val[3] = speed_wheel

        can_sgnl_velocity.header.stamp = self.get_clock().now().to_msg()
        can_sgnl_velocity.header.frame_id = "base_link"
        can_sgnl_velocity.msg_id = 451
        can_sgnl_velocity.msg_name = "WheelSpeeds"
        can_sgnl_velocity.msg_numel = 4

        self.actual_vel_publisher.publish(can_sgnl_velocity)


def main(args=None):
    rclpy.init(args=args)
    fake_bus_node = FakeBus()
    rclpy.spin(fake_bus_node)
    fake_bus_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

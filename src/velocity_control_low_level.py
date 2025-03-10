import rclpy
from rclpy.node import Node
from kfzbshtl_msgs.msg import StampedControlGuardCmdLongitudinal, StampedControlGuardInfo, PidDebugInfo
from std_msgs.msg import Float32
import time
import math
import numpy as np

class VelocityControlLowLevel(Node):

    def __init__(self):
        super().__init__('velocity_control_low_level_node')

        self.v_max = 2.0  # Maximum velocity [m/s]
        self.acc_max_forward = 0.3  # Maximum acceleration normal [m/s^2]
        self.acc_max_emergecy = 3.0  # Maximum deceleration by emergency [m/s^2]

        self.guard_mode_auto_active = False  # Initially, not in auto mode
        self.vehicle_v_actual = 0.0  # Actual vehicle velocity [m/s]
        self.desired_vel_latest = 0.0  # Desired vehicle velocity at last time step [m/s]

        self.pub_time = 0.05  # 20 hz, time step for simulation or real test [s]
        self.latest_cmd_update_time = time.time()  # Time of the last command update, initialize with current time

        self.start_time = time.time()

        self.vel_error_sum = 0.0
        self.vel_error_last = 0.0

        # Subscribers
        self.vehicle_vel_subscription = self.create_subscription(Float32, 'vehicle_v_actual', self.vehicle_vel_callback, 1)
        self.guard_info_subscription = self.create_subscription(StampedControlGuardInfo, '/act/control_guard_info', self.guard_info_callback, 1)

        # Publishers
        self.dd_vel_publisher = self.create_publisher(StampedControlGuardCmdLongitudinal, '/plan/control_cmd_long', 1)
        self.pid_debug_vel_control = self.create_publisher(PidDebugInfo, 'pid_debug/vel_control', 1)

        self.timer = self.create_timer(self.pub_time, self.timer_callback)

    def vehicle_vel_callback(self, msg):
        #self.get_logger().info('i can hear velocity')
        self.vehicle_v_actual = msg.data

    def guard_info_callback(self, msg):
        #self.get_logger().info('i can hear auto mode is %f' % msg.automatic_mode_active)
        self.guard_mode_auto_active = msg.automatic_mode_active

    def timer_callback(self):
        self.cmd_dt = time.time() - self.latest_cmd_update_time
        

        traveled_time = time.time() - self.start_time

        if traveled_time <= 20.0:
            target_vel_ref = 1.0
        else:
            target_vel_ref = 0.0
    
        Kp_vel_control = 0.03
        Ki_vel_control = 0.25
        Kd_vel_control = 0.15


        vel_error = target_vel_ref - self.vehicle_v_actual
        self.vel_error_sum += vel_error * self.cmd_dt
        vel_error_dot = (vel_error - self.vel_error_last) / self.cmd_dt
        self.vel_error_last = vel_error

        p_term_vel = Kp_vel_control * vel_error
        i_term_vel = Ki_vel_control * self.vel_error_sum
        d_term_vel = Kd_vel_control * vel_error_dot

        # anti windup:
        i_term_vel = np.clip(i_term_vel, 0, self.v_max)

        # reset by outer state
        if not self.guard_mode_auto_active:
            self.vel_error_sum = 0.0
            p_term_vel = 0.0
            i_term_vel = 0.0
            d_term_vel = 0.0

        target_vel_low_control = p_term_vel + i_term_vel + d_term_vel
        target_vel_low_control = np.clip(target_vel_low_control, 0, self.v_max)

        # Publish PID components for debugging
        pid_debug_vel_control = PidDebugInfo()
        pid_debug_vel_control.p_term = p_term_vel
        pid_debug_vel_control.i_term = i_term_vel
        pid_debug_vel_control.d_term = d_term_vel
        pid_debug_vel_control.pid_output = target_vel_low_control
        pid_debug_vel_control.set_point = target_vel_ref
        self.pid_debug_vel_control.publish(pid_debug_vel_control)

        # smooth the velocity change
        d_vel = target_vel_low_control - self.desired_vel_latest
        d_vel_max = self.acc_max_forward * self.cmd_dt
        d_vel = np.clip(d_vel, -d_vel_max, d_vel_max)
        target_vel_low_level_limited = self.desired_vel_latest + d_vel
        target_vel_low_level_limited = np.clip(target_vel_low_level_limited, 0.0, self.v_max)


        self.desired_vel_latest = target_vel_low_level_limited

        if not self.guard_mode_auto_active:
            self.desired_vel_latest = 0.0

        self.publish_control_commands()
        #self.get_logger().info("Desired velocity: %f" % self.desired_vel_latest)

    def publish_control_commands(self):

        if abs(self.desired_vel_latest) < 0.01:
        #self.get_logger().info('desired velocity is %f, set to 0.0' % self.desired_vel_latest)
            self.desired_vel_latest = 0.0
            cmd_longitudinal_to_guard = 0.0
        else:
            cmd_longitudinal_to_guard = self.desired_vel_latest
        
        vel_msg = StampedControlGuardCmdLongitudinal()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.vel = cmd_longitudinal_to_guard
        vel_msg.mode_req = 4  # [2:"Standby", 4:"Use"]
        self.dd_vel_publisher.publish(vel_msg)

        self.latest_cmd_update_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    velocity_control_low_level_node = VelocityControlLowLevel()
    rclpy.spin(velocity_control_low_level_node)
    velocity_control_low_level_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

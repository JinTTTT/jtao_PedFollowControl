import math
import numpy as np
from std_msgs.msg import Float32
from kfzbshtl_msgs.msg import PidDebugInfo

def distance_control(dist_to_ped_now, track_distance, relative_distance_rate,
                     cmd_dt, v_max, acc_max_forward, target_vel_dist_control_last,
                     vehicle_v_actual, dist_error_sum):
    """
    High level control: Distance control using forward control to maintain desired distance from pedestrian.
    """
   
    # headway distance: predict the distance after 2 seconds
    t_headway = 5.6  
    d_headway = t_headway * vehicle_v_actual

    # stop distance: min distance to stop 
    acc_stop = 0.11 * acc_max_forward
    d_stop = (vehicle_v_actual
               ** 2) / (2 * acc_stop)

    safety_margin = 0.0  
    required_distance = max(d_stop, d_headway) + track_distance + safety_margin

    error = dist_to_ped_now - required_distance
    error_rate = relative_distance_rate  
    dist_error_sum += error * cmd_dt

    # anti-windup
    max_dist_error_sum = 10.0
    dist_error_sum = np.clip(dist_error_sum, -max_dist_error_sum, max_dist_error_sum)

    Kp = 0.4
    Ki = 0.15
    Kd = 0.5

    proportional_contribution = Kp * error
    integral_contribution = Ki * dist_error_sum
    derivative_contribution = Kd * error_rate

    desired_target_vel = proportional_contribution + integral_contribution + derivative_contribution
    
    desired_target_vel = np.clip(desired_target_vel, 0.0, v_max)

    delta_v = desired_target_vel - target_vel_dist_control_last

    if delta_v > 0:
        delta_v_max = acc_max_forward * cmd_dt
        delta_v = min(delta_v, delta_v_max)
    else:
        delta_v_max = acc_max_forward * cmd_dt
        delta_v = max(delta_v, -delta_v_max)

    target_vel_dist_control = target_vel_dist_control_last + delta_v

    return target_vel_dist_control, required_distance, proportional_contribution, integral_contribution, derivative_contribution, d_stop, d_headway, dist_error_sum


def velocity_control(target_vel_high_level, vehicle_v_actual, vel_error_sum, vel_error_last,
                     cmd_dt, v_max, acc_max_forward, desired_vel_latest):
    """
    Low level control: Velocity control to achieve desired velocity.

    Args:
        target_vel_high_level (float): Target velocity from high-level control
        vehicle_v_actual (float): Actual vehicle velocity
        vel_error_sum (float): Accumulated velocity error
        vel_error_last (float): Last velocity error
        cmd_dt (float): Time step
        v_max (float): Maximum velocity
        acc_max_forward (float): Maximum forward acceleration
        desired_vel_latest (float): Latest desired velocity
        guard_mode_auto_active (bool): Whether automatic mode is active

    Returns:
        tuple: (target_vel_low_level_limited, vel_error_sum, vel_error_last, pid_debug_vel_control)
    """
    target_vel_ref = target_vel_high_level
    
    # Kp_vel_control = 0.02
    # Ki_vel_control = 0.15
    # Kd_vel_control = 0.001

    Kp_vel_control = 0.03
    Ki_vel_control = 0.25
    Kd_vel_control = 0.15


    vel_error = target_vel_ref - vehicle_v_actual
    vel_error_sum += vel_error * cmd_dt
    vel_error_dot = (vel_error - vel_error_last) / cmd_dt
    vel_error_last = vel_error

    p_term_vel = Kp_vel_control * vel_error
    i_term_vel = Ki_vel_control * vel_error_sum
    d_term_vel = Kd_vel_control * vel_error_dot

    # Anti-windup
    i_term_vel = np.clip(i_term_vel, 0, v_max)
        
    target_vel_low_control = p_term_vel + i_term_vel + d_term_vel
    target_vel_low_control = np.clip(target_vel_low_control, 0, v_max)
    
    # if target_vel_high_level < 0.01, it means need the vehicle as quick as possible to stop, set the target_vel_low_control to 0
    if target_vel_high_level < 0.01:
        target_vel_low_control = 0.0

    # Smooth the velocity change
    d_vel = target_vel_low_control - desired_vel_latest
    d_vel_max = acc_max_forward * cmd_dt
    d_vel = np.clip(d_vel, -d_vel_max, d_vel_max)
    target_vel_low_level_limited = desired_vel_latest + d_vel
    target_vel_low_level_limited = np.clip(target_vel_low_level_limited, 0.0, v_max)

    # Prepare debug message
    pid_debug_vel_control = PidDebugInfo()
    pid_debug_vel_control.p_term = p_term_vel
    pid_debug_vel_control.i_term = i_term_vel
    pid_debug_vel_control.d_term = d_term_vel
    pid_debug_vel_control.pid_output = target_vel_low_level_limited
    pid_debug_vel_control.set_point = target_vel_ref

    
    #return target_vel_low_level_limited, vel_error_sum, vel_error_last, pid_debug_vel_control
    return target_vel_low_level_limited, vel_error_sum, vel_error_last, pid_debug_vel_control


def steering_control(rel_ped_y, global_ped_x, global_ped_y, vehicle_x, vehicle_y, vehicle_v_actual, desired_steering_latest, 
                     cmd_dt, steering_change_max, steering_range_max):
    """
    High level control: Stanley Controller using relative pedestrian positions.

    Args:
        rel_ped_x (float): Pedestrian's X coordinate in vehicle's frame (forward)
        rel_ped_y (float): Pedestrian's Y coordinate in vehicle's frame (left)
        vehicle_v_actual (float): Actual velocity of vehicle
        desired_steering_latest (float): Latest desired steering angle
        cmd_dt (float): Time step
        steering_change_max (float): Maximum steering angle change per time step
        steering_range_max (float): Maximum steering angle

    Returns:
        tuple: (desired_steering_angle, heading_error, cross_error, steering_angle_stanley)
    """
    
    # Compute heading error (angle to pedestrian in vehicle frame)
    x_diff = global_ped_x - vehicle_x
    y_diff = global_ped_y - vehicle_y
    heading_error = math.atan2(y_diff, x_diff)

    # Compute cross-track error (lateral offset)
    cross_track_error = rel_ped_y

    # Stanley controller parameters
    Ke = 0.1  # Cross-track error gain
    softening_constant = 1.0  # To prevent division by zero

    # Compute cross error term
    cross_error = math.atan2(Ke * cross_track_error, vehicle_v_actual + softening_constant)

    # Compute steering angle using Stanley controller formula
    steering_angle_stanley = heading_error + cross_error

    # ppc

    # wheelbase = 2.9
    # alpha = math.atan2(rel_ped_y, rel_ped_x + wheelbase + 0.6)  # angle between the vehicle and the pedestrian, begin from rear axle

    # # set lookahead distance as distance to ped
    # ld = 3.0
    
    # desired_steering_agnle = math.atan2(2 * wheelbase * math.sin(alpha), ld)  # delta in radians
    # heading_error = 0.0
    # cross_error = 0.0
    # steering_angle_stanley = 0.0
    

    # Limit the steering angle change per time step
    d_st_agl_max = steering_change_max * cmd_dt
    d_st_angle = steering_angle_stanley - desired_steering_latest
    d_st_angle = np.clip(d_st_angle, -d_st_agl_max, d_st_agl_max)

    # Update desired steering angle
    desired_steering_angle = desired_steering_latest + d_st_angle

    # Limit output steering angle to allowable range
    desired_steering_angle = np.clip(desired_steering_angle, -steering_range_max, steering_range_max)

    return desired_steering_angle, heading_error, cross_error, steering_angle_stanley


def prepare_steering_debug_msgs(heading_error, cross_error, steering_angle_stanley):
    """
    Prepare debug messages for steering control visualization.

    Args:
        heading_error (float): Heading error in radians
        cross_error (float): Cross track error in radians
        steering_angle_stanley (float): Calculated steering angle in radians

    Returns:
        tuple: (head_msg, cross_msg, target_steering_msg)
    """
    head_msg = Float32()
    head_msg.data = heading_error * 180 / math.pi

    cross_msg = Float32()
    cross_msg.data = cross_error * 180 / math.pi

    target_steering_msg = Float32()
    target_steering_msg.data = steering_angle_stanley * 180 / math.pi

    return head_msg, cross_msg, target_steering_msg

def quaternion_to_euler(q):
        """
        Convert quaternion to Euler angles (yaw only for 2D).
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
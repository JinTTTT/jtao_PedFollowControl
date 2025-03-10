from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import math

# Global vehicle parameters
WHEELBASE = 2.9
WHEELRADIUS = 0.34
MAGIC_NAVYA_STEERING_FACTOR = 375.9 / 24.3 / 1.014492675339367
ACC_MAX_FORWARD = 0.5
ACC_MAX_EMERGENCY = 3.0
STEERING_RANGE_MAX = 24 * math.pi / 180
STEERING_CHANGE_MAX = 11.4 * math.pi / 180

def generate_launch_description():
    # Declare launch arguments
    track_distance_arg = DeclareLaunchArgument(
        'track_distance', default_value='10.0',
        description='Track distance for pedestrian following'
    )
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance', default_value='8.0',
        description='Safe distance for pedestrian following'
    )
    v_max_arg = DeclareLaunchArgument(
        'v_max', default_value='1.5',
        description='Maximum velocity for the vehicle'
    )

    # Retrieve launch configurations
    track_distance = LaunchConfiguration('track_distance')
    safe_distance = LaunchConfiguration('safe_distance')
    v_max = LaunchConfiguration('v_max')

    # List to hold all launch actions
    actions = [
        track_distance_arg,
        safe_distance_arg,
        v_max_arg
    ]

    # Define the GUI node
    gui_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='gui_node_exec',
        name='gui_node',
        parameters=[{
            'track_distance': track_distance,
            'safe_distance': safe_distance,
            'v_max': v_max
        }]
    )
    actions.append(gui_node)

    # Define the target selector node
    target_selector_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='get_nearest_obj_exec',
        name='get_nearest_obj'
    )
    actions.append(target_selector_node)

    # Define get gesture node
    get_gesture_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='get_gesture_exec',
        name='get_gesture'
    )
    actions.append(get_gesture_node)

    # Define the motion estimator node
    motion_estimator_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='motion_estimator_exec',
        name='motion_estimator',
        parameters=[{
            'wheelbase': WHEELBASE,
            'wheelradius': WHEELRADIUS,
            'magic_navya_steering_factor': MAGIC_NAVYA_STEERING_FACTOR
        }]
    )
    actions.append(motion_estimator_node)

    # Include the guard teleop launch file
    guard_teleop_share_dir = get_package_share_directory('kfzbshtl_control_guard_teleop')
    guard_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(guard_teleop_share_dir, 'launch', 'control_guard_teleop.launch.py')
        )
    )
    actions.append(guard_teleop_launch)

    # Define the pedestrian follow node
    ped_follow_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='ped_follow_exec',
        name='ped_follow',
        parameters=[{
            'track_distance': track_distance,
            'safe_distance': safe_distance,
            'v_max': v_max,
            'acc_max_forward': ACC_MAX_FORWARD,
            'acc_max_emergency': ACC_MAX_EMERGENCY,
            'steering_range_max': STEERING_RANGE_MAX,
            'steering_change_max': STEERING_CHANGE_MAX
        }]
    )
    actions.append(ped_follow_node)

    # Define the low level controller node
    low_level_controller_node = Node(
        package='kfzbshtl_control_ped_follow_jin_tao',
        executable='low_level_controller_exec',
        name='low_level_controller',
        parameters=[{
            'v_max': v_max,
            'acc_max_forward': ACC_MAX_FORWARD,
            'acc_max_emergency': ACC_MAX_EMERGENCY,
            'steering_range_max': STEERING_RANGE_MAX,
            'steering_change_max': STEERING_CHANGE_MAX
        }]
    )
    actions.append(low_level_controller_node)

    return LaunchDescription(actions)

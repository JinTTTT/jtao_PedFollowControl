from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kfzbshtl_control_ped_follow_jin_tao',
            executable='fake_pedestrian_exec',
            name='fake_pedestrian'
        ),
        Node(
            package='kfzbshtl_control_ped_follow_jin_tao',
            executable='input_fake_ped_ui_exec',
            name='input_fake_ped_ui'
        ),
    ])
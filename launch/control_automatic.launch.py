from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import subprocess
import os

def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    input_script = os.path.join(current_dir, 'parameter_input_and_launch.py')
    result = subprocess.run(['python3', input_script], capture_output=True, text=True)
    params = {}
    for line in result.stdout.splitlines():
        key, value = line.split('=')
        params[key] = float(value)

    track_distance = LaunchConfiguration('track_distance', default=str(params['track_distance']))
    safe_distance = LaunchConfiguration('safe_distance', default=str(params['safe_distance']))
    v_max = LaunchConfiguration('v_max', default=str(params['v_max']))

    original_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([current_dir, '/ped_follow.launch.py']),
        launch_arguments={
            'track_distance': track_distance,
            'safe_distance': safe_distance,
            'v_max': v_max
        }.items()
    )

    return LaunchDescription([original_launch])

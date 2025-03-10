from setuptools import find_packages, setup

package_name = 'kfzbshtl_control_ped_follow_jin_tao'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run_sim_ped.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ped_follow.launch.py']),
        ('share/' + package_name + '/launch', ['launch/parameter_input_and_launch.py']),
        ('share/' + package_name + '/launch', ['launch/control_automatic.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jin tao',
    maintainer_email='tj19970215@gmail.com',
    description='use this package to control the bus to follow the pedestrian',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_nearest_obj_exec = kfzbshtl_control_ped_follow_jin_tao.get_nearest_obj:main',
            'get_gesture_exec = kfzbshtl_control_ped_follow_jin_tao.get_gesture:main',
            'motion_estimator_exec = kfzbshtl_control_ped_follow_jin_tao.motion_estimator:main',
            'ped_follow_exec = kfzbshtl_control_ped_follow_jin_tao.ped_follow:main',
            'low_level_controller_exec = kfzbshtl_control_ped_follow_jin_tao.low_level_controller:main',
            'gui_node_exec = kfzbshtl_control_ped_follow_jin_tao.gui_node:main',

            # for testing
            'fake_pedestrian_exec = kfzbshtl_control_ped_follow_jin_tao.fake_pedestrian:main',
            'fake_bus_exec = kfzbshtl_control_ped_follow_jin_tao.fake_bus:main',
            'input_fake_ped_ui_exec = kfzbshtl_control_ped_follow_jin_tao.input_fake_ped_ui:main',   
            'velocity_control_low_level_exec = kfzbshtl_control_ped_follow_jin_tao.velocity_control_low_level:main',
        ],
    },
)

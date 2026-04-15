import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mobile_robotics')
    params_file = os.path.join(pkg_dir, 'config', 'mobile_robotics_params.yaml')

    return LaunchDescription([
        Node(
            package='mobile_robotics',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen',
            parameters=[params_file],
        )
    ])

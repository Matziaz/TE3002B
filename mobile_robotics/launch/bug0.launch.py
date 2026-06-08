import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mobile_robotics')
    params_file = os.path.join(pkg_dir, 'config', 'bug0_params.yaml')

    return LaunchDescription([
        Node(
            package='mobile_robotics',
            executable='bug0_node',
            name='bug0_node',
            output='screen',
            parameters=[params_file],
        )
    ])

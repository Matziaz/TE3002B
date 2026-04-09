import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'square_controller'
    package_share_dir = get_package_share_directory(package_name)
    params_file = os.path.join(package_share_dir, 'config', 'path_params.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='path_generator_node',
            name='path_generator_node',
            output='screen',
            parameters=[params_file]
        )
    ])

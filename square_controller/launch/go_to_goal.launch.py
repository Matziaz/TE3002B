import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('square_controller')
    params_file = os.path.join(pkg_dir, 'config', 'go_to_goal_params.yaml')

    return LaunchDescription([
        Node(
            package='square_controller',
            executable='go_to_goal_node',
            name='go_to_goal_node',
            output='screen',
            parameters=[params_file],
        )
    ])

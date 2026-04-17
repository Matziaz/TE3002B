import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mobile_robotics')
    go_to_goal_params = os.path.join(pkg_dir, 'config', 'go_to_goal_params.yaml')
    path_gen_params = os.path.join(pkg_dir, 'config', 'path_generator_params.yaml')

    return LaunchDescription([
        Node(
            package='mobile_robotics',
            executable='path_generator_node',
            name='path_generator_node',
            output='screen',
            parameters=[path_gen_params],
        ),
        Node(
            package='mobile_robotics',
            executable='go_to_goal_node',
            name='go_to_goal_node',
            output='screen',
            parameters=[go_to_goal_params],
        )
    ])

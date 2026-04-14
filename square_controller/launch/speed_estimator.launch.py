import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('square_controller')
    params_file = os.path.join(pkg_dir, 'config', 'speed_estimator_params.yaml')

    return LaunchDescription([
        Node(
            package='square_controller',
            executable='speed_estimator_node',
            name='speed_estimator_node',
            output='screen',
            parameters=[params_file],
        )
    ])

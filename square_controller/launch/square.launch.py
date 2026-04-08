from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('square_controller')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    controller_node = Node(
        package='square_controller',
        executable='square_controller_node',
        name='square_controller',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([controller_node])

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
            executable='odometry_speed_estimator',
            name='odometry_speed_estimator',
            output='screen',
            parameters=[params_file],
        )
    ])

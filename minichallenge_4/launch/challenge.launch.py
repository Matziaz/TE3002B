from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _build_nodes(context):
    package_dir = get_package_share_directory('minichallenge_4')
    profile = LaunchConfiguration('profile').perform(context)

    odometry_config = os.path.join(package_dir, 'config', 'odometry_params.yaml')
    color_config = os.path.join(package_dir, 'config', 'color_detector_params.yaml')
    traffic_config = os.path.join(package_dir, 'config', 'traffic_light_params.yaml')
    go_to_goal_config = os.path.join(package_dir, 'config', 'go_to_goal_params.yaml')

    params = {
        'odometry': [odometry_config],
        'color': [color_config],
        'traffic': [traffic_config],
        'go_to_goal': [go_to_goal_config],
    }

    if profile and profile != 'default':
        profile_file = os.path.join(package_dir, 'config', 'profiles', f'{profile}.yaml')
        if os.path.exists(profile_file):
            for key in params:
                params[key].append(profile_file)

    return [
        Node(
            package='minichallenge_4',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=params['odometry']
        ),
        Node(
            package='minichallenge_4',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
            parameters=params['color']
        ),
        Node(
            package='minichallenge_4',
            executable='traffic_light_controller',
            name='traffic_light_controller',
            output='screen',
            parameters=params['traffic']
        ),
        Node(
            package='minichallenge_4',
            executable='go_to_goal_node',
            name='go_to_goal_node',
            output='screen',
            parameters=params['go_to_goal']
        )
    ]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Config profile name: default, PROFILE_FAST, PROFILE_LAB_SAFE, PROFILE_POOR_LIGHTING, PROFILE_PRECISION'
        ),
        OpaqueFunction(function=_build_nodes)
    ])

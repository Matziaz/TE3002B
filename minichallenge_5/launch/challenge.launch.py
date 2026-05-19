from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _build_nodes(context):
    package_dir = get_package_share_directory('minichallenge_5')
    profile = LaunchConfiguration('profile').perform(context)
    launch_camera = LaunchConfiguration('launch_camera').perform(context).lower() in ('true', '1', 'yes')

    color_config = os.path.join(package_dir, 'config', 'color_detector_params.yaml')
    traffic_config = os.path.join(package_dir, 'config', 'traffic_light_params.yaml')
    line_follower_config = os.path.join(package_dir, 'config', 'line_follower_params.yaml')

    params = {
        'color': [color_config],
        'traffic': [traffic_config],
        'line_follower': [line_follower_config],
    }

    if profile and profile != 'default':
        profile_file = os.path.join(package_dir, 'config', 'profiles', f'{profile}.yaml')
        if os.path.exists(profile_file):
            for key in params:
                params[key].append(profile_file)
        else:
            print(f'[challenge.launch.py] WARNING: profile file not found: {profile_file}')

    actions = []

    if launch_camera:
        try:
            ros_deep_learning_dir = get_package_share_directory('ros_deep_learning')
            camera_launch_file = os.path.join(
                ros_deep_learning_dir,
                'launch',
                'video_source.ros2.launch',
            )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(camera_launch_file),
                    launch_arguments={'camera_topic': LaunchConfiguration('camera_topic')},
                    condition=IfCondition(LaunchConfiguration('launch_camera')),
                )
            )
        except Exception as exc:
            print('[challenge.launch.py] WARNING: could not include ros_deep_learning camera launch.')
            print(f'[challenge.launch.py] Reason: {exc}')
            print('[challenge.launch.py] Run camera manually if needed:')
            print('  ros2 launch ros_deep_learning video_source.ros2.launch')

    actions.extend([
        Node(
            package='minichallenge_5',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
            parameters=params['color'],
        ),
        Node(
            package='minichallenge_5',
            executable='traffic_light_controller',
            name='traffic_light_controller',
            output='screen',
            parameters=params['traffic'],
        ),
        Node(
            package='minichallenge_5',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen',
            parameters=params['line_follower'],
        ),
    ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Config profile name: default, PROFILE_FAST, PROFILE_LAB_SAFE, PROFILE_POOR_LIGHTING, PROFILE_PRECISION',
        ),

        DeclareLaunchArgument(
            'launch_camera',
            default_value='false',
            description='If true, also launches ros_deep_learning video_source.ros2.launch for /camera. Default is false because the Puzzlebot camera is normally launched separately.',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera',
            description='Topic name to publish camera frames on (forwarded to the included camera launch).',
        ),
        OpaqueFunction(function=_build_nodes),
    ])

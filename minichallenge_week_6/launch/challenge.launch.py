# Launch file for the complete challenge system. It starts the camera if requested
# and launches the odometry, vision, traffic-light controller, and navigation nodes.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# This function is called at launch time, after launch arguments are known.
# It builds the final list of actions/nodes to execute.
def _build_nodes(context):
    # Locate the installed package share directory so configuration files can be
    # found regardless of where the workspace is installed.
    package_dir = get_package_share_directory('minichallenge_week_6')
    profile = LaunchConfiguration('profile').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    launch_camera = LaunchConfiguration('launch_camera').perform(context).lower() in ('true', '1', 'yes')

    odometry_config = os.path.join(package_dir, 'config', 'odometry_params.yaml')
    color_config = os.path.join(package_dir, 'config', 'color_detector_params.yaml')
    traffic_config = os.path.join(package_dir, 'config', 'traffic_light_params.yaml')
    go_to_goal_config = os.path.join(package_dir, 'config', 'go_to_goal_params.yaml')
    line_follower_config = os.path.join(package_dir, 'config', 'line_follower_params.yaml')

    # Each node receives its own base YAML file. If a profile is selected, the
    # profile file is appended so it can override default values.
    params = {
        'odometry': [odometry_config],
        'color': [color_config],
        'traffic': [traffic_config],
        'go_to_goal': [go_to_goal_config],
        'line_follower': [line_follower_config],
    }

    # Optional profiles allow changing tuning presets from the launch command
    # without editing the code or copying files manually.
    if profile and profile != 'default':
        profile_file = os.path.join(package_dir, 'config', 'profiles', f'{profile}.yaml')
        if os.path.exists(profile_file):
            for key in params:
                params[key].append(profile_file)
        else:
            print(f'[challenge.launch.py] WARNING: profile file not found: {profile_file}')

    actions = []

    # Launch the Jetson camera publisher used by the Puzzlebot.
    # This is equivalent to running:
    # ros2 launch ros_deep_learning video_source.ros2.launch
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

    # Start common project nodes used in both operation modes.
    actions.extend([
        Node(
            package='minichallenge_week_6',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=params['odometry'],
        ),
        Node(
            package='minichallenge_week_6',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
            parameters=params['color'],
        ),
        Node(
            package='minichallenge_week_6',
            executable='traffic_light_controller',
            name='traffic_light_controller',
            output='screen',
            parameters=params['traffic'],
        ),
    ])

    if mode == 'go_to_goal':
        actions.append(
            Node(
                package='minichallenge_week_6',
                executable='go_to_goal_node',
                name='go_to_goal_node',
                output='screen',
                parameters=params['go_to_goal'],
            )
        )
    else:
        actions.append(
            Node(
                package='minichallenge_week_6',
                executable='line_follower_node',
                name='line_follower_node',
                output='screen',
                parameters=params['line_follower'],
            )
        )

    return actions


# Main launch-description function required by ROS 2 launch.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Config profile name: default, PROFILE_FAST, PROFILE_LAB_SAFE, PROFILE_POOR_LIGHTING, PROFILE_PRECISION',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='line_follow',
            choices=['line_follow', 'go_to_goal'],
            description='Execution mode: line_follow launches line_follower_node, go_to_goal launches go_to_goal_node.',
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

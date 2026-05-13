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
    package_dir = get_package_share_directory('minichallenge_4')
    profile = LaunchConfiguration('profile').perform(context)
    launch_camera = LaunchConfiguration('launch_camera').perform(context).lower() in ('true', '1', 'yes')

    odometry_config = os.path.join(package_dir, 'config', 'odometry_params.yaml')
    color_config = os.path.join(package_dir, 'config', 'color_detector_params.yaml')
    traffic_config = os.path.join(package_dir, 'config', 'traffic_light_params.yaml')
    go_to_goal_config = os.path.join(package_dir, 'config', 'go_to_goal_params.yaml')

    # Each node receives its own base YAML file. If a profile is selected, the
    # profile file is appended so it can override default values.
    params = {
        'odometry': [odometry_config],
        'color': [color_config],
        'traffic': [traffic_config],
        'go_to_goal': [go_to_goal_config],
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
                    condition=IfCondition(LaunchConfiguration('launch_camera')),
                )
            )
        except Exception as exc:
            print('[challenge.launch.py] WARNING: could not include ros_deep_learning camera launch.')
            print(f'[challenge.launch.py] Reason: {exc}')
            print('[challenge.launch.py] Run camera manually if needed:')
            print('  ros2 launch ros_deep_learning video_source.ros2.launch')

    # Start all project nodes. These nodes communicate only through ROS topics,
    # so no human interaction is needed after launch.
    actions.extend([
        Node(
            package='minichallenge_4',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=params['odometry'],
        ),
        Node(
            package='minichallenge_4',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
            parameters=params['color'],
        ),
        Node(
            package='minichallenge_4',
            executable='traffic_light_controller',
            name='traffic_light_controller',
            output='screen',
            parameters=params['traffic'],
        ),
        Node(
            package='minichallenge_4',
            executable='go_to_goal_node',
            name='go_to_goal_node',
            output='screen',
            parameters=params['go_to_goal'],
        ),
    ])

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
            'launch_camera',
            default_value='false',
            description='If true, also launches ros_deep_learning video_source.ros2.launch for /video_source/raw. Default is false because the Puzzlebot camera is normally launched separately.',
        ),
        OpaqueFunction(function=_build_nodes),
    ])

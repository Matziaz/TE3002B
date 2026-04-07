"""
MCR2 Mini Challenge 1 – Launch file
=====================================
Launches the square_controller_node (FSM open-loop controller).

Usage:
  ros2 launch square_controller square.launch.py
  ros2 launch square_controller square.launch.py total_time:=30.0
  ros2 launch square_controller square.launch.py linear_speed:=0.3 angular_speed:=0.6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare tunable launch arguments ─────────────────────────────────────
    args = [
        DeclareLaunchArgument("side_length",   default_value="2.0",
                              description="Side length of the square in metres"),
        DeclareLaunchArgument("linear_speed",  default_value="0.2",
                              description="Linear speed m/s  (set <=0 to auto-tune)"),
        DeclareLaunchArgument("angular_speed", default_value="0.5",
                              description="Angular speed rad/s (set <=0 to auto-tune)"),
        DeclareLaunchArgument("total_time",    default_value="40.0",
                              description="Total time to complete square (used when speed<=0)"),
        DeclareLaunchArgument("ramp_fraction", default_value="0.2",
                              description="Fraction of each segment used for velocity ramping"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="Velocity command topic"),
        DeclareLaunchArgument("control_rate",  default_value="50.0",
                              description="Control loop frequency in Hz"),
    ]

    # ── Square controller node ────────────────────────────────────────────────
    controller_node = Node(
        package="square_controller",
        executable="square_controller_node",
        name="square_controller",
        output="screen",
        parameters=[{
            "side_length":   LaunchConfiguration("side_length"),
            "linear_speed":  LaunchConfiguration("linear_speed"),
            "angular_speed": LaunchConfiguration("angular_speed"),
            "total_time":    LaunchConfiguration("total_time"),
            "ramp_fraction": LaunchConfiguration("ramp_fraction"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "control_rate":  LaunchConfiguration("control_rate"),
        }],
    )

    return LaunchDescription(args + [controller_node])

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GoToGoalController(Node):
    def __init__(self):
        super().__init__('go_to_goal_controller')

        self.declare_parameter('robot_odometry_topic', '/robot_odometry')
        self.declare_parameter('velocity_command_topic', '/cmd_vel')

        self.declare_parameter('control_loop_rate', 20.0)

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('use_dynamic_goals', False)
        self.declare_parameter('dynamic_goal_topic', '/goal_pose')

        self.declare_parameter('proportional_velocity_gain', 0.8)
        self.declare_parameter('proportional_angular_gain', 2.5)

        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('linear_velocity_limit', 0.5)
        self.declare_parameter('angular_velocity_limit', 1.5)

        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.1)

        self.odometry_topic = str(self.get_parameter('robot_odometry_topic').value)
        self.velocity_command_topic = str(self.get_parameter('velocity_command_topic').value)

        self.control_rate = max(1.0, float(self.get_parameter('control_loop_rate').value))

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.use_dynamic_goals = bool(self.get_parameter('use_dynamic_goals').value)
        self.dynamic_goal_topic = str(self.get_parameter('dynamic_goal_topic').value)

        self.k_v = max(1e-4, float(self.get_parameter('proportional_velocity_gain').value))
        self.k_omega = max(1e-4, float(self.get_parameter('proportional_angular_gain').value))

        self.max_linear = max(1e-4, float(self.get_parameter('max_linear_velocity').value))
        self.max_angular = max(1e-4, float(self.get_parameter('max_angular_velocity').value))
        self.linear_limit = max(1e-4, float(self.get_parameter('linear_velocity_limit').value))
        self.angular_limit = max(1e-4, float(self.get_parameter('angular_velocity_limit').value))

        self.distance_tolerance = max(1e-4, float(self.get_parameter('distance_tolerance').value))
        self.heading_tolerance = max(1e-4, float(self.get_parameter('heading_tolerance').value))

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0

        self.goal_reached = False

        self.velocity_command_pub = self.create_publisher(Twist, self.velocity_command_topic, 10)
        self.create_subscription(Odometry, self.odometry_topic, self.on_odometry_update, 10)

        if self.use_dynamic_goals:
            self.create_subscription(Twist, self.dynamic_goal_topic, self.on_goal_update, 10)

        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(f'Go-To-Goal Controller initialized for goal ({self.goal_x:.3f}, {self.goal_y:.3f})')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def on_odometry_update(self, msg):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        self.robot_heading = 2.0 * math.atan2(orientation.z, orientation.w)

    def on_goal_update(self, msg):
        self.goal_x = float(msg.linear.x)
        self.goal_y = float(msg.linear.y)

    def control_loop(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)

        if distance_to_goal < self.distance_tolerance:
            self.goal_reached = True
            linear_command = 0.0
            angular_command = 0.0
            self.get_logger().info(f'GOAL REACHED! Robot at ({self.robot_x:.3f}, {self.robot_y:.3f}), Goal: ({self.goal_x:.3f}, {self.goal_y:.3f})')
        else:
            self.goal_reached = False
            desired_heading = math.atan2(dy, dx)
            heading_error = self.normalize_angle(desired_heading - self.robot_heading)

            linear_command = self.k_v * distance_to_goal
            angular_command = self.k_omega * heading_error

            linear_command = self.clamp(linear_command, -self.linear_limit, self.linear_limit)
            angular_command = self.clamp(angular_command, -self.angular_limit, self.angular_limit)

            self.get_logger().info(f'Robot: ({self.robot_x:.3f}, {self.robot_y:.3f}) | Goal: ({self.goal_x:.3f}, {self.goal_y:.3f}) | Error: dx={dx:.3f}, dy={dy:.3f}, dist={distance_to_goal:.3f} | Cmd: v={linear_command:.3f}, w={angular_command:.3f}')

        cmd_msg = Twist()
        cmd_msg.linear.x = float(linear_command)
        cmd_msg.angular.z = float(angular_command)
        self.velocity_command_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

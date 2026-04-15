#!/usr/bin/env python3
"""
Go-to-Goal Navigation Node
===========================

Control a differential drive robot to reach a target position using proportional control.

Mathematical Model:
    Error calculations (from homework):
        e_d = √((x_G - x)² + (y_G - y)²)          # Distance error
        θ_G = atan2(y_G - y, x_G - x)              # Desired heading
        e_th = θ_G - θ_R                             # Heading error (normalized)
    
    Proportional controller:
        v = k_v * e_d                              # Linear velocity command
        w = k_ω * e_th                             # Angular velocity command
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')

        # ========== TOPIC PARAMETERS ==========
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('control_rate', 20.0)

        # ========== GOAL POSITION ==========
        self.declare_parameter('x_goal', 1.0)
        self.declare_parameter('y_goal', 0.0)
        self.declare_parameter('listen_goal_topic', False)

        # ========== TOLERANCES ==========
        self.declare_parameter('e_d_tolerance', 0.05)        # Distance error threshold
        self.declare_parameter('e_theta_tolerance', 0.05)    # Heading error threshold
        self.declare_parameter('settle_time', 0.5)

        # ========== VELOCITY LIMITS ==========
        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('omega_max', 1.2)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('omega_min', 0.0)

        # ========== CONTROLLER GAINS ==========
        self.declare_parameter('k_v', 0.8)       # Linear gain
        self.declare_parameter('k_omega', 2.5)   # Angular gain

        # ========== SPECIAL BEHAVIOR ==========
        self.declare_parameter('turn_in_place_threshold', 0.8)

        # ========== PARAMETER LOADING ==========
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.x_goal = float(self.get_parameter('x_goal').value)
        self.y_goal = float(self.get_parameter('y_goal').value)
        self.listen_goal_topic = bool(self.get_parameter('listen_goal_topic').value)

        self.e_d_tolerance = float(self.get_parameter('e_d_tolerance').value)
        self.e_theta_tolerance = float(self.get_parameter('e_theta_tolerance').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.omega_max = float(self.get_parameter('omega_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.omega_min = float(self.get_parameter('omega_min').value)

        self.k_v = float(self.get_parameter('k_v').value)
        self.k_omega = float(self.get_parameter('k_omega').value)

        self.turn_in_place_threshold = float(self.get_parameter('turn_in_place_threshold').value)

        # ========== PARAMETER CONSTRAINTS ==========
        self.control_rate = max(1.0, self.control_rate)
        self.e_d_tolerance = max(0.005, self.e_d_tolerance)
        self.e_theta_tolerance = max(0.005, self.e_theta_tolerance)
        self.settle_time = max(0.0, self.settle_time)

        self.v_max = max(0.01, self.v_max)
        self.omega_max = max(0.05, self.omega_max)
        self.v_min = max(0.0, min(self.v_min, self.v_max))
        self.omega_min = max(0.0, min(self.omega_min, self.omega_max))
        self.turn_in_place_threshold = max(self.e_theta_tolerance, self.turn_in_place_threshold)

        # ========== ROBOT STATE ==========
        self.x = 0.0            # Current x position
        self.y = 0.0            # Current y position
        self.yaw = 0.0          # Current heading: yaw angle (θ)
        self.odom_ready = False
        self.goal_reached = False
        self.goal_tolerance_since = None

        self.last_control_stamp = self.get_clock().now()

        # ========== ROS SETUP ==========
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.listen_goal_topic:
            self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Go-to-goal node started.')
        self.get_logger().info(f'Goal = ({self.x_goal:.3f}, {self.y_goal:.3f}) | mode={self.listen_goal_topic}')
        self.get_logger().info(f'Gains: k_v={self.k_v}, k_ω={self.k_omega}')

    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range.
        θ = atan2(sin(θ), cos(θ))
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, vmin, vmax):
        """Clamp value between vmin and vmax."""
        return max(vmin, min(value, vmax))

    def publish_cmd(self, v, w):
        """Publish velocity command (v, w) to robot."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        """Stop the robot."""
        self.publish_cmd(0.0, 0.0)

    def goal_callback(self, msg):
        """Update goal position from /goal_pose topic."""
        self.x_goal = float(msg.pose.position.x)
        self.y_goal = float(msg.pose.position.y)
        self.goal_reached = False
        self.goal_tolerance_since = None
        self.get_logger().info(f'New goal: ({self.x_goal:.3f}, {self.y_goal:.3f})')

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        # Extract yaw (θ) from quaternion orientation
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def control_loop(self):
        """Main control loop: compute v and ω commands."""
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_control_stamp.nanoseconds) / 1e9
        self.last_control_stamp = now

        # Wait for odometry data
        if not self.odom_ready:
            self.publish_stop()
            return

        # If goal reached, maintain stop
        if self.goal_reached:
            self.publish_stop()
            return

        # ========== ERROR CALCULATIONS ==========
        # e_d = √((x_G - x)² + (y_G - y)²)
        e_d = math.hypot(self.x_goal - self.x, self.y_goal - self.y)

        # θ_G = atan2(y_G - y, x_G - x)
        theta_G = math.atan2(self.y_goal - self.y, self.x_goal - self.x)

        # θ_R = robot's current heading (yaw)
        theta_R = self.yaw

        # e_th = θ_G - θ_R (normalized to [-π, π])
        e_th = self.normalize_angle(theta_G - theta_R)

        # ========== GOAL CHECK ==========
        if e_d <= self.e_d_tolerance:
            if self.goal_tolerance_since is None:
                self.goal_tolerance_since = now
            else:
                elapsed = (now.nanoseconds - self.goal_tolerance_since.nanoseconds) / 1e9
                if elapsed >= self.settle_time:
                    self.goal_reached = True
                    self.publish_stop()
                    self.get_logger().info(f'Goal reached! e_d={e_d:.4f}, e_th={e_th:.4f}')
                    return
        else:
            self.goal_tolerance_since = None

        # ========== CONTROL: PROPORTIONAL CONTROLLER ==========
        # v = k_v * e_d
        v_cmd = self.k_v * e_d

        # w = k_w * e_th
        w_cmd = self.k_omega * e_th

        # ========== SPECIAL BEHAVIOR: TURN-IN-PLACE ==========
        # If heading error is large, only rotate (don't move forward)
        if abs(e_th) > self.turn_in_place_threshold:
            v_cmd = 0.0
        else:
            # Reduce linear velocity when heading is misaligned
            v_cmd *= max(0.0, math.cos(e_th))

        # ========== VELOCITY LIMITS ==========
        v_cmd = self.clamp(v_cmd, 0.0, self.v_max)
        w_cmd = self.clamp(w_cmd, -self.omega_max, self.omega_max)

        # ========== MINIMUM SPEED ENFORCEMENT ==========
        if 0.0 < v_cmd < self.v_min and e_d > self.e_d_tolerance:
            v_cmd = self.v_min

        w_abs = abs(w_cmd)
        if 0.0 < w_abs < self.omega_min and abs(e_th) > self.e_theta_tolerance:
            w_cmd = math.copysign(self.omega_min, w_cmd)

        # ========== PUBLISH COMMAND ==========
        self.publish_cmd(v_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    rclpy.spin(node)
    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

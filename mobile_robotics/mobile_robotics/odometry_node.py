#!/usr/bin/env python3
"""
Odometry Node - Dead Reckoning using Wheel Encoders
====================================================

Integrates encoder velocities to estimate robot pose (x, y, θ).

Mathematical Model:
    Differential drive kinematics:
        v = (v_R + v_L) / 2                    # Linear velocity
        w = (v_R - v_L) / b                    # Angular velocity
    
    Pose integration:
        θ(k) = θ(k-1) + w(k-1)·Δt                   # Update heading
        x(k) = x(k-1) + v(k-1)·cos(θ(k-1))·Δt       # Update x position
        y(k) = y(k-1) + v(k-1)·sin(θ(k-1))·Δt       # Update y position
        θ(k) = atan2(sin(θ(k)), cos(θ(k)))              # Normalize angle to [-π, π]
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ========== ENCODER TOPICS ==========
        self.declare_parameter('right_wheel_topic', '/VelocityEncR')
        self.declare_parameter('left_wheel_topic', '/VelocityEncL')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)

        # ========== ROBOT PARAMETERS ==========
        self.declare_parameter('r', 0.05)   # Wheel radius (meters)
        self.declare_parameter('b', 0.19)   # Wheel base (meters)
        self.declare_parameter('encoders_are_angular_speed', True)

        # ========== INITIAL POSE ==========
        self.declare_parameter('x_0', 0.0)
        self.declare_parameter('y_0', 0.0)
        self.declare_parameter('theta_0', 0.0)

        # ========== FRAME CONFIG ==========
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('encoder_timeout_seconds', 0.5)

        # ========== PARAMETER LOADING ==========
        self.right_wheel_topic = str(self.get_parameter('right_wheel_topic').value)
        self.left_wheel_topic = str(self.get_parameter('left_wheel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.r = float(self.get_parameter('r').value)
        self.b = float(self.get_parameter('b').value)
        self.encoders_are_angular_speed = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.x = float(self.get_parameter('x_0').value)
        self.y = float(self.get_parameter('y_0').value)
        self.theta = float(self.get_parameter('theta_0').value)

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.encoder_timeout_seconds = float(self.get_parameter('encoder_timeout_seconds').value)

        # ========== PARAMETER CONSTRAINTS ==========
        self.publish_rate = max(1.0, self.publish_rate)
        self.r = max(1e-4, self.r)
        self.b = max(1e-4, self.b)
        self.encoder_timeout_seconds = max(0.05, self.encoder_timeout_seconds)

        # ========== ENCODER STATE ==========
        self.w_R = 0.0      # Right wheel angular velocity (ω_R)
        self.w_L = 0.0      # Left wheel angular velocity (ω_L)
        self.last_right_stamp = None
        self.last_left_stamp = None
        self.last_update_stamp = self.get_clock().now()

        # ========== ROS SETUP ==========
        self.create_subscription(Float32, self.right_wheel_topic, self.right_callback, 10)
        self.create_subscription(Float32, self.left_wheel_topic, self.left_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_and_publish)

        self.get_logger().info('Odometry node started.')
        self.get_logger().info(
            f'Subscribing: ω_R={self.right_wheel_topic}, ω_L={self.left_wheel_topic} | Publishing: {self.odom_topic}'
        )
        self.get_logger().info(f'Robot params: r={self.r:.4f} m, b={self.b:.4f} m')

    def right_callback(self, msg):
        """Receive right wheel angular velocity (ω_R)."""
        self.w_R = float(msg.data)
        self.last_right_stamp = self.get_clock().now()

    def left_callback(self, msg):
        """Receive left wheel angular velocity (ω_L)."""
        self.w_L = float(msg.data)
        self.last_left_stamp = self.get_clock().now()

    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range.
        θ = atan2(sin(θ), cos(θ))
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def encoder_data_fresh(self):
        """Check if encoder data is recent (not timed out)."""
        if self.last_right_stamp is None or self.last_left_stamp is None:
            return False

        now = self.get_clock().now()
        right_age = (now.nanoseconds - self.last_right_stamp.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.last_left_stamp.nanoseconds) / 1e9
        return right_age <= self.encoder_timeout_seconds and left_age <= self.encoder_timeout_seconds

    def omega_to_linear(self, w):
        """Convert wheel angular velocity to linear velocity: v = w·r"""
        if self.encoders_are_angular_speed:
            return w * self.r
        return w

    def update_and_publish(self):
        """Update pose using odometry equations and publish Odometry message."""
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_update_stamp.nanoseconds) / 1e9
        self.last_update_stamp = now

        if dt <= 1e-6:
            return

        # ========== COMPUTE VELOCITIES ==========
        if self.encoder_data_fresh():
            # v_R = w_R · r, v_L = w_L · r
            v_R = self.omega_to_linear(self.w_R)
            v_L = self.omega_to_linear(self.w_L)

            # v = (v_R + v_L) / 2
            v = 0.5 * (v_R + v_L)

            # w = (v_R - v_L) / b
            w = (v_R - v_L) / self.b
        else:
            v = 0.0
            w = 0.0

        # ========== UPDATE POSE ==========
        # Save previous heading for integration
        theta_prev = self.theta
        
        # θ(k) = θ(k-1) + w(k-1)·Δt (then normalize)
        self.theta = self.normalize_angle(self.theta + w * dt)

        # x(k) = x(k-1) + v(k-1)·cos(θ(k-1))·Δt
        self.x += v * math.cos(theta_prev) * dt

        # y(k) = y(k-1) + v(k-1)·sin(θ(k-1))·Δt
        self.y += v * math.sin(theta_prev) * dt

        # ========== BUILD ODOMETRY MESSAGE ==========
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Pose: position (x, y)
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        # Pose: orientation (θ as quaternion)
        half_theta = 0.5 * self.theta
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(half_theta)
        odom_msg.pose.pose.orientation.w = math.cos(half_theta)

        # Twist: velocities (v, w)
        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom_msg)

        # ========== LOGGING ==========
        if int(now.nanoseconds / 1e7) % 5 == 0:
            self.get_logger().debug(
                f'Pose: x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f} | '
                f'Vel: v={v:.2f}, w={w:.2f} | Encoders: w_R={self.w_R:.2f}, w_L={self.w_L:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

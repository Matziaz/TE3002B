#!/usr/bin/env python3
"""
Speed Estimator Node
====================

Converts encoder angular velocities to robot linear and angular velocities.

Mathematical Model:
    Differential drive kinematics:
        v = (v_R + v_L) / 2     # Linear velocity (average of wheel speeds)
        w = (v_R - v_L) / b     # Angular velocity (difference divided by wheel base)
    
    Where:
        v_R = w_R · r           # Right wheel linear velocity
        v_L = w_L · r           # Left wheel linear velocity
        w_R, w_L                # Wheel angular velocities (from encoders)
        r                       # Wheel radius
        b (L)                   # Wheel base (axle length)
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class SpeedEstimatorNode(Node):
    def __init__(self):
        super().__init__('speed_estimator_node')

        # ========== ENCODER TOPICS (INPUTS) ==========
        self.declare_parameter('right_wheel_topic', '/VelocityEncR')
        self.declare_parameter('left_wheel_topic', '/VelocityEncL')

        # ========== OUTPUT TOPICS ==========
        self.declare_parameter('linear_speed_topic', '/puzzlebot/linear_speed')
        self.declare_parameter('angular_speed_topic', '/puzzlebot/angular_speed')
        self.declare_parameter('estimated_twist_topic', '/puzzlebot/estimated_twist')
        self.declare_parameter('publish_rate', 20.0)

        # ========== ROBOT PARAMETERS ==========
        self.declare_parameter('r', 0.05)   # Wheel radius
        self.declare_parameter('b', 0.19)   # Wheel base
        self.declare_parameter('encoders_are_angular_speed', True)

        # ========== TIMEOUT & LOGGING ==========
        self.declare_parameter('timeout_seconds', 0.5)
        self.declare_parameter('log_rate_hz', 2.0)

        # ========== PARAMETER LOADING ==========
        self.right_wheel_topic = str(self.get_parameter('right_wheel_topic').value)
        self.left_wheel_topic = str(self.get_parameter('left_wheel_topic').value)
        self.linear_speed_topic = str(self.get_parameter('linear_speed_topic').value)
        self.angular_speed_topic = str(self.get_parameter('angular_speed_topic').value)
        self.estimated_twist_topic = str(self.get_parameter('estimated_twist_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.r = float(self.get_parameter('r').value)
        self.b = float(self.get_parameter('b').value)
        self.encoders_are_angular_speed = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.timeout_seconds = float(self.get_parameter('timeout_seconds').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        # ========== PARAMETER CONSTRAINTS ==========
        self.publish_rate = max(1.0, self.publish_rate)
        self.r = max(1e-4, self.r)
        self.b = max(1e-4, self.b)
        self.timeout_seconds = max(0.05, self.timeout_seconds)
        self.log_rate_hz = max(0.2, self.log_rate_hz)

        # ========== ENCODER STATE ==========
        self.w_R = 0.0      # Right wheel angular velocity (ω_R)
        self.w_L = 0.0      # Left wheel angular velocity (ω_L)
        self.has_right = False
        self.has_left = False
        self.last_right_time = None
        self.last_left_time = None
        self.last_log_time = self.get_clock().now()
        self.last_diag_time = self.get_clock().now()
        self.last_publish_time = None

        # ========== ROS SETUP ==========
        self.linear_pub = self.create_publisher(Float32, self.linear_speed_topic, 10)
        self.angular_pub = self.create_publisher(Float32, self.angular_speed_topic, 10)
        self.twist_pub = self.create_publisher(Twist, self.estimated_twist_topic, 10)

        self.right_sub = self.create_subscription(Float32, self.right_wheel_topic, self.right_callback, 10)
        self.left_sub = self.create_subscription(Float32, self.left_wheel_topic, self.left_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.diag_timer = self.create_timer(1.0, self.diagnostic_callback)

        self.get_logger().info('Speed estimator node started.')
        self.get_logger().info(
            f'Subscribing: w_R={self.right_wheel_topic}, w_L={self.left_wheel_topic} | '
            f'Publishing: v={self.linear_speed_topic}, w={self.angular_speed_topic}, twist={self.estimated_twist_topic}'
        )
        self.get_logger().info(
            f'Params: r={self.r:.4f} m, b={self.b:.4f} m, '
            f'encoders_are_angular_speed={self.encoders_are_angular_speed}'
        )

    def _topic_publishers_summary(self, topic_name):
        """Get count and types of publishers on a topic."""
        infos = self.get_publishers_info_by_topic(topic_name)
        if not infos:
            return 0, []

        types = sorted({info.topic_type for info in infos})
        return len(infos), types

    def _age_seconds(self, stamp):
        """Calculate age of a timestamp in seconds."""
        if stamp is None:
            return None
        now = self.get_clock().now()
        return (now.nanoseconds - stamp.nanoseconds) / 1e9

    def diagnostic_callback(self):
        """Log diagnostic information every second."""
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.last_diag_time.nanoseconds) / 1e9
        if elapsed < 1.0:
            return

        self.last_diag_time = now

        right_pub_count, right_types = self._topic_publishers_summary(self.right_wheel_topic)
        left_pub_count, left_types = self._topic_publishers_summary(self.left_wheel_topic)

        right_age = self._age_seconds(self.last_right_time)
        left_age = self._age_seconds(self.last_left_time)
        publish_age = self._age_seconds(self.last_publish_time)
        right_age_text = f'{right_age:.3f}s' if right_age is not None else 'never'
        left_age_text = f'{left_age:.3f}s' if left_age is not None else 'never'
        publish_age_text = f'{publish_age:.3f}s' if publish_age is not None else 'never'

        # Status line printed every second to make runtime debugging deterministic.
        self.get_logger().info(
            '[diag] '
            f'w_R_topic={self.right_wheel_topic} pubs={right_pub_count} types={right_types if right_types else ["none"]}; '
            f'w_L_topic={self.left_wheel_topic} pubs={left_pub_count} types={left_types if left_types else ["none"]}; '
            f'w_R_rx_age={right_age_text}; w_L_rx_age={left_age_text}; last_pub_age={publish_age_text}'
        )

        if right_pub_count == 0 or left_pub_count == 0:
            self.get_logger().warn(
                'No encoder publishers detected on configured topics. '
                'Check topic names or namespaces in your simulator.'
            )
            return

        if right_types and any(t != 'std_msgs/msg/Float32' for t in right_types):
            self.get_logger().warn(
                f'Unexpected type on {self.right_wheel_topic}: {right_types}. Expected std_msgs/msg/Float32.'
            )

        if left_types and any(t != 'std_msgs/msg/Float32' for t in left_types):
            self.get_logger().warn(
                f'Unexpected type on {self.left_wheel_topic}: {left_types}. Expected std_msgs/msg/Float32.'
            )

        if right_age is None or left_age is None:
            self.get_logger().warn(
                'Encoder topics exist but this node has not received both streams yet.'
            )
            return

        if right_age > self.timeout_seconds or left_age > self.timeout_seconds:
            self.get_logger().warn(
                'Encoder data became stale. The estimator is intentionally not publishing until fresh data returns.'
            )

    def right_callback(self, msg):
        """Receive right wheel angular velocity (ω_R)."""
        self.w_R = float(msg.data)
        self.has_right = True
        self.last_right_time = self.get_clock().now()

    def left_callback(self, msg):
        """Receive left wheel angular velocity (ω_L)."""
        self.w_L = float(msg.data)
        self.has_left = True
        self.last_left_time = self.get_clock().now()

    def data_fresh(self):
        """Check if both encoder data streams are recent."""
        if not self.has_right or not self.has_left:
            return False

        now = self.get_clock().now()
        right_age = (now.nanoseconds - self.last_right_time.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.last_left_time.nanoseconds) / 1e9
        return right_age <= self.timeout_seconds and left_age <= self.timeout_seconds

    def compute_robot_speeds(self):
        """
        Compute robot velocities from wheel angular velocities.
        
        v_R = w_R · r
        v_L = w_L · r
        v = (v_R + v_L) / 2
        w = (v_R - v_L) / b
        """
        if self.encoders_are_angular_speed:
            v_R = self.w_R * self.r
            v_L = self.w_L * self.r
        else:
            v_R = self.w_R
            v_L = self.w_L

        v = 0.5 * (v_R + v_L)
        w = (v_R - v_L) / self.b
        return v, w

    def maybe_log(self, v, w):
        """Log speeds at configured rate."""
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.last_log_time.nanoseconds) / 1e9
        if elapsed < 1.0 / self.log_rate_hz:
            return

        self.last_log_time = now
        self.get_logger().info(
            f'w_R={self.w_R:.4f}, w_L={self.w_L:.4f} [rad/s] → v={v:.4f} m/s, w={w:.4f} rad/s'
        )

    def timer_callback(self):
        """
        Main timer callback: compute speeds from encoders and publish.
        """
        if not self.data_fresh():
            return

        v, w = self.compute_robot_speeds()

        # Publish individual speed components
        linear_msg = Float32()
        linear_msg.data = float(v)
        self.linear_pub.publish(linear_msg)

        angular_msg = Float32()
        angular_msg.data = float(w)
        self.angular_pub.publish(angular_msg)

        # Publish combined Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(v)
        twist_msg.angular.z = float(w)
        self.twist_pub.publish(twist_msg)

        self.last_publish_time = self.get_clock().now()
        self.maybe_log(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

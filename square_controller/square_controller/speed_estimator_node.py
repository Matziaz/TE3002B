#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class SpeedEstimatorNode(Node):
    def __init__(self):
        super().__init__('speed_estimator_node')

        self.declare_parameter('right_wheel_topic', '/VelocityEncR')
        self.declare_parameter('left_wheel_topic', '/VelocityEncL')
        self.declare_parameter('linear_speed_topic', '/puzzlebot/linear_speed')
        self.declare_parameter('angular_speed_topic', '/puzzlebot/angular_speed')
        self.declare_parameter('estimated_twist_topic', '/puzzlebot/estimated_twist')
        self.declare_parameter('publish_rate', 20.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('encoders_are_angular_speed', True)

        self.declare_parameter('timeout_seconds', 0.5)
        self.declare_parameter('log_rate_hz', 2.0)

        self.right_wheel_topic = str(self.get_parameter('right_wheel_topic').value)
        self.left_wheel_topic = str(self.get_parameter('left_wheel_topic').value)
        self.linear_speed_topic = str(self.get_parameter('linear_speed_topic').value)
        self.angular_speed_topic = str(self.get_parameter('angular_speed_topic').value)
        self.estimated_twist_topic = str(self.get_parameter('estimated_twist_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.encoders_are_angular_speed = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.timeout_seconds = float(self.get_parameter('timeout_seconds').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        self.publish_rate = max(1.0, self.publish_rate)
        self.wheel_radius = max(1e-4, self.wheel_radius)
        self.wheel_base = max(1e-4, self.wheel_base)
        self.timeout_seconds = max(0.05, self.timeout_seconds)
        self.log_rate_hz = max(0.2, self.log_rate_hz)

        self.wr = 0.0
        self.wl = 0.0
        self.has_right = False
        self.has_left = False
        self.last_right_time = None
        self.last_left_time = None
        self.last_log_time = self.get_clock().now()
        self.last_diag_time = self.get_clock().now()
        self.last_publish_time = None

        self.linear_pub = self.create_publisher(Float32, self.linear_speed_topic, 10)
        self.angular_pub = self.create_publisher(Float32, self.angular_speed_topic, 10)
        self.twist_pub = self.create_publisher(Twist, self.estimated_twist_topic, 10)

        self.right_sub = self.create_subscription(Float32, self.right_wheel_topic, self.right_callback, 10)
        self.left_sub = self.create_subscription(Float32, self.left_wheel_topic, self.left_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.diag_timer = self.create_timer(1.0, self.diagnostic_callback)

        self.get_logger().info('Speed estimator node started (Activity 1).')
        self.get_logger().info(
            f'Subscribing: R={self.right_wheel_topic}, L={self.left_wheel_topic} | '
            f'Publishing: v={self.linear_speed_topic}, w={self.angular_speed_topic}, twist={self.estimated_twist_topic}'
        )
        self.get_logger().info(
            f'Params: wheel_radius={self.wheel_radius:.4f} m, wheel_base={self.wheel_base:.4f} m, '
            f'encoders_are_angular_speed={self.encoders_are_angular_speed}'
        )

    def _topic_publishers_summary(self, topic_name):
        infos = self.get_publishers_info_by_topic(topic_name)
        if not infos:
            return 0, []

        types = sorted({info.topic_type for info in infos})
        return len(infos), types

    def _age_seconds(self, stamp):
        if stamp is None:
            return None
        now = self.get_clock().now()
        return (now.nanoseconds - stamp.nanoseconds) / 1e9

    def diagnostic_callback(self):
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
            f'R_topic={self.right_wheel_topic} pubs={right_pub_count} types={right_types if right_types else ["none"]}; '
            f'L_topic={self.left_wheel_topic} pubs={left_pub_count} types={left_types if left_types else ["none"]}; '
            f'R_rx_age={right_age_text}; L_rx_age={left_age_text}; last_pub_age={publish_age_text}'
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
        self.wr = float(msg.data)
        self.has_right = True
        self.last_right_time = self.get_clock().now()

    def left_callback(self, msg):
        self.wl = float(msg.data)
        self.has_left = True
        self.last_left_time = self.get_clock().now()

    def data_fresh(self):
        if not self.has_right or not self.has_left:
            return False

        now = self.get_clock().now()
        right_age = (now.nanoseconds - self.last_right_time.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.last_left_time.nanoseconds) / 1e9
        return right_age <= self.timeout_seconds and left_age <= self.timeout_seconds

    def compute_robot_speeds(self):
        if self.encoders_are_angular_speed:
            vr = self.wr * self.wheel_radius
            vl = self.wl * self.wheel_radius
        else:
            vr = self.wr
            vl = self.wl

        linear_speed = 0.5 * (vr + vl)
        angular_speed = (vr - vl) / self.wheel_base
        return linear_speed, angular_speed

    def maybe_log(self, linear_speed, angular_speed):
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.last_log_time.nanoseconds) / 1e9
        if elapsed < 1.0 / self.log_rate_hz:
            return

        self.last_log_time = now
        self.get_logger().info(
            f'wr={self.wr:.4f}, wl={self.wl:.4f} -> v={linear_speed:.4f} m/s, w={angular_speed:.4f} rad/s'
        )

    def timer_callback(self):
        if not self.data_fresh():
            return

        linear_speed, angular_speed = self.compute_robot_speeds()

        linear_msg = Float32()
        linear_msg.data = float(linear_speed)
        self.linear_pub.publish(linear_msg)

        angular_msg = Float32()
        angular_msg.data = float(angular_speed)
        self.angular_pub.publish(angular_msg)

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_speed)
        twist_msg.angular.z = float(angular_speed)
        self.twist_pub.publish(twist_msg)
        self.last_publish_time = self.get_clock().now()

        self.maybe_log(linear_speed, angular_speed)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

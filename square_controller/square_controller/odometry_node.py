#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('right_wheel_topic', '/VelocityEncR')
        self.declare_parameter('left_wheel_topic', '/VelocityEncL')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('encoders_are_angular_speed', True)

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_theta', 0.0)

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('encoder_timeout_seconds', 0.5)

        self.right_wheel_topic = str(self.get_parameter('right_wheel_topic').value)
        self.left_wheel_topic = str(self.get_parameter('left_wheel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.encoders_are_angular_speed = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.theta = float(self.get_parameter('start_theta').value)

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.encoder_timeout_seconds = float(self.get_parameter('encoder_timeout_seconds').value)

        self.publish_rate = max(1.0, self.publish_rate)
        self.wheel_radius = max(1e-4, self.wheel_radius)
        self.wheel_base = max(1e-4, self.wheel_base)
        self.encoder_timeout_seconds = max(0.05, self.encoder_timeout_seconds)

        self.wr = 0.0
        self.wl = 0.0
        self.last_right_stamp = None
        self.last_left_stamp = None
        self.last_update_stamp = self.get_clock().now()

        self.create_subscription(Float32, self.right_wheel_topic, self.right_callback, 10)
        self.create_subscription(Float32, self.left_wheel_topic, self.left_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_and_publish)

        self.get_logger().info('Odometry node started (Activity 2).')
        self.get_logger().info(
            f'Subscribing: R={self.right_wheel_topic}, L={self.left_wheel_topic} | Publishing: {self.odom_topic}'
        )

    def right_callback(self, msg):
        self.wr = float(msg.data)
        self.last_right_stamp = self.get_clock().now()

    def left_callback(self, msg):
        self.wl = float(msg.data)
        self.last_left_stamp = self.get_clock().now()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def encoder_data_fresh(self):
        if self.last_right_stamp is None or self.last_left_stamp is None:
            return False

        now = self.get_clock().now()
        right_age = (now.nanoseconds - self.last_right_stamp.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.last_left_stamp.nanoseconds) / 1e9
        return right_age <= self.encoder_timeout_seconds and left_age <= self.encoder_timeout_seconds

    def wheel_to_linear(self, wheel_value):
        if self.encoders_are_angular_speed:
            return wheel_value * self.wheel_radius
        return wheel_value

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_update_stamp.nanoseconds) / 1e9
        self.last_update_stamp = now

        if dt <= 1e-6:
            return

        if self.encoder_data_fresh():
            vr = self.wheel_to_linear(self.wr)
            vl = self.wheel_to_linear(self.wl)
            v = 0.5 * (vr + vl)
            w = (vr - vl) / self.wheel_base
        else:
            v = 0.0
            w = 0.0

        self.theta = self.normalize_angle(self.theta + w * dt)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        half = 0.5 * self.theta
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(half)
        odom_msg.pose.pose.orientation.w = math.cos(half)

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

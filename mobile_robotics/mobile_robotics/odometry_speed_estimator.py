#!/usr/bin/env python3

import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class OdometryAndSpeedEstimator(Node):
    def __init__(self):
        super().__init__('odometry_speed_estimator')

        self.declare_parameter('right_wheel_encoder_topic', '/right_wheel_encoder')
        self.declare_parameter('left_wheel_encoder_topic', '/left_wheel_encoder')
        self.declare_parameter('robot_odometry_topic', '/robot_odometry')
        self.declare_parameter('robot_linear_velocity_topic', '/robot_linear_velocity')
        self.declare_parameter('robot_angular_velocity_topic', '/robot_angular_velocity')
        self.declare_parameter('velocity_estimate_topic', '/velocity_estimate')

        self.declare_parameter('odometry_update_rate', 50.0)
        self.declare_parameter('velocity_estimation_rate', 20.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('encoders_are_angular_speed', True)

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_heading', 0.0)

        self.declare_parameter('odometry_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('encoder_timeout', 0.5)

        self.right_encoder_topic = str(self.get_parameter('right_wheel_encoder_topic').value)
        self.left_encoder_topic = str(self.get_parameter('left_wheel_encoder_topic').value)
        self.odometry_topic = str(self.get_parameter('robot_odometry_topic').value)
        self.linear_velocity_topic = str(self.get_parameter('robot_linear_velocity_topic').value)
        self.angular_velocity_topic = str(self.get_parameter('robot_angular_velocity_topic').value)
        self.velocity_estimate_topic = str(self.get_parameter('velocity_estimate_topic').value)

        self.odometry_rate = max(1.0, float(self.get_parameter('odometry_update_rate').value))
        self.velocity_rate = max(1.0, float(self.get_parameter('velocity_estimation_rate').value))

        self.wheel_radius = max(1e-4, float(self.get_parameter('wheel_radius').value))
        self.wheel_base = max(1e-4, float(self.get_parameter('wheel_base').value))
        self.is_angular_encoder = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.robot_x = float(self.get_parameter('initial_x').value)
        self.robot_y = float(self.get_parameter('initial_y').value)
        self.robot_heading = float(self.get_parameter('initial_heading').value)

        self.odom_frame = str(self.get_parameter('odometry_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.encoder_timeout = max(0.05, float(self.get_parameter('encoder_timeout').value))

        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity = 0.0
        self.right_encoder_timestamp = None
        self.left_encoder_timestamp = None

        self.estimated_linear_velocity = 0.0
        self.estimated_angular_velocity = 0.0

        self.last_odometry_time = self.get_clock().now()
        self.last_velocity_time = self.get_clock().now()

        self.odometry_pub = self.create_publisher(Odometry, self.odometry_topic, 10)
        self.linear_velocity_pub = self.create_publisher(Float32, self.linear_velocity_topic, 10)
        self.angular_velocity_pub = self.create_publisher(Float32, self.angular_velocity_topic, 10)
        self.velocity_estimate_pub = self.create_publisher(Twist, self.velocity_estimate_topic, 10)

        self.create_subscription(Float32, self.right_encoder_topic, self.on_right_encoder, 10)
        self.create_subscription(Float32, self.left_encoder_topic, self.on_left_encoder, 10)

        self.odometry_timer = self.create_timer(1.0 / self.odometry_rate, self.update_odometry)
        self.velocity_timer = self.create_timer(1.0 / self.velocity_rate, self.estimate_velocities)

        self.get_logger().info(f'Odometry and Speed Estimator initialized at ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_heading:.3f})')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def on_right_encoder(self, msg):
        self.right_wheel_velocity = float(msg.data)
        self.right_encoder_timestamp = self.get_clock().now()

    def on_left_encoder(self, msg):
        self.left_wheel_velocity = float(msg.data)
        self.left_encoder_timestamp = self.get_clock().now()

    def are_encoders_valid(self):
        if self.right_encoder_timestamp is None or self.left_encoder_timestamp is None:
            return False
        now = self.get_clock().now()
        right_age = (now.nanoseconds - self.right_encoder_timestamp.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.left_encoder_timestamp.nanoseconds) / 1e9
        return right_age <= self.encoder_timeout and left_age <= self.encoder_timeout

    def encoder_to_linear_velocity(self, angular_velocity):
        if self.is_angular_encoder:
            return angular_velocity * self.wheel_radius
        return angular_velocity

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_odometry_time.nanoseconds) / 1e9
        self.last_odometry_time = now

        if dt <= 1e-6:
            return

        if self.are_encoders_valid():
            right_linear = self.encoder_to_linear_velocity(self.right_wheel_velocity)
            left_linear = self.encoder_to_linear_velocity(self.left_wheel_velocity)
            linear_velocity = 0.5 * (right_linear + left_linear)
            angular_velocity = (right_linear - left_linear) / self.wheel_base
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        heading_prev = self.robot_heading
        self.robot_heading = self.normalize_angle(self.robot_heading + angular_velocity * dt)
        self.robot_x += linear_velocity * math.cos(heading_prev) * dt
        self.robot_y += linear_velocity * math.sin(heading_prev) * dt

        self.get_logger().debug(f'Odometry updated: x={self.robot_x:.4f}, y={self.robot_y:.4f}, theta={self.robot_heading:.4f}, v={linear_velocity:.4f}, w={angular_velocity:.4f}')

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.robot_frame

        odom_msg.pose.pose.position.x = float(self.robot_x)
        odom_msg.pose.pose.position.y = float(self.robot_y)
        odom_msg.pose.pose.position.z = 0.0

        half_heading = 0.5 * self.robot_heading
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(half_heading)
        odom_msg.pose.pose.orientation.w = math.cos(half_heading)

        odom_msg.twist.twist.linear.x = float(linear_velocity)
        odom_msg.twist.twist.angular.z = float(angular_velocity)

        self.odometry_pub.publish(odom_msg)

    def estimate_velocities(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_velocity_time.nanoseconds) / 1e9
        self.last_velocity_time = now

        if dt <= 1e-6:
            return

        if self.are_encoders_valid():
            right_linear = self.encoder_to_linear_velocity(self.right_wheel_velocity)
            left_linear = self.encoder_to_linear_velocity(self.left_wheel_velocity)
            self.estimated_linear_velocity = 0.5 * (right_linear + left_linear)
            self.estimated_angular_velocity = (right_linear - left_linear) / self.wheel_base
        else:
            self.estimated_linear_velocity = 0.0
            self.estimated_angular_velocity = 0.0

        linear_msg = Float32()
        linear_msg.data = float(self.estimated_linear_velocity)
        self.linear_velocity_pub.publish(linear_msg)

        angular_msg = Float32()
        angular_msg.data = float(self.estimated_angular_velocity)
        self.angular_velocity_pub.publish(angular_msg)

        velocity_msg = Twist()
        velocity_msg.linear.x = float(self.estimated_linear_velocity)
        velocity_msg.angular.z = float(self.estimated_angular_velocity)
        self.velocity_estimate_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryAndSpeedEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

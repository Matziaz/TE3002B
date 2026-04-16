#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class EncoderSimulator(Node):
    def __init__(self):
        super().__init__('encoder_simulator')

        self.declare_parameter('velocity_command_topic', '/cmd_vel')
        self.declare_parameter('right_wheel_encoder_topic', '/right_wheel_encoder')
        self.declare_parameter('left_wheel_encoder_topic', '/left_wheel_encoder')

        self.declare_parameter('simulation_rate', 50.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)

        self.velocity_command_topic = str(self.get_parameter('velocity_command_topic').value)
        self.right_encoder_topic = str(self.get_parameter('right_wheel_encoder_topic').value)
        self.left_encoder_topic = str(self.get_parameter('left_wheel_encoder_topic').value)

        self.sim_rate = max(1.0, float(self.get_parameter('simulation_rate').value))
        self.wheel_radius = max(1e-4, float(self.get_parameter('wheel_radius').value))
        self.wheel_base = max(1e-4, float(self.get_parameter('wheel_base').value))

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity = 0.0

        self.right_encoder_pub = self.create_publisher(Float32, self.right_encoder_topic, 10)
        self.left_encoder_pub = self.create_publisher(Float32, self.left_encoder_topic, 10)

        self.create_subscription(Twist, self.velocity_command_topic, self.on_velocity_command, 10)

        self.sim_timer = self.create_timer(1.0 / self.sim_rate, self.publish_encoder_data)

        self.get_logger().info('Encoder Simulator initialized')

    def on_velocity_command(self, msg):
        self.linear_velocity = float(msg.linear.x)
        self.angular_velocity = float(msg.angular.z)
        self.get_logger().debug(f'Velocity command received: v={self.linear_velocity:.3f}, w={self.angular_velocity:.3f}')

    def publish_encoder_data(self):
        self.right_wheel_velocity = (self.linear_velocity + 0.5 * self.angular_velocity * self.wheel_base) / self.wheel_radius
        self.left_wheel_velocity = (self.linear_velocity - 0.5 * self.angular_velocity * self.wheel_base) / self.wheel_radius

        right_msg = Float32()
        right_msg.data = float(self.right_wheel_velocity)
        self.right_encoder_pub.publish(right_msg)

        left_msg = Float32()
        left_msg.data = float(self.left_wheel_velocity)
        self.left_encoder_pub.publish(left_msg)

        self.get_logger().debug(f'Encoders published: w_R={self.right_wheel_velocity:.4f}, w_L={self.left_wheel_velocity:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

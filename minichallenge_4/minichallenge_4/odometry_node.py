#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('axle_length', 0.167)
        self.declare_parameter('update_rate', 0.02)
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('left_velocity_topic', 'VelocityEncL')
        self.declare_parameter('right_velocity_topic', 'VelocityEncR')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('axle_length').value)
        self.dt = float(self.get_parameter('update_rate').value)
        pose_topic = self.get_parameter('pose_topic').value
        left_vel_topic = self.get_parameter('left_velocity_topic').value
        right_vel_topic = self.get_parameter('right_velocity_topic').value
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_pose = self.create_publisher(Pose2D, pose_topic, qos)
        self.pub_linear_velocity = self.create_publisher(Float32, 'odom_linear_velocity', qos)
        self.pub_angular_velocity = self.create_publisher(Float32, 'odom_angular_velocity', qos)

        self.create_subscription(Float32, right_vel_topic, self.wr_cb, qos)
        self.create_subscription(Float32, left_vel_topic, self.wl_cb, qos)

        self.wl = 0.0
        self.wr = 0.0
        self.create_timer(self.dt, self.main_timer_cb)

    def wl_cb(self, msg):
        self.wl = float(msg.data)

    def wr_cb(self, msg):
        self.wr = float(msg.data)

    def main_timer_cb(self):
        v = self.r * (self.wr + self.wl) / 2.0
        omega = self.r * (self.wr - self.wl) / self.L

        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += omega * self.dt
        self.theta = float(np.arctan2(np.sin(self.theta), np.cos(self.theta)))

        pose_msg = Pose2D()
        pose_msg.x = float(self.x)
        pose_msg.y = float(self.y)
        pose_msg.theta = float(self.theta)
        self.pub_pose.publish(pose_msg)

        lin_msg = Float32()
        lin_msg.data = float(v)
        self.pub_linear_velocity.publish(lin_msg)

        ang_msg = Float32()
        ang_msg.data = float(omega)
        self.pub_angular_velocity.publish(ang_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

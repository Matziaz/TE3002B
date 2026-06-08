#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Bug0(Node):
    def __init__(self):
        super().__init__('bug0')

        self.declare_parameter('right_wheel_topic', '/VelocityEncR')
        self.declare_parameter('left_wheel_topic', '/VelocityEncL')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('r', 0.0288)
        self.declare_parameter('b', 0.19)

        self.declare_parameter('encoders_are_angular_speed', True)
        self.declare_parameter('invert_right_encoder', False)
        self.declare_parameter('invert_left_encoder', False)

        self.declare_parameter('x_goal', 8.0)
        self.declare_parameter('y_goal', 0.0)

        self.declare_parameter('v_goal', 0.75)
        self.declare_parameter('v_wall', 0.40)
        self.declare_parameter('w_max', 2.5)

        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('front_obstacle_distance', 0.55)
        self.declare_parameter('path_clear_distance', 0.90)
        self.declare_parameter('desired_wall_distance', 0.40)

        self.right_topic = self.get_parameter('right_wheel_topic').value
        self.left_topic = self.get_parameter('left_wheel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.r = float(self.get_parameter('r').value)
        self.b = float(self.get_parameter('b').value)

        self.encoders_are_angular_speed = bool(
            self.get_parameter('encoders_are_angular_speed').value
        )

        self.invert_right_encoder = bool(
            self.get_parameter('invert_right_encoder').value
        )
        self.invert_left_encoder = bool(
            self.get_parameter('invert_left_encoder').value
        )

        self.x_goal = float(self.get_parameter('x_goal').value)
        self.y_goal = float(self.get_parameter('y_goal').value)

        self.v_goal = float(self.get_parameter('v_goal').value)
        self.v_wall = float(self.get_parameter('v_wall').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.front_obstacle_distance = float(
            self.get_parameter('front_obstacle_distance').value
        )
        self.path_clear_distance = float(
            self.get_parameter('path_clear_distance').value
        )
        self.desired_wall_distance = float(
            self.get_parameter('desired_wall_distance').value
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.right_encoder = 0.0
        self.left_encoder = 0.0
        self.have_right = False
        self.have_left = False
        self.last_right_stamp = None
        self.last_left_stamp = None
        self.encoder_timeout = 0.5

        self.scan_ready = False
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        self.state = 'GO_TO_GOAL'
        self.goal_logged = False
        self.last_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.create_subscription(Float32, self.right_topic, self.right_encoder_cb, 10)
        self.create_subscription(Float32, self.left_topic, self.left_encoder_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Bug0 started.')

    def right_encoder_cb(self, msg):
        self.right_encoder = float(msg.data)
        self.have_right = True
        self.last_right_stamp = self.get_clock().now()

    def left_encoder_cb(self, msg):
        self.left_encoder = float(msg.data)
        self.have_left = True
        self.last_left_stamp = self.get_clock().now()

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        front_mask = np.abs(angles) < math.radians(25)
        left_mask = (angles > math.radians(45)) & (angles < math.radians(115))
        right_mask = (angles < math.radians(-45)) & (angles > math.radians(-115))

        self.front_dist = self.safe_min(ranges[front_mask])
        self.left_dist = self.safe_min(ranges[left_mask])
        self.right_dist = self.safe_min(ranges[right_mask])

        self.scan_ready = True

    def safe_min(self, values):
        if len(values) == 0:
            return float('inf')

        valid = values[(values > 0.12) & np.isfinite(values)]

        if len(valid) == 0:
            return float('inf')

        return float(np.min(valid))

    def encoders_are_recent(self):
        if not self.have_right or not self.have_left:
            return False

        now = self.get_clock().now()

        right_age = (now.nanoseconds - self.last_right_stamp.nanoseconds) / 1e9
        left_age = (now.nanoseconds - self.last_left_stamp.nanoseconds) / 1e9

        return right_age <= self.encoder_timeout and left_age <= self.encoder_timeout

    def update_odometry(self, dt):
        if not self.encoders_are_recent():
            return 0.0, 0.0

        if self.encoders_are_angular_speed:
            v_right = self.r * self.right_encoder
            v_left = self.r * self.left_encoder
        else:
            v_right = self.right_encoder
            v_left = self.left_encoder

        if self.invert_right_encoder:
            v_right *= -1.0

        if self.invert_left_encoder:
            v_left *= -1.0

        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.b

        if abs(w) < 1e-6:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            theta_new = wrap_to_pi(self.theta + w * dt)

            self.x += (v / w) * (math.sin(theta_new) - math.sin(self.theta))
            self.y += (v / w) * (math.cos(self.theta) - math.cos(theta_new))

            self.theta = theta_new
            return v, w

        self.theta = wrap_to_pi(self.theta + w * dt)
        return v, w

    def publish_odom(self, v, w):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.theta)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(msg)

    def distance_to_goal(self):
        return math.hypot(self.x_goal - self.x, self.y_goal - self.y)

    def angle_to_goal(self):
        desired_angle = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
        return wrap_to_pi(desired_angle - self.theta)

    def go_to_goal_control(self):
        angle_error = self.angle_to_goal()

        cmd = Twist()
        cmd.linear.x = 0.08 if abs(angle_error) > 0.6 else self.v_goal
        cmd.angular.z = max(min(2.0 * angle_error, self.w_max), -self.w_max)

        return cmd

    def wall_follow_control(self):
        cmd = Twist()

        error = self.desired_wall_distance - self.right_dist

        if self.front_dist < self.front_obstacle_distance:
            cmd.linear.x = 0.08
            cmd.angular.z = 1.25
        else:
            cmd.linear.x = self.v_wall
            cmd.angular.z = max(min(1.8 * error, self.w_max), -self.w_max)

        return cmd

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now

        dt = max(0.0, min(dt, 0.1))

        v_odom, w_odom = self.update_odometry(dt)
        self.publish_odom(v_odom, w_odom)

        if not self.scan_ready:
            return

        if self.distance_to_goal() <= self.goal_tolerance:
            self.stop_robot()

            if not self.goal_logged:
                self.get_logger().info('Goal reached.')
                self.goal_logged = True

            return

        angle_error = abs(self.angle_to_goal())

        if self.state == 'GO_TO_GOAL':
            if self.front_dist < self.front_obstacle_distance:
                self.state = 'WALL_FOLLOW'
                self.get_logger().info('Switching to WALL_FOLLOW.')

            cmd = self.go_to_goal_control()

        elif self.state == 'WALL_FOLLOW':
            if self.front_dist > self.path_clear_distance and angle_error < 0.35:
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Switching to GO_TO_GOAL.')

            cmd = self.wall_follow_control()

        else:
            self.state = 'GO_TO_GOAL'
            cmd = Twist()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
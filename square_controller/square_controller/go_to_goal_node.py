#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PIDController:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 1e-6:
            dt = 1e-3

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class GoToGoalController(Node):
    def __init__(self):
        super().__init__('go_to_goal_controller')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_rate', 20.0)

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)

        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.05)

        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('min_angular_speed', 0.0)

        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)

        self.declare_parameter('kp_angular', 2.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.0)

        self.declare_parameter('linear_integral_limit', 0.8)
        self.declare_parameter('angular_integral_limit', 1.0)

        self.declare_parameter('turn_in_place_threshold', 0.80)
        self.declare_parameter('settle_time', 0.50)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.ki_linear = float(self.get_parameter('ki_linear').value)
        self.kd_linear = float(self.get_parameter('kd_linear').value)

        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.ki_angular = float(self.get_parameter('ki_angular').value)
        self.kd_angular = float(self.get_parameter('kd_angular').value)

        self.linear_integral_limit = float(self.get_parameter('linear_integral_limit').value)
        self.angular_integral_limit = float(self.get_parameter('angular_integral_limit').value)

        self.turn_in_place_threshold = float(self.get_parameter('turn_in_place_threshold').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.control_rate = max(1.0, self.control_rate)
        self.max_linear_speed = max(0.01, self.max_linear_speed)
        self.max_angular_speed = max(0.05, self.max_angular_speed)
        self.min_linear_speed = max(0.0, min(self.min_linear_speed, self.max_linear_speed))
        self.min_angular_speed = max(0.0, min(self.min_angular_speed, self.max_angular_speed))
        self.distance_tolerance = max(0.01, self.distance_tolerance)
        self.heading_tolerance = max(0.01, self.heading_tolerance)
        self.turn_in_place_threshold = max(self.heading_tolerance, self.turn_in_place_threshold)
        self.settle_time = max(0.0, self.settle_time)

        self.linear_pid = PIDController(
            self.kp_linear,
            self.ki_linear,
            self.kd_linear,
            self.linear_integral_limit,
        )
        self.angular_pid = PIDController(
            self.kp_angular,
            self.ki_angular,
            self.kd_angular,
            self.angular_integral_limit,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_ready = False
        self.goal_reached = False
        self.goal_tolerance_since = None

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Go-to-goal controller started')
        self.get_logger().info(f'Goal = ({self.goal_x:.3f}, {self.goal_y:.3f})')
        self.get_logger().info(
            f'Linear PID: kp={self.kp_linear:.3f}, ki={self.ki_linear:.3f}, kd={self.kd_linear:.3f}'
        )
        self.get_logger().info(
            f'Angular PID: kp={self.kp_angular:.3f}, ki={self.ki_angular:.3f}, kd={self.kd_angular:.3f}'
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)

        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        self.odom_ready = True

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def is_goal_stable(self, distance_error):
        now = self.get_clock().now()
        if distance_error <= self.distance_tolerance:
            if self.goal_tolerance_since is None:
                self.goal_tolerance_since = now
                return False

            elapsed = (now.nanoseconds - self.goal_tolerance_since.nanoseconds) / 1e9
            return elapsed >= self.settle_time

        self.goal_tolerance_since = None
        return False

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now

        if not self.odom_ready:
            self.publish_stop()
            return

        if self.goal_reached:
            self.publish_stop()
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance_error = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - self.current_theta)

        if self.is_goal_stable(distance_error):
            self.goal_reached = True
            self.publish_stop()
            self.get_logger().info('Goal reached and stabilized')
            self.get_logger().info(f'Final pose: x={self.current_x:.4f}, y={self.current_y:.4f}')
            self.get_logger().info(f'xerror={self.goal_x - self.current_x:.4f}, yerror={self.goal_y - self.current_y:.4f}')
            return

        v_cmd = self.linear_pid.update(distance_error, dt)
        w_cmd = self.angular_pid.update(heading_error, dt)

        # Both controllers run simultaneously; linear speed is reduced when heading error is large.
        if abs(heading_error) > self.turn_in_place_threshold:
            v_cmd = 0.0
        else:
            v_cmd *= max(0.0, math.cos(heading_error))

        v_cmd = self.clamp(v_cmd, 0.0, self.max_linear_speed)
        w_cmd = self.clamp(w_cmd, -self.max_angular_speed, self.max_angular_speed)

        if 0.0 < v_cmd < self.min_linear_speed and distance_error > self.distance_tolerance:
            v_cmd = self.min_linear_speed

        w_mag = abs(w_cmd)
        if 0.0 < w_mag < self.min_angular_speed and abs(heading_error) > self.heading_tolerance:
            w_cmd = math.copysign(self.min_angular_speed, w_cmd)

        self.publish_cmd(v_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalController()
    rclpy.spin(node)
    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

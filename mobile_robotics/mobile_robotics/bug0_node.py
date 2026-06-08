#!/usr/bin/env python3
"""
Bug 0 reactive navigation node for ROS 2.

States:
    GtG: move toward the goal with proportional control.
    WF: follow the obstacle boundary when the frontal path is blocked.
    DONE: stop after reaching the goal.

The switch back to GtG is done when the goal is approximately in front of
robot and the frontal LiDAR sector is clear. This matches the Bug 0 idea: leave
wall-following as soon as the robot can move directly toward the goal again.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Bug0Node(Node):
    GtG = 'GtG'
    WF = 'WF'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('bug0_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('listen_goal_topic', False)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('x_goal', 5.0)
        self.declare_parameter('y_goal', 0.0)

        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('heading_tolerance_to_leave_wall', 0.35)
        self.declare_parameter('front_block_distance', 0.65)
        self.declare_parameter('front_clear_distance', 0.85)
        self.declare_parameter('side_clear_distance', 0.75)
        self.declare_parameter('target_wall_distance', 0.40)
        self.declare_parameter('danger_distance', 0.27)

        self.declare_parameter('v_gtg_max', 0.28)
        self.declare_parameter('v_wall', 0.12)
        self.declare_parameter('omega_max', 1.2)
        self.declare_parameter('k_v', 0.8)
        self.declare_parameter('k_omega_gtg', 1.8)
        self.declare_parameter('k_omega_wall', 1.4)
        self.declare_parameter('k_wall_distance', 0.9)
        self.declare_parameter('WF_direction', 'CCW')
        self.declare_parameter('min_WF_time', 1.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.listen_goal_topic = bool(self.get_parameter('listen_goal_topic').value)
        self.control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.x_goal = float(self.get_parameter('x_goal').value)
        self.y_goal = float(self.get_parameter('y_goal').value)

        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.heading_tolerance_to_leave_wall = float(self.get_parameter('heading_tolerance_to_leave_wall').value)
        self.front_block_distance = float(self.get_parameter('front_block_distance').value)
        self.front_clear_distance = float(self.get_parameter('front_clear_distance').value)
        self.side_clear_distance = float(self.get_parameter('side_clear_distance').value)
        self.target_wall_distance = float(self.get_parameter('target_wall_distance').value)
        self.danger_distance = float(self.get_parameter('danger_distance').value)

        self.v_gtg_max = float(self.get_parameter('v_gtg_max').value)
        self.v_wall = float(self.get_parameter('v_wall').value)
        self.omega_max = float(self.get_parameter('omega_max').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_omega_gtg = float(self.get_parameter('k_omega_gtg').value)
        self.k_omega_wall = float(self.get_parameter('k_omega_wall').value)
        self.k_wall_distance = float(self.get_parameter('k_wall_distance').value)
        self.WF_direction = str(self.get_parameter('WF_direction').value).upper()
        self.min_WF_time = float(self.get_parameter('min_WF_time').value)

        if self.WF_direction not in ('CW', 'CCW'):
            self.WF_direction = 'CCW'

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.scan_ready = False

        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.closest_range = math.inf
        self.theta_closest = 0.0

        self.state = self.GtG
        self.wall_start_time = None
        self.last_state = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        if self.listen_goal_topic:
            self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)

        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Bug 0 node started.')
        self.get_logger().info(f'Goal: ({self.x_goal:.2f}, {self.y_goal:.2f}) | wall mode: {self.WF_direction}')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def seconds_since(self, stamp):
        if stamp is None:
            return math.inf
        now = self.get_clock().now()
        return (now.nanoseconds - stamp.nanoseconds) / 1e9

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def goal_cb(self, msg):
        self.x_goal = float(msg.pose.position.x)
        self.y_goal = float(msg.pose.position.y)
        self.state = self.GtG
        self.wall_start_time = None
        self.get_logger().info(f'New goal: ({self.x_goal:.2f}, {self.y_goal:.2f})')

    def odom_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def scan_cb(self, msg):
        front_min = math.inf
        left_min = math.inf
        right_min = math.inf
        closest_range = math.inf
        theta_closest = 0.0

        angle = float(msg.angle_min)
        for r_raw in msg.ranges:
            r = float(r_raw)
            theta = self.normalize_angle(angle)
            valid = math.isfinite(r) and r > max(0.05, float(msg.range_min))
            if msg.range_max > 0.0:
                valid = valid and r < float(msg.range_max)

            if valid:
                if r < closest_range:
                    closest_range = r
                    theta_closest = theta

                if abs(theta) <= math.radians(25.0):
                    front_min = min(front_min, r)
                elif math.radians(25.0) < theta <= math.radians(100.0):
                    left_min = min(left_min, r)
                elif -math.radians(100.0) <= theta < -math.radians(25.0):
                    right_min = min(right_min, r)

            angle += float(msg.angle_increment)

        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min
        self.closest_range = closest_range
        self.theta_closest = theta_closest
        self.scan_ready = True

    def goal_error(self):
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(theta_goal - self.yaw)
        return distance, heading_error

    def path_to_goal_is_clear(self, heading_error):
        front_clear = (not math.isfinite(self.front_min)) or self.front_min > self.front_clear_distance
        goal_in_front = abs(heading_error) < self.heading_tolerance_to_leave_wall
        waited_enough = self.seconds_since(self.wall_start_time) >= self.min_WF_time
        return front_clear and goal_in_front and waited_enough

    def update_state(self, distance, heading_error):
        if distance <= self.goal_tolerance:
            self.state = self.DONE
            return

        obstacle_front = math.isfinite(self.front_min) and self.front_min < self.front_block_distance

        if self.state == self.GtG and obstacle_front:
            self.state = self.WF
            self.wall_start_time = self.get_clock().now()
        elif self.state == self.WF and self.path_to_goal_is_clear(heading_error):
            self.state = self.GtG
            self.wall_start_time = None

        if self.state != self.last_state:
            self.get_logger().info(f'State: {self.state}')
            self.last_state = self.state

    def GtG_control(self, distance, heading_error):
        v = self.clamp(self.k_v * distance, 0.0, self.v_gtg_max)
        if abs(heading_error) > 0.55:
            v = 0.0
        else:
            v *= max(0.0, math.cos(heading_error))
        w = self.clamp(self.k_omega_gtg * heading_error, -self.omega_max, self.omega_max)
        return v, w

    def WF_control(self):
        if not math.isfinite(self.closest_range):
            return 0.06, 0.45

        if self.WF_direction == 'CW':
            theta_fw = self.theta_closest - math.pi / 2.0
            distance_error = self.closest_range - self.target_wall_distance
            theta_fw += self.k_wall_distance * distance_error
            escape_sign = 1.0
        else:
            theta_fw = self.theta_closest + math.pi / 2.0
            distance_error = self.closest_range - self.target_wall_distance
            theta_fw -= self.k_wall_distance * distance_error
            escape_sign = -1.0

        theta_fw = self.normalize_angle(theta_fw)
        w = self.k_omega_wall * theta_fw

        v = self.v_wall
        if math.isfinite(self.front_min) and self.front_min < self.danger_distance:
            v = 0.0
            w = escape_sign * self.omega_max
        elif math.isfinite(self.front_min) and self.front_min < self.front_block_distance:
            v = 0.05

        w = self.clamp(w, -self.omega_max, self.omega_max)
        return v, w

    def control_loop(self):
        if not self.odom_ready or not self.scan_ready:
            self.stop()
            return

        distance, heading_error = self.goal_error()
        self.update_state(distance, heading_error)

        if self.state == self.DONE:
            self.stop()
            return
        if self.state == self.GtG:
            v, w = self.GtG_control(distance, heading_error)
        else:
            v, w = self.WF_control()

        self.publish_cmd(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

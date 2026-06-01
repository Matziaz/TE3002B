#!/usr/bin/env python3
"""
Go-to-Goal Navigation Node
===========================

Control a differential drive robot to reach a target position using proportional control.

Mathematical Model:
    Error calculations (from homework):
        e_d = √((x_G - x)² + (y_G - y)²)          # Distance error
        θ_G = atan2(y_G - y, x_G - x)              # Desired heading
        e_th = θ_G - θ_R                             # Heading error (normalized)
    
    Proportional controller:
        v = k_v * e_d                              # Linear velocity command
        w = k_ω * e_th                             # Angular velocity command
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')

        # ========== TOPIC PARAMETERS ==========
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('control_rate', 30.0)

        # ========== GOAL POSITION ==========
        self.declare_parameter('x_goal', 1.0)
        self.declare_parameter('y_goal', 0.0)
        self.declare_parameter('listen_goal_topic', False)

        # ========== TOLERANCES ==========
        self.declare_parameter('e_d_tolerance', 0.05)
        self.declare_parameter('e_theta_tolerance', 0.05)
        self.declare_parameter('settle_time', 0.5)

        # ========== VELOCITY LIMITS ==========
        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('v_min', 0.01)
        self.declare_parameter('omega_min', 0.05)

        # ========== GO-TO-GOAL GAINS ==========
        self.declare_parameter('k_v', 1.0)
        self.declare_parameter('k_omega', 2.5)

        # ========== SPECIAL BEHAVIOR ==========
        self.declare_parameter('turn_in_place_threshold', 0.8)

        # ========== OBSTACLE AVOIDANCE PARAMETERS ==========
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('scan_topic', '/scan')

        # Distances [m]
        self.declare_parameter('d_safety', 0.35)      
        self.declare_parameter('d_start_ao', 0.85)    
        self.declare_parameter('d_clear_ao', 1.10)    

        # Angles [rad]
        self.declare_parameter('avoid_front_angle', 1.0472) 
        self.declare_parameter('avoid_side_angle', 1.5708)  

        # Avoidance controller
        self.declare_parameter('avoid_linear_speed', 0.12)   
        self.declare_parameter('escape_angular_speed', 0.75) 
        self.declare_parameter('avoid_angular_speed', 0.55)  
        self.declare_parameter('goal_blend_during_avoid', 0.25)  
        self.declare_parameter('avoid_min_time', 0.8)        
        self.declare_parameter('front_bias', 0.08)           

        # ========== PARAMETER LOADING ==========
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.x_goal = float(self.get_parameter('x_goal').value)
        self.y_goal = float(self.get_parameter('y_goal').value)
        self.listen_goal_topic = bool(self.get_parameter('listen_goal_topic').value)

        self.e_d_tolerance = float(self.get_parameter('e_d_tolerance').value)
        self.e_theta_tolerance = float(self.get_parameter('e_theta_tolerance').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.omega_max = float(self.get_parameter('omega_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.omega_min = float(self.get_parameter('omega_min').value)

        self.k_v = float(self.get_parameter('k_v').value)
        self.k_omega = float(self.get_parameter('k_omega').value)

        self.turn_in_place_threshold = float(self.get_parameter('turn_in_place_threshold').value)

        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.d_safety = float(self.get_parameter('d_safety').value)
        self.d_start_ao = float(self.get_parameter('d_start_ao').value)
        self.d_clear_ao = float(self.get_parameter('d_clear_ao').value)
        self.avoid_front_angle = float(self.get_parameter('avoid_front_angle').value)
        self.avoid_side_angle = float(self.get_parameter('avoid_side_angle').value)
        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.escape_angular_speed = float(self.get_parameter('escape_angular_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.goal_blend_during_avoid = float(self.get_parameter('goal_blend_during_avoid').value)
        self.avoid_min_time = float(self.get_parameter('avoid_min_time').value)
        self.front_bias = float(self.get_parameter('front_bias').value)

        # ========== PARAMETER CONSTRAINTS ==========
        self.control_rate = max(1.0, self.control_rate)
        self.e_d_tolerance = max(0.005, self.e_d_tolerance)
        self.e_theta_tolerance = max(0.005, self.e_theta_tolerance)
        self.settle_time = max(0.0, self.settle_time)

        self.v_max = max(0.01, self.v_max)
        self.omega_max = max(0.05, self.omega_max)
        self.v_min = self.clamp(self.v_min, 0.0, self.v_max)
        self.omega_min = self.clamp(self.omega_min, 0.0, self.omega_max)
        self.turn_in_place_threshold = max(self.e_theta_tolerance, self.turn_in_place_threshold)

        self.d_safety = max(0.05, self.d_safety)
        self.d_start_ao = max(self.d_safety + 0.05, self.d_start_ao)
        self.d_clear_ao = max(self.d_start_ao + 0.05, self.d_clear_ao)
        self.avoid_front_angle = self.clamp(self.avoid_front_angle, 0.10, math.pi)
        self.avoid_side_angle = self.clamp(self.avoid_side_angle, self.avoid_front_angle, math.pi)
        self.avoid_linear_speed = self.clamp(self.avoid_linear_speed, 0.0, self.v_max)
        self.escape_angular_speed = self.clamp(self.escape_angular_speed, 0.05, self.omega_max)
        self.avoid_angular_speed = self.clamp(self.avoid_angular_speed, 0.05, self.omega_max)
        self.goal_blend_during_avoid = self.clamp(self.goal_blend_during_avoid, 0.0, 1.0)
        self.avoid_min_time = max(0.0, self.avoid_min_time)
        self.front_bias = max(0.0, self.front_bias)

        # ========== ROBOT STATE ==========
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.goal_reached = False
        self.goal_tolerance_since = None

        
        self.scan_ready = False
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.closest_range = math.inf
        self.theta_closest = 0.0

        self.avoid_active = False
        self.avoid_direction = 1.0  
        self.avoid_started_time = None
        self.last_avoid_log_time = None

        self.last_control_stamp = self.get_clock().now()

        # ========== ROS SETUP ==========
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.enable_obstacle_avoidance:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        if self.listen_goal_topic:
            self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Go-to-goal node with obstacle avoidance started.')
        self.get_logger().info(f'Goal = ({self.x_goal:.3f}, {self.y_goal:.3f}) | listen_goal_topic={self.listen_goal_topic}')
        self.get_logger().info(f'Gains: k_v={self.k_v:.3f}, k_omega={self.k_omega:.3f}')
        self.get_logger().info(
            f'Obstacle avoidance={self.enable_obstacle_avoidance} | '
            f'd_safety={self.d_safety:.2f}, d_start_ao={self.d_start_ao:.2f}, d_clear_ao={self.d_clear_ao:.2f}'
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def seconds_since(self, start_time, now):
        if start_time is None:
            return math.inf
        return (now.nanoseconds - start_time.nanoseconds) / 1e9

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def goal_callback(self, msg):
        self.x_goal = float(msg.pose.position.x)
        self.y_goal = float(msg.pose.position.y)
        self.goal_reached = False
        self.goal_tolerance_since = None

        self.avoid_active = False
        self.avoid_started_time = None

        self.get_logger().info(f'New goal: ({self.x_goal:.3f}, {self.y_goal:.3f})')

    def odom_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def scan_callback(self, msg):
        front_min = math.inf
        left_min = math.inf
        right_min = math.inf
        closest_range = math.inf
        theta_closest = 0.0

        angle = float(msg.angle_min)
        angle_increment = float(msg.angle_increment)

        for r in msg.ranges:
            r = float(r)
            theta = self.normalize_angle(angle)

            valid = math.isfinite(r)
            if msg.range_min > 0.0:
                valid = valid and r >= float(msg.range_min)
            if msg.range_max > 0.0:
                valid = valid and r <= float(msg.range_max)

            if valid and abs(theta) <= self.avoid_side_angle:
                if r < closest_range:
                    closest_range = r
                    theta_closest = theta

                if abs(theta) <= self.avoid_front_angle:
                    front_min = min(front_min, r)
                elif theta > 0.0:
                    left_min = min(left_min, r)
                else:
                    right_min = min(right_min, r)

            angle += angle_increment

        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min
        self.closest_range = closest_range
        self.theta_closest = theta_closest
        self.scan_ready = True

    # ============================================================
    # Obstacle avoidance
    # ============================================================
    def choose_avoid_direction(self):
        
        left_clearance = self.left_min
        right_clearance = self.right_min

        if not math.isfinite(left_clearance):
            left_clearance = self.d_clear_ao + 1.0
        if not math.isfinite(right_clearance):
            right_clearance = self.d_clear_ao + 1.0

        if left_clearance > right_clearance + self.front_bias:
            return 1.0
        if right_clearance > left_clearance + self.front_bias:
            return -1.0

        if self.theta_closest > 0.0:
            return -1.0
        return 1.0

    def update_avoidance_state(self, now):
        
        if not self.enable_obstacle_avoidance or not self.scan_ready:
            self.avoid_active = False
            return

        obstacle_in_front = math.isfinite(self.front_min) and self.front_min < self.d_start_ao
        front_is_clear = (not math.isfinite(self.front_min)) or self.front_min > self.d_clear_ao
        min_time_done = self.seconds_since(self.avoid_started_time, now) >= self.avoid_min_time

        if not self.avoid_active:
            if obstacle_in_front:
                self.avoid_active = True
                self.avoid_started_time = now
                self.avoid_direction = self.choose_avoid_direction()
                self.get_logger().info(
                    f'AVOID start | dir={"left" if self.avoid_direction > 0 else "right"} | '
                    f'front={self.front_min:.2f} m'
                )
        else:
            if front_is_clear and min_time_done:
                self.avoid_active = False
                self.avoid_started_time = None
                self.get_logger().info('AVOID end | front clear')

    def apply_obstacle_avoidance(self, v_gtg, w_gtg, e_th):
        
        if not self.enable_obstacle_avoidance or not self.scan_ready or not self.avoid_active:
            return v_gtg, w_gtg

        if math.isfinite(self.front_min) and self.front_min <= self.d_safety:
            v_cmd = 0.0
            w_cmd = self.avoid_direction * self.escape_angular_speed
            return v_cmd, w_cmd

        v_cmd = min(self.avoid_linear_speed, self.v_max)

        w_goal_small = self.goal_blend_during_avoid * w_gtg
        w_avoid = self.avoid_direction * self.avoid_angular_speed
        w_cmd = w_avoid + w_goal_small

        if abs(e_th) > self.turn_in_place_threshold:
            v_cmd *= 0.50

        return v_cmd, w_cmd

    def control_loop(self):
        now = self.get_clock().now()
        self.last_control_stamp = now

        if not self.odom_ready:
            self.publish_stop()
            return

        if self.goal_reached:
            self.publish_stop()
            return

        # ========== ERROR CALCULATIONS ==========
        e_d = math.hypot(self.x_goal - self.x, self.y_goal - self.y)
        theta_goal = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
        e_th = self.normalize_angle(theta_goal - self.yaw)

        # ========== GOAL CHECK ==========
        if e_d <= self.e_d_tolerance:
            if self.goal_tolerance_since is None:
                self.goal_tolerance_since = now
            else:
                elapsed = self.seconds_since(self.goal_tolerance_since, now)
                if elapsed >= self.settle_time:
                    self.goal_reached = True
                    self.avoid_active = False
                    self.publish_stop()
                    self.get_logger().info(f'Goal reached! e_d={e_d:.4f}, e_th={e_th:.4f}')
                    return
        else:
            self.goal_tolerance_since = None

        # ========== GO-TO-GOAL CONTROLLER ==========
        v_gtg = self.k_v * e_d
        w_gtg = self.k_omega * e_th

        if abs(e_th) > self.turn_in_place_threshold:
            v_gtg = 0.0
        else:
            v_gtg *= max(0.0, math.cos(e_th))

        # ========== STATE MACHINE ==========
        self.update_avoidance_state(now)
        v_cmd, w_cmd = self.apply_obstacle_avoidance(v_gtg, w_gtg, e_th)

        # ========== VELOCITY LIMITS ==========
        v_cmd = self.clamp(v_cmd, 0.0, self.v_max)
        w_cmd = self.clamp(w_cmd, -self.omega_max, self.omega_max)

        too_close = self.avoid_active and math.isfinite(self.front_min) and self.front_min <= self.d_safety
        if not too_close and 0.0 < v_cmd < self.v_min and e_d > self.e_d_tolerance:
            v_cmd = self.v_min

        w_abs = abs(w_cmd)
        if 0.0 < w_abs < self.omega_min and abs(e_th) > self.e_theta_tolerance:
            w_cmd = math.copysign(self.omega_min, w_cmd)

        self.publish_cmd(v_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
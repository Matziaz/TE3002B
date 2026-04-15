#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        self.declare_parameter('right_wheel_encoder_topic', '/right_wheel_encoder')
        self.declare_parameter('left_wheel_encoder_topic', '/left_wheel_encoder')
        self.declare_parameter('robot_odometry_topic', '/robot_odometry')
        self.declare_parameter('robot_linear_velocity_topic', '/robot_linear_velocity')
        self.declare_parameter('robot_angular_velocity_topic', '/robot_angular_velocity')
        self.declare_parameter('velocity_estimate_topic', '/velocity_estimate')
        self.declare_parameter('velocity_command_topic', '/velocity_command')
        self.declare_parameter('navigation_goal_topic', '/navigation_goal')

        self.declare_parameter('odometry_update_rate', 50.0)
        self.declare_parameter('velocity_estimation_rate', 20.0)
        self.declare_parameter('control_loop_rate', 20.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('encoders_are_angular_speed', True)

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_heading', 0.0)

        self.declare_parameter('odometry_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('encoder_timeout', 0.5)

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('use_dynamic_goals', False)

        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('goal_settle_time', 0.5)

        self.declare_parameter('max_linear_velocity', 0.35)
        self.declare_parameter('max_angular_velocity', 1.2)
        self.declare_parameter('min_linear_velocity', 0.03)
        self.declare_parameter('min_angular_velocity', 0.05)

        self.declare_parameter('k_v', 0.8)
        self.declare_parameter('k_omega', 2.5)
        self.declare_parameter('rotation_threshold', 0.8)

        self.right_encoder_topic = str(self.get_parameter('right_wheel_encoder_topic').value)
        self.left_encoder_topic = str(self.get_parameter('left_wheel_encoder_topic').value)
        self.odometry_topic = str(self.get_parameter('robot_odometry_topic').value)
        self.linear_velocity_topic = str(self.get_parameter('robot_linear_velocity_topic').value)
        self.angular_velocity_topic = str(self.get_parameter('robot_angular_velocity_topic').value)
        self.velocity_estimate_topic = str(self.get_parameter('velocity_estimate_topic').value)
        self.velocity_command_topic = str(self.get_parameter('velocity_command_topic').value)
        self.goal_topic = str(self.get_parameter('navigation_goal_topic').value)

        self.odometry_rate = max(1.0, float(self.get_parameter('odometry_update_rate').value))
        self.velocity_rate = max(1.0, float(self.get_parameter('velocity_estimation_rate').value))
        self.control_rate = max(1.0, float(self.get_parameter('control_loop_rate').value))

        self.wheel_radius = max(1e-4, float(self.get_parameter('wheel_radius').value))
        self.wheel_base = max(1e-4, float(self.get_parameter('wheel_base').value))
        self.is_angular_encoder = bool(self.get_parameter('encoders_are_angular_speed').value)

        self.robot_x = float(self.get_parameter('initial_x').value)
        self.robot_y = float(self.get_parameter('initial_y').value)
        self.robot_heading = float(self.get_parameter('initial_heading').value)

        self.odom_frame = str(self.get_parameter('odometry_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.encoder_timeout = max(0.05, float(self.get_parameter('encoder_timeout').value))

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.use_dynamic_goals = bool(self.get_parameter('use_dynamic_goals').value)

        self.distance_tolerance = max(0.005, float(self.get_parameter('distance_tolerance').value))
        self.heading_tolerance = max(0.005, float(self.get_parameter('heading_tolerance').value))
        self.settle_time = max(0.0, float(self.get_parameter('goal_settle_time').value))

        self.max_linear_velocity = max(0.01, float(self.get_parameter('max_linear_velocity').value))
        self.max_angular_velocity = max(0.05, float(self.get_parameter('max_angular_velocity').value))
        self.min_linear_velocity = max(0.0, min(float(self.get_parameter('min_linear_velocity').value), self.max_linear_velocity))
        self.min_angular_velocity = max(0.0, min(float(self.get_parameter('min_angular_velocity').value), self.max_angular_velocity))

        self.linear_gain = float(self.get_parameter('k_v').value)
        self.angular_gain = float(self.get_parameter('k_omega').value)
        self.rotation_threshold = max(self.heading_tolerance, float(self.get_parameter('rotation_threshold').value))

        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity = 0.0
        self.right_encoder_timestamp = None
        self.left_encoder_timestamp = None

        self.estimated_linear_velocity = 0.0
        self.estimated_angular_velocity = 0.0

        self.odometry_initialized = False
        self.goal_reached = False
        self.goal_settled_timestamp = None

        self.last_control_time = self.get_clock().now()
        self.last_odometry_time = self.get_clock().now()
        self.last_velocity_time = self.get_clock().now()

        self.velocity_command_pub = self.create_publisher(Twist, self.velocity_command_topic, 10)
        self.odometry_pub = self.create_publisher(Odometry, self.odometry_topic, 10)
        self.linear_velocity_pub = self.create_publisher(Float32, self.linear_velocity_topic, 10)
        self.angular_velocity_pub = self.create_publisher(Float32, self.angular_velocity_topic, 10)
        self.velocity_estimate_pub = self.create_publisher(Twist, self.velocity_estimate_topic, 10)

        self.create_subscription(Float32, self.right_encoder_topic, self.on_right_encoder, 10)
        self.create_subscription(Float32, self.left_encoder_topic, self.on_left_encoder, 10)

        if self.use_dynamic_goals:
            self.create_subscription(PoseStamped, self.goal_topic, self.on_goal_update, 10)

        self.odometry_timer = self.create_timer(1.0 / self.odometry_rate, self.update_odometry)
        self.velocity_timer = self.create_timer(1.0 / self.velocity_rate, self.estimate_velocities)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(f'Differential Drive Controller initialized at ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_heading:.3f})')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def on_right_encoder(self, msg):
        self.right_wheel_velocity = float(msg.data)
        self.right_encoder_timestamp = self.get_clock().now()

    def on_left_encoder(self, msg):
        self.left_wheel_velocity = float(msg.data)
        self.left_encoder_timestamp = self.get_clock().now()

    def on_goal_update(self, msg):
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.goal_reached = False
        self.goal_settled_timestamp = None
        self.get_logger().info(f'Goal updated: ({self.goal_x:.3f}, {self.goal_y:.3f})')

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

        self.odometry_initialized = True

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

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_control_time.nanoseconds) / 1e9
        self.last_control_time = now

        if not self.odometry_initialized:
            self.publish_stop()
            return

        if self.goal_reached:
            self.publish_stop()
            return

        delta_x = self.goal_x - self.robot_x
        delta_y = self.goal_y - self.robot_y
        distance_error = math.sqrt(delta_x * delta_x + delta_y * delta_y)

        desired_heading = math.atan2(delta_y, delta_x)
        current_heading = self.robot_heading
        heading_error = self.normalize_angle(desired_heading - current_heading)

        if distance_error <= self.distance_tolerance:
            if self.goal_settled_timestamp is None:
                self.goal_settled_timestamp = now
            else:
                elapsed = (now.nanoseconds - self.goal_settled_timestamp.nanoseconds) / 1e9
                if elapsed >= self.settle_time:
                    self.goal_reached = True
                    self.publish_stop()
                    return
        else:
            self.goal_settled_timestamp = None

        normalized_distance = distance_error / max(self.distance_tolerance, 0.01)
        heading_magnitude = abs(heading_error)

        if heading_magnitude > self.rotation_threshold:
            linear_command = 0.0
            angular_command = self.angular_gain * heading_error
        else:
            decay_factor = math.exp(-normalized_distance * 0.5)
            linear_component = self.linear_gain * distance_error * math.cos(heading_error)
            linear_command = linear_component * decay_factor

            primary_angular = self.angular_gain * heading_error
            cross_track_gain = 1.5 * self.linear_gain / max(distance_error, 0.15)
            cross_track_correction = cross_track_gain * distance_error * math.sin(heading_error)
            heading_damping = 0.3 * self.angular_gain * heading_error * math.cos(heading_error)

            angular_command = primary_angular + cross_track_correction + heading_damping

        linear_command = self.clamp(linear_command, -self.max_linear_velocity, self.max_linear_velocity)
        angular_command = self.clamp(angular_command, -self.max_angular_velocity, self.max_angular_velocity)

        if abs(linear_command) < self.min_linear_velocity and linear_command != 0:
            linear_command = self.min_linear_velocity if linear_command > 0 else -self.min_linear_velocity

        if abs(angular_command) < self.min_angular_velocity and angular_command != 0:
            angular_command = self.min_angular_velocity if angular_command > 0 else -self.min_angular_velocity

        self.publish_velocity_command(linear_command, angular_command)

    def publish_velocity_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.velocity_command_pub.publish(msg)

    def publish_stop(self):
        self.publish_velocity_command(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MobileRoboticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

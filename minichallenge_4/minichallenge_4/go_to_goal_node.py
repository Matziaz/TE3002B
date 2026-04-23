#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String


class Controller(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')

        self.declare_parameter('goal_list', [[-1.2, -1.2]])
        self.declare_parameter('control_loop_rate', 0.05)
        self.declare_parameter('kv', 0.4)
        self.declare_parameter('kw', 0.4)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.3)
        self.declare_parameter('dist_threshold', 0.08)
        self.declare_parameter('angle_trigger', 0.3)
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')

        goal_list_raw = self.get_parameter('goal_list').value
        self.goal_list = [tuple(goal) for goal in goal_list_raw]
        control_loop_rate = float(self.get_parameter('control_loop_rate').value)
        self.kv = float(self.get_parameter('kv').value)
        self.kw = float(self.get_parameter('kw').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.dist_threshold = float(self.get_parameter('dist_threshold').value)
        self.angle_trigger = float(self.get_parameter('angle_trigger').value)
        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        velocity_scale_topic = self.get_parameter('velocity_scale_topic').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        self.current_goal = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity_scale = 1.0

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.distance_error_pub = self.create_publisher(Float32, 'distance_error', qos)
        self.heading_error_pub = self.create_publisher(Float32, 'heading_error', qos)
        self.current_waypoint_pub = self.create_publisher(Int32, 'current_waypoint', qos)
        self.nav_status_pub = self.create_publisher(String, 'nav_status', qos)

        self.create_subscription(Pose2D, pose_topic, self.pose_cb, qos)
        self.create_subscription(Float32, velocity_scale_topic, self.velocity_scale_cb, qos)
        self.create_timer(control_loop_rate, self.control_loop)

        self.get_logger().info(f'Go-to-goal initialized with goals: {self.goal_list}')

    def pose_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def velocity_scale_cb(self, msg):
        self.velocity_scale = float(msg.data)

    def control_loop(self):
        waypoint_msg = Int32()
        waypoint_msg.data = int(self.current_goal)
        self.current_waypoint_pub.publish(waypoint_msg)

        if self.current_goal >= len(self.goal_list):
            self.publish_vel(0.0, 0.0)
            status = String()
            status.data = 'mission_complete'
            self.nav_status_pub.publish(status)
            return

        gx, gy = self.goal_list[self.current_goal]
        dx, dy = gx - self.x, gy - self.y
        ed = float(np.hypot(dx, dy))
        target_angle = float(np.arctan2(dy, dx))
        e_theta = float(np.arctan2(np.sin(target_angle - self.theta), np.cos(target_angle - self.theta)))

        self._publish_metrics(ed, e_theta)

        if ed < self.dist_threshold:
            self.get_logger().info(f'Waypoint {self.current_goal + 1} reached.')
            self.current_goal += 1
            return

        if abs(e_theta) > self.angle_trigger:
            v = 0.0
            w = self.kw * e_theta
            nav_status = 'rotate_only'
        else:
            v = self.kv * ed
            w = self.kw * e_theta
            nav_status = 'advance_and_correct'

        scaled_v_limit = max(0.0, self.v_max * self.velocity_scale)
        scaled_w_limit = max(0.0, self.w_max * self.velocity_scale)
        v_scaled = float(np.clip(v * self.velocity_scale, 0.0, scaled_v_limit))
        w_scaled = float(np.clip(w * self.velocity_scale, -scaled_w_limit, scaled_w_limit))

        self.publish_vel(v_scaled, w_scaled)
        status = String()
        status.data = nav_status
        self.nav_status_pub.publish(status)

    def _publish_metrics(self, distance_error, heading_error):
        d_msg = Float32()
        d_msg.data = distance_error
        self.distance_error_pub.publish(d_msg)

        h_msg = Float32()
        h_msg.data = heading_error
        self.heading_error_pub.publish(h_msg)

    def publish_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

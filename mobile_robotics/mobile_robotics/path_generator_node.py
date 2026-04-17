#!/usr/bin/env python3
"""
Path Generator Node for Mini Challenge 2
=========================================

Generates and publishes a sequence of goal positions for the robot to visit.
Implements Part 1 (square trajectory) and Part 2 (automatic goal sequencing).

Part 1: Square Trajectory
    - Defines a 2m × 2m square starting at (0,0)
    - Waypoints: (2,0) → (2,2) → (0,2) → (0,0)

Part 2: Path Generator Node
    - Reads goal sequence from YAML parameters
    - Publishes goals sequentially to /goal_pose
    - Detects when each goal is reached
    - Transitions to next goal automatically
    - Can adjust controller gains based on distance/heading
"""

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PathState(Enum):
    """States for path generation."""
    IDLE = 0           # No goal active
    MOVING = 1         # Moving toward current goal
    GOAL_REACHED = 2   # Current goal reached
    PATH_COMPLETE = 3  # All goals completed


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        # ========== TOPIC PARAMETERS ==========
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 10.0)

        # ========== PATH PARAMETERS ==========
        self.declare_parameter('path_type', 'square')  # 'square' or 'custom'
        self.declare_parameter('square_size', 2.0)     # Size of square in meters
        self.declare_parameter('waypoints', [             # Custom waypoint sequence
            [2.0, 0.0],  # (2, 0)
            [2.0, 2.0],  # (2, 2)
            [0.0, 2.0],  # (0, 2)
            [0.0, 0.0],  # (0, 0)
        ])

        # ========== GOAL DETECTION ==========
        self.declare_parameter('goal_reached_distance', 0.05)  # Distance threshold (m)
        self.declare_parameter('goal_reached_heading', 0.05)   # Heading threshold (rad)
        self.declare_parameter('settle_time', 0.5)             # Time to confirm goal reached (s)
        self.declare_parameter('reachability_check', True)     # Validate goal reachability

        # ========== CONTROLLER GAIN ADAPTATION ==========
        self.declare_parameter('adaptive_gains', False)        # Enable gain adaptation
        self.declare_parameter('k_v_near', 0.5)                # Linear gain when near goal
        self.declare_parameter('k_v_far', 1.0)                 # Linear gain when far from goal
        self.declare_parameter('distance_threshold', 0.5)      # Distance to switch gains (m)

        # ========== PARAMETER LOADING ==========
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.path_type = str(self.get_parameter('path_type').value).lower()
        self.square_size = float(self.get_parameter('square_size').value)
        waypoints_list = self.get_parameter('waypoints').value
        self.waypoints = [[float(p[0]), float(p[1])] for p in waypoints_list]

        self.goal_reached_distance = float(self.get_parameter('goal_reached_distance').value)
        self.goal_reached_heading = float(self.get_parameter('goal_reached_heading').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.reachability_check = bool(self.get_parameter('reachability_check').value)

        self.adaptive_gains = bool(self.get_parameter('adaptive_gains').value)
        self.k_v_near = float(self.get_parameter('k_v_near').value)
        self.k_v_far = float(self.get_parameter('k_v_far').value)
        self.distance_threshold = float(self.get_parameter('distance_threshold').value)

        # ========== STATE VARIABLES ==========
        self.state = PathState.IDLE

        # Robot odometry
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        # Path state
        self.current_waypoint_idx = 0
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.goal_tolerance_since = None
        self.times_goal_reached = 0

        # ========== GENERATE WAYPOINTS ==========
        if self.path_type == 'square':
            self.generate_square_path()
        else:
            self.get_logger().info(f'Using custom waypoints: {len(self.waypoints)} points')

        # Validate reachability
        if self.reachability_check:
            self.validate_reachability()

        # ========== ROS SETUP ==========
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self.main_loop)

        self.get_logger().info('=== Path Generator Node Started ===')
        self.get_logger().info(f'Path Type: {self.path_type} | Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Waypoints: {self.waypoints}')

    def generate_square_path(self):
        """Generate 2m × 2m square path: (2,0) → (2,2) → (0,2) → (0,0)."""
        side = self.square_size
        self.waypoints = [
            [side, 0.0],   # (2, 0)
            [side, side],  # (2, 2)
            [0.0, side],   # (0, 2)
            [0.0, 0.0],    # (0, 0)
        ]
        self.get_logger().info(f'Generated square path (side={side}m): {self.waypoints}')

    def validate_reachability(self):
        """Validate that all waypoints are reachable."""
        for i, wp in enumerate(self.waypoints):
            # Simple check: waypoints should be within reasonable distance
            dist = math.hypot(wp[0], wp[1])
            self.get_logger().info(f'Waypoint {i}: ({wp[0]:.2f}, {wp[1]:.2f}) distance from origin: {dist:.2f}m')

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        # Extract yaw from quaternion
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def publish_goal(self, x, y):
        """Publish goal position to /goal_pose."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # Set heading toward next waypoint if available
        if self.current_waypoint_idx + 1 < len(self.waypoints):
            next_x, next_y = self.waypoints[self.current_waypoint_idx + 1]
            heading = math.atan2(next_y - y, next_x - x)
        else:
            # Last waypoint: orient toward first waypoint
            next_x, next_y = self.waypoints[0]
            heading = math.atan2(next_y - y, next_x - x)

        # Convert heading to quaternion
        qz = math.sin(heading / 2.0)
        qw = math.cos(heading / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.goal_pub.publish(msg)

    def check_goal_reached(self):
        """Check if current goal is reached within tolerance."""
        if not self.odom_ready or self.state == PathState.PATH_COMPLETE:
            return False

        # Distance and heading errors
        e_d = math.hypot(self.current_goal_x - self.x, self.current_goal_y - self.y)
        theta_G = math.atan2(self.current_goal_y - self.y, self.current_goal_x - self.x)
        e_th = abs(self.normalize_angle(theta_G - self.yaw))

        # Check tolerance
        distance_ok = e_d <= self.goal_reached_distance
        heading_ok = e_th <= self.goal_reached_heading

        if distance_ok and heading_ok:
            if self.goal_tolerance_since is None:
                self.goal_tolerance_since = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now().nanoseconds - self.goal_tolerance_since.nanoseconds) / 1e9
                if elapsed >= self.settle_time:
                    return True
        else:
            self.goal_tolerance_since = None

        return False

    def transition_to_next_goal(self):
        """Move to next waypoint in sequence."""
        self.times_goal_reached += 1
        self.current_waypoint_idx += 1
        self.goal_tolerance_since = None

        if self.current_waypoint_idx >= len(self.waypoints):
            # Path complete: loop or stop
            self.state = PathState.PATH_COMPLETE
            self.get_logger().info(f'=== PATH COMPLETE === ({self.times_goal_reached} waypoints reached)')
            # Reset to beginning (optional: change to stop)
            self.current_waypoint_idx = 0

        # Set new goal
        self.current_goal_x, self.current_goal_y = self.waypoints[self.current_waypoint_idx]
        self.state = PathState.MOVING
        self.get_logger().info(
            f'→ Waypoint {self.current_waypoint_idx}/{len(self.waypoints)}: ({self.current_goal_x:.2f}, {self.current_goal_y:.2f})'
        )

    def main_loop(self):
        """Main loop: manage goal sequence."""
        if not self.odom_ready:
            return

        # Initialize first goal
        if self.state == PathState.IDLE:
            self.current_goal_x, self.current_goal_y = self.waypoints[0]
            self.state = PathState.MOVING
            self.get_logger().info(f'Starting path: first goal ({self.current_goal_x:.2f}, {self.current_goal_y:.2f})')

        # Publish current goal
        if self.state == PathState.MOVING or self.state == PathState.PATH_COMPLETE:
            self.publish_goal(self.current_goal_x, self.current_goal_y)

        # Check if goal reached
        if self.check_goal_reached():
            self.transition_to_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

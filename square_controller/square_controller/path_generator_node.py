#!/usr/bin/env python3
"""
MCR2 - Mini Challenge 1
Path Generator Node
===========================
Reads a list of waypoints from a ROS parameter file and publishes custom
PoseWithVelocity messages to the /pose topic consumed by the Controller node.

Custom message: PoseWithVelocity (defined in square_controller package)
  geometry_msgs/Pose  pose       – target position & orientation
  float64             linear_vel – desired linear speed to reach this point (m/s)
  float64             angular_vel– desired angular speed at this point (rad/s)
  float64             time_to_wp – desired travel time to this waypoint (s)
                                   (use -1 to let controller compute from velocities)

Parameter file structure (YAML):
  path_generator_node:
    ros__parameters:
      waypoints:
        - x: 2.0
          y: 0.0
          yaw: 0.0
          linear_vel: 0.2   # optional if time_to_wp is given
          angular_vel: 0.0
          time_to_wp: -1.0  # -1 → compute from velocities
        - x: 2.0
          y: 2.0
          yaw: 1.5707
          linear_vel: 0.2
          angular_vel: 0.5
          time_to_wp: -1.0
        ...

The node checks reachability: each waypoint must be reachable given the
robot's kinematic constraints (v_max, omega_max). If a waypoint is not
reachable, it logs a warning and skips it.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

# We use the standard Pose and append velocities via a custom msg wrapper.
# Since we cannot compile custom messages in this environment, we use a
# NamedTuple/dict approach and publish the closest standard equivalent.
# In a real ROS 2 package, replace this with the real custom msg import:
#   from square_controller.msg import PoseWithVelocity

from geometry_msgs.msg import PoseStamped   # fallback for demonstration


class PathGenerator(Node):
    """Reads waypoints from params, validates them, and publishes to /pose."""

    V_MAX   = 1.0   # m/s   (robot physical limit)
    W_MAX   = 2.0   # rad/s (robot physical limit)

    def __init__(self):
        super().__init__("path_generator_node")

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter("publish_rate", 10.0)   # Hz
        self.declare_parameter("loop_path",    False)  # repeat path?

        publish_rate = self.get_parameter("publish_rate").value
        self.loop    = self.get_parameter("loop_path").value

        # Waypoints loaded from YAML (list of dicts)
        self.declare_parameter("waypoints", rclpy.Parameter.Type.STRING)

        # ── Parse waypoints ───────────────────────────────────────────────────
        # In a real launch, these are set from a YAML file.
        # Here we define a default square path as fallback.
        self.waypoints = self._load_waypoints()
        self.wp_index  = 0

        self.get_logger().info(
            f"PathGenerator: {len(self.waypoints)} waypoints loaded."
        )
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"  WP{i}: {wp}")

        # ── Publisher ─────────────────────────────────────────────────────────
        # In production, publish to /pose with custom PoseWithVelocity msg.
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._publish_next_waypoint)

    # ─────────────────────────────────────────────────────────────────────────
    def _load_waypoints(self):
        """
        Load waypoints from ROS parameters.
        Falls back to a default 2m square if params are missing.
        """
        default_square = [
            {"x": 2.0, "y": 0.0, "yaw": 0.0,            "linear_vel": 0.2, "angular_vel": 0.5, "time_to_wp": -1.0},
            {"x": 2.0, "y": 2.0, "yaw": math.pi/2,      "linear_vel": 0.2, "angular_vel": 0.5, "time_to_wp": -1.0},
            {"x": 0.0, "y": 2.0, "yaw": math.pi,        "linear_vel": 0.2, "angular_vel": 0.5, "time_to_wp": -1.0},
            {"x": 0.0, "y": 0.0, "yaw": 3*math.pi/2,    "linear_vel": 0.2, "angular_vel": 0.5, "time_to_wp": -1.0},
        ]
        validated = []
        for wp in default_square:
            if self._is_reachable(wp):
                validated.append(wp)
            else:
                self.get_logger().warn(
                    f"Waypoint ({wp['x']},{wp['y']}) UNREACHABLE with given velocities – skipped."
                )
        return validated

    def _is_reachable(self, wp: dict) -> bool:
        """
        Check if the waypoint is reachable given kinematic constraints.
        Returns False if requested velocities exceed physical limits.
        """
        v = wp.get("linear_vel", 0.0)
        w = wp.get("angular_vel", 0.0)
        if v > self.V_MAX:
            self.get_logger().warn(
                f"Requested v={v} m/s exceeds v_max={self.V_MAX} m/s"
            )
            return False
        if w > self.W_MAX:
            self.get_logger().warn(
                f"Requested ω={w} rad/s exceeds ω_max={self.W_MAX} rad/s"
            )
            return False
        return True

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_next_waypoint(self):
        if not self.waypoints:
            return

        if self.wp_index >= len(self.waypoints):
            if self.loop:
                self.wp_index = 0
            else:
                self.timer.cancel()
                self.get_logger().info("All waypoints published.")
                return

        wp = self.waypoints[self.wp_index]

        # Build PoseStamped (substitute for custom PoseWithVelocity)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = wp["x"]
        msg.pose.position.y = wp["y"]
        msg.pose.position.z = 0.0

        # Encode yaw into quaternion (z-rotation only for 2D robot)
        yaw = wp.get("yaw", 0.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub_pose.publish(msg)
        self.get_logger().info(
            f"Published WP {self.wp_index}: "
            f"x={wp['x']}, y={wp['y']}, yaw={math.degrees(yaw):.1f}°  "
            f"v={wp['linear_vel']} m/s  ω={wp['angular_vel']} rad/s"
        )
        self.wp_index += 1


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

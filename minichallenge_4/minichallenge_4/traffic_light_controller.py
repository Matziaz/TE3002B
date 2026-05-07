#!/usr/bin/env python3
"""Traffic light controller: integrates color detection with velocity scaling."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32


class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller')

        # Velocity scales for each color state
        self.declare_parameter('red_velocity_scale', 0.0)
        self.declare_parameter('yellow_velocity_scale', 0.3)
        self.declare_parameter('green_velocity_scale', 1.0)
        self.declare_parameter('unknown_velocity_scale', 0.0)
        self.declare_parameter('color_topic', 'detected_color')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('state_history_length', 3)

        self.red_scale = float(self.get_parameter('red_velocity_scale').value)
        self.yellow_scale = float(self.get_parameter('yellow_velocity_scale').value)
        self.green_scale = float(self.get_parameter('green_velocity_scale').value)
        self.unknown_scale = float(self.get_parameter('unknown_velocity_scale').value)
        color_topic = self.get_parameter('color_topic').value
        velocity_scale_topic = self.get_parameter('velocity_scale_topic').value
        self.history_length = int(self.get_parameter('state_history_length').value)

        self.color_scale_map = {
            'red': self.red_scale,
            'yellow': self.yellow_scale,
            'green': self.green_scale,
            'unknown': self.unknown_scale,
        }

        self.state_history = []
        self.current_velocity_scale = 1.0
        self.last_valid_color = 'green'
        self.red_locked = False  # State variable: True when red is detected, locked until green

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.velocity_scale_pub = self.create_publisher(Float32, velocity_scale_topic, qos)
        self.create_subscription(String, color_topic, self.color_callback, qos)

        self.get_logger().info(
            f'Traffic light controller initialized: '
            f'Red={self.red_scale}, Yellow={self.yellow_scale}, Green={self.green_scale}'
        )

    def color_callback(self, msg):
        detected_color = msg.data

        self.state_history.append(detected_color)
        if len(self.state_history) > self.history_length:
            self.state_history.pop(0)

        smoothed_color = self._smooth_state(self.state_history)

        # Latched state logic for red light
        if smoothed_color == 'red':
            # Red detected: lock the robot
            self.red_locked = True
            velocity_scale = 0.0
            self.get_logger().info('RED LIGHT DETECTED - Robot locked!')
        elif self.red_locked:
            # Robot is locked: only green can unlock it
            if smoothed_color == 'green':
                # Green signal detected while locked: unlock and resume movement
                self.red_locked = False
                velocity_scale = self.green_scale
                self.get_logger().info('GREEN LIGHT DETECTED - Robot unlocked!')
            else:
                # Maintain stopped state: anything other than green keeps it locked
                velocity_scale = 0.0
        else:
            # Normal operation: use smoothed color velocity scale
            velocity_scale = self.color_scale_map.get(smoothed_color, self.unknown_scale)

        if smoothed_color != 'unknown':
            self.last_valid_color = smoothed_color

        self.current_velocity_scale = velocity_scale

        scale_msg = Float32()
        scale_msg.data = float(velocity_scale)
        self.velocity_scale_pub.publish(scale_msg)

        self.get_logger().debug(
            f'Color: {detected_color} -> Smoothed: {smoothed_color} -> Locked: {self.red_locked} -> Scale: {velocity_scale:.2f}'
        )

    def _smooth_state(self, history):
        if not history:
            return self.last_valid_color
        valid = [color for color in history if color != 'unknown']
        if not valid:
            return self.last_valid_color
        counts = {color: valid.count(color) for color in ('red', 'yellow', 'green')}
        winner = max(counts, key=counts.get)
        return winner if counts[winner] >= max(1, len(valid) // 2 + 1) else self.last_valid_color

    def _is_clear_green(self, history):
        """Check if we have a clear, unambiguous green signal (no red in history)."""
        if not history:
            return False
        # Filter out unknowns
        valid = [color for color in history if color != 'unknown']
        if not valid:
            return False
        # Clear green: all valid colors must be green (no red or yellow)
        return all(color == 'green' for color in valid)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

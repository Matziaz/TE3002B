#!/usr/bin/env python3
"""Traffic light controller with red latch and confidence-aware transitions."""

from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray, String


class TrafficState(Enum):
    STOPPED = 'stopped'
    SLOW = 'slow'
    MOVING = 'moving'
    UNKNOWN = 'unknown'


@dataclass
class StateTransitionLog:
    timestamp: float
    from_state: str
    to_state: str
    reason: str
    color_detected: str


class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller')

        self.declare_parameter('signal_loss_timeout', 2.0)
        self.declare_parameter('slow_velocity_factor', 0.3)
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('transition_frames', 3)
        self.declare_parameter('enable_logging', True)

        self.signal_loss_timeout = float(self.get_parameter('signal_loss_timeout').value)
        self.slow_velocity_factor = float(self.get_parameter('slow_velocity_factor').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.transition_frames = int(self.get_parameter('transition_frames').value)
        self.enable_logging = bool(self.get_parameter('enable_logging').value)

        self.current_state = TrafficState.UNKNOWN
        self.latest_detected_color = 'unknown'
        self.latest_confidences = [0.0, 0.0, 0.0]
        self.last_valid_detection_time = self.get_clock().now()
        self.signal_loss_detected = True
        self.red_latched = False

        self.transition_counter = 0
        self.pending_state = None
        self.velocity_scale = 0.0

        self.red_detection_count = 0
        self.transition_log = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.state_pub = self.create_publisher(String, 'traffic_state', qos)
        self.velocity_scale_pub = self.create_publisher(Float32, 'velocity_scale', qos)
        self.controller_status_pub = self.create_publisher(String, 'traffic_controller_status', qos)

        self.create_subscription(String, 'detected_color', self.color_callback, qos)
        self.create_subscription(Float32MultiArray, 'color_confidence', self.confidence_callback, qos)

        self.create_timer(0.1, self.state_machine_update)
        self.create_timer(1.0, self.log_status)

        self.get_logger().info(
            'Traffic light controller initialized. '
            f'signal_loss_timeout={self.signal_loss_timeout}s, '
            f'confidence_threshold={self.confidence_threshold}, '
            f'transition_frames={self.transition_frames}'
        )

    def color_callback(self, msg):
        self.latest_detected_color = msg.data

    def confidence_callback(self, msg):
        if len(msg.data) >= 3:
            self.latest_confidences = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

    def _effective_color(self):
        red_conf, yellow_conf, green_conf = self.latest_confidences
        confidence_map = {'red': red_conf, 'yellow': yellow_conf, 'green': green_conf}
        confidence_color = max(confidence_map, key=confidence_map.get)
        max_conf = confidence_map[confidence_color]

        if self.latest_detected_color == confidence_color and max_conf >= self.confidence_threshold:
            self.last_valid_detection_time = self.get_clock().now()
            self.signal_loss_detected = False
            return self.latest_detected_color

        dt = (self.get_clock().now() - self.last_valid_detection_time).nanoseconds / 1e9
        self.signal_loss_detected = dt > self.signal_loss_timeout
        return 'unknown' if self.signal_loss_detected else self.latest_detected_color

    def state_machine_update(self):
        effective_color = self._effective_color()

        if effective_color == 'red':
            self.red_detection_count += 1
            self.red_latched = True

        if self.red_latched:
            if effective_color == 'green':
                self._process_state_transition(TrafficState.MOVING, 'Green detected after latched red')
            else:
                self._force_state(TrafficState.STOPPED, 'Red latch active; waiting for green')
        else:
            if effective_color == 'red':
                self._process_state_transition(TrafficState.STOPPED, 'Red light detected')
            elif effective_color == 'yellow':
                self._process_state_transition(TrafficState.SLOW, 'Yellow light detected')
            elif effective_color == 'green':
                self._process_state_transition(TrafficState.MOVING, 'Green light detected')
            else:
                self._process_state_transition(TrafficState.UNKNOWN, 'Signal lost or low confidence')

        self._publish_state()
        self._publish_velocity_scale(self.velocity_scale)

    def _process_state_transition(self, target_state, reason):
        if target_state == self.current_state:
            self.transition_counter = 0
            self.pending_state = None
            return

        if target_state != self.pending_state:
            self.pending_state = target_state
            self.transition_counter = 1
            return

        self.transition_counter += 1
        if self.transition_counter >= self.transition_frames:
            self._force_state(target_state, reason)
            self.transition_counter = 0
            self.pending_state = None

    def _force_state(self, new_state, reason):
        if new_state == self.current_state and reason != 'Red latch active; waiting for green':
            return

        old_state = self.current_state
        self.current_state = new_state

        if new_state == TrafficState.STOPPED:
            self.velocity_scale = 0.0
        elif new_state == TrafficState.SLOW:
            self.velocity_scale = self.slow_velocity_factor
        elif new_state == TrafficState.MOVING:
            self.velocity_scale = 1.0
            if self.red_latched:
                self.red_latched = False
        else:
            self.velocity_scale = 0.0

        if old_state != new_state:
            self.transition_log.append(
                StateTransitionLog(
                    timestamp=self.get_clock().now().nanoseconds / 1e9,
                    from_state=old_state.value,
                    to_state=new_state.value,
                    reason=reason,
                    color_detected=self.latest_detected_color,
                )
            )
            self.get_logger().info(
                f'State transition: {old_state.value} -> {new_state.value} | {reason} | scale={self.velocity_scale:.2f}'
            )

    def _publish_state(self):
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)

    def _publish_velocity_scale(self, scale):
        msg = Float32()
        msg.data = float(scale)
        self.velocity_scale_pub.publish(msg)

    def log_status(self):
        if not self.enable_logging:
            return
        status = String()
        status.data = (
            f'state={self.current_state.value}, scale={self.velocity_scale:.2f}, '
            f'color={self.latest_detected_color}, conf={self.latest_confidences}, '
            f'signal_loss={self.signal_loss_detected}, red_latched={self.red_latched}, '
            f'transitions={len(self.transition_log)}'
        )
        self.controller_status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

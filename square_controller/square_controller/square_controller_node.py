#!/usr/bin/env python3
"""
MCR2 - Mini Challenge 1
Square Path Controller Node (Open Loop)
========================================
Drives a differential-drive robot (real or Gazebo) in a square of side_length 2 m.

Architecture:
  - Finite State Machine: Stop → Straight → Turn → (repeat N_sides times) → Stop
  - Auto-tuned: user provides EITHER the desired linear speed OR the total time to
    complete the square; the node computes the other quantities automatically.
  - Robustness strategies:
      1. Time-based transitions (no sensor drift).
      2. Velocity ramping (smooth accel/decel) to reduce wheel-slip overshoot.
      3. Configurable safety margins (ramp_fraction) to absorb nonlinearities.
      4. Watchdog: stops the robot if ROS shuts down mid-motion.

Parameters (set via ROS parameter file or CLI):
  side_length   [float, m]    – side length of the square  (default 2.0)
  linear_speed  [float, m/s]  – desired translational speed (auto if <= 0)
  angular_speed [float, rad/s]– desired rotational speed    (auto if <= 0)
  total_time    [float, s]    – total time to complete square (used when speed<=0)
  ramp_fraction [float, 0-1]  – fraction of segment spent ramping up/down (default 0.2)
  cmd_vel_topic [string]      – topic to publish velocity commands (default /cmd_vel)
  control_rate  [float, Hz]   – control loop frequency (default 50)

Usage examples:
  # Fixed speed
  ros2 run square_controller square_controller_node \
      --ros-args -p linear_speed:=0.2 -p angular_speed:=0.5

  # Fixed total time (auto-tune speeds)
  ros2 run square_controller square_controller_node \
      --ros-args -p total_time:=40.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


# ── Finite State Machine states ───────────────────────────────────────────────
class State:
    STOP     = "STOP"
    STRAIGHT = "STRAIGHT"
    TURN     = "TURN"
    DONE     = "DONE"


class SquareController(Node):
    """Open-loop FSM controller to drive a robot in a square path."""

    N_SIDES = 4  # A square has 4 straight segments and 4 turns

    def __init__(self):
        super().__init__("square_controller")

        # ── Declare & read parameters ─────────────────────────────────────────
        self.declare_parameter("side_length",   2.0)
        self.declare_parameter("linear_speed",  0.2)    # m/s  (<=0 → auto-tune)
        self.declare_parameter("angular_speed", 0.5)    # rad/s(<=0 → auto-tune)
        self.declare_parameter("total_time",    40.0)   # s (used when speed<=0)
        self.declare_parameter("ramp_fraction", 0.2)    # fraction for ramping
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_rate",  50.0)   # Hz

        side_length   = self.get_parameter("side_length").value
        linear_speed  = self.get_parameter("linear_speed").value
        angular_speed = self.get_parameter("angular_speed").value
        total_time    = self.get_parameter("total_time").value
        self.ramp_frac = float(self.get_parameter("ramp_fraction").value)
        cmd_topic     = self.get_parameter("cmd_vel_topic").value
        rate_hz       = self.get_parameter("control_rate").value

        # ── Auto-tune velocities ──────────────────────────────────────────────
        # Square perimeter = 4 * side_length; 4 quarter-turns of 90° each.
        # We split total_time equally between straight and turn phases.
        #   t_straight = side_length / v_linear
        #   t_turn     = (π/2)      / v_angular
        # Auto-tune: if speed not specified, derive from total_time:
        #   total_time ≈ 4*t_straight + 4*t_turn
        #              = 4*(L/v + π/(2*ω))
        # We assume v and ω are linked by the robot's kinematic ratio or we use
        # a default ratio v/ω = side_length/(π/2).

        if linear_speed <= 0 or angular_speed <= 0:
            # Distribute total_time: half to straights, half to turns
            t_straight = total_time / (2 * self.N_SIDES)
            t_turn     = total_time / (2 * self.N_SIDES)
            linear_speed  = side_length / t_straight
            angular_speed = (math.pi / 2.0) / t_turn
            self.get_logger().info(
                f"[Auto-tune] total_time={total_time:.1f}s → "
                f"v={linear_speed:.3f} m/s, ω={angular_speed:.3f} rad/s"
            )

        self.v_lin = linear_speed
        self.v_ang = angular_speed

        # Segment durations
        self.t_straight = side_length / self.v_lin        # seconds per side
        self.t_turn     = (math.pi / 2.0) / self.v_ang   # seconds per 90° turn

        self.get_logger().info(
            f"Square controller ready | side={side_length} m | "
            f"v={self.v_lin:.3f} m/s | ω={self.v_ang:.3f} rad/s | "
            f"t_straight={self.t_straight:.2f}s | t_turn={self.t_turn:.2f}s"
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        # ── FSM initialisation ────────────────────────────────────────────────
        self.state        = State.STOP
        self.turn_count   = 0
        self.segment_start = None   # wall-clock time when current segment began

        # ── Control timer ─────────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.control_loop)

        # Allow one cycle for simulation/hardware to warm up
        self._ready_after = time.time() + 1.0

    # ─────────────────────────────────────────────────────────────────────────
    # Velocity ramping helper
    # ─────────────────────────────────────────────────────────────────────────
    def _ramp(self, elapsed: float, duration: float, cmd_vel: float) -> float:
        """
        Apply a trapezoidal velocity profile.
        The first and last `ramp_frac` of the segment are used for accel/decel.
        This reduces overshoot caused by wheel inertia and ground friction.
        """
        ramp_time = self.ramp_frac * duration
        if ramp_time < 1e-6:
            return cmd_vel
        if elapsed < ramp_time:
            scale = elapsed / ramp_time
        elif elapsed > duration - ramp_time:
            scale = max(0.0, (duration - elapsed) / ramp_time)
        else:
            scale = 1.0
        return cmd_vel * scale

    # ─────────────────────────────────────────────────────────────────────────
    # FSM control loop (called at `control_rate` Hz)
    # ─────────────────────────────────────────────────────────────────────────
    def control_loop(self):
        now = time.time()
        msg = Twist()  # default: all zeros → stop

        # ── STOP (initial / final) ────────────────────────────────────────────
        if self.state == State.STOP:
            if now < self._ready_after:
                # Wait for system to be ready
                self.pub_cmd.publish(msg)
                return
            # Check variables are set before transitioning
            if self.v_lin > 0 and self.v_ang > 0:
                self.get_logger().info("FSM → STRAIGHT (side 1)")
                self.state         = State.STRAIGHT
                self.segment_start = now
                self.turn_count    = 0
            else:
                self.get_logger().warn("Variables not ready, staying in STOP")
                self.pub_cmd.publish(msg)
                return

        # ── STRAIGHT ──────────────────────────────────────────────────────────
        if self.state == State.STRAIGHT:
            elapsed = now - self.segment_start
            if elapsed >= self.t_straight:
                # Transition → TURN
                self.get_logger().info(
                    f"FSM → TURN (turn {self.turn_count + 1}/{self.N_SIDES})"
                )
                self.state         = State.TURN
                self.segment_start = now
            else:
                v = self._ramp(elapsed, self.t_straight, self.v_lin)
                msg.linear.x = v
                msg.angular.z = 0.0

        # ── TURN ──────────────────────────────────────────────────────────────
        if self.state == State.TURN:
            elapsed = now - self.segment_start
            if elapsed >= self.t_turn:
                self.turn_count += 1
                if self.turn_count >= self.N_SIDES:
                    # Square complete
                    self.get_logger().info("Square complete! FSM → DONE")
                    self.state = State.DONE
                else:
                    self.get_logger().info(
                        f"FSM → STRAIGHT (side {self.turn_count + 1})"
                    )
                    self.state         = State.STRAIGHT
                    self.segment_start = now
            else:
                v = self._ramp(elapsed, self.t_turn, self.v_ang)
                msg.linear.x  = 0.0
                msg.angular.z = v

        # ── DONE ──────────────────────────────────────────────────────────────
        if self.state == State.DONE:
            # Publish zero velocity once, then cancel timer
            self.pub_cmd.publish(msg)
            self.timer.cancel()
            self.get_logger().info("Controller stopped. Robot should be at origin.")
            return

        self.pub_cmd.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        """Watchdog: ensure robot stops on shutdown."""
        self.get_logger().info("Shutting down — sending stop command.")
        stop = Twist()
        self.pub_cmd.publish(stop)
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

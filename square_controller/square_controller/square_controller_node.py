import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareController(Node):

    def __init__(self):
        super().__init__('square_controller')

        # =========================
        # Parámetros ROS2
        # =========================
        self.declare_parameter('mode', 'speed')  # 'speed' o 'time'
        self.declare_parameter('side_length', 2.0)

        # Modo speed
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.35)

        # Modo time
        self.declare_parameter('total_time', 80.0)
        self.declare_parameter('turn_time_ratio', 0.30)  # % del tiempo para giros

        # Robustez / límites
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('min_angular_speed', 0.10)

        # Pausas
        self.declare_parameter('initial_stop_time', 2.0)
        self.declare_parameter('point_stop_time', 2.0)
        self.declare_parameter('final_stop_time', 2.0)

        # Perfil suave de velocidad
        self.declare_parameter('ramp_time', 0.25)

        # ROS
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate', 20.0)

        # Giro
        self.declare_parameter('turn_direction', 1.0)  # +1 izquierda, -1 derecha

        # =========================
        # Leer parámetros
        # =========================
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.side_length = float(self.get_parameter('side_length').value)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.total_time = float(self.get_parameter('total_time').value)
        self.turn_time_ratio = float(self.get_parameter('turn_time_ratio').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        self.initial_stop_time = float(self.get_parameter('initial_stop_time').value)
        self.point_stop_time = float(self.get_parameter('point_stop_time').value)
        self.final_stop_time = float(self.get_parameter('final_stop_time').value)
        self.ramp_time = float(self.get_parameter('ramp_time').value)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.turn_direction = float(self.get_parameter('turn_direction').value)

        # =========================
        # Validaciones básicas
        # =========================
        if self.mode not in ['speed', 'time']:
            self.get_logger().warn(f"Invalid mode '{self.mode}'. Using 'speed'.")
            self.mode = 'speed'

        if self.side_length <= 0.0:
            self.get_logger().warn("side_length <= 0. Using default 2.0")
            self.side_length = 2.0

        if self.control_rate <= 0.0:
            self.get_logger().warn("control_rate <= 0. Using default 20.0")
            self.control_rate = 20.0

        if self.ramp_time < 0.0:
            self.get_logger().warn("ramp_time < 0. Using 0.0")
            self.ramp_time = 0.0

        if self.turn_direction >= 0.0:
            self.turn_direction = 1.0
        else:
            self.turn_direction = -1.0

        # =========================
        # Auto-tuning
        # =========================
        self.compute_motion_parameters()

        # =========================
        # Variables FSM
        # =========================
        self.state = "initial_stop"
        self.segment_count = 0
        self.current_point_index = 1  # arranca en p1, primer objetivo es p2
        self.start_time = self.get_clock().now()
        self.current_phase_duration = 0.0

        # =========================
        # ROS interfaces
        # =========================
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vel = Twist()

        # =========================
        # Logs de configuración
        # =========================
        self.get_logger().info("Node initialized!!!")
        self.get_logger().info(f"Mode: {self.mode}")
        self.get_logger().info(f"side_length = {self.side_length:.3f} m")
        self.get_logger().info(f"linear_speed = {self.linear_speed:.3f} m/s")
        self.get_logger().info(f"angular_speed = {self.angular_speed:.3f} rad/s")
        self.get_logger().info(f"forward_time = {self.forward_time:.3f} s")
        self.get_logger().info(f"turn_time = {self.turn_time:.3f} s")
        self.get_logger().info(
            f"Pauses: initial={self.initial_stop_time:.2f}s, "
            f"point={self.point_stop_time:.2f}s, final={self.final_stop_time:.2f}s"
        )
        self.get_logger().info(f"ramp_time = {self.ramp_time:.2f} s")
        self.get_logger().info(
            f"Limits: vmax={self.max_linear_speed:.3f} m/s, "
            f"wmax={self.max_angular_speed:.3f} rad/s"
        )
        self.get_logger().info(
            f"cmd_vel_topic={self.cmd_vel_topic}, control_rate={self.control_rate:.1f} Hz"
        )
        self.get_logger().info("Robot must start at p1 facing toward p2.")

    def ramp_factor(self):
        if self.ramp_time <= 1e-6:
            return 1.0

        phase_time = self.elapsed_time()

        if phase_time < self.ramp_time:
            return max(0.0, min(1.0, phase_time / self.ramp_time))

        if phase_time > self.current_phase_duration - self.ramp_time:
            remaining = max(0.0, self.current_phase_duration - phase_time)
            return max(0.0, min(1.0, remaining / self.ramp_time))

        return 1.0

    # ==================================================
    # Auto-tuning / robustez
    # ==================================================
    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def compute_motion_parameters(self):
        """
        Modo speed:
            usuario da linear_speed y angular_speed
            -> se calculan forward_time y turn_time

        Modo time:
            usuario da total_time
            -> se calculan linear_speed y angular_speed
        """

        # Validación de pausas
        total_pause_time = self.initial_stop_time + 4.0 * self.point_stop_time + self.final_stop_time

        if self.mode == 'speed':
            if self.linear_speed <= 0.0:
                self.get_logger().warn("linear_speed <= 0. Using 0.15")
                self.linear_speed = 0.15

            if self.angular_speed <= 0.0:
                self.get_logger().warn("angular_speed <= 0. Using 0.35")
                self.angular_speed = 0.35

            # Saturación para robustez
            if self.linear_speed > self.max_linear_speed:
                self.get_logger().warn(
                    f"Requested linear_speed={self.linear_speed:.3f} exceeds max. "
                    f"Clamping to {self.max_linear_speed:.3f}"
                )
                self.linear_speed = self.max_linear_speed

            if self.angular_speed > self.max_angular_speed:
                self.get_logger().warn(
                    f"Requested angular_speed={self.angular_speed:.3f} exceeds max. "
                    f"Clamping to {self.max_angular_speed:.3f}"
                )
                self.angular_speed = self.max_angular_speed

            if self.linear_speed < self.min_linear_speed:
                self.get_logger().warn(
                    f"Requested linear_speed too low. Raising to {self.min_linear_speed:.3f}"
                )
                self.linear_speed = self.min_linear_speed

            if self.angular_speed < self.min_angular_speed:
                self.get_logger().warn(
                    f"Requested angular_speed too low. Raising to {self.min_angular_speed:.3f}"
                )
                self.angular_speed = self.min_angular_speed

            self.forward_time = self.side_length / self.linear_speed
            self.turn_time = (math.pi / 2.0) / self.angular_speed

            if self.ramp_time > 0.0:
                self.forward_time += self.ramp_time
                self.turn_time += self.ramp_time

            estimated_total = 4.0 * self.forward_time + 4.0 * self.turn_time + total_pause_time
            self.get_logger().info(
                f"[AUTO-TUNE] Speed mode -> estimated total execution time = {estimated_total:.2f} s"
            )

        else:  # mode == 'time'
            if self.total_time <= 0.0:
                self.get_logger().warn("total_time <= 0. Using 80.0")
                self.total_time = 80.0

            # asegurar rango razonable
            self.turn_time_ratio = self.clamp(self.turn_time_ratio, 0.10, 0.50)

            available_motion_time = self.total_time - total_pause_time

            if available_motion_time <= 0.0:
                self.get_logger().warn(
                    "total_time is too small after subtracting pauses. "
                    "Using minimum feasible motion time."
                )
                available_motion_time = 20.0

            move_total = (1.0 - self.turn_time_ratio) * available_motion_time
            turn_total = self.turn_time_ratio * available_motion_time

            self.forward_time = move_total / 4.0
            self.turn_time = turn_total / 4.0

            # evitar tiempos imposibles
            self.forward_time = max(self.forward_time, 0.1)
            self.turn_time = max(self.turn_time, 0.1)

            raw_linear_speed = self.side_length / self.forward_time
            raw_angular_speed = (math.pi / 2.0) / self.turn_time

            reachable = True

            # Saturación
            self.linear_speed = raw_linear_speed
            self.angular_speed = raw_angular_speed

            if self.linear_speed > self.max_linear_speed:
                self.get_logger().warn(
                    f"[AUTO-TUNE] Computed linear_speed={self.linear_speed:.3f} exceeds max "
                    f"{self.max_linear_speed:.3f}. Clamping."
                )
                self.linear_speed = self.max_linear_speed
                reachable = False

            if self.angular_speed > self.max_angular_speed:
                self.get_logger().warn(
                    f"[AUTO-TUNE] Computed angular_speed={self.angular_speed:.3f} exceeds max "
                    f"{self.max_angular_speed:.3f}. Clamping."
                )
                self.angular_speed = self.max_angular_speed
                reachable = False

            if self.linear_speed < self.min_linear_speed:
                self.get_logger().warn(
                    f"[AUTO-TUNE] Computed linear_speed too low. Raising to {self.min_linear_speed:.3f}"
                )
                self.linear_speed = self.min_linear_speed

            if self.angular_speed < self.min_angular_speed:
                self.get_logger().warn(
                    f"[AUTO-TUNE] Computed angular_speed too low. Raising to {self.min_angular_speed:.3f}"
                )
                self.angular_speed = self.min_angular_speed

            # Recalcular tiempos reales después de saturar
            self.forward_time = self.side_length / self.linear_speed
            self.turn_time = (math.pi / 2.0) / self.angular_speed

            if self.ramp_time > 0.0:
                self.forward_time += self.ramp_time
                self.turn_time += self.ramp_time

            actual_total = 4.0 * self.forward_time + 4.0 * self.turn_time + total_pause_time

            if reachable:
                self.get_logger().info(
                    "[AUTO-TUNE] Time mode -> requested time is dynamically reachable."
                )
            else:
                self.get_logger().warn(
                    "[AUTO-TUNE] Requested total_time was not fully reachable with robot limits. "
                    "Controller adjusted to safe max/min speeds."
                )

            self.get_logger().info(
                f"[AUTO-TUNE] Time mode -> adjusted total execution time = {actual_total:.2f} s"
            )

    # ==================================================
    # Utilidades de tiempo
    # ==================================================
    def elapsed_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time.nanoseconds) / 1e9

    def reset_time_reference(self):
        self.start_time = self.get_clock().now()

    # ==================================================
    # Publicación de velocidades
    # ==================================================
    def publish_stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_forward(self):
        self.vel.linear.x = self.linear_speed * self.ramp_factor()
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_turn(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = self.turn_direction * self.angular_speed * self.ramp_factor()
        self.cmd_vel_pub.publish(self.vel)

    def next_point_name(self):
        names = ["p1", "p2", "p3", "p4"]
        return names[self.current_point_index % 4]

    # ==================================================
    # FSM
    # ==================================================
    def timer_callback(self):

        if self.state == "initial_stop":
            self.publish_stop()

            if self.elapsed_time() >= self.initial_stop_time:
                self.state = "move_forward"
                self.reset_time_reference()
                self.current_phase_duration = self.forward_time
                self.get_logger().info("Starting motion: p1 -> p2")

        elif self.state == "move_forward":
            self.current_phase_duration = self.forward_time
            self.publish_forward()

            if self.elapsed_time() >= self.forward_time:
                self.publish_stop()
                self.state = "point_stop"
                self.reset_time_reference()

                reached_point = self.next_point_name()
                self.get_logger().info(
                    f"Reached {reached_point}. Hold position for measurement."
                )

        elif self.state == "point_stop":
            self.publish_stop()

            if self.elapsed_time() >= self.point_stop_time:
                if self.segment_count >= 3:
                    self.state = "final_stop"
                    self.reset_time_reference()
                    self.current_phase_duration = self.final_stop_time
                    self.get_logger().info("Square completed. Final stop.")
                else:
                    self.state = "turn"
                    self.reset_time_reference()
                    self.current_phase_duration = self.turn_time
                    self.get_logger().info("Turning 90 degrees to next segment.")

        elif self.state == "turn":
            self.current_phase_duration = self.turn_time
            self.publish_turn()

            if self.elapsed_time() >= self.turn_time:
                self.publish_stop()

                self.segment_count += 1
                self.current_point_index += 1

                next_target = self.next_point_name()
                self.state = "move_forward"
                self.reset_time_reference()
                self.current_phase_duration = self.forward_time
                self.get_logger().info(f"Starting segment toward {next_target}")

        elif self.state == "final_stop":
            self.publish_stop()

            if self.elapsed_time() >= self.final_stop_time:
                self.state = "done"
                self.get_logger().info("Done.")

        elif self.state == "done":
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = SquareController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

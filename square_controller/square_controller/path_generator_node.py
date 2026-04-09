#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        # =========================================================
        # Parámetros ROS2
        # =========================================================
        self.declare_parameter('mode', 'speed')  # 'speed' o 'time'
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate', 20.0)

        # Límites / robustez
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('min_angular_speed', 0.10)

        # Velocidades por defecto para mode='speed'
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.35)

        # Pausas
        self.declare_parameter('initial_stop_time', 2.0)
        self.declare_parameter('point_stop_time', 2.0)
        self.declare_parameter('final_stop_time', 2.0)

        # Geofencing opcional
        self.declare_parameter('use_geofence', False)
        self.declare_parameter('min_x', -10.0)
        self.declare_parameter('max_x', 10.0)
        self.declare_parameter('min_y', -10.0)
        self.declare_parameter('max_y', 10.0)

        # Pose inicial asumida del robot
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_theta', 0.0)

        # Lista de waypoints en JSON
        self.declare_parameter('waypoints_json', '[]')

        # =========================================================
        # Leer parámetros
        # =========================================================
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        self.linear_speed_default = float(self.get_parameter('linear_speed').value)
        self.angular_speed_default = float(self.get_parameter('angular_speed').value)

        self.initial_stop_time = float(self.get_parameter('initial_stop_time').value)
        self.point_stop_time = float(self.get_parameter('point_stop_time').value)
        self.final_stop_time = float(self.get_parameter('final_stop_time').value)

        self.use_geofence = bool(self.get_parameter('use_geofence').value)
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_theta = float(self.get_parameter('start_theta').value)

        self.waypoints_json = str(self.get_parameter('waypoints_json').value)

        # =========================================================
        # Validaciones base
        # =========================================================
        if self.mode not in ['speed', 'time']:
            self.get_logger().warn(f"Modo inválido '{self.mode}'. Usando 'speed'.")
            self.mode = 'speed'

        if self.control_rate <= 0.0:
            self.get_logger().warn("control_rate <= 0. Usando 20 Hz.")
            self.control_rate = 20.0

        if self.linear_speed_default <= 0.0:
            self.get_logger().warn("linear_speed <= 0. Usando 0.15 m/s.")
            self.linear_speed_default = 0.15

        if self.angular_speed_default <= 0.0:
            self.get_logger().warn("angular_speed <= 0. Usando 0.35 rad/s.")
            self.angular_speed_default = 0.35

        # Saturación defaults
        self.linear_speed_default = self.clamp(
            self.linear_speed_default,
            self.min_linear_speed,
            self.max_linear_speed
        )
        self.angular_speed_default = self.clamp(
            self.angular_speed_default,
            self.min_angular_speed,
            self.max_angular_speed
        )

        # =========================================================
        # Cargar y procesar trayectoria
        # =========================================================
        raw_waypoints = self.load_waypoints()
        self.segments, self.route_ready = self.build_segments(raw_waypoints)

        # =========================================================
        # Estado FSM
        # =========================================================
        self.state = 'initial_stop'
        self.segment_index = 0
        self.start_time_ref = self.get_clock().now()

        # Variables activas del segmento
        self.current_turn_angle = 0.0
        self.current_distance = 0.0
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.current_turn_time = 0.0
        self.current_forward_time = 0.0

        # =========================================================
        # ROS interfaces
        # =========================================================
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.vel = Twist()

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # =========================================================
        # Logs
        # =========================================================
        self.get_logger().info("Path Generator Node initialized.")
        self.get_logger().info(f"Mode: {self.mode}")
        self.get_logger().info(f"cmd_vel_topic: {self.cmd_vel_topic}")
        self.get_logger().info(
            f"Limits: vmax={self.max_linear_speed:.3f} m/s, "
            f"wmax={self.max_angular_speed:.3f} rad/s"
        )
        self.get_logger().info(
            f"Default speeds: v={self.linear_speed_default:.3f} m/s, "
            f"w={self.angular_speed_default:.3f} rad/s"
        )
        self.get_logger().info(f"Valid segments loaded: {len(self.segments)}")

        for i, seg in enumerate(self.segments, start=1):
            self.get_logger().info(
                f"[SEG {i}] target=({seg['target_x']:.3f}, {seg['target_y']:.3f}) "
                f"dist={seg['distance']:.3f} m, "
                f"turn={seg['delta_theta']:.3f} rad, "
                f"v={seg['linear_speed']:.3f} m/s, "
                f"w={seg['angular_speed']:.3f} rad/s, "
                f"t_turn={seg['turn_time']:.3f} s, "
                f"t_fwd={seg['forward_time']:.3f} s"
            )

        if not self.route_ready:
            self.get_logger().error("Route precheck FAILED. Node will remain stopped.")
        elif not self.segments:
            self.get_logger().warn("No valid reachable segments. Node will remain stopped.")

    # =============================================================
    # Utilidades
    # =============================================================
    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def elapsed_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time_ref.nanoseconds) / 1e9

    def reset_time_reference(self):
        self.start_time_ref = self.get_clock().now()

    def publish_stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_turn(self, angular_speed_signed):
        self.vel.linear.x = 0.0
        self.vel.angular.z = angular_speed_signed
        self.cmd_vel_pub.publish(self.vel)

    def publish_forward(self, linear_speed):
        self.vel.linear.x = linear_speed
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    # =============================================================
    # Carga de waypoints
    # =============================================================
    def load_waypoints(self):
        try:
            waypoints = json.loads(self.waypoints_json)
            if not isinstance(waypoints, list):
                raise ValueError("waypoints_json must decode to a list")
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoints_json: {e}")
            return []

        validated = []
        for i, wp in enumerate(waypoints, start=1):
            if not isinstance(wp, dict):
                self.get_logger().warn(f"Waypoint {i} inválido: no es objeto JSON. Se omite.")
                continue

            if 'x' not in wp or 'y' not in wp:
                self.get_logger().warn(f"Waypoint {i} sin x/y. Se omite.")
                continue

            x = float(wp['x'])
            y = float(wp['y'])

            if self.use_geofence:
                if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
                    self.get_logger().warn(
                        f"Waypoint {i} ({x:.3f}, {y:.3f}) fuera del geofence. Se omite."
                    )
                    continue

            time_to_wp = None
            if 'time_to_wp' in wp and wp['time_to_wp'] is not None:
                time_to_wp = float(wp['time_to_wp'])

            validated.append({
                'x': x,
                'y': y,
                'time_to_wp': time_to_wp
            })

        return validated

    # =============================================================
    # Construcción de segmentos + precheck completo
    # =============================================================
    def build_segments(self, waypoints):
        segments = []
        route_valid = True

        curr_x = self.start_x
        curr_y = self.start_y
        curr_theta = self.start_theta

        total_min_motion_time = 0.0
        total_budget_motion_time = 0.0

        if not waypoints:
            self.get_logger().error("[PRECHECK] No hay waypoints válidos.")
            return [], False

        for i, wp in enumerate(waypoints, start=1):
            target_x = wp['x']
            target_y = wp['y']
            time_to_wp = wp['time_to_wp']

            dx = target_x - curr_x
            dy = target_y - curr_y

            distance = math.hypot(dx, dy)

            if distance < 1e-6:
                self.get_logger().error(
                    f"[PRECHECK] Waypoint {i} coincide con la posición actual."
                )
                route_valid = False
                curr_x = target_x
                curr_y = target_y
                continue

            target_theta = math.atan2(dy, dx)
            delta_theta = self.normalize_angle(target_theta - curr_theta)
            abs_turn = abs(delta_theta)

            # Tiempo mínimo físicamente necesario para este tramo
            min_turn_time = abs_turn / self.max_angular_speed if abs_turn > 1e-6 else 0.0
            min_forward_time = distance / self.max_linear_speed
            min_total_time = min_turn_time + min_forward_time

            total_min_motion_time += min_total_time

            if self.mode == 'speed':
                linear_speed = self.linear_speed_default
                angular_speed = self.angular_speed_default if abs_turn > 1e-6 else 0.0

                turn_time = abs_turn / angular_speed if abs_turn > 1e-6 else 0.0
                forward_time = distance / linear_speed

                total_budget_motion_time += (turn_time + forward_time)

            else:
                # mode == 'time'
                if time_to_wp is None:
                    self.get_logger().error(
                        f"[PRECHECK] Waypoint {i} no trae time_to_wp en mode='time'."
                    )
                    route_valid = False
                    curr_x = target_x
                    curr_y = target_y
                    curr_theta = target_theta
                    continue

                if time_to_wp <= 0.0:
                    self.get_logger().error(
                        f"[PRECHECK] Waypoint {i} tiene time_to_wp <= 0."
                    )
                    route_valid = False
                    curr_x = target_x
                    curr_y = target_y
                    curr_theta = target_theta
                    continue

                total_budget_motion_time += time_to_wp

                if time_to_wp + 1e-9 < min_total_time:
                    self.get_logger().error(
                        f"[PRECHECK] Waypoint {i} NO alcanzable: "
                        f"requiere al menos {min_total_time:.3f} s "
                        f"(giro={min_turn_time:.3f} s, avance={min_forward_time:.3f} s), "
                        f"pero time_to_wp={time_to_wp:.3f} s."
                    )
                    route_valid = False
                    curr_x = target_x
                    curr_y = target_y
                    curr_theta = target_theta
                    continue

                # Reparto del tiempo disponible entre giro y avance
                extra_time = time_to_wp - min_total_time
                turn_share = (min_turn_time / min_total_time) if min_total_time > 1e-6 else 0.0

                turn_time = min_turn_time + extra_time * turn_share
                forward_time = min_forward_time + extra_time * (1.0 - turn_share)

                turn_time = max(turn_time, 1e-6)
                forward_time = max(forward_time, 1e-6)

                linear_speed = distance / forward_time
                angular_speed = abs_turn / turn_time if abs_turn > 1e-6 else 0.0

            # Validación de límites
            if not (self.min_linear_speed <= linear_speed <= self.max_linear_speed):
                self.get_logger().error(
                    f"[PRECHECK] Waypoint {i}: v requerida={linear_speed:.3f} "
                    f"fuera de límites [{self.min_linear_speed:.3f}, {self.max_linear_speed:.3f}]."
                )
                route_valid = False

            if abs_turn > 1e-6 and not (self.min_angular_speed <= angular_speed <= self.max_angular_speed):
                self.get_logger().error(
                    f"[PRECHECK] Waypoint {i}: w requerida={angular_speed:.3f} "
                    f"fuera de límites [{self.min_angular_speed:.3f}, {self.max_angular_speed:.3f}]."
                )
                route_valid = False

            segments.append({
                'target_x': target_x,
                'target_y': target_y,
                'distance': distance,
                'target_theta': target_theta,
                'delta_theta': delta_theta,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'turn_time': turn_time,
                'forward_time': forward_time
            })

            # La planeación sigue la ruta completa intencional
            curr_x = target_x
            curr_y = target_y
            curr_theta = target_theta

        total_waypoints = len(waypoints)

        total_min_full_time = (
            self.initial_stop_time
            + total_min_motion_time
            + total_waypoints * self.point_stop_time
            + self.final_stop_time
        )

        total_budget_full_time = (
            self.initial_stop_time
            + total_budget_motion_time
            + total_waypoints * self.point_stop_time
            + self.final_stop_time
        )

        self.get_logger().info(
            f"[PRECHECK] Tiempo mínimo de movimiento: {total_min_motion_time:.3f} s"
        )
        self.get_logger().info(
            f"[PRECHECK] Presupuesto de movimiento: {total_budget_motion_time:.3f} s"
        )
        self.get_logger().info(
            f"[PRECHECK] Tiempo mínimo total (con pausas): {total_min_full_time:.3f} s"
        )
        self.get_logger().info(
            f"[PRECHECK] Presupuesto total (con pausas): {total_budget_full_time:.3f} s"
        )

        if self.mode == 'time' and total_min_motion_time > total_budget_motion_time + 1e-9:
            self.get_logger().error(
                "[PRECHECK] La ruta completa excede el presupuesto total definido por time_to_wp."
            )
            route_valid = False

        if not route_valid:
            self.get_logger().error(
                "[PRECHECK] Ruta inválida. No se ejecutará ninguna parte de la secuencia."
            )
            return [], False

        self.get_logger().info("[PRECHECK] Ruta válida. Se ejecutará la secuencia completa.")
        return segments, True

    # =============================================================
    # FSM
    # =============================================================
    def timer_callback(self):
        if not self.route_ready or not self.segments:
            self.publish_stop()
            return

        if self.state == 'initial_stop':
            self.publish_stop()
            if self.elapsed_time() >= self.initial_stop_time:
                self.load_current_segment()
                self.state = 'turn'
                self.reset_time_reference()
                self.get_logger().info(
                    f"Starting segment {self.segment_index + 1}: first rotate, then move."
                )

        elif self.state == 'turn':
            if abs(self.current_turn_angle) < 1e-6 or self.current_turn_time <= 1e-6:
                self.publish_stop()
                self.state = 'move_forward'
                self.reset_time_reference()
                self.get_logger().info("No rotation needed. Starting forward motion.")
            else:
                signed_w = math.copysign(self.current_angular_speed, self.current_turn_angle)
                self.publish_turn(signed_w)
                if self.elapsed_time() >= self.current_turn_time:
                    self.publish_stop()
                    self.state = 'move_forward'
                    self.reset_time_reference()
                    self.get_logger().info("Rotation complete. Starting forward motion.")

        elif self.state == 'move_forward':
            self.publish_forward(self.current_linear_speed)
            if self.elapsed_time() >= self.current_forward_time:
                self.publish_stop()
                self.state = 'point_stop'
                self.reset_time_reference()
                self.get_logger().info(
                    f"Reached target point {self.segment_index + 1}. Hold for measurement."
                )

        elif self.state == 'point_stop':
            self.publish_stop()
            if self.elapsed_time() >= self.point_stop_time:
                self.segment_index += 1
                if self.segment_index >= len(self.segments):
                    self.state = 'final_stop'
                    self.reset_time_reference()
                    self.get_logger().info("Path completed. Final stop.")
                else:
                    self.load_current_segment()
                    self.state = 'turn'
                    self.reset_time_reference()
                    self.get_logger().info(
                        f"Starting segment {self.segment_index + 1}: rotate, then move."
                    )

        elif self.state == 'final_stop':
            self.publish_stop()
            if self.elapsed_time() >= self.final_stop_time:
                self.state = 'done'
                self.get_logger().info("Done.")

        elif self.state == 'done':
            self.publish_stop()

    def load_current_segment(self):
        seg = self.segments[self.segment_index]
        self.current_turn_angle = seg['delta_theta']
        self.current_distance = seg['distance']
        self.current_linear_speed = seg['linear_speed']
        self.current_angular_speed = seg['angular_speed']
        self.current_turn_time = seg['turn_time']
        self.current_forward_time = seg['forward_time']

        self.get_logger().info(
            f"[EXEC SEG {self.segment_index + 1}] "
            f"target=({seg['target_x']:.3f}, {seg['target_y']:.3f}) "
            f"turn={seg['delta_theta']:.3f} rad in {seg['turn_time']:.3f} s, "
            f"forward={seg['distance']:.3f} m in {seg['forward_time']:.3f} s "
            f"with v={seg['linear_speed']:.3f} m/s and w={seg['angular_speed']:.3f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
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

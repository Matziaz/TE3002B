import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PIDController:
    def __init__(self, kp, ki, kd, integral_limit, derivative_filter_alpha):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.alpha = max(0.0, min(1.0, derivative_filter_alpha))

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 1e-6:
            dt = 1e-3

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        raw_derivative = (error - self.prev_error) / dt
        derivative = self.alpha * self.prev_derivative + (1.0 - self.alpha) * raw_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_derivative = derivative
        return output


class SquareController(Node):

    def __init__(self):
        super().__init__('square_controller')

        # Compatibilidad con config existente
        self.declare_parameter('mode', 'pid')

        # Geometria
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_theta', 0.0)

        # ROS
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_rate', 20.0)

        # Tolerancias
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.06)
        self.declare_parameter('heading_priority_threshold', 0.45)

        # Limites
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('min_angular_speed', 0.10)

        # PID lineal
        self.declare_parameter('kp_linear', 0.9)
        self.declare_parameter('ki_linear', 0.02)
        self.declare_parameter('kd_linear', 0.08)

        # PID angular
        self.declare_parameter('kp_angular', 2.8)
        self.declare_parameter('ki_angular', 0.04)
        self.declare_parameter('kd_angular', 0.16)

        # Anti-windup y filtro derivativo
        self.declare_parameter('linear_integral_limit', 0.5)
        self.declare_parameter('angular_integral_limit', 0.8)
        self.declare_parameter('derivative_filter_alpha', 0.80)

        # Pausas para medicion
        self.declare_parameter('initial_stop_time', 2.0)
        self.declare_parameter('point_stop_time', 2.0)
        self.declare_parameter('final_stop_time', 2.0)

        self.mode = str(self.get_parameter('mode').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_theta = float(self.get_parameter('start_theta').value)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        self.heading_priority_threshold = float(self.get_parameter('heading_priority_threshold').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.ki_linear = float(self.get_parameter('ki_linear').value)
        self.kd_linear = float(self.get_parameter('kd_linear').value)

        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.ki_angular = float(self.get_parameter('ki_angular').value)
        self.kd_angular = float(self.get_parameter('kd_angular').value)

        self.linear_integral_limit = float(self.get_parameter('linear_integral_limit').value)
        self.angular_integral_limit = float(self.get_parameter('angular_integral_limit').value)
        self.derivative_filter_alpha = float(self.get_parameter('derivative_filter_alpha').value)

        self.initial_stop_time = float(self.get_parameter('initial_stop_time').value)
        self.point_stop_time = float(self.get_parameter('point_stop_time').value)
        self.final_stop_time = float(self.get_parameter('final_stop_time').value)

        if self.side_length <= 0.0:
            self.get_logger().warn('side_length <= 0. Using default 2.0')
            self.side_length = 2.0

        if self.control_rate <= 0.0:
            self.get_logger().warn('control_rate <= 0. Using default 20.0')
            self.control_rate = 20.0

        self.distance_tolerance = max(0.01, self.distance_tolerance)
        self.heading_tolerance = max(0.01, self.heading_tolerance)
        self.heading_priority_threshold = max(self.heading_tolerance, self.heading_priority_threshold)

        self.max_linear_speed = max(0.01, self.max_linear_speed)
        self.max_angular_speed = max(0.05, self.max_angular_speed)
        self.min_linear_speed = max(0.0, min(self.min_linear_speed, self.max_linear_speed))
        self.min_angular_speed = max(0.0, min(self.min_angular_speed, self.max_angular_speed))

        # Ruta cuadrada: p1 -> p2 -> p3 -> p4 -> p1
        p1 = (self.start_x, self.start_y)
        p2 = (self.start_x + self.side_length, self.start_y)
        p3 = (self.start_x + self.side_length, self.start_y + self.side_length)
        p4 = (self.start_x, self.start_y + self.side_length)
        self.waypoints = [p2, p3, p4, p1]
        self.vertex_headings = [math.pi / 2.0, math.pi, -math.pi / 2.0, 0.0]

        # Estado
        self.state = 'initial_stop'
        self.target_index = 0
        self.start_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()

        self.odom_ready = False
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_theta = self.start_theta

        # Controladores PID
        self.linear_pid = PIDController(
            self.kp_linear,
            self.ki_linear,
            self.kd_linear,
            self.linear_integral_limit,
            self.derivative_filter_alpha,
        )
        self.angular_pid = PIDController(
            self.kp_angular,
            self.ki_angular,
            self.kd_angular,
            self.angular_integral_limit,
            self.derivative_filter_alpha,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vel = Twist()

        self.get_logger().info('Closed-loop PID square controller initialized.')
        self.get_logger().info(f'side_length={self.side_length:.3f} m | control_rate={self.control_rate:.1f} Hz')
        self.get_logger().info(f'cmd_vel_topic={self.cmd_vel_topic} | odom_topic={self.odom_topic}')
        self.get_logger().info(
            f'PID linear=(kp={self.kp_linear:.3f}, ki={self.ki_linear:.3f}, kd={self.kd_linear:.3f}) | '
            f'PID angular=(kp={self.kp_angular:.3f}, ki={self.ki_angular:.3f}, kd={self.kd_angular:.3f})'
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(value, vmax))

    def elapsed_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time.nanoseconds) / 1e9

    def reset_time_reference(self):
        self.start_time = self.get_clock().now()

    def current_dt(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_control_time.nanoseconds) / 1e9
        self.last_control_time = now
        if dt <= 1e-6:
            return 1.0 / self.control_rate
        return dt

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)

        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        self.odom_ready = True

    def publish_stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_cmd(self, linear, angular):
        self.vel.linear.x = float(linear)
        self.vel.angular.z = float(angular)
        self.cmd_vel_pub.publish(self.vel)

    def reset_controllers(self):
        self.linear_pid.reset()
        self.angular_pid.reset()

    def compute_go_to_point_cmd(self, goal_x, goal_y, dt):
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - self.current_theta)

        if distance_error <= self.distance_tolerance:
            return True, 0.0, 0.0

        linear_cmd = self.linear_pid.update(distance_error, dt)
        angular_cmd = self.angular_pid.update(heading_error, dt)

        # Prioriza orientar el robot cuando el error angular es grande.
        if abs(heading_error) > self.heading_priority_threshold:
            linear_cmd = 0.0

        linear_cmd = self.clamp(linear_cmd, 0.0, self.max_linear_speed)
        if 0.0 < linear_cmd < self.min_linear_speed and distance_error > self.distance_tolerance:
            linear_cmd = self.min_linear_speed

        angular_mag = self.clamp(abs(angular_cmd), 0.0, self.max_angular_speed)
        if 0.0 < angular_mag < self.min_angular_speed and abs(heading_error) > self.heading_tolerance:
            angular_mag = self.min_angular_speed
        angular_cmd = math.copysign(angular_mag, angular_cmd)

        return False, linear_cmd, angular_cmd

    def compute_heading_align_cmd(self, heading_ref, dt):
        heading_error = self.normalize_angle(heading_ref - self.current_theta)
        if abs(heading_error) <= self.heading_tolerance:
            return True, 0.0

        angular_cmd = self.angular_pid.update(heading_error, dt)
        angular_mag = self.clamp(abs(angular_cmd), 0.0, self.max_angular_speed)
        if 0.0 < angular_mag < self.min_angular_speed:
            angular_mag = self.min_angular_speed
        return False, math.copysign(angular_mag, angular_cmd)

    def timer_callback(self):
        dt = self.current_dt()

        if not self.odom_ready:
            self.publish_stop()
            return

        if self.state == 'initial_stop':
            self.publish_stop()

            if self.elapsed_time() >= self.initial_stop_time:
                self.state = 'go_to_point'
                self.reset_controllers()
                self.reset_time_reference()
                self.get_logger().info('Starting closed-loop square tracking.')

        elif self.state == 'go_to_point':
            goal_x, goal_y = self.waypoints[self.target_index]
            reached, linear_cmd, angular_cmd = self.compute_go_to_point_cmd(goal_x, goal_y, dt)
            self.publish_cmd(linear_cmd, angular_cmd)

            if reached:
                self.publish_stop()
                self.state = 'align_vertex'
                self.reset_controllers()
                self.reset_time_reference()
                self.get_logger().info(f'Reached waypoint {self.target_index + 1}. Aligning heading.')

        elif self.state == 'align_vertex':
            heading_ref = self.vertex_headings[self.target_index]
            aligned, angular_cmd = self.compute_heading_align_cmd(heading_ref, dt)
            self.publish_cmd(0.0, angular_cmd)

            if aligned:
                self.publish_stop()
                self.state = 'point_stop'
                self.reset_time_reference()
                self.get_logger().info('Heading aligned. Hold position for measurement.')

        elif self.state == 'point_stop':
            self.publish_stop()

            if self.elapsed_time() >= self.point_stop_time:
                self.target_index += 1
                if self.target_index >= len(self.waypoints):
                    self.state = 'final_stop'
                    self.reset_time_reference()
                    self.get_logger().info('Square completed. Final stop.')
                else:
                    self.state = 'go_to_point'
                    self.reset_controllers()
                    self.reset_time_reference()
                    self.get_logger().info(f'Starting segment {self.target_index + 1}.')

        elif self.state == 'final_stop':
            self.publish_stop()

            if self.elapsed_time() >= self.final_stop_time:
                self.state = 'done'
                self.get_logger().info('Done.')

        elif self.state == 'done':
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = SquareController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquarePathClass(Node):

    def __init__(self):
        super().__init__('square_path')

        # ===== Declarar parámetros =====
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('initial_stop_time', 2.0)
        self.declare_parameter('point_stop_time', 2.0)
        self.declare_parameter('final_stop_time', 2.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate', 20.0)

        # ===== Leer parámetros =====
        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.initial_stop_time = self.get_parameter('initial_stop_time').value
        self.point_stop_time = self.get_parameter('point_stop_time').value
        self.final_stop_time = self.get_parameter('final_stop_time').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_rate = self.get_parameter('control_rate').value

        # ===== Validación básica =====
        if self.linear_speed <= 0.0:
            self.get_logger().warn("linear_speed <= 0. Using default 0.15")
            self.linear_speed = 0.15

        if self.angular_speed <= 0.0:
            self.get_logger().warn("angular_speed <= 0. Using default 0.35")
            self.angular_speed = 0.35

        if self.side_length <= 0.0:
            self.get_logger().warn("side_length <= 0. Using default 2.0")
            self.side_length = 2.0

        if self.control_rate <= 0.0:
            self.get_logger().warn("control_rate <= 0. Using default 20.0")
            self.control_rate = 20.0

        # ===== Tiempos calculados =====
        self.forward_time = self.side_length / self.linear_speed
        self.turn_time = (math.pi / 2.0) / self.angular_speed

        # ===== Variables de estado =====
        self.state = "initial_stop"
        self.segment_count = 0
        self.current_point_index = 1
        self.start_time = self.get_clock().now()

        # ===== ROS =====
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vel = Twist()

        self.get_logger().info("Node initialized!!!")
        self.get_logger().info(
            f"Square path config: side_length={self.side_length} m, "
            f"linear_speed={self.linear_speed} m/s, "
            f"angular_speed={self.angular_speed} rad/s"
        )
        self.get_logger().info(
            f"initial_stop_time={self.initial_stop_time} s, "
            f"point_stop_time={self.point_stop_time} s, "
            f"final_stop_time={self.final_stop_time} s"
        )
        self.get_logger().info(
            f"forward_time={self.forward_time:.2f} s, "
            f"turn_time={self.turn_time:.2f} s"
        )
        self.get_logger().info(
            f"cmd_vel_topic={self.cmd_vel_topic}, control_rate={self.control_rate} Hz"
        )
        self.get_logger().info("Robot must start at p1 facing toward p2.")

    def elapsed_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time.nanoseconds) / 1e9

    def reset_time_reference(self):
        self.start_time = self.get_clock().now()

    def publish_stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_forward(self):
        self.vel.linear.x = self.linear_speed
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    def publish_turn(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(self.vel)

    def next_point_name(self):
        names = ["p1", "p2", "p3", "p4"]
        return names[self.current_point_index % 4]

    def timer_callback(self):

        if self.state == "initial_stop":
            self.publish_stop()

            if self.elapsed_time() >= self.initial_stop_time:
                self.state = "move_forward"
                self.reset_time_reference()
                self.get_logger().info("Starting motion: p1 -> p2")

        elif self.state == "move_forward":
            self.publish_forward()

            if self.elapsed_time() >= self.forward_time:
                self.publish_stop()
                self.state = "point_stop"
                self.reset_time_reference()

                reached_point = self.next_point_name()
                self.get_logger().info(f"Reached {reached_point}. Hold position for measurement.")

        elif self.state == "point_stop":
            self.publish_stop()

            if self.elapsed_time() >= self.point_stop_time:
                if self.segment_count >= 3:
                    self.state = "final_stop"
                    self.reset_time_reference()
                    self.get_logger().info("Square completed. Final stop.")
                else:
                    self.state = "turn"
                    self.reset_time_reference()
                    self.get_logger().info("Turning 90 degrees to next segment.")

        elif self.state == "turn":
            self.publish_turn()

            if self.elapsed_time() >= self.turn_time:
                self.publish_stop()

                self.segment_count += 1
                self.current_point_index += 1

                next_target = self.next_point_name()
                self.state = "move_forward"
                self.reset_time_reference()
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

    node = SquarePathClass()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

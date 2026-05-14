#!/usr/bin/env python3
"""Line follower node with PID control and traffic light integration."""

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # --- Declaración de Parámetros ---
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('line_color', 'white')
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_lower', 200)
        self.declare_parameter('v_upper', 255)
        self.declare_parameter('roi_start_fraction', 0.67)
        self.declare_parameter('roi_end_fraction', 1.0)
        
        # Ganancias PID
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.01)  # Nueva ganancia Integral
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('i_limit', 0.5) # Límite para evitar integral windup
        
        self.declare_parameter('v_base', 0.2)
        self.declare_parameter('w_max', 0.5)
        self.declare_parameter('control_loop_rate', 0.033)
        self.declare_parameter('enable_line_follower', True)

        # --- Lectura de Parámetros ---
        self.image_topic = self.get_parameter('image_topic').value
        self.velocity_scale_topic = self.get_parameter('velocity_scale_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.line_color = self.get_parameter('line_color').value
        self.h_lower = int(self.get_parameter('h_lower').value)
        self.h_upper = int(self.get_parameter('h_upper').value)
        self.s_lower = int(self.get_parameter('s_lower').value)
        self.s_upper = int(self.get_parameter('s_upper').value)
        self.v_lower = int(self.get_parameter('v_lower').value)
        self.v_upper = int(self.get_parameter('v_upper').value)
        self.roi_start_fraction = float(self.get_parameter('roi_start_fraction').value)
        self.roi_end_fraction = float(self.get_parameter('roi_end_fraction').value)
        
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.i_limit = float(self.get_parameter('i_limit').value)
        
        self.v_base = float(self.get_parameter('v_base').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.dt = float(self.get_parameter('control_loop_rate').value)
        self.enable = self.get_parameter('enable_line_follower').value

        # --- Estado Interno ---
        self.velocity_scale = 1.0
        self.prev_error = 0.0
        self.integral_error = 0.0  # Acumulador integral
        self.current_error = 0.0
        self.cv_bridge = CvBridge()

        # Configuración de QoS y Comunicaciones (igual que antes)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.create_subscription(Float32, self.velocity_scale_topic, self.velocity_scale_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos_reliable)
        self.error_pub = self.create_publisher(Float32, 'lateral_error', qos)
        
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f'PID Line Follower: kp={self.kp}, ki={self.ki}, kd={self.kd}')

    def image_cb(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def velocity_scale_cb(self, msg: Float32):
        self.velocity_scale = float(msg.data)
        # Opcional: Resetear integral si el robot se detiene por semáforo
        if self.velocity_scale == 0.0:
            self.integral_error = 0.0

    def process_image(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height = cv_image.shape[0]
        roi_start = int(height * self.roi_start_fraction)
        roi_end = int(height * self.roi_end_fraction)
        hsv_roi = hsv[roi_start:roi_end, :]
        
        if self.line_color.lower() == 'white':
            lower = np.array([self.h_lower, self.s_lower, self.v_lower], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, self.v_upper], dtype=np.uint8)
        else:
            lower = np.array([self.h_lower, self.s_lower, 0], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, 100], dtype=np.uint8)
        
        mask = cv2.inRange(hsv_roi, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                image_center_x = cv_image.shape[1] / 2.0
                error = (cx - image_center_x) / (image_center_x + 1e-6)
                self.current_error = np.clip(error, -1.0, 1.0)
                return
        self.current_error = 0.0

    def control_loop(self):
        """Controlador PID."""
        if not self.enable:
            self.cmd_pub.publish(Twist())
            return
        
        error = self.current_error
        
        # 1. Término Proporcional
        p_term = self.kp * error
        
        # 2. Término Integral (con anti-windup)
        # Sumamos el error actual multiplicado por el tiempo transcurrido
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.i_limit, self.i_limit)
        i_term = self.ki * self.integral_error
        
        # 3. Término Derivativo
        d_term = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error
        
        # Cálculo de velocidad angular total: w = P + I + D
        w = p_term + i_term + d_term
        w = np.clip(w * self.velocity_scale, -self.w_max, self.w_max)
        
        v = self.v_base * self.velocity_scale
        
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        
        # Debug
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
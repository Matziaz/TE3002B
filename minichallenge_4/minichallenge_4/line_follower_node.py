#!/usr/bin/env python3
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

        # Declaración de parámetros
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('line_color', 'white')
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 50)
        self.declare_parameter('v_lower', 200)
        self.declare_parameter('v_upper', 255)
        self.declare_parameter('roi_start_fraction', 0.6) # Un poco más alto para anticipar
        self.declare_parameter('roi_end_fraction', 0.9)   # Evitar el borde extremo inferior
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('v_base', 0.15)
        self.declare_parameter('w_max', 1.2) # Mayor capacidad de giro
        self.declare_parameter('control_loop_rate', 0.033)
        self.declare_parameter('enable_line_follower', True)

        # Cargar parámetros
        self.line_color = self.get_parameter('line_color').value
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.v_base = float(self.get_parameter('v_base').value)
        self.w_max = float(self.get_parameter('w_max').value)
        
        # Estado interno
        self.velocity_scale = 1.0
        self.prev_error = 0.0
        self.current_error = 0.0
        self.line_detected = False
        self.cv_bridge = CvBridge()

        # Configuración de QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Suscriptores y Publicadores
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_cb, qos)
        self.create_subscription(Float32, self.get_parameter('velocity_scale_topic').value, self.velocity_scale_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, qos_reliable)
        self.error_pub = self.create_publisher(Float32, 'lateral_error', qos)

        self.create_timer(self.get_parameter('control_loop_rate').value, self.control_loop)
        self.get_logger().info('Line follower con memoria de error iniciado.')

    def velocity_scale_cb(self, msg):
        self.velocity_scale = float(msg.data)

    def image_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error en imagen: {e}')

    def process_image(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        
        # ROI
        r_start = int(h * self.get_parameter('roi_start_fraction').value)
        r_end = int(h * self.get_parameter('roi_end_fraction').value)
        hsv_roi = hsv[r_start:r_end, :]

        # Máscara
        if self.line_color.lower() == 'white':
            lower = np.array([self.get_parameter('h_lower').value, self.get_parameter('s_lower').value, self.get_parameter('v_lower').value])
            upper = np.array([self.get_parameter('h_upper').value, self.get_parameter('s_upper').value, self.get_parameter('v_upper').value])
        else: # Black
            lower = np.array([0, 0, 0])
            upper = np.array([180, 255, 100])

        mask = cv2.inRange(hsv_roi, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 100: # Umbral mínimo de píxeles
                cx = M['m10'] / M['m00']
                error = (cx - w / 2.0) / (w / 2.0)
                self.current_error = np.clip(error, -1.0, 1.0)
                self.line_detected = True
                return

        # Si llegamos aquí, no hay línea
        self.line_detected = False

    def control_loop(self):
        if not self.get_parameter('enable_line_follower').value:
            self.cmd_pub.publish(Twist())
            return

        # Lógica de Recuperación: Si pierde la línea, mantener el giro último
        if not self.line_detected:
            # Si el último error fue positivo (derecha), forzar giro derecha
            # Si fue negativo (izquierda), forzar giro izquierda
            error = 1.2 * (1.0 if self.prev_error > 0 else -1.0)
            derivative = 0.0
        else:
            error = self.current_error
            derivative = error - self.prev_error
        
        self.prev_error = error

        # PD
        w = (self.kp * error) + (self.kd * derivative)
        w = np.clip(w * self.velocity_scale, -self.w_max, self.w_max)
        
        # Reducir velocidad lineal en curvas fuertes para no derrapar
        v = self.v_base * self.velocity_scale * (1.0 - abs(error) * 0.5)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

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
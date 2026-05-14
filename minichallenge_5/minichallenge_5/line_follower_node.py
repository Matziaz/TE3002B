#!/usr/bin/env python3
"""
Nodo de seguidor de línea con PID Robusto.
Soluciona pérdida en curvas y avance sin detección.
"""

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

        # --- Parámetros de ROS 2 ---
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('line_color', 'white')
        
        # Umbrales de Color (HSV)
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 50)
        self.declare_parameter('v_lower', 200)
        self.declare_parameter('v_upper', 255)
        
        # Ganancias PID (Ajustadas para curvas)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('i_limit', 0.3)
        
        # Velocidades
        self.declare_parameter('v_base', 0.2)
        self.declare_parameter('w_max', 1.0) # Más alto para curvas cerradas
        self.declare_parameter('control_loop_rate', 0.033)

        # --- Inicialización de Variables ---
        self.cv_bridge = CvBridge()
        self.current_error = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_valid_error = 0.0  # Memoria para curvas
        self.line_detected = False   # Flag de seguridad
        self.velocity_scale = 1.0

        # Lectura de parámetros
        self.v_base = self.get_parameter('v_base').value
        self.w_max = self.get_parameter('w_max').value
        self.dt = self.get_parameter('control_loop_rate').value

        # QoS para video (Best Effort) y para comandos (Reliable)
        qos_video = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Suscriptores y Publicadores
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_cb, qos_video)
        self.create_subscription(Float32, 'velocity_scale', self.velocity_scale_cb, qos_video)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, qos_cmd)
        
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("PID Line Follower PRO inicializado")

    def velocity_scale_cb(self, msg):
        self.velocity_scale = msg.data

    def image_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error en imagen: {e}")

    def process_image(self, cv_image):
        # 1. Convertir a HSV y aplicar ROI
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        
        # ROI: Franja delgada entre el 70% y 85% de la imagen (mira hacia adelante)
        roi_start = int(h * 0.70)
        roi_end = int(h * 0.85)
        hsv_roi = hsv[roi_start:roi_end, :]

        # 2. Máscara de color
        lower = np.array([self.get_parameter('h_lower').value, 
                          self.get_parameter('s_lower').value, 
                          self.get_parameter('v_lower').value])
        upper = np.array([self.get_parameter('h_upper').value, 
                          self.get_parameter('s_upper').value, 
                          self.get_parameter('v_upper').value])
        
        mask = cv2.inRange(hsv_roi, lower, upper)

        # 3. Limpieza de ruido (Erosión y Dilatación)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 4. Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 150: # Evita ruidos pequeños
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    # Error normalizado [-1.0, 1.0]
                    self.current_error = (cx - (w / 2.0)) / (w / 2.0)
                    self.last_valid_error = self.current_error
                    self.line_detected = True
                    return

        # Si perdemos la línea
        self.line_detected = False

    def control_loop(self):
        cmd = Twist()
        
        # Lógica de Error: Si no hay línea, usamos el último error pero amplificado
        # para que el robot gire agresivamente hasta encontrarla.
        if self.line_detected:
            error = self.current_error
        else:
            # Si se perdió, intentamos girar sobre nuestro eje para buscar
            error = self.last_valid_error * 1.5 
        
        # --- CÁLCULO PID ---
        # Proporcional
        p_term = self.get_parameter('kp').value * error
        
        # Integral (con anti-windup)
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, 
                                     -self.get_parameter('i_limit').value, 
                                     self.get_parameter('i_limit').value)
        i_term = self.get_parameter('ki').value * self.integral_error
        
        # Derivativo
        d_term = self.get_parameter('kd').value * (error - self.prev_error) / self.dt
        self.prev_error = error

        # Salida Angular
        w = p_term + i_term + d_term
        w = np.clip(w * self.velocity_scale, -self.w_max, self.w_max)

        # --- VELOCIDAD ADAPTATIVA ---
        if self.line_detected:
            # Fórmula: Si el error es 0 -> v_base. Si el error es grande -> frena.
            # $v = v_{base} * (1.0 - |error| * 0.8)$
            v = self.v_base * (1.0 - abs(error) * 0.8)
        else:
            # Si no detecta nada, se detiene y solo gira para buscar
            v = 0.02 # Muy lento para rotar sobre su eje
            
        cmd.linear.x = float(v * self.velocity_scale)
        cmd.angular.z = float(w)
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # Detener al cerrar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
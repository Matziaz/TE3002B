#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # Parámetros (Ajusta kp y kd si oscila mucho)
        self.declare_parameter('kp', 0.8)      # Aumentado para reaccionar más rápido
        self.declare_parameter('ki', 0.001)
        self.declare_parameter('kd', 0.2)      # Aumentado para suavizar el rebote
        self.declare_parameter('v_base', 0.2)
        self.declare_parameter('w_max', 1.2)   # Aumentado para permitir giros más cerrados

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.v_base = self.get_parameter('v_base').value
        self.w_max = self.get_parameter('w_max').value
        
        # Estado interno
        self.current_error = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_valid_error = 0.0 # <--- MEMORIA
        self.line_detected = False
        
        self.cv_bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Image, '/camera', self.image_cb, 10)
        self.create_timer(0.033, self.control_loop)

    def process_image(self, cv_image):
        # 1. Preprocesamiento
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # ROI: Tomamos una franja horizontal en el medio-bajo
        # Mirar demasiado abajo es muy tarde, mirar muy arriba es muy inestable.
        h, w, _ = cv_image.shape
        roi_top = int(h * 0.70)
        roi_bottom = int(h * 0.85)
        mask = cv2.inRange(hsv[roi_top:roi_bottom, :], 
                          np.array([0, 0, 200]), # Ajustar según tu color (Blanco)
                          np.array([180, 50, 255]))

        # Limpiar ruido
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100: # Filtro de tamaño mínimo
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    # Error normalizado de -1 a 1
                    self.current_error = (cx - (w / 2)) / (w / 2)
                    self.last_valid_error = self.current_error
                    self.line_detected = True
                    return
        
        # Si llegamos aquí, no se detectó la línea
        self.line_detected = False

    def control_loop(self):
        # Si no ve la línea, usa el último error para seguir girando
        error = self.current_error if self.line_detected else self.last_valid_error * 1.2
        
        # PID
        self.integral_error = np.clip(self.integral_error + error, -1.0, 1.0)
        derivative = error - self.prev_error
        
        w = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative)
        w = np.clip(w, -self.w_max, self.w_max)
        
        # --- ESTRATEGIA DE VELOCIDAD ADAPTATIVA ---
        # Si el error es alto (curva), v baja. Si el error es bajo (recta), v es v_base.
        v = self.v_base * (1.0 - abs(error) * 0.7) 
        
        # Si perdió la línea por completo hace mucho, girar sobre su propio eje para buscar
        if not self.line_detected:
            v = v * 0.2  # Casi detenido para encontrar la línea
        
        cmd = Twist()
        cmd.linear.x = float(max(v, 0.02)) # Que no sea 0 o negativo
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        
        self.prev_error = error

    def image_cb(self, msg):
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_image(cv_img)

def main():
    rclpy.init()
    rclpy.spin(LineFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
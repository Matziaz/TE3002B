#!/usr/bin/env python3
"""
Line follower node using black line detection and PID control.
Detects black lines by thresholding pixel intensity (dark pixels).
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
    """
    Black line follower with PID control.
    
    Detects black lines by thresholding grayscale intensity.
    Uses PID (Proportional-Integral-Derivative) control for smooth steering.
    
    Subscribes to:
    - /camera (sensor_msgs/Image): Camera feed
    - velocity_scale (std_msgs/Float32): Traffic light velocity scale
    
    Publishes to:
    - cmd_vel (geometry_msgs/Twist): Motor commands
    - line_confidence (std_msgs/Float32): Detection confidence [0,1]
    """

    def __init__(self):
        super().__init__('line_follower_node')

        # ===== ROS Topic Configuration =====
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        
        # ===== Vision Parameters (Black Line Detection) =====
        # ROI: analyze these rows for line detection
        self.declare_parameter('roi_start_fraction', 0.4)   # Start at 40% of height
        self.declare_parameter('roi_end_fraction', 1.0)     # End at 100% (bottom)
        
        # Number of horizontal rows to sample for line detection
        self.declare_parameter('detection_rows', 6)
        
        # Black line intensity threshold: pixels below this are considered part of the line
        self.declare_parameter('black_threshold', 100)      # Grayscale value (0-255)
        
        # Blur kernel size for noise reduction (must be odd)
        self.declare_parameter('blur_kernel', 5)
        
        # ===== Control Parameters (PID) =====
        self.declare_parameter('kp', 0.8)           # Proportional gain for angular velocity
        self.declare_parameter('ki', 0.08)          # Integral gain - reduces steady-state error
        self.declare_parameter('kd', 0.15)          # Derivative gain - improves stability
        self.declare_parameter('v_base', 0.25)      # Base linear velocity (m/s)
        self.declare_parameter('w_max', 0.8)        # Max angular velocity (rad/s)
        
        # Confidence scaling: reduce speed if line not clearly detected
        self.declare_parameter('confidence_threshold', 0.3)  # Min confidence to move
        self.declare_parameter('confidence_scaling', True)   # Enable confidence-based speed adjustment
        
        self.declare_parameter('control_loop_rate', 0.033)  # ~30 Hz
        self.declare_parameter('enable_line_follower', True)
        
        # ===== Read Parameters =====
        self.image_topic = self.get_parameter('image_topic').value
        self.velocity_scale_topic = self.get_parameter('velocity_scale_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        self.roi_start_fraction = float(self.get_parameter('roi_start_fraction').value)
        self.roi_end_fraction = float(self.get_parameter('roi_end_fraction').value)
        self.detection_rows = int(self.get_parameter('detection_rows').value)
        self.black_threshold = int(self.get_parameter('black_threshold').value)
        self.blur_kernel = int(self.get_parameter('blur_kernel').value)
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1  # Ensure odd
        
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.v_base = float(self.get_parameter('v_base').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.confidence_scaling = self.get_parameter('confidence_scaling').value
        
        control_loop_rate = float(self.get_parameter('control_loop_rate').value)
        self.enable = self.get_parameter('enable_line_follower').value
        
        # ===== Internal State =====
        self.velocity_scale = 1.0
        self.current_error = 0.0
        self.prev_error = 0.0      # For derivative term
        self.integral_error = 0.0
        self.line_confidence = 0.0
        self.cv_bridge = CvBridge()
        self.dt = 0.033            # Assuming ~30Hz loop
        
        # QoS profile for best-effort topics (image is time-sensitive).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Use reliable QoS for `cmd_vel` so motor drivers that expect RELIABLE messages
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscriptions
        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.create_subscription(Float32, self.velocity_scale_topic, self.velocity_scale_cb, qos)
        
        # Create publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos_reliable)
        self.confidence_pub = self.create_publisher(Float32, 'line_confidence', qos)
        
        # Control loop timer
        self.create_timer(control_loop_rate, self.control_loop)
        
        self.get_logger().info(
            f'Line Follower (Black Line Detection with PID): roi=[{self.roi_start_fraction:.2f}, {self.roi_end_fraction:.2f}], '
            f'rows={self.detection_rows}, black_threshold={self.black_threshold}, '
            f'kp={self.kp}, ki={self.ki}, kd={self.kd}, v_base={self.v_base}'
        )

    def image_cb(self, msg: Image):
        """Receive and process camera image."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def velocity_scale_cb(self, msg: Float32):
        """Receive velocity scale from traffic light controller."""
        self.velocity_scale = max(0.0, float(msg.data))

    def process_image(self, cv_image):
        """
        Detect black line by thresholding grayscale intensity.
        Sets self.current_error and self.line_confidence.
        """
        height, width = cv_image.shape[:2]
        
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        
        # ===== Black Line Detection =====
        # Create binary image: black pixels (low intensity) = 255, others = 0
        _, binary = cv2.threshold(gray, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # ===== Multi-row Detection =====
        # Sample multiple rows from ROI and vote for line position
        roi_start = int(height * self.roi_start_fraction)
        roi_end = int(height * self.roi_end_fraction)
        roi_height = max(1, roi_end - roi_start)
        
        # Sample rows evenly distributed across ROI
        row_indices = np.linspace(roi_start, roi_end - 1, self.detection_rows, dtype=int)
        
        detections = []  # List of (x_position, confidence) for each row
        valid_rows = 0
        
        for row_idx in row_indices:
            row_pixels = binary[row_idx, :]
            
            # Find black pixels (value = 255)
            black_pixels = np.where(row_pixels == 255)[0]
            
            if len(black_pixels) > 0:
                # Line position = center of black pixel cluster
                line_x = np.mean(black_pixels)
                
                # Confidence = fraction of pixels that are black in this row
                confidence = len(black_pixels) / width
                
                detections.append((line_x, confidence))
                valid_rows += 1
        
        # ===== Voting =====
        if detections:
            positions, confidences = np.array(detections).T
            
            # Weighted average position
            line_x = np.average(positions, weights=confidences)
            
            # Overall confidence
            self.line_confidence = float(np.mean(confidences))
        else:
            # No line detected: assume center
            line_x = width / 2.0
            self.line_confidence = 0.0
        
        # ===== Lateral Error Calculation =====
        # Error: normalized distance from image center
        # Range: [-1, 1] where -1 = line far left, 0 = centered, +1 = line far right
        image_center_x = width / 2.0
        error = (line_x - image_center_x) / (image_center_x + 1e-6)
        self.current_error = np.clip(error, -1.0, 1.0)

    def control_loop(self):
        """
        PID controller with confidence-weighted speed adjustment.
        Generates cmd_vel based on lateral error and detection confidence.
        """
        if not self.enable:
            # Publish zero velocity if disabled
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        error = self.current_error
        
        # ===== PID Control =====
        # Proportional term
        p_term = self.kp * error
        
        # Integral accumulation (prevents steady-state error)
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -1.0, 1.0)  # Anti-windup
        i_term = self.ki * self.integral_error
        
        # Derivative term (reduces overshoot and improves stability)
        derivative = (error - self.prev_error) / (self.dt + 1e-6)
        d_term = self.kd * derivative
        self.prev_error = error
        
        # Total angular velocity from PID
        w = p_term + i_term + d_term
        w = np.clip(w, -self.w_max, self.w_max)
        
        # ===== Confidence-based Speed Adjustment =====
        confidence_factor = 1.0
        if self.confidence_scaling:
            if self.line_confidence < self.confidence_threshold:
                # No clear line: reduce speed significantly
                confidence_factor = 0.1
            else:
                # Scale: confidence goes from threshold to 1.0
                confidence_factor = (self.line_confidence - self.confidence_threshold) / (1.0 - self.confidence_threshold)
                confidence_factor = np.clip(confidence_factor, 0.0, 1.0)
        
        # Apply velocity scale (traffic light) and confidence
        v = self.v_base * self.velocity_scale * confidence_factor
        w = w * self.velocity_scale
        
        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        
        # Publish line confidence for monitoring
        confidence_msg = Float32()
        confidence_msg.data = float(self.line_confidence)
        self.confidence_pub.publish(confidence_msg)


def main(args=None):
    """Entry point for the line_follower_node executable."""
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

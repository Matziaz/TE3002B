#!/usr/bin/env python3
"""Line follower node with PD control and traffic light integration."""

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
    """PD-based line follower node.
    
    Subscribes to:
    - /video_source/raw (sensor_msgs/Image): Camera feed
    - velocity_scale (std_msgs/Float32): Traffic light velocity scale
    
    Publishes to:
    - cmd_vel (geometry_msgs/Twist): Motor commands
    - lateral_error (std_msgs/Float32): Lateral error for debugging
    """

    def __init__(self):
        super().__init__('line_follower_node')

        # Declare all configurable parameters.
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        
        # Line detection parameters.
        self.declare_parameter('line_color', 'white')  # 'white' or 'black'
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_lower', 200)
        self.declare_parameter('v_upper', 255)
        
        # Region of interest: detect line in the lower third of the image.
        self.declare_parameter('roi_start_fraction', 0.67)  # Start at 67% of image height
        self.declare_parameter('roi_end_fraction', 1.0)     # End at 100% (bottom)
        
        # Control parameters.
        self.declare_parameter('kp', 0.5)          # Proportional gain for angular velocity
        self.declare_parameter('kd', 0.1)          # Derivative gain for angular velocity
        self.declare_parameter('v_base', 0.2)      # Base linear velocity (m/s)
        self.declare_parameter('w_max', 0.5)       # Max angular velocity (rad/s)
        self.declare_parameter('control_loop_rate', 0.033)  # ~30 Hz
        
        # Enable/disable.
        self.declare_parameter('enable_line_follower', True)
        
        # Read parameters.
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
        self.kd = float(self.get_parameter('kd').value)
        self.v_base = float(self.get_parameter('v_base').value)
        self.w_max = float(self.get_parameter('w_max').value)
        control_loop_rate = float(self.get_parameter('control_loop_rate').value)
        self.enable = self.get_parameter('enable_line_follower').value
        
        # Internal state.
        self.velocity_scale = 1.0
        self.prev_error = 0.0
        self.cv_bridge = CvBridge()
        
        # QoS profile for best-effort topics (image is time-sensitive).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Use reliable QoS for `cmd_vel` so motor drivers that expect RELIABLE
        # messages will receive commands.
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscriptions.
        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.create_subscription(Float32, self.velocity_scale_topic, self.velocity_scale_cb, qos)
        
        # Create publishers.
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos_reliable)
        self.error_pub = self.create_publisher(Float32, 'lateral_error', qos)
        
        # Control loop timer.
        self.create_timer(control_loop_rate, self.control_loop)
        
        # Debug logging.
        self.get_logger().info(
            f'Line follower initialized: color={self.line_color}, '
            f'kp={self.kp}, kd={self.kd}, v_base={self.v_base}, w_max={self.w_max}'
        )

    def image_cb(self, msg: Image):
        """Receive camera image and process it."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def velocity_scale_cb(self, msg: Float32):
        """Receive velocity scale from traffic light controller."""
        self.velocity_scale = float(msg.data)

    def process_image(self, cv_image):
        """Detect line in the image and calculate lateral error."""
        # Convert BGR to HSV.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Extract region of interest (lower third of image).
        height = cv_image.shape[0]
        roi_start = int(height * self.roi_start_fraction)
        roi_end = int(height * self.roi_end_fraction)
        hsv_roi = hsv[roi_start:roi_end, :]
        
        # Create mask based on line color.
        if self.line_color.lower() == 'white':
            # White line: high V, low S
            lower = np.array([self.h_lower, self.s_lower, self.v_lower], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, self.v_upper], dtype=np.uint8)
        else:  # black
            # Black line: low V
            lower = np.array([self.h_lower, self.s_lower, 0], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, 100], dtype=np.uint8)
        
        mask = cv2.inRange(hsv_roi, lower, upper)
        
        # Find contours to locate the line.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour (the line).
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate centroid.
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                # cx is relative to the ROI, so add the offset.
                cx += 0
                
                # Normalize lateral error to [-1, 1].
                image_center_x = cv_image.shape[1] / 2.0
                error = (cx - image_center_x) / (image_center_x + 1e-6)
                error = np.clip(error, -1.0, 1.0)
                
                self.current_error = error
            else:
                self.current_error = 0.0
        else:
            # No line detected.
            self.current_error = 0.0

    def control_loop(self):
        """PD controller to generate cmd_vel based on lateral error."""
        if not self.enable:
            # Publish zero velocity if disabled.
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        # PD control: w = kp * error + kd * d(error)/dt
        error = self.current_error if hasattr(self, 'current_error') else 0.0
        error_derivative = error - self.prev_error
        self.prev_error = error
        
        # Calculate angular velocity.
        w = self.kp * error + self.kd * error_derivative
        w = np.clip(w, -self.w_max, self.w_max)
        
        # Linear velocity is the base velocity, scaled by traffic light.
        v = self.v_base * self.velocity_scale
        
        # Create and publish Twist message.
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        
        # Publish lateral error for debugging.
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)


def main(args=None):
    """Entry point for the line_follower_node executable."""
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
    """PID-based line follower node.
    
    Detects line using HSV color thresholding and contour analysis.
    Uses PID control for smooth and stable steering.
    
    Subscribes to:
    - /camera (sensor_msgs/Image): Camera feed
    - velocity_scale (std_msgs/Float32): Traffic light velocity scale
    
    Publishes to:
    - cmd_vel (geometry_msgs/Twist): Motor commands
    - lateral_error (std_msgs/Float32): Lateral error for debugging
    """

    def __init__(self):
        super().__init__('line_follower_node')

        # Declare all configurable parameters.
        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        
        # Line detection parameters (HSV color range).
        self.declare_parameter('line_color', 'black')  # 'white' or 'black'
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_lower', 0)
        self.declare_parameter('v_upper', 100)
        
        # Region of interest: detect line in the lower portion of the image.
        self.declare_parameter('roi_start_fraction', 0.4)   # Start at 40% of image height
        self.declare_parameter('roi_end_fraction', 1.0)     # End at 100% (bottom)
        
        # Control parameters (PID).
        self.declare_parameter('kp', 0.8)           # Proportional gain for angular velocity
        self.declare_parameter('ki', 0.08)          # Integral gain - reduces steady-state error
        self.declare_parameter('kd', 0.15)          # Derivative gain - improves stability
        self.declare_parameter('v_base', 0.25)      # Base linear velocity (m/s)
        self.declare_parameter('w_max', 0.8)        # Max angular velocity (rad/s)
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
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.v_base = float(self.get_parameter('v_base').value)
        self.w_max = float(self.get_parameter('w_max').value)
        control_loop_rate = float(self.get_parameter('control_loop_rate').value)
        self.enable = self.get_parameter('enable_line_follower').value
        
        # Internal state (PID).
        self.velocity_scale = 1.0
        self.prev_error = 0.0
        self.current_error = 0.0
        self.integral_error = 0.0
        self.cv_bridge = CvBridge()
        self.dt = control_loop_rate  # Time step for PID integral and derivative
        
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
            f'Line follower initialized (PID): color={self.line_color}, '
            f'kp={self.kp}, ki={self.ki}, kd={self.kd}, v_base={self.v_base}, w_max={self.w_max}'
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
        """Detect line using row-by-row analysis.
        
        Analyzes each horizontal row in the ROI to find black line pixels.
        For multiple parallel lines, selects the one closest to image center.
        This approach is more robust than contour detection with parallel lines.
        """
        # Convert BGR to HSV.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Extract region of interest (lower portion of image).
        height = cv_image.shape[0]
        width = cv_image.shape[1]
        roi_start = int(height * self.roi_start_fraction)
        roi_end = int(height * self.roi_end_fraction)
        hsv_roi = hsv[roi_start:roi_end, :]
        
        # Create mask based on line color.
        if self.line_color.lower() == 'white':
            lower = np.array([self.h_lower, self.s_lower, self.v_lower], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, self.v_upper], dtype=np.uint8)
        else:  # black
            lower = np.array([self.h_lower, self.s_lower, self.v_lower], dtype=np.uint8)
            upper = np.array([self.h_upper, self.s_upper, self.v_upper], dtype=np.uint8)
        
        mask = cv2.inRange(hsv_roi, lower, upper)
        
        image_center_x = width / 2.0
        line_positions = []  # Store detected line x-positions for each row
        
        # Analyze each row to find line position closest to center.
        for row_idx in range(mask.shape[0]):
            row_pixels = mask[row_idx, :]
            
            # Find all pixels that are part of a line (white in mask).
            line_pixel_indices = np.where(row_pixels == 255)[0]
            
            if len(line_pixel_indices) > 0:
                # Find clusters of consecutive pixels (separate lines).
                # Calculate gaps between consecutive line pixels.
                gaps = np.diff(line_pixel_indices)
                
                # Threshold for gap size (minimum gap to consider it a separate line).
                # Adjust based on expected line width in pixels.
                gap_threshold = 10
                
                # Find cluster boundaries.
                gap_positions = np.where(gaps > gap_threshold)[0]
                
                if len(gap_positions) > 0:
                    # Multiple clusters detected; split them.
                    cluster_starts = [line_pixel_indices[0]]
                    cluster_ends = []
                    
                    for gap_pos in gap_positions:
                        cluster_ends.append(line_pixel_indices[gap_pos])
                        cluster_starts.append(line_pixel_indices[gap_pos + 1])
                    cluster_ends.append(line_pixel_indices[-1])
                    
                    clusters = list(zip(cluster_starts, cluster_ends))
                    
                    # Find cluster closest to image center.
                    closest_cluster = min(clusters, 
                                        key=lambda c: abs((c[0] + c[1]) / 2.0 - image_center_x))
                    cluster_center = (closest_cluster[0] + closest_cluster[1]) / 2.0
                else:
                    # Single cluster; take its center.
                    cluster_center = np.mean(line_pixel_indices)
                
                line_positions.append(cluster_center)
        
        # Calculate overall line position from collected row positions.
        if line_positions:
            line_x = np.mean(line_positions)
            
            # Normalize lateral error to [-1, 1].
            error = (line_x - image_center_x) / (image_center_x + 1e-6)
            error = np.clip(error, -1.0, 1.0)
            
            self.current_error = error
        else:
            # No line detected.
            self.current_error = 0.0

    def control_loop(self):
        """PID controller to generate cmd_vel based on lateral error."""
        if not self.enable:
            # Publish zero velocity if disabled.
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        error = self.current_error
        
        # ===== PID Control =====
        # Proportional term
        p_term = self.kp * error
        
        # Integral accumulation (prevents steady-state error).
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -1.0, 1.0)  # Anti-windup
        i_term = self.ki * self.integral_error
        
        # Derivative term (reduces overshoot and improves stability).
        derivative = (error - self.prev_error) / (self.dt + 1e-6)
        d_term = self.kd * derivative
        self.prev_error = error
        
        # Calculate angular velocity from PID.
        w = p_term + i_term + d_term
        w = np.clip(w * self.velocity_scale, -self.w_max, self.w_max)
        
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

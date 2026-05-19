"""Line follower node with PD control."""

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class LineFollower(Node):
    
    
    def __init__(self):
        super().__init__('line_follower_node')

        # ROS topics
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('velocity_scale_topic', 'velocity_scale')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # Line detection
        self.declare_parameter('line_color', 'black')

        # HSV thresholds for black line
        self.declare_parameter('h_lower', 0)
        self.declare_parameter('h_upper', 180)
        self.declare_parameter('s_lower', 0)
        self.declare_parameter('s_upper', 255)
        self.declare_parameter('v_lower', 0)
        self.declare_parameter('v_upper', 90)

        # ROI: lower image band
        self.declare_parameter('roi_start_fraction', 0.66)
        self.declare_parameter('roi_end_fraction', 0.96)

        # Detection filtering
        self.declare_parameter('min_line_area', 100.0)
        self.declare_parameter('morphology_kernel_size', 5)

        # PD control
        self.declare_parameter('kp', 0.75)
        self.declare_parameter('kd', 0.45)
        self.declare_parameter('w_max', 1.25)

        # Adaptive speed
        self.declare_parameter('v_base', 0.32)
        self.declare_parameter('v_min', 0.16)
        self.declare_parameter('v_max', 0.52)
        self.declare_parameter('slow_down_error', 0.32)

        # Inertia control: limit acceleration/deceleration per control cycle
        self.declare_parameter('max_accel_step', 0.020)
        self.declare_parameter('max_decel_step', 0.030)

        # Direction correction
        # If the robot turns the wrong way, change this to 1.0 in YAML.
        self.declare_parameter('steering_sign', -1.0)

        # Behavior when line is lost
        self.declare_parameter('stop_when_line_lost', True)

        # Control loop period
        self.declare_parameter('control_loop_rate', 0.033)

        # Enable/disable
        self.declare_parameter('enable_line_follower', True)

        # Read parameters
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.velocity_scale_topic = str(self.get_parameter('velocity_scale_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.line_color = str(self.get_parameter('line_color').value)

        self.h_lower = int(self.get_parameter('h_lower').value)
        self.h_upper = int(self.get_parameter('h_upper').value)
        self.s_lower = int(self.get_parameter('s_lower').value)
        self.s_upper = int(self.get_parameter('s_upper').value)
        self.v_lower = int(self.get_parameter('v_lower').value)
        self.v_upper = int(self.get_parameter('v_upper').value)

        self.roi_start_fraction = float(self.get_parameter('roi_start_fraction').value)
        self.roi_end_fraction = float(self.get_parameter('roi_end_fraction').value)

        self.min_line_area = float(self.get_parameter('min_line_area').value)

        kernel_size = int(self.get_parameter('morphology_kernel_size').value)
        if kernel_size < 3:
            kernel_size = 3
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.v_base = float(self.get_parameter('v_base').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.slow_down_error = float(self.get_parameter('slow_down_error').value)

        self.max_accel_step = float(self.get_parameter('max_accel_step').value)
        self.max_decel_step = float(self.get_parameter('max_decel_step').value)

        self.steering_sign = float(self.get_parameter('steering_sign').value)
        self.stop_when_line_lost = bool(self.get_parameter('stop_when_line_lost').value)

        control_loop_rate = float(self.get_parameter('control_loop_rate').value)
        self.enable = bool(self.get_parameter('enable_line_follower').value)

        # Internal state
        self.velocity_scale = 1.0
        self.prev_error = 0.0
        self.current_error = 0.0
        self.current_linear_x = 0.0
        self.line_detected = False
        self.prev_line_center_x = None

        self.cv_bridge = CvBridge()

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_best_effort,
        )

        self.create_subscription(
            Float32,
            self.velocity_scale_topic,
            self.velocity_scale_cb,
            qos_best_effort,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            qos_reliable,
        )

        self.error_pub = self.create_publisher(
            Float32,
            'lateral_error',
            qos_best_effort,
        )

        self.create_timer(control_loop_rate, self.control_loop)

        self.get_logger().info(
            f'Line follower initialized | '
            f'kp={self.kp}, kd={self.kd}, '
            f'v_min={self.v_min}, v_max={self.v_max}, '
            f'w_max={self.w_max}, steering_sign={self.steering_sign}'
        )

    def image_cb(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
            self.process_image(cv_image)

        except Exception as exc:
            self.get_logger().error(f'Image processing error: {exc}')

    def velocity_scale_cb(self, msg: Float32):
        self.velocity_scale = float(msg.data)

    def process_image(self, cv_image):
        """Detect the line inside a lower ROI and compute normalized error."""

        height, width = cv_image.shape[:2]

        roi_start = int(height * self.roi_start_fraction)
        roi_end = int(height * self.roi_end_fraction)

        roi_start = max(0, min(height - 1, roi_start))
        roi_end = max(roi_start + 1, min(height, roi_end))

        roi = cv_image[roi_start:roi_end, 0:width]

        line_color = self.line_color.lower().strip()

        if line_color == 'black':
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

            mask = cv2.adaptiveThreshold(
                gray_roi,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                31,
                7,
            )

        elif line_color == 'white':
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

            mask = cv2.adaptiveThreshold(
                gray_roi,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                7,
            )

        else:
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            hsv_roi = cv2.GaussianBlur(hsv_roi, (5, 5), 0)

            lower = np.array(
                [self.h_lower, self.s_lower, self.v_lower],
                dtype=np.uint8,
            )

            upper = np.array(
                [self.h_upper, self.s_upper, self.v_upper],
                dtype=np.uint8,
            )

            mask = cv2.inRange(hsv_roi, lower, upper)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel,
            iterations=1,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.kernel,
            iterations=2,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            self.line_detected = False
            return

        # Filter contours by minimum area
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_line_area]

        if not valid:
            self.line_detected = False
            return

        # Compute centroid for each valid contour
        centroids = []
        for c in valid:
            m = cv2.moments(c)
            if m.get('m00', 0) == 0:
                continue
            cx = float(m['m10'] / m['m00'])
            cy = float(m['m01'] / m['m00'])
            area = float(cv2.contourArea(c))
            centroids.append((c, cx, cy, area))

        if not centroids:
            self.line_detected = False
            return

        # Choose contour based on continuity: prefer centroid close to previous detection.
        if self.prev_line_center_x is not None:
            # score = distance to previous centroid (lower is better), break ties by larger area
            best = min(
                centroids,
                key=lambda tup: (abs(tup[1] - self.prev_line_center_x), -tup[3]),
            )
            chosen_contour, line_center_x, _, area = best
        else:
            # No history: pick largest area
            chosen_contour, line_center_x, _, area = max(centroids, key=lambda tup: tup[3])

        camera_center_x = width / 2.0

        # Error lateral normalizado:
        # -1.0 = izquierda extrema
        #  0.0 = centro
        #  1.0 = derecha extrema
        error = (line_center_x - camera_center_x) / camera_center_x
        error = float(np.clip(error, -1.0, 1.0))

        # Save previous centroid (in pixels) to keep continuity across frames
        try:
            self.prev_line_center_x = float(line_center_x)
        except Exception:
            self.prev_line_center_x = None

        self.current_error = error
        self.line_detected = True

    def control_loop(self):
        if not self.enable:
            self.current_linear_x = 0.0
            self.publish_cmd(0.0, 0.0)
            return

        if not self.line_detected:
            if self.stop_when_line_lost:
                # Mantener velocidad actual en lugar de frenar abruptamente
                # para pasar curvas cerradas
                self.current_linear_x = self._smooth_speed(self.current_linear_x * 0.85)
                self.publish_cmd(self.current_linear_x, 0.0)
            else:
                self.publish_cmd(0.0, 0.0)

            self.publish_error(self.current_error)
            return

        error = self.current_error
        error_derivative = error - self.prev_error
        self.prev_error = error

        # Control PD:
        pd_output = self.kp * error + self.kd * error_derivative

        angular_z = self.steering_sign * pd_output

        angular_z = float(
            np.clip(
                angular_z,
                -self.w_max,
                self.w_max,
            )
        )

        error_abs = abs(error)

        if self.slow_down_error <= 0.0:
            speed_factor = 1.0
        else:
            speed_factor = 1.0 - min(error_abs / self.slow_down_error, 1.0)

        target_linear_x = self.v_min + (self.v_max - self.v_min) * speed_factor

        target_linear_x *= self.velocity_scale
        angular_z *= self.velocity_scale

        if self.velocity_scale < 1.0:
            old_max_decel_step = self.max_decel_step
            self.max_decel_step = max(self.max_decel_step, 0.12)
            linear_x = self._smooth_speed(target_linear_x)
            self.max_decel_step = old_max_decel_step
        else:
            linear_x = self._smooth_speed(target_linear_x)


        self.publish_cmd(linear_x, angular_z)
        self.publish_error(error)

    def _smooth_speed(self, target_linear_x):
        """Limit acceleration and deceleration to reduce inertia problems."""

        delta_v = target_linear_x - self.current_linear_x

        if delta_v > self.max_accel_step:
            delta_v = self.max_accel_step

        elif delta_v < -self.max_decel_step:
            delta_v = -self.max_decel_step

        self.current_linear_x += delta_v

        self.current_linear_x = float(
            np.clip(
                self.current_linear_x,
                0.0,
                self.v_max,
            )
        )

        return self.current_linear_x

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def publish_error(self, error):
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
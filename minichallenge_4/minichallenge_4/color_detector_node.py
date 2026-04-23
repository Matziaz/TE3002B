#!/usr/bin/env python3
"""Robust traffic light color detector based on HSV segmentation."""

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        # Red thresholds
        self.declare_parameter('red_lower_h', 0)
        self.declare_parameter('red_upper_h', 10)
        self.declare_parameter('red_lower_h2', 170)
        self.declare_parameter('red_upper_h2', 180)
        self.declare_parameter('red_lower_s', 50)
        self.declare_parameter('red_upper_s', 255)
        self.declare_parameter('red_lower_v', 50)
        self.declare_parameter('red_upper_v', 255)

        # Yellow thresholds
        self.declare_parameter('yellow_lower_h', 20)
        self.declare_parameter('yellow_upper_h', 40)
        self.declare_parameter('yellow_lower_s', 50)
        self.declare_parameter('yellow_upper_s', 255)
        self.declare_parameter('yellow_lower_v', 50)
        self.declare_parameter('yellow_upper_v', 255)

        # Green thresholds
        self.declare_parameter('green_lower_h', 40)
        self.declare_parameter('green_upper_h', 80)
        self.declare_parameter('green_lower_s', 50)
        self.declare_parameter('green_upper_s', 255)
        self.declare_parameter('green_lower_v', 50)
        self.declare_parameter('green_upper_v', 255)

        # Robustness parameters
        self.declare_parameter('min_blob_size', 100)
        self.declare_parameter('dominance_ratio', 1.25)
        self.declare_parameter('morphology_kernel_size', 5)
        self.declare_parameter('history_length', 5)
        self.declare_parameter('camera_topic', '/image_raw')

        self.red_lower_h = int(self.get_parameter('red_lower_h').value)
        self.red_upper_h = int(self.get_parameter('red_upper_h').value)
        self.red_lower_h2 = int(self.get_parameter('red_lower_h2').value)
        self.red_upper_h2 = int(self.get_parameter('red_upper_h2').value)
        self.red_lower_s = int(self.get_parameter('red_lower_s').value)
        self.red_upper_s = int(self.get_parameter('red_upper_s').value)
        self.red_lower_v = int(self.get_parameter('red_lower_v').value)
        self.red_upper_v = int(self.get_parameter('red_upper_v').value)

        self.yellow_lower_h = int(self.get_parameter('yellow_lower_h').value)
        self.yellow_upper_h = int(self.get_parameter('yellow_upper_h').value)
        self.yellow_lower_s = int(self.get_parameter('yellow_lower_s').value)
        self.yellow_upper_s = int(self.get_parameter('yellow_upper_s').value)
        self.yellow_lower_v = int(self.get_parameter('yellow_lower_v').value)
        self.yellow_upper_v = int(self.get_parameter('yellow_upper_v').value)

        self.green_lower_h = int(self.get_parameter('green_lower_h').value)
        self.green_upper_h = int(self.get_parameter('green_upper_h').value)
        self.green_lower_s = int(self.get_parameter('green_lower_s').value)
        self.green_upper_s = int(self.get_parameter('green_upper_s').value)
        self.green_lower_v = int(self.get_parameter('green_lower_v').value)
        self.green_upper_v = int(self.get_parameter('green_upper_v').value)

        self.min_blob_size = float(self.get_parameter('min_blob_size').value)
        self.dominance_ratio = float(self.get_parameter('dominance_ratio').value)
        kernel_size = int(self.get_parameter('morphology_kernel_size').value)
        self.history_length = int(self.get_parameter('history_length').value)
        camera_topic = self.get_parameter('camera_topic').value

        self.bridge = cv_bridge.CvBridge()
        self.detection_history = []
        self.last_detected_color = 'unknown'
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.color_pub = self.create_publisher(String, 'detected_color', qos)
        self.confidence_pub = self.create_publisher(Float32MultiArray, 'color_confidence', qos)
        self.debug_pub = self.create_publisher(Image, 'color_debug_image', qos)
        self.create_subscription(Image, camera_topic, self.image_callback, qos)

        self.get_logger().info(f'Color detector listening on: {camera_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            red_mask = self._detect_red(hsv)
            yellow_mask = self._detect_yellow(hsv)
            green_mask = self._detect_green(hsv)

            red_area = self._largest_blob_area(red_mask)
            yellow_area = self._largest_blob_area(yellow_mask)
            green_area = self._largest_blob_area(green_mask)

            detected_color, scores = self._determine_color(red_area, yellow_area, green_area)

            self.detection_history.append(detected_color)
            if len(self.detection_history) > self.history_length:
                self.detection_history.pop(0)
            smoothed_color = self._smooth_detection(self.detection_history)

            color_msg = String()
            color_msg.data = smoothed_color
            self.color_pub.publish(color_msg)

            conf_msg = Float32MultiArray()
            conf_msg.data = [float(scores['red']), float(scores['yellow']), float(scores['green'])]
            self.confidence_pub.publish(conf_msg)

            debug_image = self._create_debug_image(
                cv_image, red_mask, yellow_mask, green_mask, smoothed_color,
                red_area, yellow_area, green_area,
            )
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
            self.last_detected_color = smoothed_color
        except Exception as exc:
            self.get_logger().error(f'Error in image processing: {exc}')

    def _detect_red(self, hsv):
        lower_red1 = np.array([self.red_lower_h, self.red_lower_s, self.red_lower_v])
        upper_red1 = np.array([self.red_upper_h, self.red_upper_s, self.red_upper_v])
        lower_red2 = np.array([self.red_lower_h2, self.red_lower_s, self.red_lower_v])
        upper_red2 = np.array([self.red_upper_h2, self.red_upper_s, self.red_upper_v])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        return self._postprocess_mask(cv2.bitwise_or(mask1, mask2))

    def _detect_yellow(self, hsv):
        lower = np.array([self.yellow_lower_h, self.yellow_lower_s, self.yellow_lower_v])
        upper = np.array([self.yellow_upper_h, self.yellow_upper_s, self.yellow_upper_v])
        return self._postprocess_mask(cv2.inRange(hsv, lower, upper))

    def _detect_green(self, hsv):
        lower = np.array([self.green_lower_h, self.green_lower_s, self.green_lower_v])
        upper = np.array([self.green_upper_h, self.green_upper_s, self.green_upper_v])
        return self._postprocess_mask(cv2.inRange(hsv, lower, upper))

    def _postprocess_mask(self, mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morphology_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel)
        return mask

    def _largest_blob_area(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        return float(max(cv2.contourArea(contour) for contour in contours))

    def _determine_color(self, red_area, yellow_area, green_area):
        areas = {'red': red_area, 'yellow': yellow_area, 'green': green_area}
        ranked = sorted(areas.items(), key=lambda item: item[1], reverse=True)
        winner, winner_area = ranked[0]
        second_area = ranked[1][1]
        total_area = red_area + yellow_area + green_area
        scores = {
            color: (area / total_area) if total_area > 0.0 else 0.0
            for color, area in areas.items()
        }

        enough_area = winner_area >= self.min_blob_size
        dominant_enough = winner_area >= (self.dominance_ratio * max(second_area, 1.0))

        if enough_area and dominant_enough:
            return winner, scores
        return 'unknown', scores

    def _smooth_detection(self, history):
        if not history:
            return 'unknown'
        valid = [color for color in history if color != 'unknown']
        if not valid:
            return 'unknown'
        counts = {color: valid.count(color) for color in ('red', 'yellow', 'green')}
        winner = max(counts, key=counts.get)
        return winner if counts[winner] >= max(1, len(valid) // 2 + 1) else 'unknown'

    def _create_debug_image(self, image, red_mask, yellow_mask, green_mask, detected_color,
                            red_area, yellow_area, green_area):
        debug = image.copy()
        mask_debug = np.zeros_like(debug)
        mask_debug[:, :, 2] = red_mask
        mask_debug[:, :, 1] = cv2.bitwise_or(yellow_mask, green_mask)
        overlay = cv2.addWeighted(debug, 0.7, mask_debug, 0.3, 0)
        cv2.putText(overlay, f'Detected: {detected_color.upper()}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(overlay, f'Areas R/Y/G: {red_area:.0f}/{yellow_area:.0f}/{green_area:.0f}',
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return overlay


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

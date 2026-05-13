#!/usr/bin/env python3
"""Traffic-light color detector with noise filtering and square area gating.

This node subscribes to a camera image, segments red/yellow/green regions in HSV,
selects the largest valid blob, draws a square bounding box around it, and only
publishes a color when the square area is large enough to be considered relevant
for the robot.
"""

# dataclass is used only to store one detection result in a clean structure.
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# OpenCV handles the image processing operations such as HSV conversion, masks,
# morphology operations, contour extraction, and debug drawing.
import cv2
# cv_bridge converts ROS Image messages into OpenCV images and back.
import cv_bridge
# NumPy is used for array operations and HSV threshold ranges.
import numpy as np
# rclpy is the Python client library used to create ROS 2 nodes.
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String


@dataclass
class Detection:
    # This structure groups the result of one detected colored object.
    # It stores the detected color, the contour size, the square used for
    # visualization, and the center of the object in the image.
    color: str
    contour_area: float
    square_area: float
    x: int
    y: int
    side: int
    center_x: int
    center_y: int
    score: float = 0.0


class ColorDetector(Node):
    # This node is responsible only for vision. It does not move the robot.
    # Its output is the detected traffic-light color and useful debug data.
    def __init__(self):
        super().__init__('color_detector_node')

        # Camera/topic parameters
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)

        # Red thresholds. Red is split in two hue ranges in OpenCV HSV.
        self.declare_parameter('red_lower_h', 0)
        self.declare_parameter('red_upper_h', 8)
        self.declare_parameter('red_lower_h2', 172)
        self.declare_parameter('red_upper_h2', 179)
        self.declare_parameter('red_lower_s', 90)
        self.declare_parameter('red_upper_s', 255)
        self.declare_parameter('red_lower_v', 80)
        self.declare_parameter('red_upper_v', 255)

        # Yellow thresholds. These are intentionally tighter than before to avoid
        # detecting white/gray reflections as yellow.
        self.declare_parameter('yellow_lower_h', 22)
        self.declare_parameter('yellow_upper_h', 35)
        self.declare_parameter('yellow_lower_s', 90)
        self.declare_parameter('yellow_upper_s', 255)
        self.declare_parameter('yellow_lower_v', 90)
        self.declare_parameter('yellow_upper_v', 255)

        # Green thresholds. These are tighter to reduce false green speckles.
        self.declare_parameter('green_lower_h', 48)
        self.declare_parameter('green_upper_h', 78)
        self.declare_parameter('green_lower_s', 80)
        self.declare_parameter('green_upper_s', 255)
        self.declare_parameter('green_lower_v', 70)
        self.declare_parameter('green_upper_v', 255)

        # Noise/area filtering. The detector uses contour area to remove small
        # noise, but publishes/decides using the square bounding-box area.
        self.declare_parameter('min_contour_area', 1200.0)
        self.declare_parameter('min_square_area', 8000.0)
        self.declare_parameter('dominance_ratio', 1.35)
        self.declare_parameter('morphology_kernel_size', 7)
        self.declare_parameter('morphology_open_iterations', 2)
        self.declare_parameter('morphology_close_iterations', 2)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('history_length', 5)
        self.declare_parameter('debug_log_every_n_frames', 5)

        # Optional ROI. Keep full image by default. Values are normalized [0, 1].
        # Useful if the traffic light is expected in the center of the image.
        self.declare_parameter('use_roi', False)
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.0)
        self.declare_parameter('roi_y_min', 0.0)
        self.declare_parameter('roi_y_max', 1.0)

        # Read the final parameter values after declaration. These values can come
        # from the YAML files loaded by the launch file, so the code does not need
        # to be edited when tuning thresholds or topics.
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)

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

        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.min_square_area = float(self.get_parameter('min_square_area').value)
        self.dominance_ratio = float(self.get_parameter('dominance_ratio').value)
        self.open_iterations = int(self.get_parameter('morphology_open_iterations').value)
        self.close_iterations = int(self.get_parameter('morphology_close_iterations').value)
        self.blur_kernel_size = int(self.get_parameter('blur_kernel_size').value)
        self.history_length = int(self.get_parameter('history_length').value)
        self.debug_log_every_n_frames = int(self.get_parameter('debug_log_every_n_frames').value)

        self.use_roi = bool(self.get_parameter('use_roi').value)
        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)

        # Force the morphology kernel to be a valid odd size. This prevents
        # invalid OpenCV kernels and keeps the filtering behavior consistent.
        kernel_size = int(self.get_parameter('morphology_kernel_size').value)
        if kernel_size < 3:
            kernel_size = 3
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (kernel_size, kernel_size),
        )

        if self.blur_kernel_size < 3:
            self.blur_kernel_size = 3
        if self.blur_kernel_size % 2 == 0:
            self.blur_kernel_size += 1

        # Runtime state used by the detector. The history list smooths the output
        # across several frames so one noisy frame does not immediately change
        # the detected traffic-light command.
        self.bridge = cv_bridge.CvBridge()
        self.detection_history = []
        self.mission_complete = False
        self.completion_debug_published = False
        self.image_frame_count = 0

        # A BEST_EFFORT QoS is used because camera streams and debug images are
        # high-frequency data. Dropping an old frame is better than blocking the node.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Published topics:
        # - detected_color: final color decision used by the traffic controller.
        # - color_confidence: numeric confidence for red, yellow, and green.
        # - detected_color_area: size of the chosen color region, useful for tuning.
        # - color_debug_image: camera image with visual overlays for rqt_image_view.
        self.color_pub = self.create_publisher(String, 'detected_color', qos)
        self.confidence_pub = self.create_publisher(Float32MultiArray, 'color_confidence', qos)
        self.object_area_pub = self.create_publisher(Float32, 'detected_color_area', qos)
        self.debug_pub = self.create_publisher(Image, 'color_debug_image', qos)

        self.create_subscription(Image, self.camera_topic, self.image_callback, qos)
        self.create_subscription(String, 'nav_status', self.nav_status_callback, qos)

        self.get_logger().info(f'Color detector listening on: {self.camera_topic}')
        self.get_logger().info(
            f'Area gate: min_contour_area={self.min_contour_area:.0f}, '
            f'min_square_area={self.min_square_area:.0f}, dominance_ratio={self.dominance_ratio:.2f}'
        )

    # The navigation node publishes mission_complete when the route is done.
    # After that, this node stops publishing normal detection overlays to avoid
    # confusing the final result with unnecessary debug frames.
    def nav_status_callback(self, msg):
        if msg.data == 'mission_complete' and not self.mission_complete:
            self.mission_complete = True
            self.get_logger().info('Route completed. Color detection debug output stopped.')

    # Main image-processing callback. Every incoming ROS image is converted to
    # OpenCV format, filtered by color, evaluated, smoothed, and published.
    def image_callback(self, msg):
        try:
            # Convert from ROS Image to an OpenCV BGR image. OpenCV uses BGR by
            # default, so the desired encoding is set to bgr8.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = self._resize_frame(cv_image)

            # If the path is already finished, publish one final debug frame and
            # return without running the color detector again.
            if self.mission_complete:
                if not self.completion_debug_published:
                    debug_image = self._create_completion_debug_image(cv_image)
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
                    self.completion_debug_published = True
                return

            self.image_frame_count += 1

            # Convert BGR to HSV because hue/saturation/value thresholds are more
            # stable for color segmentation than raw RGB/BGR values.
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (self.blur_kernel_size, self.blur_kernel_size), 0)

            # Build one mask per traffic-light color. Each mask is cleaned using
            # morphology before contours are selected.
            roi_bounds = self._get_roi_bounds(cv_image)
            red_mask = self._detect_red(hsv, roi_bounds)
            yellow_mask = self._detect_yellow(hsv, roi_bounds)
            green_mask = self._detect_green(hsv, roi_bounds)

            # For each color, keep only the largest valid object. This avoids
            # reacting to small speckles or multiple weak detections in the image.
            detections = {
                'red': self._largest_valid_detection(red_mask, 'red'),
                'yellow': self._largest_valid_detection(yellow_mask, 'yellow'),
                'green': self._largest_valid_detection(green_mask, 'green'),
            }

            # Choose the strongest color using area and dominance. The winner is
            # later smoothed with previous detections to reduce flickering.
            detected_color, scores, winner_detection = self._determine_color(detections)

            self.detection_history.append(detected_color)
            if len(self.detection_history) > self.history_length:
                self.detection_history.pop(0)
            smoothed_color = self._smooth_detection(self.detection_history)

            # Only draw the smoothed/winning object if it is still the current winner.
            draw_detection = winner_detection if smoothed_color == detected_color else None

            self._publish_detection(smoothed_color, scores, draw_detection)

            if self.debug_log_every_n_frames > 0 and self.image_frame_count % self.debug_log_every_n_frames == 0:
                self._log_detection(detections, detected_color, smoothed_color, scores, draw_detection)

            debug_image = self._create_debug_image(
                cv_image,
                smoothed_color,
                scores,
                detections,
                draw_detection,
                roi_bounds,
            )
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))

        except Exception as exc:
            self.get_logger().error(f'Error in image processing: {exc}')

    def _resize_frame(self, image):
        if self.resize_width > 0 and self.resize_height > 0:
            return cv2.resize(image, (self.resize_width, self.resize_height))
        return image

    def _get_roi_bounds(self, image) -> Tuple[int, int, int, int]:
        h, w = image.shape[:2]
        if not self.use_roi:
            return 0, 0, w, h

        x_min = int(max(0.0, min(1.0, self.roi_x_min)) * w)
        x_max = int(max(0.0, min(1.0, self.roi_x_max)) * w)
        y_min = int(max(0.0, min(1.0, self.roi_y_min)) * h)
        y_max = int(max(0.0, min(1.0, self.roi_y_max)) * h)

        if x_max <= x_min or y_max <= y_min:
            return 0, 0, w, h
        return x_min, y_min, x_max, y_max

    # Remove detections outside the selected ROI by zeroing out those mask pixels.
    def _apply_roi(self, mask, roi_bounds):
        x_min, y_min, x_max, y_max = roi_bounds
        if x_min == 0 and y_min == 0 and x_max == mask.shape[1] and y_max == mask.shape[0]:
            return mask
        roi_mask = np.zeros_like(mask)
        roi_mask[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
        return roi_mask

    # Red wraps around the HSV hue scale, so two separate ranges are combined.
    def _detect_red(self, hsv, roi_bounds):
        lower_red1 = np.array([self.red_lower_h, self.red_lower_s, self.red_lower_v])
        upper_red1 = np.array([self.red_upper_h, self.red_upper_s, self.red_upper_v])
        lower_red2 = np.array([self.red_lower_h2, self.red_lower_s, self.red_lower_v])
        upper_red2 = np.array([self.red_upper_h2, self.red_upper_s, self.red_upper_v])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        return self._postprocess_mask(self._apply_roi(cv2.bitwise_or(mask1, mask2), roi_bounds))

    # Yellow detection uses a single HSV range and then applies the same cleanup.
    def _detect_yellow(self, hsv, roi_bounds):
        lower = np.array([self.yellow_lower_h, self.yellow_lower_s, self.yellow_lower_v])
        upper = np.array([self.yellow_upper_h, self.yellow_upper_s, self.yellow_upper_v])
        return self._postprocess_mask(self._apply_roi(cv2.inRange(hsv, lower, upper), roi_bounds))

    # Green detection also uses one HSV range with the common mask cleanup.
    def _detect_green(self, hsv, roi_bounds):
        lower = np.array([self.green_lower_h, self.green_lower_s, self.green_lower_v])
        upper = np.array([self.green_upper_h, self.green_upper_s, self.green_upper_v])
        return self._postprocess_mask(self._apply_roi(cv2.inRange(hsv, lower, upper), roi_bounds))

    def _postprocess_mask(self, mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel, iterations=self.open_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morphology_kernel, iterations=self.close_iterations)
        return mask

    def _largest_valid_detection(self, mask, color: str) -> Optional[Detection]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_detection = None
        best_square_area = 0.0

        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area < self.min_contour_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            side = int(max(w, h))
            square_area = float(side * side)
            if square_area < self.min_square_area:
                continue

            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            square_x = int(center_x - side / 2)
            square_y = int(center_y - side / 2)

            if square_area > best_square_area:
                best_square_area = square_area
                best_detection = Detection(
                    color=color,
                    contour_area=contour_area,
                    square_area=square_area,
                    x=square_x,
                    y=square_y,
                    side=side,
                    center_x=center_x,
                    center_y=center_y,
                )

        return best_detection

    # Decide the final color for the current frame. A color is accepted only if
    # its square area is large enough and it dominates the second-best color.
    def _determine_color(self, detections: Dict[str, Optional[Detection]]):
        areas = {
            color: (det.square_area if det is not None else 0.0)
            for color, det in detections.items()
        }
        total_area = sum(areas.values())
        scores = {
            color: (area / total_area) if total_area > 0.0 else 0.0
            for color, area in areas.items()
        }

        ranked = sorted(areas.items(), key=lambda item: item[1], reverse=True)
        winner_color, winner_area = ranked[0]
        second_area = ranked[1][1]

        if winner_area <= 0.0:
            return 'unknown', scores, None

        dominant_enough = winner_area >= self.dominance_ratio * max(second_area, 1.0)
        if not dominant_enough:
            return 'unknown', scores, None

        winner_detection = detections[winner_color]
        if winner_detection is not None:
            winner_detection.score = scores[winner_color]
        return winner_color, scores, winner_detection

    # Smooth the detected color using a short history window. This reduces rapid
    # changes caused by noise, lighting changes, or momentary missed detections.
    def _smooth_detection(self, history):
        if not history:
            return 'unknown'
        valid = [color for color in history if color != 'unknown']
        if not valid:
            return 'unknown'
        counts = {color: valid.count(color) for color in ('red', 'yellow', 'green')}
        winner = max(counts, key=counts.get)
        return winner if counts[winner] >= max(1, len(valid) // 2 + 1) else 'unknown'

    def _publish_detection(self, detected_color, scores, detection):
        color_msg = String()
        color_msg.data = detected_color
        self.color_pub.publish(color_msg)

        conf_msg = Float32MultiArray()
        conf_msg.data = [float(scores['red']), float(scores['yellow']), float(scores['green'])]
        self.confidence_pub.publish(conf_msg)

        area_msg = Float32()
        area_msg.data = float(detection.square_area if detection is not None else 0.0)
        self.object_area_pub.publish(area_msg)

    def _log_detection(self, detections, detected_color, smoothed_color, scores, detection):
        red_area = detections['red'].square_area if detections['red'] is not None else 0.0
        yellow_area = detections['yellow'].square_area if detections['yellow'] is not None else 0.0
        green_area = detections['green'].square_area if detections['green'] is not None else 0.0
        selected_area = detection.square_area if detection is not None else 0.0
        self.get_logger().info(
            f'Square areas R/Y/G: {red_area:.0f}/{yellow_area:.0f}/{green_area:.0f} | '
            f'Raw: {detected_color} | Smoothed: {smoothed_color} | Selected area: {selected_area:.0f} | '
            f'Scores R={scores["red"]:.3f}, Y={scores["yellow"]:.3f}, G={scores["green"]:.3f}'
        )

    def _create_debug_image(self, image, detected_color, scores, detections, detection, roi_bounds):
        debug = image.copy()
        h, w = debug.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        color_map = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'unknown': (128, 128, 128),
        }
        color_bgr = color_map.get(detected_color, (128, 128, 128))

        # ROI boundary if enabled.
        if self.use_roi:
            x_min, y_min, x_max, y_max = roi_bounds
            cv2.rectangle(debug, (x_min, y_min), (x_max, y_max), (255, 255, 255), 1)

        # Draw only the selected valid object, not every speckle in the masks.
        if detection is not None:
            x1 = max(0, detection.x)
            y1 = max(0, detection.y)
            x2 = min(w - 1, detection.x + detection.side)
            y2 = min(h - 1, detection.y + detection.side)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color_bgr, 3)
            cv2.circle(debug, (detection.center_x, detection.center_y), 6, (255, 255, 255), -1)
            cv2.circle(debug, (detection.center_x, detection.center_y), 3, color_bgr, -1)
            cv2.putText(
                debug,
                f'{detected_color.upper()} area={int(detection.square_area)}',
                (x1, max(25, y1 - 10)),
                font,
                0.65,
                color_bgr,
                2,
                cv2.LINE_AA,
            )

        # Status panel.
        panel_h = 130
        cv2.rectangle(debug, (0, 0), (min(w, 430), panel_h), (0, 0, 0), -1)
        cv2.rectangle(debug, (0, 0), (min(w, 430), panel_h), color_bgr, 2)

        red_area = detections['red'].square_area if detections['red'] is not None else 0.0
        yellow_area = detections['yellow'].square_area if detections['yellow'] is not None else 0.0
        green_area = detections['green'].square_area if detections['green'] is not None else 0.0
        selected_area = detection.square_area if detection is not None else 0.0

        lines = [
            f'Detected: {detected_color.upper()}',
            f'Square area: {selected_area:.0f}  Min: {self.min_square_area:.0f}',
            f'Areas R/Y/G: {red_area:.0f}/{yellow_area:.0f}/{green_area:.0f}',
            f'Scores R={scores["red"]:.2f} Y={scores["yellow"]:.2f} G={scores["green"]:.2f}',
        ]
        y = 25
        for line in lines:
            cv2.putText(debug, line, (10, y), font, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            y += 28

        # Large color strip on the right.
        box_width = 70
        box_left = w - box_width
        cv2.rectangle(debug, (box_left, 0), (w, h), color_bgr, -1)
        cv2.rectangle(debug, (box_left, 0), (w, h), (255, 255, 255), 2)
        text = detected_color.upper()
        text_size = cv2.getTextSize(text, font, 0.85, 2)[0]
        text_x = box_left + max(2, (box_width - text_size[0]) // 2)
        text_y = h // 2
        cv2.putText(debug, text, (text_x, text_y), font, 0.85, (255, 255, 255), 2, cv2.LINE_AA)

        return debug

    def _create_completion_debug_image(self, image):
        debug = image.copy()
        h, w = debug.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_1 = 'ROUTE COMPLETED'
        text_2 = 'Mission complete - robot stopped'

        cv2.rectangle(debug, (0, 0), (w, h), (0, 0, 0), -1)
        cv2.rectangle(debug, (20, 20), (w - 20, h - 20), (255, 255, 255), 3)

        text_1_size = cv2.getTextSize(text_1, font, 1.2, 3)[0]
        text_2_size = cv2.getTextSize(text_2, font, 0.7, 2)[0]
        cv2.putText(debug, text_1, ((w - text_1_size[0]) // 2, h // 2 - 20), font, 1.2, (0, 255, 0), 3)
        cv2.putText(debug, text_2, ((w - text_2_size[0]) // 2, h // 2 + 30), font, 0.7, (255, 255, 255), 2)
        return debug


# Standard ROS 2 Python entry point: initialize ROS, spin the node, and cleanly
# shut it down when the process exits.
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

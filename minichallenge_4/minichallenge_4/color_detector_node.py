"""Traffic-light color detector with noise filtering and square area gating."""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String


@dataclass
class Detection:
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
    def __init__(self):
        super().__init__('color_detector_node')

        # Camera/topic parameters
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)

        # Red thresholds
        self.declare_parameter('red_lower_h', 0)
        self.declare_parameter('red_upper_h', 8)
        self.declare_parameter('red_lower_h2', 172)
        self.declare_parameter('red_upper_h2', 179)
        self.declare_parameter('red_lower_s', 90)
        self.declare_parameter('red_upper_s', 255)
        self.declare_parameter('red_lower_v', 80)
        self.declare_parameter('red_upper_v', 255)

        # Yellow thresholds
        self.declare_parameter('yellow_lower_h', 22)
        self.declare_parameter('yellow_upper_h', 35)
        self.declare_parameter('yellow_lower_s', 90)
        self.declare_parameter('yellow_upper_s', 255)
        self.declare_parameter('yellow_lower_v', 90)
        self.declare_parameter('yellow_upper_v', 255)

        # Green thresholds
        self.declare_parameter('green_lower_h', 48)
        self.declare_parameter('green_upper_h', 78)
        self.declare_parameter('green_lower_s', 80)
        self.declare_parameter('green_upper_s', 255)
        self.declare_parameter('green_lower_v', 70)
        self.declare_parameter('green_upper_v', 255)

        # Noise/area filtering
        self.declare_parameter('min_contour_area', 1200.0)
        self.declare_parameter('min_square_area', 30000.0)
        self.declare_parameter('dominance_ratio', 1.35)
        self.declare_parameter('morphology_kernel_size', 7)
        self.declare_parameter('morphology_open_iterations', 2)
        self.declare_parameter('morphology_close_iterations', 2)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('history_length', 5)
        self.declare_parameter('debug_log_every_n_frames', 5)

        # Optional ROI
        self.declare_parameter('use_roi', False)
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.0)
        self.declare_parameter('roi_y_min', 0.0)
        self.declare_parameter('roi_y_max', 1.0)

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

        self.bridge = cv_bridge.CvBridge()
        self.detection_history = []
        self.mission_complete = False
        self.completion_debug_published = False
        self.image_frame_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.color_pub = self.create_publisher(String, 'detected_color', qos)
        self.confidence_pub = self.create_publisher(Float32MultiArray, 'color_confidence', qos)
        self.object_area_pub = self.create_publisher(Float32, 'detected_color_area', qos)
        self.debug_pub = self.create_publisher(Image, 'color_debug_image', qos)

        self.create_subscription(Image, self.camera_topic, self.image_callback, qos)
        self.create_subscription(String, 'nav_status', self.nav_status_callback, qos)

        self.get_logger().info(f'Color detector listening on: {self.camera_topic}')
        self.get_logger().info(
            f'Area gate: min_contour_area={self.min_contour_area:.0f}, '
            f'min_square_area={self.min_square_area:.0f}, '
            f'dominance_ratio={self.dominance_ratio:.2f}'
        )

    def nav_status_callback(self, msg):
        if msg.data == 'mission_complete' and not self.mission_complete:
            self.mission_complete = True
            self.get_logger().info('Route completed. Color detection debug output stopped.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = self._resize_frame(cv_image)

            if self.mission_complete:
                if not self.completion_debug_published:
                    debug_image = self._create_completion_debug_image(cv_image)
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
                    self.completion_debug_published = True
                return

            self.image_frame_count += 1

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(
                hsv,
                (self.blur_kernel_size, self.blur_kernel_size),
                0,
            )

            roi_bounds = self._get_roi_bounds(cv_image)

            red_mask = self._detect_red(hsv, roi_bounds)
            yellow_mask = self._detect_yellow(hsv, roi_bounds)
            green_mask = self._detect_green(hsv, roi_bounds)

            detections = {
                'red': self._largest_valid_detection(red_mask, 'red'),
                'yellow': self._largest_valid_detection(yellow_mask, 'yellow'),
                'green': self._largest_valid_detection(green_mask, 'green'),
            }

            detected_color, scores, winner_detection = self._determine_color(detections)

            self.detection_history.append(detected_color)

            if len(self.detection_history) > self.history_length:
                self.detection_history.pop(0)

            smoothed_color = self._smooth_detection(self.detection_history)

            draw_detection = winner_detection if smoothed_color == detected_color else None

            self._publish_detection(smoothed_color, scores, draw_detection)

            if (
                self.debug_log_every_n_frames > 0
                and self.image_frame_count % self.debug_log_every_n_frames == 0
            ):
                self._log_detection(
                    detections,
                    detected_color,
                    smoothed_color,
                    scores,
                    draw_detection,
                )

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

    def _apply_roi(self, mask, roi_bounds):
        x_min, y_min, x_max, y_max = roi_bounds

        if (
            x_min == 0
            and y_min == 0
            and x_max == mask.shape[1]
            and y_max == mask.shape[0]
        ):
            return mask

        roi_mask = np.zeros_like(mask)
        roi_mask[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]

        return roi_mask

    def _detect_red(self, hsv, roi_bounds):
        lower_red1 = np.array([self.red_lower_h, self.red_lower_s, self.red_lower_v])
        upper_red1 = np.array([self.red_upper_h, self.red_upper_s, self.red_upper_v])

        lower_red2 = np.array([self.red_lower_h2, self.red_lower_s, self.red_lower_v])
        upper_red2 = np.array([self.red_upper_h2, self.red_upper_s, self.red_upper_v])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        return self._postprocess_mask(
            self._apply_roi(cv2.bitwise_or(mask1, mask2), roi_bounds)
        )

    def _detect_yellow(self, hsv, roi_bounds):
        lower = np.array([
            self.yellow_lower_h,
            self.yellow_lower_s,
            self.yellow_lower_v,
        ])

        upper = np.array([
            self.yellow_upper_h,
            self.yellow_upper_s,
            self.yellow_upper_v,
        ])

        return self._postprocess_mask(
            self._apply_roi(cv2.inRange(hsv, lower, upper), roi_bounds)
        )

    def _detect_green(self, hsv, roi_bounds):
        lower = np.array([
            self.green_lower_h,
            self.green_lower_s,
            self.green_lower_v,
        ])

        upper = np.array([
            self.green_upper_h,
            self.green_upper_s,
            self.green_upper_v,
        ])

        return self._postprocess_mask(
            self._apply_roi(cv2.inRange(hsv, lower, upper), roi_bounds)
        )

    def _postprocess_mask(self, mask):
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.morphology_kernel,
            iterations=self.open_iterations,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.morphology_kernel,
            iterations=self.close_iterations,
        )

        return mask

    def _largest_valid_detection(self, mask, color: str) -> Optional[Detection]:
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

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

    def _smooth_detection(self, history):
        if not history:
            return 'unknown'

        valid = [color for color in history if color != 'unknown']

        if not valid:
            return 'unknown'

        counts = {
            color: valid.count(color)
            for color in ('red', 'yellow', 'green')
        }

        winner = max(counts, key=counts.get)

        return winner if counts[winner] >= max(1, len(valid) // 2 + 1) else 'unknown'

    def _publish_detection(self, detected_color, scores, detection):
        color_msg = String()
        color_msg.data = detected_color
        self.color_pub.publish(color_msg)

        conf_msg = Float32MultiArray()
        conf_msg.data = [
            float(scores['red']),
            float(scores['yellow']),
            float(scores['green']),
        ]

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
            f'Raw: {detected_color} | '
            f'Smoothed: {smoothed_color} | '
            f'Selected area: {selected_area:.0f} | '
            f'Scores R={scores["red"]:.3f}, '
            f'Y={scores["yellow"]:.3f}, '
            f'G={scores["green"]:.3f}'
        )

    def _draw_follow_line(self, debug):
        h, w = debug.shape[:2]

        # Buscar la línea principalmente en la parte inferior de la imagen
        roi_y_start = int(h * 0.45)
        roi = debug[roi_y_start:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detectar línea negra
        _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        # Línea central de referencia de la cámara
        cv2.line(
            debug,
            (w // 2, roi_y_start),
            (w // 2, h),
            (255, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            "CENTER",
            (w // 2 + 8, roi_y_start + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if not contours:
            return debug

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 500:
            return debug

        # Convertir coordenadas del ROI a coordenadas de imagen completa
        largest[:, :, 1] += roi_y_start

        # Dibujar contorno de la línea detectada
        cv2.drawContours(
            debug,
            [largest],
            -1,
            (255, 0, 255),
            3,
        )

        moments = cv2.moments(largest)

        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            error = cx - (w // 2)

            # Punto central de la línea detectada
            cv2.circle(
                debug,
                (cx, cy),
                8,
                (0, 0, 255),
                -1,
            )

            # Línea vertical sobre el centro de la línea detectada
            cv2.line(
                debug,
                (cx, roi_y_start),
                (cx, h),
                (0, 0, 255),
                2,
            )

            cv2.putText(
                debug,
                "FOLLOW LINE",
                (max(10, cx - 90), max(30, cy - 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                debug,
                f"Line error: {error}",
                (10, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return debug

    def _create_debug_image(
        self,
        frame,
        detected_color,
        scores,
        detections,
        selected_detection,
        roi_bounds,
    ):
        debug = frame.copy()
        h, w = debug.shape[:2]

        # Dibujar la línea que sigue el robot
        debug = self._draw_follow_line(debug)

        font = cv2.FONT_HERSHEY_SIMPLEX

        color_map = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'unknown': (160, 160, 160),
        }

        selected_color_bgr = color_map.get(detected_color, (160, 160, 160))

        # Dibujar ROI si está activo
        if self.use_roi:
            x_min, y_min, x_max, y_max = roi_bounds

            cv2.rectangle(
                debug,
                (x_min, y_min),
                (x_max, y_max),
                (255, 255, 255),
                2,
            )

        # Dibujar cuadro de la detección seleccionada
        if selected_detection is not None:
            x = selected_detection.x
            y = selected_detection.y
            side = selected_detection.side

            cv2.rectangle(
                debug,
                (x, y),
                (x + side, y + side),
                color_map.get(selected_detection.color, (255, 255, 255)),
                3,
            )

            cv2.circle(
                debug,
                (selected_detection.center_x, selected_detection.center_y),
                5,
                color_map.get(selected_detection.color, (255, 255, 255)),
                -1,
            )

        red_area = int(detections['red'].square_area) if detections['red'] is not None else 0
        yellow_area = int(detections['yellow'].square_area) if detections['yellow'] is not None else 0
        green_area = int(detections['green'].square_area) if detections['green'] is not None else 0

        selected_area = int(selected_detection.square_area) if selected_detection is not None else 0

        # Panel negro del lado derecho
        panel_w = 390
        panel = np.zeros((h, panel_w, 3), dtype=np.uint8)

        # Franja de color arriba del panel
        cv2.rectangle(
            panel,
            (0, 0),
            (panel_w, 65),
            selected_color_bgr,
            -1,
        )

        cv2.putText(
            panel,
            detected_color.upper(),
            (20, 42),
            font,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        lines = [
            f"Detected: {detected_color.upper()}",
            f"Square area: {selected_area}",
            f"Min area: {int(self.min_square_area)}",
            f"Areas R/Y/G:",
            f"{red_area}/{yellow_area}/{green_area}",
            f"Scores:",
            f"R={scores['red']:.2f}",
            f"Y={scores['yellow']:.2f}",
            f"G={scores['green']:.2f}",
            "Line:",
            "Purple = detected line",
            "Red = line center",
            "White = camera center",
        ]

        y = 105

        for line in lines:
            cv2.putText(
                panel,
                line,
                (20, y),
                font,
                0.62,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            y += 33

        # Unir imagen original con panel derecho
        debug = np.hstack((debug, panel))

        return debug

    def _create_completion_debug_image(self, image):
        debug = image.copy()
        h, w = debug.shape[:2]

        font = cv2.FONT_HERSHEY_SIMPLEX

        text_1 = 'ROUTE COMPLETED'
        text_2 = 'Mission complete - robot stopped'

        cv2.rectangle(debug, (0, 0), (w, h), (0, 0, 0), -1)

        cv2.rectangle(
            debug,
            (20, 20),
            (w - 20, h - 20),
            (255, 255, 255),
            3,
        )

        text_1_size = cv2.getTextSize(text_1, font, 1.2, 3)[0]
        text_2_size = cv2.getTextSize(text_2, font, 0.7, 2)[0]

        cv2.putText(
            debug,
            text_1,
            ((w - text_1_size[0]) // 2, h // 2 - 20),
            font,
            1.2,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug,
            text_2,
            ((w - text_2_size[0]) // 2, h // 2 + 30),
            font,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return debug


def main(args=None):
    rclpy.init(args=args)

    node = ColorDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
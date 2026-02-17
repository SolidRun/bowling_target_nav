#!/usr/bin/env python3
"""
Vision Detection Node for Bowling Target Navigation
====================================================

ROS2 node that captures camera frames, runs object detection,
and publishes target pose for navigation.

Subscriptions:
    None (uses OpenCV for direct camera access)

Publications:
    /target_pose (geometry_msgs/PoseStamped) - Target position in base_link frame
    /target_detection (std_msgs/String) - JSON detection info for debugging
    /vision/image_raw (sensor_msgs/Image) - Optional: annotated camera image

Parameters:
    detector_type: "yolo" or "binary"
    model_path: Path to ONNX model (for YOLO)
    binary_path: Path to binary app (for binary detector)
    camera_index: Camera device index (default 0)
    target_class: Class name to track (default "Pins")
    conf_threshold: Minimum confidence (default 0.3)
    reference_box_height: Box height at reference distance (calibration)
    reference_distance: Reference distance in meters (calibration)
    publish_rate: Detection rate in Hz (default 10)
    frame_width: Camera frame width (default 320)
    frame_height: Camera frame height (default 240)

Usage:
    ros2 run bowling_target_nav vision_node
    ros2 run bowling_target_nav vision_node --ros-args -p detector_type:=yolo
"""

import json
import math
import os
import subprocess
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from bowling_target_nav.detectors import (
    YoloOnnxDetector as YoloDetector,
    DrpBinaryDetector as BinDetector,
    MockDetector as BinDetectorStub,
)
from bowling_target_nav.utils import DistanceEstimator


class VisionNode(Node):
    """
    ROS2 node for vision-based target detection.

    Captures frames from USB camera, runs detection (YOLO or binary),
    estimates target distance/angle, and publishes pose for navigation.
    """

    def __init__(self):
        super().__init__('vision_node')

        # Declare parameters
        self.declare_parameter('detector_type', 'yolo')
        self.declare_parameter('model_path', '')
        self.declare_parameter('binary_path', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('target_class', 'bowling-pins')
        self.declare_parameter('filter_classes', ['bowling-pins', 'Bowlingpin-bowlingball'])
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('reference_box_height', 100.0)
        self.declare_parameter('reference_distance', 1.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('horizontal_fov', 60.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('consecutive_misses_to_lose', 5)
        self.declare_parameter('camera_retry_count', 3)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('show_video', True)

        # Get parameters
        self.detector_type = self.get_parameter('detector_type').value
        self.model_path = self.get_parameter('model_path').value
        self.binary_path = self.get_parameter('binary_path').value
        self.camera_index = self.get_parameter('camera_index').value
        self.target_class = self.get_parameter('target_class').value
        self.filter_classes = list(self.get_parameter('filter_classes').value)
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.consecutive_misses_to_lose = self.get_parameter('consecutive_misses_to_lose').value
        self.camera_retry_count = self.get_parameter('camera_retry_count').value
        self.ref_box_height = self.get_parameter('reference_box_height').value
        self.ref_distance = self.get_parameter('reference_distance').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.horizontal_fov = self.get_parameter('horizontal_fov').value
        self.base_frame = self.get_parameter('base_frame').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.show_video = self.get_parameter('show_video').value

        # Initialize display window
        self.window_name = "Vision Node - Press 'q' to quit"
        if self.show_video:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.get_logger().info("Video display enabled")
            except cv2.error as e:
                self.get_logger().warn(f"Cannot open display window: {e}")
                self.get_logger().warn("Disabling video display. Set WAYLAND_DISPLAY and XDG_RUNTIME_DIR for Wayland.")
                self.show_video = False

        # Find model path if not specified
        if not self.model_path and self.detector_type == 'yolo':
            self.model_path = self._find_model_path()

        # Initialize detector
        self.detector = self._create_detector()

        # Initialize distance estimator
        self.estimator = DistanceEstimator(
            reference_box_height=self.ref_box_height,
            reference_distance=self.ref_distance,
            frame_width=self.frame_width,
            frame_height=self.frame_height,
            horizontal_fov=self.horizontal_fov
        )

        # Initialize camera
        self.cap = self._init_camera()

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', qos)
        self.detection_pub = self.create_publisher(String, '/target_detection', qos)

        # Timer for detection loop
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.detection_callback)

        # Stats
        self.frame_count = 0
        self.detection_count = 0

        # Temporal tracking state
        self._consecutive_misses = 0
        self._last_detection = None        # Last valid Detection object
        self._last_distance = float('inf')
        self._last_angle = 0.0
        self._tracking_active = False

        self.get_logger().info(f"Vision node started: detector={self.detector_type}, "
                               f"target_class={self.target_class}, "
                               f"filter_classes={self.filter_classes}, "
                               f"conf={self.conf_threshold}, nms={self.nms_threshold}, "
                               f"miss_tolerance={self.consecutive_misses_to_lose}")

    def _find_model_path(self) -> str:
        """Find YOLO model in standard locations."""
        search_paths = [
            # Installed location
            '/opt/ros/humble/share/bowling_target_nav/models/bowling_yolov5.onnx',
            # Development locations
            os.path.expanduser('~/ros2_ws/src/bowling_target_nav/models/bowling_yolov5.onnx'),
            '/tmp/bowling_yolov5.onnx',
            # Relative to package
            os.path.join(os.path.dirname(__file__), '..', '..', 'models', 'bowling_yolov5.onnx'),
        ]

        for path in search_paths:
            if os.path.exists(path):
                self.get_logger().info(f"Found model: {path}")
                return path

        self.get_logger().warn("No model found in standard locations")
        return ''

    def _create_detector(self):
        """Create and initialize detector based on configuration."""
        detector = None

        if self.detector_type == 'yolo':
            if not self.model_path or not os.path.exists(self.model_path):
                self.get_logger().error(f"YOLO model not found: {self.model_path}")
                return None

            detector = YoloDetector(
                model_path=self.model_path,
                confidence_threshold=self.conf_threshold,
                nms_threshold=self.nms_threshold,
                target_class=self.target_class,
                filter_classes=self.filter_classes,
            )

        elif self.detector_type == 'binary':
            if not self.binary_path:
                self.get_logger().warn("No binary path specified, using stub")
                detector = BinDetectorStub()
            else:
                detector = BinDetector(
                    binary_path=self.binary_path,
                    confidence_threshold=self.conf_threshold
                )

        else:
            self.get_logger().error(f"Unknown detector type: {self.detector_type}")
            return None

        if detector is not None:
            if not detector.initialize():
                self.get_logger().error(f"Failed to initialize {self.detector_type} detector")
                return None
            self.get_logger().info(f"Detector initialized: {detector.name}")

        return detector

    def _release_camera_if_busy(self, device_path: str):
        """Try to release camera if held by another process."""
        try:
            result = subprocess.run(
                ['fuser', device_path],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.stdout.strip():
                pids = result.stdout.strip().split()
                current_pid = str(os.getpid())
                for pid in pids:
                    if pid != current_pid:
                        self.get_logger().warn(f"Camera {device_path} busy by PID {pid}, attempting release")
                        try:
                            subprocess.run(['kill', '-9', pid], timeout=2)
                        except Exception:
                            pass
                time.sleep(0.5)
        except Exception as e:
            self.get_logger().debug(f"Camera busy check skipped: {e}")

    def _init_camera(self):
        """Initialize camera capture."""
        # Disable OpenCL for RZ/V2N compatibility
        cv2.ocl.setUseOpenCL(False)

        # Try V4L2 backend first (more reliable on embedded systems)
        cap = None

        # Method 1: Use V4L2 backend explicitly with device path
        device_path = f"/dev/video{self.camera_index}"

        # Try to release camera if busy
        self._release_camera_if_busy(device_path)
        self.get_logger().info(f"Trying V4L2 backend with {device_path}")
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

        if not cap.isOpened():
            # Method 2: Try with index and V4L2 backend
            self.get_logger().info(f"Trying V4L2 backend with index {self.camera_index}")
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not cap.isOpened():
            # Method 3: Fallback to default
            self.get_logger().info("Trying default backend")
            cap = cv2.VideoCapture(self.camera_index)

        if not cap.isOpened():
            self.get_logger().error(f"Cannot open camera {self.camera_index}")
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FPS, self.publish_rate)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera opened: {actual_w}x{actual_h}")

        # Update frame dimensions if different
        if actual_w != self.frame_width or actual_h != self.frame_height:
            self.frame_width = actual_w
            self.frame_height = actual_h
            self.estimator = DistanceEstimator(
                reference_box_height=self.ref_box_height,
                reference_distance=self.ref_distance,
                frame_width=self.frame_width,
                frame_height=self.frame_height,
                horizontal_fov=self.horizontal_fov
            )

        return cap

    def detection_callback(self):
        """Main detection loop - called by timer."""
        if self.cap is None or self.detector is None:
            return

        # Capture frame with retry (USB cameras drop frames occasionally)
        frame = None
        for attempt in range(self.camera_retry_count):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                break
            if attempt < self.camera_retry_count - 1:
                time.sleep(0.01)

        if frame is None:
            self.get_logger().warn("Failed to read frame after retries", throttle_duration_sec=5.0)
            return

        self.frame_count += 1

        # Run detection
        result = self.detector.detect(frame)
        detections = result.detections if result.success else []

        # Log status every 100 frames
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Frame {self.frame_count}: {len(detections)} detections, "
                f"{self.detection_count} targets, tracking={self._tracking_active}"
            )

        # Drawing
        display_frame = frame.copy() if self.show_video else None
        if self.show_video:
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{det.class_name} {det.confidence:.2f}"
                cv2.putText(display_frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Get best detection (largest = closest)
        best = max(detections, key=lambda d: d.area, default=None)

        if best:
            # Target found - reset miss counter, update tracking
            self._consecutive_misses = 0
            self._tracking_active = True
            self._last_detection = best
            self.detection_count += 1

            x, y = self.estimator.estimate_position(best)
            distance, angle = self.estimator.estimate(best)
            self._last_distance = distance
            self._last_angle = angle

            self._publish_pose(x, y, angle)
            self._publish_detection(best, distance, angle)

            if self.show_video:
                bx1, by1, bx2, by2 = best.bbox
                cv2.rectangle(display_frame, (bx1, by1), (bx2, by2), (0, 0, 255), 3)
                info = f"DIST: {distance:.2f}m ANG: {math.degrees(angle):.1f}deg"
                cv2.putText(display_frame, info, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            self.get_logger().debug(
                f"TARGET: conf={best.confidence:.2f} dist={distance:.2f}m "
                f"angle={math.degrees(angle):.1f}deg"
            )

        else:
            # No detection this frame - temporal tracking
            self._consecutive_misses += 1

            if self._tracking_active and self._consecutive_misses < self.consecutive_misses_to_lose:
                # Still within tolerance: publish last known position
                # This prevents flickering when detection drops a single frame
                if self._last_detection is not None:
                    self._publish_pose(
                        self._last_distance * math.cos(self._last_angle),
                        -self._last_distance * math.sin(self._last_angle),
                        self._last_angle
                    )
            elif self._tracking_active:
                # Exceeded miss tolerance: target truly lost
                self._tracking_active = False
                self._last_detection = None
                self.get_logger().info(
                    f"Target lost after {self._consecutive_misses} consecutive misses"
                )

        # Show video
        if self.show_video and display_frame is not None:
            status = f"[{self.target_class}] F:{self.frame_count} D:{len(detections)}"
            if self._tracking_active:
                status += f" | TRACKING (miss:{self._consecutive_misses})"
            else:
                status += " | NO TARGET"
            cv2.putText(display_frame, status,
                        (10, display_frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(self.window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quit requested via keyboard")
                raise KeyboardInterrupt

    def _publish_pose(self, x: float, y: float, yaw: float):
        """Publish target pose as PoseStamped."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # Quaternion from yaw
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pose_pub.publish(msg)

    def _publish_detection(self, detection, distance: float, angle: float):
        """Publish detection details as JSON string."""
        info = {
            'class_name': detection.class_name,
            'class_id': detection.class_id,
            'confidence': round(detection.confidence, 3),
            'bbox': {
                'x1': detection.bbox[0],
                'y1': detection.bbox[1],
                'x2': detection.bbox[2],
                'y2': detection.bbox[3],
                'width': detection.width,
                'height': detection.height,
            },
            'estimated': {
                'distance_m': round(distance, 3),
                'angle_rad': round(angle, 4),
                'angle_deg': round(math.degrees(angle), 1),
            }
        }

        msg = String()
        msg.data = json.dumps(info)
        self.detection_pub.publish(msg)

    def cleanup(self):
        """Cleanup resources (call before destroy_node)."""
        print("[vision_node] Cleaning up...")

        # Cancel timer first
        try:
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
                self.timer = None
        except Exception:
            pass

        # Close video window
        try:
            if hasattr(self, 'show_video') and self.show_video:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
        except Exception:
            pass

        # Release camera
        try:
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
                self.cap = None
        except Exception:
            pass

        # Cleanup detector
        try:
            if hasattr(self, 'detector') and self.detector:
                self.detector.cleanup()
                self.detector = None
        except Exception:
            pass

        print("[vision_node] Cleanup complete")


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

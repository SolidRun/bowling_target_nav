"""
YOLOv5 ONNX Detector Implementation
====================================

Implements DetectorBase using YOLOv5 ONNX model via OpenCV DNN.
Based on reference implementation for RZ/V2N bowling detection.

Usage:
    detector = YoloDetector("path/to/model.onnx")
    detections = detector.detect(frame)
    for det in detections:
        print(f"{det.class_name}: {det.confidence:.2f}")
"""

import cv2
import numpy as np
from typing import List
from .detector_base import DetectorBase, Detection


class YoloDetector(DetectorBase):
    """
    YOLOv5 object detector using OpenCV DNN backend.

    Args:
        model_path: Path to ONNX model file
        input_size: Model input size (default 320)
        conf_threshold: Minimum confidence to accept (default 0.25)
        iou_threshold: IoU threshold for NMS (default 0.45)
        class_names: List of class names (default: bowling classes)
    """

    # Default class names for bowling detection model
    DEFAULT_CLASSES = ["Pin+Ball", "Ball", "Pins", "Sweep"]

    def __init__(self,
                 model_path: str,
                 input_size: int = 320,
                 conf_threshold: float = 0.25,
                 iou_threshold: float = 0.45,
                 class_names: List[str] = None):

        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.class_names = class_names or self.DEFAULT_CLASSES

        # Disable OpenCL to prevent DRP-AI conflicts on RZ/V2N
        cv2.ocl.setUseOpenCL(False)

        # Load ONNX model using OpenCV DNN
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        print(f"[YoloDetector] Loaded model: input={self.input_size}x{self.input_size}, "
              f"conf_thresh={self.conf_threshold}, classes={self.class_names}")

        # Store preprocessing info for coordinate conversion
        self._ratio = 1.0
        self._pad_x = 0
        self._pad_y = 0
        self._orig_w = 0
        self._orig_h = 0

    def get_class_names(self) -> List[str]:
        """Return list of detectable class names."""
        return self.class_names

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run YOLOv5 detection on frame.

        Args:
            frame: BGR image as numpy array

        Returns:
            List of Detection objects
        """
        # Preprocess
        blob = self._preprocess(frame)

        # Inference with OpenCV DNN
        self.net.setInput(blob)
        output = self.net.forward()

        # Postprocess based on output format
        return self._postprocess(output)

    def cleanup(self):
        """Cleanup."""
        self.net = None

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for YOLO inference.
        Applies letterbox padding to maintain aspect ratio.
        """
        h, w = frame.shape[:2]
        self._orig_h, self._orig_w = h, w

        # Calculate scaling ratio
        self._ratio = min(self.input_size / w, self.input_size / h)
        new_w = int(w * self._ratio)
        new_h = int(h * self._ratio)

        # Resize
        resized = cv2.resize(frame, (new_w, new_h))

        # Letterbox padding (gray = 114)
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        self._pad_x = (self.input_size - new_w) // 2
        self._pad_y = (self.input_size - new_h) // 2
        padded[self._pad_y:self._pad_y + new_h,
               self._pad_x:self._pad_x + new_w] = resized

        # Create blob
        blob = cv2.dnn.blobFromImage(
            padded, 1 / 255.0,
            (self.input_size, self.input_size),
            swapRB=True, crop=False
        )
        return blob

    def _postprocess(self, output: np.ndarray) -> List[Detection]:
        """
        Parse YOLO output into Detection objects.
        Handles both modern and legacy output formats.
        """
        detections = []

        # Format 1: Modern (1, N, 6) - [x1, y1, x2, y2, conf, class_id]
        if len(output.shape) == 3 and output.shape[2] == 6:
            for pred in output[0]:
                x1, y1, x2, y2, conf, cls_id = pred
                if conf < self.conf_threshold:
                    continue

                # Convert to original image coordinates
                x1 = int((x1 - self._pad_x) / self._ratio)
                y1 = int((y1 - self._pad_y) / self._ratio)
                x2 = int((x2 - self._pad_x) / self._ratio)
                y2 = int((y2 - self._pad_y) / self._ratio)

                # Clamp to image bounds
                x1 = max(0, min(self._orig_w, x1))
                y1 = max(0, min(self._orig_h, y1))
                x2 = max(0, min(self._orig_w, x2))
                y2 = max(0, min(self._orig_h, y2))

                cls_id = int(cls_id)
                class_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"

                detections.append(Detection(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    confidence=float(conf),
                    class_id=cls_id,
                    class_name=class_name
                ))

        # Format 2: Legacy (1, C+5, N) - needs transpose and NMS
        elif len(output.shape) == 3:
            output = output[0].T  # Transpose to (N, C+5)
            detections = self._parse_legacy_output(output)

        return detections

    def _parse_legacy_output(self, output: np.ndarray) -> List[Detection]:
        """Parse legacy YOLO output format with NMS."""
        detections = []
        boxes = []
        scores = []
        class_ids = []

        for row in output:
            cx, cy, w, h = row[:4]
            class_scores = row[4:]  # Use raw scores (this model outputs low values)

            cls_id = int(np.argmax(class_scores))
            conf = float(class_scores[cls_id])

            if conf < self.conf_threshold:
                continue

            # Convert center format to corner format
            x1 = int((cx - w / 2 - self._pad_x) / self._ratio)
            y1 = int((cy - h / 2 - self._pad_y) / self._ratio)
            x2 = int((cx + w / 2 - self._pad_x) / self._ratio)
            y2 = int((cy + h / 2 - self._pad_y) / self._ratio)

            # Clamp
            x1 = max(0, min(self._orig_w, x1))
            y1 = max(0, min(self._orig_h, y1))
            x2 = max(0, min(self._orig_w, x2))
            y2 = max(0, min(self._orig_h, y2))

            # Skip invalid boxes
            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2 - x1, y2 - y1])  # x, y, w, h for NMS
            scores.append(conf)
            class_ids.append(cls_id)

        # Apply NMS
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, self.iou_threshold)
            if len(indices) > 0:
                indices = indices.flatten()
                for i in indices:
                    x, y, w, h = boxes[i]
                    cls_id = class_ids[i]
                    class_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"

                    detections.append(Detection(
                        x1=x, y1=y, x2=x + w, y2=y + h,
                        confidence=scores[i],
                        class_id=cls_id,
                        class_name=class_name
                    ))

        return detections

    def draw_detections(self, frame: np.ndarray, detections: List[Detection],
                        colors: List[tuple] = None) -> np.ndarray:
        """
        Draw bounding boxes on frame.

        Args:
            frame: Image to draw on
            detections: List of detections
            colors: Optional list of BGR colors per class

        Returns:
            Frame with drawn detections
        """
        default_colors = [
            (107, 107, 255),  # Red-ish
            (196, 205, 78),   # Cyan-ish
            (209, 183, 69),   # Blue-ish
            (180, 206, 150),  # Green-ish
        ]
        colors = colors or default_colors

        for det in detections:
            color = colors[det.class_id % len(colors)]
            cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, 2)

            label = f"{det.class_name} {det.confidence:.2f}"
            cv2.putText(frame, label, (det.x1, det.y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return frame

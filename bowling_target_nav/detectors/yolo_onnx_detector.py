"""
YOLO ONNX Detector
==================

Object detection using YOLO model in ONNX format.
Uses ONNX Runtime for inference.
"""

import logging
from pathlib import Path
from typing import List, Optional
import numpy as np

from .base import DetectorBase, Detection, DetectionResult

logger = logging.getLogger(__name__)


class YoloOnnxDetector(DetectorBase):
    """
    YOLO detector using ONNX Runtime.

    Supports YOLOv5 and compatible models in ONNX format.

    Usage:
        detector = YoloOnnxDetector(
            model_path="models/bowling_yolov5.onnx",
            confidence_threshold=0.5,
            input_size=(640, 640),
            class_names=["bowling_pin"]
        )
        detector.initialize()
        result = detector.detect(frame)
    """

    def __init__(
        self,
        model_path: str = "models/bowling_yolov5.onnx",
        input_size: tuple = (640, 640),
        class_names: Optional[List[str]] = None,
        confidence_threshold: float = 0.5,
        nms_threshold: float = 0.45,
        target_class: str = "bowling_pin",
        filter_classes: Optional[List[str]] = None,
        **kwargs
    ):
        """
        Initialize YOLO ONNX detector.

        Args:
            model_path: Path to ONNX model file
            input_size: Model input size (width, height)
            class_names: List of class names
            confidence_threshold: Minimum detection confidence
            nms_threshold: Non-maximum suppression threshold
            target_class: Primary target class to detect
            filter_classes: Only return detections for these classes (None = all)
        """
        super().__init__(
            confidence_threshold=confidence_threshold,
            target_class=target_class,
            **kwargs
        )

        self.model_path = model_path
        self.input_size = input_size
        self.class_names = class_names or ["bowling_pin"]
        self.nms_threshold = nms_threshold
        self.filter_classes = filter_classes

        self._session = None
        self._input_name = None
        self._output_names = None

    @property
    def name(self) -> str:
        return "YOLO ONNX Detector"

    @property
    def supported_classes(self) -> List[str]:
        return self.class_names

    def _load_model(self) -> None:
        """Load ONNX model."""
        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError("onnxruntime not installed. Install with: pip install onnxruntime")

        # Find model file
        model_path = self._find_model_path()
        if model_path is None:
            raise FileNotFoundError(f"Model not found: {self.model_path}")

        logger.info(f"Loading ONNX model from: {model_path}")

        # Create inference session
        providers = ['CPUExecutionProvider']

        # Try to use hardware acceleration if available
        available_providers = ort.get_available_providers()
        if 'CUDAExecutionProvider' in available_providers:
            providers.insert(0, 'CUDAExecutionProvider')
        elif 'OpenVINOExecutionProvider' in available_providers:
            providers.insert(0, 'OpenVINOExecutionProvider')

        self._session = ort.InferenceSession(str(model_path), providers=providers)

        # Get input/output info
        input_info = self._session.get_inputs()[0]
        self._input_name = input_info.name
        self._output_names = [o.name for o in self._session.get_outputs()]

        # Auto-detect input size from model
        input_shape = input_info.shape  # e.g. [1, 3, 320, 320]
        if len(input_shape) == 4 and isinstance(input_shape[2], int):
            model_h, model_w = input_shape[2], input_shape[3]
            if (model_w, model_h) != self.input_size:
                logger.info(f"Overriding input_size {self.input_size} -> ({model_w}, {model_h}) from model")
                self.input_size = (model_w, model_h)

        # Auto-detect class names from model metadata
        meta = self._session.get_modelmeta()
        if meta.custom_metadata_map and 'names' in meta.custom_metadata_map:
            import ast
            try:
                names_dict = ast.literal_eval(meta.custom_metadata_map['names'])
                self.class_names = [names_dict[i] for i in sorted(names_dict.keys())]
                logger.info(f"Classes from model: {self.class_names}")
            except Exception:
                pass

        # Detect YOLOv8 vs YOLOv5 format from output shape
        out_shape = self._session.get_outputs()[0].shape
        self._is_yolov8 = (len(out_shape) == 3 and
                           isinstance(out_shape[1], int) and
                           out_shape[1] < out_shape[2])  # [1, 8, 2100] vs [1, 2100, 8]

        logger.info(f"Model loaded. Input: {self._input_name} shape={input_shape}, Outputs: {self._output_names}")
        logger.info(f"Format: {'YOLOv8' if self._is_yolov8 else 'YOLOv5'}, Classes: {self.class_names}")
        if self.filter_classes:
            logger.info(f"Filtering to classes: {self.filter_classes}")
        logger.info(f"Using providers: {self._session.get_providers()}")

    def _find_model_path(self) -> Optional[Path]:
        """Find model file in common locations."""
        search_paths = [
            Path(self.model_path),
            Path(__file__).parent.parent.parent / self.model_path,
            Path(__file__).parent.parent.parent / "models" / Path(self.model_path).name,
            Path.home() / "ros2_ws/src/bowling_target_nav" / self.model_path,
        ]

        for path in search_paths:
            if path.exists():
                return path

        return None

    def _cleanup(self) -> None:
        """Cleanup ONNX session."""
        self._session = None
        self._input_name = None
        self._output_names = None

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for YOLO inference.

        Args:
            frame: Input BGR image

        Returns:
            Preprocessed tensor
        """
        import cv2

        # Resize
        img = cv2.resize(frame, self.input_size)

        # BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Normalize to 0-1
        img = img.astype(np.float32) / 255.0

        # HWC to CHW format
        img = np.transpose(img, (2, 0, 1))

        # Add batch dimension
        img = np.expand_dims(img, axis=0)

        return img

    def _postprocess(
        self,
        outputs: np.ndarray,
        original_size: tuple
    ) -> List[Detection]:
        """
        Postprocess YOLO outputs (supports both YOLOv5 and YOLOv8 formats).
        """
        detections = []
        output = outputs[0]

        # Remove batch dimension
        if len(output.shape) == 3:
            output = output[0]  # [num_classes+4, num_candidates] or [num_candidates, num_classes+5]

        # YOLOv8 format: [num_classes+4, num_candidates] -> transpose
        if self._is_yolov8:
            output = output.T  # -> [num_candidates, num_classes+4]

        orig_w, orig_h = original_size
        input_w, input_h = self.input_size
        scale_x = orig_w / input_w
        scale_y = orig_h / input_h
        num_classes = len(self.class_names)

        boxes = []
        confidences = []
        class_ids = []

        # Build set of allowed class IDs for fast filtering
        allowed_ids = None
        if self.filter_classes:
            allowed_ids = set()
            for i, name in enumerate(self.class_names):
                if name in self.filter_classes:
                    allowed_ids.add(i)

        for row in output:
            if self._is_yolov8:
                # YOLOv8: [cx, cy, w, h, class_score_0, class_score_1, ...]
                class_scores = row[4:4 + num_classes]
                class_id = np.argmax(class_scores)
                confidence = float(class_scores[class_id])
            else:
                # YOLOv5: [cx, cy, w, h, obj_conf, class_score_0, ...]
                obj_conf = row[4]
                class_scores = row[5:5 + num_classes]
                class_id = np.argmax(class_scores)
                confidence = float(obj_conf * class_scores[class_id])

            if confidence < self.confidence_threshold:
                continue

            # Skip classes not in filter list
            if allowed_ids is not None and int(class_id) not in allowed_ids:
                continue

            cx, cy, w, h = row[:4]
            x1 = int((cx - w / 2) * scale_x)
            y1 = int((cy - h / 2) * scale_y)
            x2 = int((cx + w / 2) * scale_x)
            y2 = int((cy + h / 2) * scale_y)

            x1 = max(0, min(x1, orig_w - 1))
            y1 = max(0, min(y1, orig_h - 1))
            x2 = max(0, min(x2, orig_w - 1))
            y2 = max(0, min(y2, orig_h - 1))

            boxes.append([x1, y1, x2, y2])
            confidences.append(confidence)
            class_ids.append(int(class_id))

        if boxes:
            indices = self._nms(boxes, confidences)
            for i in indices:
                class_name = self.class_names[class_ids[i]] if class_ids[i] < len(self.class_names) else "unknown"
                detections.append(Detection(
                    class_name=class_name,
                    class_id=class_ids[i],
                    confidence=confidences[i],
                    bbox=tuple(boxes[i])
                ))

        return detections

    def _nms(self, boxes: List, scores: List) -> List[int]:
        """
        Apply Non-Maximum Suppression.

        Args:
            boxes: List of bounding boxes
            scores: List of confidence scores

        Returns:
            Indices of kept boxes
        """
        if not boxes:
            return []

        boxes = np.array(boxes)
        scores = np.array(scores)

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0, xx2 - xx1)
            h = np.maximum(0, yy2 - yy1)

            intersection = w * h
            iou = intersection / (areas[i] + areas[order[1:]] - intersection)

            inds = np.where(iou <= self.nms_threshold)[0]
            order = order[inds + 1]

        return keep

    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Run YOLO detection on frame."""
        # Preprocess
        input_tensor = self._preprocess(frame)

        # Run inference
        outputs = self._session.run(self._output_names, {self._input_name: input_tensor})

        # Postprocess
        original_size = (frame.shape[1], frame.shape[0])
        detections = self._postprocess(outputs, original_size)

        return DetectionResult(
            detections=detections,
            success=True
        )

"""
Binary Application Detector (Stub for Future Implementation)
============================================================

This module provides the interface for using a custom binary application
as the detection backend. Currently a stub - implement when binary is ready.

The binary application should:
    1. Accept an image file path or raw image data
    2. Output detection results in a parseable format (JSON, CSV, etc.)
    3. Return object locations based on bounding box size

Usage (future):
    detector = BinDetector("/path/to/detection_binary")
    detections = detector.detect(frame)

Expected binary output format (customize as needed):
    class_id,x1,y1,x2,y2,confidence
    2,100,50,200,150,0.85
    1,300,200,400,350,0.72
"""

import subprocess
import tempfile
import json
import os
import cv2
import numpy as np
from typing import List
from .detector_base import DetectorBase, Detection


class BinDetector(DetectorBase):
    """
    Detector that wraps an external binary application.

    The binary should accept an image and return detections.
    Customize the _parse_output method for your binary's output format.

    Args:
        binary_path: Path to the detection binary/executable
        class_names: List of class names the binary can detect
        conf_threshold: Minimum confidence threshold
        timeout: Maximum seconds to wait for binary (default 5.0)
    """

    def __init__(self,
                 binary_path: str,
                 class_names: List[str] = None,
                 conf_threshold: float = 0.3,
                 timeout: float = 5.0):

        self.binary_path = binary_path
        self.class_names = class_names or ["object"]
        self.conf_threshold = conf_threshold
        self.timeout = timeout

        # Verify binary exists
        if binary_path and not os.path.exists(binary_path):
            raise FileNotFoundError(f"Binary not found: {binary_path}")

    def get_class_names(self) -> List[str]:
        """Return list of detectable class names."""
        return self.class_names

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run detection using external binary.

        Currently returns empty list - implement when binary is ready.

        Args:
            frame: BGR image as numpy array

        Returns:
            List of Detection objects
        """
        if not self.binary_path:
            return []

        # Save frame to temp file
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as f:
            temp_path = f.name
            cv2.imwrite(temp_path, frame)

        try:
            # Call binary with image path
            result = subprocess.run(
                [self.binary_path, temp_path],
                capture_output=True,
                text=True,
                timeout=self.timeout
            )

            if result.returncode != 0:
                print(f"Binary error: {result.stderr}")
                return []

            # Parse output
            return self._parse_output(result.stdout, frame.shape)

        except subprocess.TimeoutExpired:
            print(f"Binary timeout after {self.timeout}s")
            return []
        except Exception as e:
            print(f"Binary execution error: {e}")
            return []
        finally:
            # Cleanup temp file
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def _parse_output(self, output: str, frame_shape: tuple) -> List[Detection]:
        """
        Parse binary output into Detection objects.

        Override this method for your specific binary output format.

        Expected formats (implement one):
            1. JSON: {"detections": [{"class_id": 0, "x1": 10, ...}, ...]}
            2. CSV: class_id,x1,y1,x2,y2,confidence (one per line)
            3. Custom: adapt as needed

        Args:
            output: Raw stdout from binary
            frame_shape: (height, width, channels) for bounds checking

        Returns:
            List of Detection objects
        """
        detections = []
        h, w = frame_shape[:2]

        # Try JSON format first
        try:
            data = json.loads(output)
            if isinstance(data, dict) and 'detections' in data:
                for det in data['detections']:
                    if det.get('confidence', 1.0) < self.conf_threshold:
                        continue

                    cls_id = det.get('class_id', 0)
                    class_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"

                    detections.append(Detection(
                        x1=max(0, min(w, det['x1'])),
                        y1=max(0, min(h, det['y1'])),
                        x2=max(0, min(w, det['x2'])),
                        y2=max(0, min(h, det['y2'])),
                        confidence=det.get('confidence', 1.0),
                        class_id=cls_id,
                        class_name=class_name
                    ))
            return detections
        except json.JSONDecodeError:
            pass

        # Try CSV format
        for line in output.strip().split('\n'):
            if not line or line.startswith('#'):
                continue

            parts = line.split(',')
            if len(parts) >= 5:
                try:
                    cls_id = int(parts[0])
                    x1, y1, x2, y2 = map(int, parts[1:5])
                    conf = float(parts[5]) if len(parts) > 5 else 1.0

                    if conf < self.conf_threshold:
                        continue

                    class_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"

                    detections.append(Detection(
                        x1=max(0, min(w, x1)),
                        y1=max(0, min(h, y1)),
                        x2=max(0, min(w, x2)),
                        y2=max(0, min(h, y2)),
                        confidence=conf,
                        class_id=cls_id,
                        class_name=class_name
                    ))
                except (ValueError, IndexError):
                    continue

        return detections


class BinDetectorStub(DetectorBase):
    """
    Stub detector for testing without actual binary.
    Returns no detections - use for development/testing.
    """

    def __init__(self, class_names: List[str] = None):
        self.class_names = class_names or ["object"]

    def get_class_names(self) -> List[str]:
        return self.class_names

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Returns empty list - stub implementation."""
        return []

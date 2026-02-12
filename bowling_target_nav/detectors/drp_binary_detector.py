"""
DRP Binary Detector
===================

Object detection using V2N DRP (Dynamic Reconfigurable Processor) binary application.
Interfaces with external binary through shared memory or file I/O.
"""

import json
import logging
import mmap
import os
import struct
import subprocess
import tempfile
import time
from pathlib import Path
from typing import List, Optional
import numpy as np

from .base import DetectorBase, Detection, DetectionResult

logger = logging.getLogger(__name__)


class DrpBinaryDetector(DetectorBase):
    """
    Detector using V2N DRP binary application.

    This detector interfaces with an external binary application that performs
    object detection using the V2N's DRP (Dynamic Reconfigurable Processor).

    Communication methods:
    1. Shared memory (fastest) - Uses /dev/shm for frame and result exchange
    2. File-based - Uses temporary files for frame and result exchange
    3. Stdout - Reads JSON results from binary stdout

    Usage:
        detector = DrpBinaryDetector(
            binary_path="/opt/drp/bowling_detector",
            input_method="shared_memory",
            output_method="shared_memory"
        )
        detector.initialize()
        result = detector.detect(frame)

    DRP Binary Protocol:
        Input (shared memory at input_path):
            - Header (16 bytes): width(4), height(4), channels(4), format(4)
            - Image data: raw BGR pixels

        Output (shared memory at output_path):
            - Header (8 bytes): num_detections(4), status(4)
            - Per detection (28 bytes): x1(4), y1(4), x2(4), y2(4), class_id(4), confidence(4), reserved(4)
    """

    # Protocol constants
    INPUT_HEADER_SIZE = 16
    OUTPUT_HEADER_SIZE = 8
    DETECTION_SIZE = 28
    MAX_DETECTIONS = 100

    def __init__(
        self,
        binary_path: str = "/opt/drp/bowling_detector",
        input_method: str = "shared_memory",
        input_path: str = "/dev/shm/v2n_camera_frame",
        output_method: str = "shared_memory",
        output_path: str = "/dev/shm/v2n_detection_result",
        timeout: float = 1.0,
        confidence_threshold: float = 0.5,
        target_class: str = "bowling_pin",
        class_names: Optional[List[str]] = None,
        **kwargs
    ):
        """
        Initialize DRP Binary detector.

        Args:
            binary_path: Path to DRP detection binary
            input_method: How to send frames ("shared_memory" | "file")
            input_path: Path for input frame
            output_method: How to receive results ("shared_memory" | "file" | "stdout")
            output_path: Path for output results
            timeout: Execution timeout in seconds
            confidence_threshold: Minimum detection confidence
            target_class: Primary target class
            class_names: List of class names
        """
        super().__init__(
            confidence_threshold=confidence_threshold,
            target_class=target_class,
            **kwargs
        )

        self.binary_path = binary_path
        self.input_method = input_method
        self.input_path = input_path
        self.output_method = output_method
        self.output_path = output_path
        self.timeout = timeout
        self.class_names = class_names or ["bowling_pin"]

        self._input_mmap = None
        self._output_mmap = None
        self._input_fd = None
        self._output_fd = None

    @property
    def name(self) -> str:
        return "DRP Binary Detector"

    @property
    def supported_classes(self) -> List[str]:
        return self.class_names

    def _load_model(self) -> None:
        """Verify binary exists and setup communication channels."""
        # Verify binary exists
        if not Path(self.binary_path).exists():
            raise FileNotFoundError(f"DRP binary not found: {self.binary_path}")

        # Check binary is executable
        if not os.access(self.binary_path, os.X_OK):
            raise PermissionError(f"DRP binary not executable: {self.binary_path}")

        # Setup shared memory if needed
        if self.input_method == "shared_memory":
            self._setup_input_shared_memory()

        if self.output_method == "shared_memory":
            self._setup_output_shared_memory()

        logger.info(f"DRP binary verified: {self.binary_path}")
        logger.info(f"Input method: {self.input_method} -> {self.input_path}")
        logger.info(f"Output method: {self.output_method} -> {self.output_path}")

    def _setup_input_shared_memory(self) -> None:
        """Setup input shared memory region."""
        # Calculate size for max resolution (1920x1080 BGR)
        max_size = self.INPUT_HEADER_SIZE + (1920 * 1080 * 3)

        # Create or open shared memory
        try:
            self._input_fd = os.open(
                self.input_path,
                os.O_RDWR | os.O_CREAT,
                0o666
            )
            os.ftruncate(self._input_fd, max_size)
            self._input_mmap = mmap.mmap(
                self._input_fd,
                max_size,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE
            )
            logger.debug(f"Input shared memory created: {self.input_path}")
        except Exception as e:
            logger.error(f"Failed to create input shared memory: {e}")
            raise

    def _setup_output_shared_memory(self) -> None:
        """Setup output shared memory region."""
        # Calculate size for max detections
        max_size = self.OUTPUT_HEADER_SIZE + (self.MAX_DETECTIONS * self.DETECTION_SIZE)

        try:
            self._output_fd = os.open(
                self.output_path,
                os.O_RDWR | os.O_CREAT,
                0o666
            )
            os.ftruncate(self._output_fd, max_size)
            self._output_mmap = mmap.mmap(
                self._output_fd,
                max_size,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE
            )
            logger.debug(f"Output shared memory created: {self.output_path}")
        except Exception as e:
            logger.error(f"Failed to create output shared memory: {e}")
            raise

    def _cleanup(self) -> None:
        """Cleanup shared memory and file descriptors."""
        if self._input_mmap:
            self._input_mmap.close()
            self._input_mmap = None

        if self._output_mmap:
            self._output_mmap.close()
            self._output_mmap = None

        if self._input_fd:
            os.close(self._input_fd)
            self._input_fd = None

        if self._output_fd:
            os.close(self._output_fd)
            self._output_fd = None

    def _write_frame_shared_memory(self, frame: np.ndarray) -> None:
        """Write frame to shared memory."""
        h, w, c = frame.shape

        # Write header
        header = struct.pack('IIII', w, h, c, 0)  # format=0 for BGR
        self._input_mmap.seek(0)
        self._input_mmap.write(header)

        # Write image data
        self._input_mmap.write(frame.tobytes())

    def _write_frame_file(self, frame: np.ndarray) -> None:
        """Write frame to file."""
        import cv2
        cv2.imwrite(self.input_path, frame)

    def _read_result_shared_memory(self) -> List[Detection]:
        """Read detection results from shared memory."""
        detections = []

        self._output_mmap.seek(0)

        # Read header
        header = self._output_mmap.read(self.OUTPUT_HEADER_SIZE)
        num_detections, status = struct.unpack('II', header)

        if status != 0:
            logger.warning(f"DRP binary returned status: {status}")
            return []

        # Read detections
        for _ in range(min(num_detections, self.MAX_DETECTIONS)):
            det_data = self._output_mmap.read(self.DETECTION_SIZE)
            x1, y1, x2, y2, class_id, conf_int, _ = struct.unpack('IIIIIIf', det_data)

            # Convert confidence from int (0-1000) to float if needed
            confidence = conf_int / 1000.0 if conf_int > 1 else conf_int

            if confidence >= self.confidence_threshold:
                class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"
                detections.append(Detection(
                    class_name=class_name,
                    class_id=class_id,
                    confidence=confidence,
                    bbox=(x1, y1, x2, y2)
                ))

        return detections

    def _read_result_file(self) -> List[Detection]:
        """Read detection results from file."""
        detections = []

        try:
            with open(self.output_path, 'r') as f:
                data = json.load(f)

            for det in data.get('detections', []):
                confidence = det.get('confidence', 0.0)
                if confidence >= self.confidence_threshold:
                    class_id = det.get('class_id', 0)
                    class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"
                    detections.append(Detection(
                        class_name=class_name,
                        class_id=class_id,
                        confidence=confidence,
                        bbox=(det['x1'], det['y1'], det['x2'], det['y2'])
                    ))
        except Exception as e:
            logger.error(f"Failed to read result file: {e}")

        return detections

    def _read_result_stdout(self, stdout: str) -> List[Detection]:
        """Parse detection results from stdout JSON."""
        detections = []

        try:
            data = json.loads(stdout)

            for det in data.get('detections', []):
                confidence = det.get('confidence', 0.0)
                if confidence >= self.confidence_threshold:
                    class_id = det.get('class_id', 0)
                    class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"
                    detections.append(Detection(
                        class_name=class_name,
                        class_id=class_id,
                        confidence=confidence,
                        bbox=(det['x1'], det['y1'], det['x2'], det['y2'])
                    ))
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse stdout JSON: {e}")
        except Exception as e:
            logger.error(f"Failed to parse stdout: {e}")

        return detections

    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Run DRP binary detection on frame."""
        # Write frame
        if self.input_method == "shared_memory":
            self._write_frame_shared_memory(frame)
        else:
            self._write_frame_file(frame)

        # Build command
        cmd = [self.binary_path]
        if self.input_method == "shared_memory":
            cmd.extend(['--input-shm', self.input_path])
        else:
            cmd.extend(['--input-file', self.input_path])

        if self.output_method == "shared_memory":
            cmd.extend(['--output-shm', self.output_path])
        elif self.output_method == "file":
            cmd.extend(['--output-file', self.output_path])
        # stdout method doesn't need extra args

        # Run binary
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                timeout=self.timeout,
                text=True
            )

            if result.returncode != 0:
                logger.warning(f"DRP binary returned {result.returncode}: {result.stderr}")

            # Read results
            if self.output_method == "shared_memory":
                detections = self._read_result_shared_memory()
            elif self.output_method == "file":
                detections = self._read_result_file()
            else:  # stdout
                detections = self._read_result_stdout(result.stdout)

            return DetectionResult(
                detections=detections,
                success=True
            )

        except subprocess.TimeoutExpired:
            logger.error(f"DRP binary timed out after {self.timeout}s")
            return DetectionResult(
                success=False,
                error="Detection timeout"
            )
        except Exception as e:
            logger.error(f"DRP binary execution failed: {e}")
            return DetectionResult(
                success=False,
                error=str(e)
            )

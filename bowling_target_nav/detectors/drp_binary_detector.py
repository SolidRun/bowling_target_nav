"""
DRP-AI Pipe Detector
====================

Object detection using V2N DRP-AI hardware accelerator via a persistent
C++ subprocess. Communicates through stdin/stdout pipes for low-latency,
high-throughput inference.

The C++ binary (yolo_detection --pipe) loads the DRP-AI model once at startup,
then continuously reads BGR frames from stdin and writes JSON detections to stdout.

Protocol:
    Python → C++ stdin:  [uint32 width][uint32 height][BGR pixel bytes]
    C++ stdout → Python: {"detections":[...],"inference_ms":float}\n
    Startup:             C++ writes "READY\n" when model is loaded
"""

import json
import logging
import os
import signal
import struct
import subprocess
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

from .base import DetectorBase, Detection, DetectionResult

logger = logging.getLogger(__name__)


# Default paths on V2N
DRP_BINARY_PATHS = [
    '/opt/drp/yolo_detection',
    '/home/root/deploy/yolo_detection',
    '/home/root/deploy/app_yolo_cam',
    '/home/root/drp/yolo_detection',
]

DRP_MODEL_PATHS = [
    '/opt/drp/drpai_model',
    '/home/root/deploy/drpai_model',
    '/home/root/deploy/model',
    '/home/root/drp/drpai_model',
]


def find_drp_binary() -> Optional[str]:
    """Find the DRP-AI detection binary on the system."""
    for path in DRP_BINARY_PATHS:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None


def find_drp_model() -> Optional[str]:
    """Find the DRP-AI model directory on the system."""
    for path in DRP_MODEL_PATHS:
        if os.path.isdir(path):
            # Verify it contains the model subgraph
            for entry in os.listdir(path):
                if entry.startswith('sub_') and entry.endswith('_CPU_DRP_TVM'):
                    return path
    return None


class DrpBinaryDetector(DetectorBase):
    """
    Detector using V2N DRP-AI hardware accelerator via persistent subprocess.

    Launches the C++ yolo_detection binary in --pipe mode. The binary loads
    the DRP-AI model once, then processes frames sent through stdin and returns
    JSON detection results through stdout.

    This is ~10-20x faster than ONNX Runtime on CPU for the same model.

    Usage:
        detector = DrpBinaryDetector(
            binary_path="/opt/drp/yolo_detection",
            model_dir="/opt/drp/drpai_model"
        )
        detector.initialize()
        result = detector.detect(frame)  # Returns DetectionResult
    """

    def __init__(
        self,
        binary_path: Optional[str] = None,
        model_dir: Optional[str] = None,
        confidence_threshold: float = 0.5,
        target_class: str = "bowling-pins",
        class_names: Optional[List[str]] = None,
        startup_timeout: float = 30.0,
        **kwargs
    ):
        super().__init__(
            confidence_threshold=confidence_threshold,
            target_class=target_class,
            **kwargs
        )

        self.binary_path = binary_path or find_drp_binary()
        self.model_dir = model_dir or find_drp_model()
        self.class_names = class_names or ["bowling-pins"]
        self.startup_timeout = startup_timeout
        self._proc = None

    @property
    def name(self) -> str:
        return "DRP-AI Pipe Detector"

    @property
    def supported_classes(self) -> List[str]:
        return self.class_names

    def _load_model(self) -> None:
        """Launch C++ subprocess and wait for READY signal."""
        if not self.binary_path:
            raise FileNotFoundError(
                f"DRP-AI binary not found. Searched: {DRP_BINARY_PATHS}")

        if not os.path.isfile(self.binary_path):
            raise FileNotFoundError(
                f"DRP-AI binary not found: {self.binary_path}")

        if not os.access(self.binary_path, os.X_OK):
            raise PermissionError(
                f"DRP-AI binary not executable: {self.binary_path}")

        if not self.model_dir:
            raise FileNotFoundError(
                f"DRP-AI model dir not found. Searched: {DRP_MODEL_PATHS}")

        if not os.path.isdir(self.model_dir):
            raise FileNotFoundError(
                f"DRP-AI model dir not found: {self.model_dir}")

        # Launch the C++ binary in pipe mode
        cmd = [self.binary_path, '--pipe', self.model_dir]
        logger.info(f"Launching DRP-AI subprocess: {' '.join(cmd)}")

        # Disable any OpenCV hardware acceleration in subprocess to prevent
        # DRP contention (the binary links OpenCV but pipe mode doesn't use it)
        env = os.environ.copy()
        env['OPENCV_OPENCL_DEVICE'] = ''
        env['OPENCV_OPENCL_RUNTIME'] = ''

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,  # Unbuffered
            env=env,
        )

        # Wait for READY signal
        start = time.time()
        while time.time() - start < self.startup_timeout:
            # Check if process died
            if self._proc.poll() is not None:
                stderr = self._proc.stderr.read().decode('utf-8', errors='replace')
                raise RuntimeError(
                    f"DRP-AI process exited with code {self._proc.returncode}: {stderr}")

            # Try to read READY line (non-blocking via timeout)
            try:
                line = self._proc.stdout.readline()
                if line:
                    text = line.decode('utf-8', errors='replace').strip()
                    if text == 'READY':
                        logger.info("DRP-AI subprocess ready")
                        return
                    else:
                        logger.debug(f"DRP-AI startup output: {text}")
            except Exception:
                pass

            time.sleep(0.1)

        # Timeout
        self._kill_proc()
        raise TimeoutError(
            f"DRP-AI subprocess did not become ready within {self.startup_timeout}s")

    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Send frame to DRP-AI subprocess, receive JSON detections."""
        if self._proc is None or self._proc.poll() is not None:
            return DetectionResult(
                success=False,
                error="DRP-AI subprocess not running"
            )

        try:
            h, w = frame.shape[:2]

            # Write frame header (8 bytes: width + height as uint32 LE)
            header = struct.pack('<II', w, h)
            self._proc.stdin.write(header)

            # Write BGR pixel data
            self._proc.stdin.write(frame.tobytes())
            self._proc.stdin.flush()

            # Read JSON response line
            line = self._proc.stdout.readline()
            if not line:
                return DetectionResult(
                    success=False,
                    error="DRP-AI subprocess closed stdout"
                )

            data = json.loads(line.decode('utf-8', errors='replace'))

            # Parse detections
            detections = []
            for d in data.get('detections', []):
                confidence = d.get('confidence', 0.0)
                if confidence < self.confidence_threshold:
                    continue
                class_id = d.get('class_id', 0)
                class_name = d.get('class_name', '')
                if not class_name and class_id < len(self.class_names):
                    class_name = self.class_names[class_id]

                detections.append(Detection(
                    class_name=class_name,
                    class_id=class_id,
                    confidence=confidence,
                    bbox=(
                        int(d.get('x1', 0)),
                        int(d.get('y1', 0)),
                        int(d.get('x2', 0)),
                        int(d.get('y2', 0)),
                    )
                ))

            return DetectionResult(
                detections=detections,
                inference_time=data.get('inference_ms', 0.0) / 1000.0,
                success=True
            )

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse DRP-AI JSON: {e}")
            return DetectionResult(success=False, error=f"JSON parse error: {e}")
        except BrokenPipeError:
            logger.error("DRP-AI subprocess pipe broken")
            return DetectionResult(success=False, error="Pipe broken")
        except Exception as e:
            logger.error(f"DRP-AI detection failed: {e}")
            return DetectionResult(success=False, error=str(e))

    def _kill_proc(self):
        """Terminate the subprocess."""
        if self._proc is not None:
            try:
                self._proc.stdin.close()
            except Exception:
                pass
            try:
                self._proc.send_signal(signal.SIGTERM)
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            self._proc = None

    def shutdown(self) -> None:
        """Stop DRP-AI subprocess."""
        logger.info("Shutting down DRP-AI subprocess")
        self._kill_proc()
        super().shutdown()

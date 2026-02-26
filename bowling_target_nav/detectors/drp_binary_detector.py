"""
DRP-AI Detectors
================

Object detection using V2N DRP-AI hardware accelerator via C++ subprocess.

Two modes:
  - Pipe mode:   Python sends BGR frames via stdin, reads JSON from stdout.
  - Stream mode: C++ owns camera, writes frames to shared memory,
                 writes JSON detections to stdout. Much faster.

Protocol (pipe mode):
    Python → C++ stdin:  [uint32 width][uint32 height][BGR pixel bytes]
    C++ stdout → Python: {"detections":[...],"inference_ms":float}\\n
    Startup:             C++ writes "READY\\n" when model is loaded

Protocol (stream mode):
    C++ stdout → Python: {"detections":[...],"inference_ms":float}\\n
    C++ → /dev/shm/v2n_camera: [uint32 w][uint32 h][uint64 seq][uint64 ts][BGR data]
    Startup:             C++ writes "READY\\n" when model is loaded
"""

import json
import logging
import mmap
import os
import signal
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

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
            cwd=os.path.dirname(self.binary_path),
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


SHM_NAME = '/dev/shm/v2n_camera'
SHM_HEADER_SIZE = 24  # uint32 w + uint32 h + uint64 seq + uint64 ts


class DrpStreamDetector:
    """
    DRP-AI stream mode: C++ owns camera + inference, Python reads results.

    The C++ binary runs with --stream flag:
      - Captures camera frames internally (multi-threaded)
      - Runs DRP-AI inference (multi-threaded)
      - Writes camera frames to shared memory /dev/shm/v2n_camera
      - Writes JSON detections to stdout

    Python just reads:
      - get_frame() → latest BGR frame from shared memory (zero-copy mmap)
      - get_detections() → latest JSON detections from stdout reader thread

    This eliminates ~921KB pipe transfer per frame → ~10x faster than pipe mode.
    """

    def __init__(
        self,
        binary_path: Optional[str] = None,
        model_dir: Optional[str] = None,
        confidence_threshold: float = 0.5,
        class_names: Optional[List[str]] = None,
        startup_timeout: float = 30.0,
    ):
        self.binary_path = binary_path or find_drp_binary()
        self.model_dir = model_dir or find_drp_model()
        self.class_names = class_names or ["bowling-pins"]
        self.confidence_threshold = confidence_threshold
        self.startup_timeout = startup_timeout
        self._proc = None
        self._shm_fd = -1
        self._shm = None
        self._reader_thread = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_detections: List[dict] = []
        self._latest_infer_ms = 0.0
        self._last_seq = 0

    def initialize(self) -> bool:
        """Launch C++ stream subprocess, wait for READY, open shared memory."""
        if not self.binary_path or not os.path.isfile(self.binary_path):
            logger.error("DRP-AI binary not found")
            return False
        if not self.model_dir or not os.path.isdir(self.model_dir):
            logger.error("DRP-AI model dir not found")
            return False

        cmd = [self.binary_path, '--stream', self.model_dir]
        logger.info(f"Launching DRP-AI stream: {' '.join(cmd)}")

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            cwd=os.path.dirname(self.binary_path),
        )

        # Wait for READY
        start = time.time()
        while time.time() - start < self.startup_timeout:
            if self._proc.poll() is not None:
                stderr = self._proc.stderr.read().decode('utf-8', errors='replace')
                logger.error(f"Stream process died: {stderr}")
                return False
            try:
                line = self._proc.stdout.readline()
                if line:
                    text = line.decode('utf-8', errors='replace').strip()
                    if text == 'READY':
                        logger.info("DRP-AI stream ready")
                        break
                    else:
                        logger.debug(f"Stream startup: {text}")
            except Exception:
                pass
            time.sleep(0.1)
        else:
            logger.error("Stream startup timeout")
            self.shutdown()
            return False

        # Wait briefly for shared memory to be populated
        for _ in range(20):
            if os.path.exists(SHM_NAME):
                break
            time.sleep(0.1)

        if not os.path.exists(SHM_NAME):
            logger.error("Shared memory not created")
            self.shutdown()
            return False

        # Open shared memory
        try:
            self._shm_fd = os.open(SHM_NAME, os.O_RDONLY)
            shm_size = os.fstat(self._shm_fd).st_size
            self._shm = mmap.mmap(self._shm_fd, shm_size, access=mmap.ACCESS_READ)
            logger.info(f"Shared memory opened: {shm_size} bytes")
        except Exception as e:
            logger.error(f"Failed to open shared memory: {e}")
            self.shutdown()
            return False

        # Start JSON reader thread
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._read_json_loop, daemon=True)
        self._reader_thread.start()

        return True

    def _read_json_loop(self):
        """Background thread: read JSON detection lines from stdout."""
        while self._running and self._proc and self._proc.poll() is None:
            try:
                line = self._proc.stdout.readline()
                if not line:
                    break
                data = json.loads(line.decode('utf-8', errors='replace'))
                dets = data.get('detections', [])
                ms = data.get('inference_ms', 0.0)
                with self._lock:
                    self._latest_detections = dets
                    self._latest_infer_ms = ms
            except json.JSONDecodeError:
                continue
            except Exception:
                break
        logger.info("JSON reader thread stopped")

    def get_frame(self) -> Optional[np.ndarray]:
        """Read latest camera frame from shared memory. Returns BGR numpy array."""
        if self._shm is None:
            return None
        try:
            self._shm.seek(0)
            header = self._shm.read(SHM_HEADER_SIZE)
            if len(header) < SHM_HEADER_SIZE:
                return None
            w, h, seq, ts = struct.unpack('<IIqq', header)
            if w == 0 or h == 0 or seq == 0:
                return None
            if seq == self._last_seq:
                return None  # No new frame
            self._last_seq = seq
            nbytes = w * h * 3
            data = self._shm.read(nbytes)
            if len(data) < nbytes:
                return None
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3).copy()
        except Exception:
            return None

    def get_detections(self) -> Tuple[List[dict], float]:
        """Get latest detections and inference time."""
        with self._lock:
            return self._latest_detections[:], self._latest_infer_ms

    def shutdown(self):
        """Stop stream subprocess and clean up."""
        logger.info("Shutting down DRP-AI stream")
        self._running = False
        if self._shm is not None:
            try:
                self._shm.close()
            except Exception:
                pass
            self._shm = None
        if self._shm_fd >= 0:
            try:
                os.close(self._shm_fd)
            except Exception:
                pass
            self._shm_fd = -1
        if self._proc is not None:
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

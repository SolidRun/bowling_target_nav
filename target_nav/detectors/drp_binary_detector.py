"""
DRP-AI Detectors
================

Object detection using the V2N DRP-AI hardware accelerator via a persistent
C++ subprocess (yolo_detection binary).

Two detector classes are provided:

DrpBinaryDetector (pipe mode):
    Python owns the camera, sends BGR frames to the C++ binary via stdin,
    and reads JSON detection results from stdout. Suitable when Python needs
    direct camera access (e.g., for frame annotation before display).

    Protocol:
      Python -> C++ stdin:  [uint32 width][uint32 height][BGR pixel bytes]
      C++ stdout -> Python: {"detections":[...],"inference_ms":float}\\n
      Startup:              C++ writes "READY\\n" when model is loaded

DrpStreamDetector (stream mode):
    C++ owns the camera and runs multi-threaded capture + inference.
    Frames are written to /dev/shm/v2n_camera (zero-copy mmap read by Python).
    Detection JSON is streamed on stdout. This eliminates the ~921KB per-frame
    pipe transfer and is roughly 10x faster than pipe mode.

    Protocol:
      C++ stdout -> Python: {"detections":[...],"inference_ms":float}\\n
      C++ -> /dev/shm/v2n_camera: [uint32 w][uint32 h][uint64 seq][uint64 ts][BGR data]
      Startup:              C++ writes "READY\\n" when model is loaded

Helper functions find_drp_binary() and find_drp_model() search standard
V2N filesystem paths for the detection binary and model directory.
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
from typing import List, Optional, Tuple

import numpy as np

from target_nav.config import TARGET_CLASS_NAME
from .base import DetectorBase, Detection, DetectionResult

logger = logging.getLogger(__name__)


def _wait_for_ready(proc: subprocess.Popen, timeout: float, label: str) -> bool:
    """Wait for a subprocess to print 'READY' on stdout.

    Blocks until the subprocess writes a line containing exactly 'READY',
    the timeout elapses, or the process exits prematurely.

    Args:
        proc: The subprocess whose stdout to read.
        timeout: Maximum seconds to wait.
        label: Human-readable name for log messages (e.g. 'pipe', 'stream').

    Returns:
        True if READY was received within the timeout.

    Raises:
        RuntimeError: If the subprocess exits before sending READY.
        TimeoutError: If the timeout elapses without a READY signal.
    """
    start = time.time()
    while time.time() - start < timeout:
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode('utf-8', errors='replace')
            raise RuntimeError(
                f"DRP-AI {label} process exited with code {proc.returncode}: {stderr}")
        try:
            line = proc.stdout.readline()
            if line:
                text = line.decode('utf-8', errors='replace').strip()
                if text == 'READY':
                    logger.info("DRP-AI %s subprocess ready", label)
                    return True
                else:
                    logger.debug("DRP-AI %s startup output: %s", label, text)
        except Exception:
            pass
        time.sleep(0.1)

    raise TimeoutError(
        f"DRP-AI {label} subprocess did not become ready within {timeout}s")


def _terminate_process(proc: Optional[subprocess.Popen],
                       close_stdin: bool = False) -> None:
    """Terminate a subprocess gracefully (SIGTERM), then SIGKILL if needed.

    Args:
        proc: The subprocess to terminate. No-op if None.
        close_stdin: If True, close the process's stdin before sending signals.
    """
    if proc is None:
        return
    if close_stdin:
        try:
            proc.stdin.close()
        except Exception:
            pass
    try:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


# Default paths on V2N (root home is /root, not /home/root)
DRP_BINARY_PATHS = [
    '/root/deploy/app_yolo_cam',
    '/root/deploy/yolo_detection',
    '/opt/drp/yolo_detection',
    '/opt/drp/app_yolo_cam',
]

DRP_MODEL_PATHS = [
    '/root/deploy/model',
    '/root/deploy/drpai_model',
    '/opt/drp/drpai_model',
    '/opt/drp/model',
]


def find_drp_binary() -> Optional[str]:
    """Search standard V2N paths for the yolo_detection binary.

    Returns:
        Absolute path to the first executable binary found, or None.
    """
    for path in DRP_BINARY_PATHS:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None


def find_drp_model() -> Optional[str]:
    """Search standard V2N paths for a DRP-AI model directory.

    Validates that the directory contains a TVM subgraph (sub_*_CPU_DRP_TVM).

    Returns:
        Absolute path to the first valid model directory, or None.
    """
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

    Hardware-accelerated inference on DRP-AI coprocessor.

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
        target_class: str = TARGET_CLASS_NAME,
        class_names: Optional[List[str]] = None,
        startup_timeout: float = 30.0,
        **kwargs
    ):
        """Initialize the DRP-AI pipe detector.

        Args:
            binary_path: Path to yolo_detection binary. Auto-detected if None.
            model_dir: Path to DRP-AI model directory. Auto-detected if None.
            confidence_threshold: Minimum confidence to keep a detection.
            target_class: Primary target class name.
            class_names: List of class names indexed by class_id.
            startup_timeout: Seconds to wait for C++ READY signal.
            **kwargs: Passed to DetectorBase.
        """
        super().__init__(
            confidence_threshold=confidence_threshold,
            target_class=target_class,
            **kwargs
        )

        self.binary_path = binary_path or find_drp_binary()
        self.model_dir = model_dir or find_drp_model()
        self.class_names = class_names or [target_class]
        self.startup_timeout = startup_timeout
        self._proc = None

    @property
    def name(self) -> str:
        return "DRP-AI Pipe Detector"

    @property
    def supported_classes(self) -> List[str]:
        return self.class_names

    def _load_model(self) -> None:
        """Launch the C++ yolo_detection subprocess in pipe mode and wait for READY.

        Raises:
            FileNotFoundError: If binary or model directory is missing.
            PermissionError: If binary is not executable.
            TimeoutError: If subprocess does not send READY within startup_timeout.
            RuntimeError: If subprocess exits prematurely.
        """
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

        # Wait for READY signal (raises on timeout or premature exit)
        try:
            _wait_for_ready(self._proc, self.startup_timeout, 'pipe')
        except (TimeoutError, RuntimeError):
            self._kill_proc()
            raise

    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Send a BGR frame to the DRP-AI subprocess and parse JSON detections.

        Args:
            frame: Input image as numpy array (BGR, HxWx3).

        Returns:
            DetectionResult with parsed detections or error status.
        """
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
        """Terminate the C++ subprocess gracefully (SIGTERM), then SIGKILL if needed."""
        _terminate_process(self._proc, close_stdin=True)
        self._proc = None

    def shutdown(self) -> None:
        """Stop the DRP-AI subprocess and release resources."""
        logger.info("Shutting down DRP-AI subprocess")
        self._kill_proc()
        super().shutdown()


from target_nav.config import SHM_CAMERA_PATH
SHM_NAME = SHM_CAMERA_PATH
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
        """Initialize the DRP-AI stream detector.

        Args:
            binary_path: Path to yolo_detection binary. Auto-detected if None.
            model_dir: Path to DRP-AI model directory. Auto-detected if None.
            confidence_threshold: Minimum confidence for detections.
            class_names: List of class names indexed by class_id.
            startup_timeout: Seconds to wait for C++ READY signal.
        """
        self.binary_path = binary_path or find_drp_binary()
        self.model_dir = model_dir or find_drp_model()
        self.class_names = class_names or [TARGET_CLASS_NAME]
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
        """Launch C++ stream subprocess, wait for READY, open shared memory.

        Starts the yolo_detection binary with --stream flag, waits for
        the READY signal, maps /dev/shm/v2n_camera for frame access, and
        starts a background thread to read JSON detection lines.

        Returns:
            True if initialization succeeded, False on any failure.
        """
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

        # Wait for READY signal
        try:
            _wait_for_ready(self._proc, self.startup_timeout, 'stream')
        except (TimeoutError, RuntimeError) as e:
            logger.error("Stream startup failed: %s", e)
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
        """Background thread: continuously read JSON detection lines from C++ stdout.

        Runs until self._running is False or the subprocess exits. Updates
        self._latest_detections and self._latest_infer_ms under lock.

        Note:
            Thread safety: writes to _latest_detections/_latest_infer_ms are
            protected by self._lock. get_detections() reads under the same lock.
        """
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
        """Read the latest camera frame from shared memory with torn-frame detection.

        Uses double-read validation: reads the sequence number before and after
        the pixel data.  If seq changed mid-read, the C++ writer overwrote the
        buffer — the frame is discarded to prevent corrupted display.

        Thread safety: _last_seq is accessed only by the CameraDetection thread
        (via this method in camera_worker._run_stream_mode).  The _read_json_loop
        thread never touches this field.  No additional lock is needed.

        Returns:
            BGR numpy array (HxWx3) if a new, intact frame is available.
            None if no new frame, shared memory unavailable, or torn frame.
        """
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
            if w > 2048 or h > 2048:
                return None
            if seq == self._last_seq:
                return None  # No new frame

            nbytes = w * h * 3
            data = self._shm.read(nbytes)
            if len(data) != nbytes:
                return None

            # Re-read sequence number to detect torn frame
            self._shm.seek(8)  # offset of seq field in header
            seq_after = struct.unpack('<q', self._shm.read(8))[0]
            if seq_after != seq:
                return None  # Torn frame — C++ wrote new data mid-read

            self._last_seq = seq
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3).copy()
        except Exception:
            return None

    def get_detections(self) -> Tuple[List[dict], float]:
        """Get latest detections and inference time from the JSON reader thread.

        Returns:
            Tuple of (detections list, inference_time_ms). The detections list
            is a copy so callers can mutate it safely.
        """
        with self._lock:
            return self._latest_detections[:], self._latest_infer_ms

    def shutdown(self):
        """Stop the stream subprocess, close shared memory, and clean up resources.

        Joins the reader thread before terminating the subprocess to ensure
        no data race between the thread reading stdout and process termination.
        """
        logger.info("Shutting down DRP-AI stream")
        self._running = False
        # Join reader thread before killing subprocess — prevents race on stdout
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
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
        _terminate_process(self._proc)
        self._proc = None

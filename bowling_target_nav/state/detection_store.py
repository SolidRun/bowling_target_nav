"""Thread-safe storage for camera frames, detections, and tunable parameters."""

import threading
import time


class DetectionStore:
    """Thread-safe container for camera/detection data and tunable params."""

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
        self._camera_frame = None
        self._detections = []
        self._detection_info = "Initializing..."
        self._detection_time = 0.0

        # Tunable params (accessed by camera thread, modified by settings)
        self._detect_interval = 2.0
        self._detect_expiry = 1.5
        self._confidence_threshold = 0.35
        self._ref_box_height = 100.0
        self._ref_distance = 1.0

    def _try_lock(self, timeout=None):
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # -- Camera frame + detections --
    def set_camera(self, frame, detections, info, fresh_detection=False):
        if not self._try_lock():
            return False
        try:
            self._camera_frame = frame
            self._detections = detections
            self._detection_info = info
            if fresh_detection:
                self._detection_time = time.time()
            return True
        finally:
            self._release()

    def get_camera(self):
        if not self._try_lock():
            return None, [], "Lock timeout", 0.0
        try:
            frame = self._camera_frame.copy() if self._camera_frame is not None else None
            det_age = time.time() - self._detection_time if self._detection_time > 0 else 999.0
            return frame, self._detections[:], self._detection_info, det_age
        finally:
            self._release()

    # -- Tunable parameters --
    def set_detect_interval(self, val):
        if self._try_lock():
            try: self._detect_interval = val
            finally: self._release()

    def get_detect_interval(self):
        if self._try_lock():
            try: return self._detect_interval
            finally: self._release()
        return 2.0

    def set_detect_expiry(self, val):
        if self._try_lock():
            try: self._detect_expiry = val
            finally: self._release()

    def get_detect_expiry(self):
        if self._try_lock():
            try: return self._detect_expiry
            finally: self._release()
        return 1.5

    def set_confidence_threshold(self, val):
        if self._try_lock():
            try: self._confidence_threshold = val
            finally: self._release()

    def get_confidence_threshold(self):
        if self._try_lock():
            try: return self._confidence_threshold
            finally: self._release()
        return 0.35

    def set_calibration(self, box_height, distance):
        if self._try_lock():
            try:
                self._ref_box_height = box_height
                self._ref_distance = distance
            finally: self._release()

    def get_calibration(self):
        if self._try_lock():
            try: return self._ref_box_height, self._ref_distance
            finally: self._release()
        return 100.0, 1.0

"""SharedState facade composing domain-specific stores."""

import threading

from .sensor_store import SensorStore
from .detection_store import DetectionStore
from .nav_store import NavStore


class SharedState:
    """Thin facade over domain stores + lifecycle management.

    All three threads (ROS, Camera, GTK) import the singleton from state/__init__.py.
    """

    def __init__(self):
        self._shutdown_event = threading.Event()
        self.sensors = SensorStore()
        self.detection = DetectionStore()
        self.nav = NavStore()

        # ROS node reference (set by ros_thread, used by SettingsWindow)
        self._ros_node = None

        # Error tracking
        self._error_lock = threading.RLock()
        self._errors = []

    @property
    def running(self):
        return not self._shutdown_event.is_set()

    def request_shutdown(self):
        self._shutdown_event.set()

    def add_error(self, source, error):
        if self._error_lock.acquire(timeout=0.1):
            try:
                self._errors.append(f"{source}: {error}")
                if len(self._errors) > 10:
                    self._errors = self._errors[-10:]
            finally:
                self._error_lock.release()

    def get_errors(self):
        if self._error_lock.acquire(timeout=0.1):
            try:
                return self._errors[:]
            finally:
                self._error_lock.release()
        return []

    def clear_errors(self):
        if self._error_lock.acquire(timeout=0.1):
            try:
                self._errors.clear()
            finally:
                self._error_lock.release()

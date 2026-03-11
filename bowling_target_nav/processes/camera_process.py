"""Camera+Detection process — runs on a dedicated CPU core.

Owns:
    - Camera capture (V4L2 or DRP-AI stream)
    - DRP-AI inference (C++ subprocess on DRP-AI core)
    - Distance estimation

Syncs with other processes via IPCHub:
    - Writes: camera frame, detections, detector mode
    - Reads: settings_changed event (reloads calibration)
"""

import os
import signal
import time
import traceback

_TAG = "[Camera Process]"


def _log(msg):
    print(f"{_TAG} {msg}", flush=True)


class _CameraIPCBridge:
    """Adapter that wraps IPCHub to match the SharedState.detection API.

    The camera_worker.py module expects a ``shared_state.detection`` object
    with methods like set_camera(), get_calibration(), etc.  This bridge
    satisfies that interface by:
        - Routing frame/detection writes to IPCHub shared memory (so the
          GUI process can read them without ROS).
        - Routing calibration/setting reads to a process-local
          DetectionStore (loaded from calibration.json on disk).
        - Exposing reload_calibration() for the settings watcher thread.
    """

    def __init__(self, hub):
        """Initialize with IPCHub for writes and a local DetectionStore for reads.

        Args:
            hub: IPCHub instance shared across processes.
        """
        self._hub = hub
        # Local detection store for calibration/settings
        from bowling_target_nav.state.detection_store import DetectionStore
        self._local = DetectionStore()

    def set_camera(self, frame, detections, info, fresh_detection=False):
        """Write frame + detections to IPCHub shared memory."""
        if frame is not None:
            self._hub.write_frame(frame)
        # Only update detection data when a new inference result arrives.
        # Writing ts=0.0 on non-fresh frames would make the GUI think
        # detections are 999s old and drop them immediately.
        if fresh_detection:
            self._hub.write_detections(detections or [], info or "", time.time())

    def get_camera(self):
        """Not used by camera_worker, but provided for completeness."""
        frame, _ = self._hub.read_frame()
        dets, info, ts = self._hub.read_detections()
        det_age = time.time() - ts if ts > 0 else 999.0
        return frame, dets, info, det_age

    # Settings — delegate to local store
    def get_calibration(self):
        return self._local.get_calibration()

    def set_calibration(self, *args):
        self._local.set_calibration(*args)

    def is_calibration_dirty(self):
        return self._local.is_calibration_dirty()

    def get_camera_fov(self):
        return self._local.get_camera_fov()

    def get_confidence_threshold(self):
        return self._local.get_confidence_threshold()

    def get_detect_expiry(self):
        return self._local.get_detect_expiry()

    def get_filter_max_aspect(self):
        return self._local.get_filter_max_aspect()

    def get_filter_max_box_pct(self):
        return self._local.get_filter_max_box_pct()

    def get_filter_size_tolerance(self):
        return self._local.get_filter_size_tolerance()

    def get_filter_tracker_hits(self):
        return self._local.get_filter_tracker_hits()

    def get_filter_tracker_dist(self):
        return self._local.get_filter_tracker_dist()

    def get_filter_bbox_window(self):
        return self._local.get_filter_bbox_window()

    def get_filter_bbox_factor(self):
        return self._local.get_filter_bbox_factor()

    def request_bbox_reset(self):
        pass  # Camera side doesn't request, only checks

    def check_and_clear_bbox_reset(self):
        """Check if ROS process signaled a bbox tracker reset via IPC."""
        if self._hub.bbox_reset_cam.is_set():
            self._hub.bbox_reset_cam.clear()
            return True
        return False

    def set_stream_mode(self, enabled):
        self._local.set_stream_mode(enabled)

    def set_detector_mode(self, mode):
        self._local.set_detector_mode(mode)
        self._hub.write_detector_mode(mode)

    def get_detector_mode(self):
        return self._local.get_detector_mode()

    def reload_calibration(self):
        """Reload calibration from disk (called when settings_changed fires)."""
        self._local._load_calibration()
        self._local._calibration_dirty = True
        _log("Calibration reloaded from disk")


class _CameraStateProxy:
    """Minimal SharedState proxy for camera_thread().

    Provides the subset of the SharedState interface that camera_thread()
    actually uses:
        - ``running`` property (delegates to hub.shutdown).
        - ``detection`` attribute (_CameraIPCBridge instance).
        - ``add_error(source, msg)`` (forwards to hub error queue).
    """

    def __init__(self, hub):
        self._hub = hub
        self.detection = _CameraIPCBridge(hub)

    @property
    def running(self):
        return not self._hub.shutdown.is_set()

    def add_error(self, source, error):
        self._hub.add_error(source, error)


def camera_process_main(hub, core_id=2):
    """Camera process entry point.

    Lifecycle:
        1. Pin to CPU core, ignore SIGINT (parent handles signals).
        2. Create _CameraStateProxy wrapping the IPCHub.
        3. Start a settings watcher daemon thread that reloads calibration
           from disk when hub.settings_changed_cam fires.
        4. Run camera_thread(proxy) which blocks until shutdown.
        5. Close shm handles on exit.

    Args:
        hub: IPCHub instance (shared with other processes).
        core_id: CPU core to pin to (default 2).
    """
    _log("Starting...")

    # Pin to CPU core
    try:
        os.sched_setaffinity(0, {core_id})
        _log(f"Pinned to core {core_id}")
    except Exception:
        _log(f"Could not pin to core {core_id}")

    # Ignore SIGINT in child — parent handles it
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        # Create proxy state for camera_thread
        proxy = _CameraStateProxy(hub)

        # Import camera_thread (this also imports DRP-AI modules)
        from bowling_target_nav.threads.camera_worker import camera_thread

        # Start settings watcher in background
        import threading

        def _settings_watcher():
            while not hub.shutdown.is_set():
                if hub.settings_changed_cam.wait(timeout=1.0):
                    hub.settings_changed_cam.clear()
                    proxy.detection.reload_calibration()

        watcher = threading.Thread(
            target=_settings_watcher, name="CamSettingsWatch", daemon=True)
        watcher.start()

        # Run camera_thread (blocks until shutdown)
        camera_thread(proxy)

    except Exception as e:
        _log(f"Fatal error: {e}")
        traceback.print_exc()
        hub.add_error('camera_process', str(e))
    finally:
        # Close shared memory handles in child (without unlink)
        try:
            hub.close_child()
        except Exception:
            pass
        _log("Stopped")

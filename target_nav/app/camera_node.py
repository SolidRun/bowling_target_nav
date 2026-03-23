"""Camera detection ROS2 node.

Manages the DRP-AI detection pipeline and publishes results as ROS2 topics.
Communicates with the C++ DRP-AI binary via /dev/shm (not ROS2) because
the binary is a standalone process, not a ROS2 node.

Published topics:
    /detections (std_msgs/String): JSON-encoded detection list per frame.
    /detector_mode (std_msgs/String): Current detector mode ("Stream"/"Pipe").

Subscribed topics:
    /settings_changed (std_msgs/String): Reload calibration from disk.

Shared memory (C++ binary, NOT ROS2):
    /dev/shm/v2n_camera:       C++ writes annotated BGR frames (GUI reads directly).
    /dev/shm/v2n_detections:   C++ writes detection structs.
    /dev/shm/v2n_calibration:  Python writes calibration for C++ to pick up.
"""

import json
import os
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from target_nav.ipc.shm_struct import DetShmWriter
from target_nav.state.settings_store import SettingsStore
from target_nav.utils.log import get_logger

logger = get_logger('app.camera_node')


class _CameraStateAdapter:
    """Adapter that lets camera_worker.camera_thread() run inside a ROS2 node.

    camera_worker expects a ``shared_state`` object with:
        - ``running`` property (bool)
        - ``detection`` attribute with set_camera(), get_calibration(), etc.
        - ``add_error(source, msg)``

    This adapter satisfies that interface by:
        - Routing calibration/setting reads to a process-local SettingsStore.
        - Publishing detections to a ROS2 topic.
        - Keeping /dev/shm frame writes for the GUI to read directly.

    The ``detection`` attribute is ``self`` -- this class implements all the
    detection-store methods that camera_worker calls on ``shared_state.detection``.
    """

    def __init__(self, node: 'CameraNode'):
        """Initialize with a reference to the parent ROS2 node.

        Args:
            node: The CameraNode that owns this adapter.
        """
        self._node = node
        self._store = SettingsStore()
        self._shutdown = threading.Event()
        self._bbox_reset = threading.Event()
        self._drpai_restart = threading.Event()
        self._last_mode = ""
        # Expose self as the detection sub-object
        self.detection = self

    # ── SharedState interface ───────────────────────────────────────────

    @property
    def running(self):
        """Return True while the node has not been asked to shut down."""
        return not self._shutdown.is_set()

    def add_error(self, source, error):
        """Log an error from the detection pipeline.

        Args:
            source: Component name that raised the error.
            error: Human-readable error message.
        """
        logger.error("[%s] %s", source, error)

    def request_shutdown(self):
        """Signal the detection pipeline to stop."""
        self._shutdown.set()

    # ── Frame / detection writes ────────────────────────────────────────

    def set_camera(self, frame, detections, info, fresh_detection=False):
        """Publish detections to the ROS2 /detections topic.

        Frames are NOT published over ROS2 -- the GUI reads them directly
        from /dev/shm/v2n_camera written by the C++ binary.  Only detection
        metadata (tiny JSON) goes through the topic.

        Args:
            frame: BGR numpy array or None (ignored -- shm handles frames).
            detections: List of Det objects to publish.
            info: Human-readable summary string (unused by ROS2, kept for API compat).
            fresh_detection: If True, this is a new inference result worth publishing.
        """
        if not fresh_detection:
            return

        det_list = []
        for d in (detections or []):
            det_list.append({
                "confidence": round(d.confidence, 3),
                "bbox": list(d.bbox),
                "distance": round(d.distance, 3),
                "angle": round(d.angle, 4),
                "class_name": d.class_name,
                "bbox_clipped": bool(d.bbox_clipped),
            })

        # Write to struct SHM (GUI reads directly, no Bridge needed)
        try:
            self._node._det_shm.write(
                timestamp=time.time(),
                detections=det_list,
                info=info or '',
            )
        except Exception:
            pass  # SHM write failure must not break the pipeline

        # Publish to ROS2 /detections topic (nav_node subscribes)
        payload = {
            "detections": det_list,
            "info": info,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._node.detections_pub.publish(msg)

    def get_camera(self):
        """Read frame + detections (not used by camera_worker on the write side).

        Returns:
            Tuple of (None, [], '', 999.0) -- reads are not supported from this adapter.
        """
        return None, [], '', 999.0

    # ── Calibration / settings (delegate to local SettingsStore) ────────

    def get_calibration(self):
        """Return (ref_box_height, ref_distance) from the local settings store."""
        return self._store.get_calibration()

    def set_calibration(self, *args):
        """Set calibration reference values in the local settings store.

        Args:
            *args: Positional args forwarded to SettingsStore.set_calibration().
        """
        self._store.set_calibration(*args)

    def is_calibration_dirty(self):
        """Check and clear the calibration dirty flag.

        Returns:
            True if calibration was changed since the last check.
        """
        return self._store.is_calibration_dirty()

    def get_camera_fov(self):
        """Return the camera horizontal FOV in degrees."""
        return self._store.get_camera_fov()

    def get_confidence_threshold(self):
        """Return the minimum detection confidence threshold (0.0--1.0)."""
        return self._store.get_confidence_threshold()

    def get_detect_expiry(self):
        """Return the detection expiry time in seconds."""
        return self._store.get_detect_expiry()

    def request_bbox_reset(self):
        """No-op: the camera side only checks for resets, never requests them."""

    def check_and_clear_bbox_reset(self):
        """Check if a bbox tracker reset has been requested and clear the flag.

        Returns:
            True if a reset was requested since the last check.
        """
        if self._bbox_reset.is_set():
            self._bbox_reset.clear()
            return True
        return False

    def set_stream_mode(self, enabled):
        """Set the stream-mode flag in the local settings store.

        Args:
            enabled: True if C++ owns the camera (stream mode).
        """
        self._store.set_stream_mode(enabled)

    def set_detector_mode(self, mode):
        """Set the detector mode string and publish it to the /detector_mode topic.

        Args:
            mode: Human-readable mode string (e.g. "DRP-AI Stream", "DRP-AI Pipe").
        """
        self._store.set_detector_mode(mode)
        if mode != self._last_mode:
            self._last_mode = mode
            msg = String()
            msg.data = mode
            self._node.mode_pub.publish(msg)

    def get_detector_mode(self):
        """Return the current detector mode string."""
        return self._store.get_detector_mode()

    def reload_calibration(self):
        """Reload calibration from SHM (fast, ~1ms) or disk (fallback, ~100ms).

        Called when the /settings_changed topic fires.  Tries shared memory
        first for real-time sync, falls back to calibration.json on disk.
        """
        from target_nav.state.settings_store import SettingsShmReader
        if not hasattr(self, '_shm_reader'):
            self._shm_reader = SettingsShmReader()
        if not self._store.load_from_shm(self._shm_reader):
            self._store._load_calibration()
            logger.info("Calibration reloaded from disk (SHM unavailable)")
        self._store.mark_calibration_dirty()

    def check_and_clear_drpai_restart(self) -> bool:
        """Check and clear the DRP-AI restart flag.

        Set by CameraNode when it receives a /drpai_restart message from
        the GUI process.  camera_worker checks this in its main loop to
        restart the C++ subprocess with updated config.ini.

        Returns:
            True if a restart was requested since the last check.
        """
        if self._drpai_restart.is_set():
            self._drpai_restart.clear()
            return True
        return False

    def request_drpai_restart(self):
        """Request that the DRP-AI subprocess be restarted."""
        self._drpai_restart.set()


class CameraNode(Node):
    """ROS2 node that runs the DRP-AI detection pipeline.

    Publishes detection results as JSON on /detections and the current
    detector mode on /detector_mode.  Subscribes to /settings_changed to
    reload calibration when the GUI saves new values.

    The camera_worker detection loop runs in a background daemon thread,
    communicating back through the _CameraStateAdapter which publishes
    to ROS2 topics.
    """

    def __init__(self):
        """Initialize the CameraNode with publishers, subscriber, and state adapter."""
        super().__init__('camera_node')

        # Publishers
        self.detections_pub = self.create_publisher(String, '/detections', 10)
        self.mode_pub = self.create_publisher(String, '/detector_mode', 10)

        # Subscribers
        self.create_subscription(
            String, '/settings_changed', self._on_settings_changed, 10)
        self.create_subscription(
            String, '/drpai_restart', self._on_drpai_restart, 1)

        # State adapter for camera_worker
        self._adapter = _CameraStateAdapter(self)

        # Struct SHM writer for detections (camera -> GUI, bypasses Bridge)
        self._det_shm = DetShmWriter()

        # Detection thread (started in start())
        self._thread = None

        self.get_logger().info("CameraNode initialized")

    def start(self):
        """Start the camera detection pipeline in a background thread.

        The thread runs camera_worker.camera_thread() with the adapter as
        its shared_state.
        """
        self._thread = threading.Thread(
            target=self._run_detection, name="CameraDetection", daemon=True)
        self._thread.start()
        self.get_logger().info("Detection thread started")

    def _run_detection(self):
        """Run the camera_worker detection pipeline (blocks until shutdown).

        This is the thread target.  It imports camera_thread lazily to
        avoid loading DRP-AI modules at import time, then calls it with
        the adapter.
        """
        try:
            from target_nav.app.camera_worker import camera_thread
            camera_thread(self._adapter)
        except Exception as e:
            self.get_logger().error("Detection thread fatal: %s" % e)
            traceback.print_exc()

    def _on_settings_changed(self, msg):
        """Handle /settings_changed messages by reloading calibration from disk.

        Args:
            msg: std_msgs/String message (payload is ignored).
        """
        self.get_logger().info("Settings changed, reloading calibration")
        self._adapter.detection.reload_calibration()

    def _on_drpai_restart(self, msg):
        """Handle /drpai_restart messages by signaling the camera worker.

        The camera worker checks ``check_and_clear_drpai_restart()`` in its
        main loop and restarts the C++ DRP-AI subprocess when the flag is set.

        Args:
            msg: std_msgs/String message (payload is ignored).
        """
        self.get_logger().info("DRP-AI restart requested via ROS2 topic")
        self._adapter.request_drpai_restart()

    def destroy_node(self):
        """Shut down the detection thread and clean up resources."""
        self.get_logger().info("Shutting down...")
        self._adapter.request_shutdown()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                self.get_logger().warning("Detection thread did not stop in 5s")
        self._det_shm.close()
        super().destroy_node()


def main(args=None):
    """Entry point for the camera_node ROS2 node.

    Initializes rclpy, pins the process to CPU core 2 for deterministic
    scheduling alongside the DRP-AI accelerator, creates the CameraNode,
    starts the detection thread, and spins until SIGINT / shutdown.
    """
    # Pin to CPU core 2
    try:
        os.sched_setaffinity(0, {2})
        logger.info("Pinned to core 2")
    except Exception:
        logger.warning("Could not pin to core 2")

    rclpy.init(args=args)
    node = CameraNode()

    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        logger.info("camera_node stopped")


if __name__ == '__main__':
    main()

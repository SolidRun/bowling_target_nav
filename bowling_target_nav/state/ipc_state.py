"""IPC-backed state facades for the GUI process.

These classes expose the same API as SensorStore, DetectionStore, NavStore
but read from/write to IPCHub shared memory instead of local thread-safe stores.

The GUI process uses these to read data produced by the ROS and Camera processes.
Settings (calibration, map params, nav params) stay local in DetectionStore
since the GUI owns them and syncs via calibration file + settings_changed event.
"""

import math
import os
import struct
import time

import numpy as np


class IPCSensorStore:
    """GUI-side sensor store that reads from IPCHub shared memory.

    Provides the same read API as SensorStore (get_map, get_robot_pose,
    get_laser, get_diagnostics) but backed by IPCHub.  Caches map and
    laser data using sequence numbers to avoid redundant multi-MB copies.
    """

    def __init__(self, hub):
        """Initialize with a reference to the shared IPCHub.

        Args:
            hub: IPCHub instance created by the main process.
        """
        self._hub = hub
        self._last_map_seq = -1
        self._cached_map = None
        self._cached_map_info = None
        self._last_laser_seq = -1
        self._cached_laser = None
        self._cached_laser_time = 0.0

    def get_map(self):
        """Returns (map_img, MapInfo, count).

        Caches the map image and only re-reads from shared memory when
        the sequence number changes, avoiding expensive ~12MB copies at 30 FPS.
        Uses a single lock-protected call to avoid TOCTOU race conditions.
        """
        img, info, seq = self._hub.read_map()
        if img is None:
            if self._cached_map is not None:
                return self._cached_map.copy(), self._cached_map_info, self._last_map_seq
            return None, None, 0
        if seq == self._last_map_seq and self._cached_map is not None:
            return self._cached_map.copy(), self._cached_map_info, seq
        self._last_map_seq = seq
        self._cached_map = img
        self._cached_map_info = info
        return img.copy(), info, seq

    def get_robot_pose(self):
        """Returns (x, y, theta)."""
        return self._hub.read_pose()

    def get_laser(self):
        """Returns (points, count, laser_time).

        Caches laser points and only re-reads when sequence changes.
        Uses a single lock-protected call to avoid TOCTOU race conditions.
        """
        pts, seq, lt = self._hub.read_laser()
        if seq == self._last_laser_seq and self._cached_laser is not None:
            return self._cached_laser.copy(), seq, self._cached_laser_time
        self._last_laser_seq = seq
        self._cached_laser = pts
        self._cached_laser_time = lt
        return pts.copy(), seq, lt

    def get_diagnostics(self):
        """Return diagnostic snapshot from ROS process."""
        return self._hub.read_diagnostics()

    def get_lidar_distance_at_angle(self, angle_rad, window=0.15):
        """Not available in GUI process — returns inf."""
        return float('inf')


class IPCNavStore:
    """GUI-side nav store that reads from IPCHub and writes commands.

    Read methods delegate to hub.read_nav_snapshot().  Write methods
    (request_go, request_stop) send commands via hub.cmd_queue.
    Mutator methods used only by the navigator (set_nav_state, set_obstacle,
    etc.) are no-ops since the GUI process does not own navigation.
    """

    def __init__(self, hub):
        """Initialize with a reference to the shared IPCHub.

        Args:
            hub: IPCHub instance created by the main process.
        """
        self._hub = hub

    def request_go(self):
        """Send a 'go' command to the ROS process via the command queue."""
        try:
            self._hub.cmd_queue.put_nowait({'type': 'go'})
        except Exception:
            pass

    def request_stop(self):
        """Send an emergency stop to the ROS process.

        Uses both the mp.Event (never dropped, even if queue is full)
        and the command queue (for the nav state machine).
        """
        self._hub.emergency_stop.set()
        try:
            self._hub.cmd_queue.put_nowait({'type': 'stop'})
        except Exception:
            pass  # event guarantees delivery even if queue is full

    def get_nav_state(self):
        """Returns (state_str, nav_target)."""
        snap = self._hub.read_nav_snapshot()
        return snap['nav_state'], snap['nav_target']

    def get_gui_snapshot(self):
        """Return all GUI-needed data in one call."""
        return self._hub.read_nav_snapshot()

    def get_nav_target_map(self):
        """Return (x, y) target position in map frame, or None."""
        snap = self._hub.read_nav_snapshot()
        return snap['nav_target_map']

    def get_current_cmd_vel(self):
        """Return (vx, vy, wz) current velocity command tuple."""
        snap = self._hub.read_nav_snapshot()
        return snap['cmd_vel']

    def get_lidar_at_target(self):
        """Return LiDAR distance at target angle from ROS process."""
        snap = self._hub.read_nav_snapshot()
        return snap.get('lidar_at_target', float('inf'))

    # These are no-ops in GUI process (nav is controlled by ROS process)
    def set_nav_state(self, *args, **kwargs):
        pass

    def set_obstacle(self, *args, **kwargs):
        pass

    def set_nav_target_map(self, *args, **kwargs):
        pass

    def clear_nav_target_map(self):
        pass

    def set_current_cmd_vel(self, *args, **kwargs):
        pass

    def update_target_seen(self):
        pass

    def start_search(self):
        pass


class _DirectShmReader:
    """Reads camera frames directly from C++ DRP-AI shared memory.

    In stream mode, C++ writes annotated BGR frames to /dev/shm/v2n_camera.
    This reader lets the GUI read them directly, bypassing the camera
    process IPC entirely (eliminates ~5 copies of 921KB per frame).
    """

    SHM_PATH = '/dev/shm/v2n_camera'
    HEADER_SIZE = 24  # uint32 w + uint32 h + int64 seq + int64 ts

    def __init__(self):
        self._fd = None
        self._mmap = None
        self._last_seq = -1
        self._cached_frame = None

    def read_frame(self):
        """Read latest BGR frame from C++ shm, convert to RGB.

        Returns cached frame if no new frame available.
        Returns None if shm not available.
        """
        # Lazy open
        if self._mmap is None:
            if not os.path.exists(self.SHM_PATH):
                return self._cached_frame
            try:
                import mmap
                self._fd = os.open(self.SHM_PATH, os.O_RDONLY)
                size = os.fstat(self._fd).st_size
                if size < self.HEADER_SIZE:
                    os.close(self._fd)
                    self._fd = None
                    return None
                self._mmap = mmap.mmap(self._fd, size, access=mmap.ACCESS_READ)
            except Exception:
                if self._fd is not None:
                    try:
                        os.close(self._fd)
                    except Exception:
                        pass
                    self._fd = None
                return self._cached_frame

        try:
            self._mmap.seek(0)
            header = self._mmap.read(self.HEADER_SIZE)
            if len(header) < self.HEADER_SIZE:
                return self._cached_frame
            w, h, seq, ts = struct.unpack('<IIqq', header)
            if w == 0 or h == 0 or seq == 0:
                return self._cached_frame
            # Reject unreasonable dimensions (max 2048x2048)
            if w > 2048 or h > 2048:
                return self._cached_frame
            if seq == self._last_seq:
                return self._cached_frame  # No new frame
            self._last_seq = seq
            nbytes = w * h * 3
            data = self._mmap.read(nbytes)
            if len(data) < nbytes:
                return self._cached_frame
            # BGR→RGB conversion: single copy
            bgr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            self._cached_frame = bgr[:, :, ::-1].copy()
            return self._cached_frame
        except Exception:
            return self._cached_frame

    def close(self):
        if self._mmap is not None:
            try:
                self._mmap.close()
            except Exception:
                pass
            self._mmap = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = None


class IPCDetectionStore:
    """GUI-side detection store.

    Reads frame+detections from IPCHub shared memory.
    In stream mode, reads frames directly from C++ shm for zero-latency video.
    Delegates settings (calibration, map_params, nav_params) to a local
    DetectionStore instance that the GUI owns.
    """

    def __init__(self, hub, local_detection_store):
        """Initialize the detection store facade.

        Args:
            hub: IPCHub instance for frame/detection reads.
            local_detection_store: DetectionStore instance that owns
                calibration and parameter settings on the GUI side.
        """
        self._hub = hub
        self._local = local_detection_store
        self._last_frame_seq = -1
        self._cached_frame = None
        self._direct_reader = _DirectShmReader()
        self._stream_mode = None  # None = not checked yet

    # -- Frame + detections (from Camera process via IPC) --

    def get_camera(self):
        """Returns (frame_rgb, detections, info_str, det_age).

        In stream mode: reads frame directly from C++ shm (1 copy).
        In pipe mode: reads from IPC shm with seq caching (1 copy).
        """
        # Check stream mode (cached after first check)
        if self._stream_mode is None:
            mode = self._hub.read_detector_mode()
            if mode and mode != 'Initializing':
                self._stream_mode = 'Stream' in mode

        # Read frame
        if self._stream_mode:
            # Direct from C++ shm — bypasses camera→IPC→GUI path
            frame = self._direct_reader.read_frame()
        else:
            # Pipe mode — read from IPC with lock-protected call
            ipc_frame, seq = self._hub.read_frame()
            if ipc_frame is not None and seq != self._last_frame_seq:
                self._cached_frame = ipc_frame
                self._last_frame_seq = seq
            frame = self._cached_frame

        dets, info, ts = self._hub.read_detections()
        det_age = time.time() - ts if ts > 0 else 999.0
        return frame, dets, info, det_age

    def get_detector_mode(self):
        """Return the current detector mode string (e.g. 'Stream', 'Pipe')."""
        return self._hub.read_detector_mode()

    # -- All settings methods delegate to local DetectionStore --

    def get_calibration(self):
        return self._local.get_calibration()

    def set_calibration(self, box_height, distance):
        self._local.set_calibration(box_height, distance)

    def is_calibration_dirty(self):
        return self._local.is_calibration_dirty()

    def get_camera_fov(self):
        return self._local.get_camera_fov()

    def set_camera_fov(self, fov_deg):
        self._local.set_camera_fov(fov_deg)

    def get_camera_pos(self):
        return self._local.get_camera_pos()

    def set_camera_pos(self, x, y, z):
        self._local.set_camera_pos(x, y, z)

    def get_camera_yaw(self):
        return self._local.get_camera_yaw()

    def set_camera_yaw(self, yaw_deg):
        self._local.set_camera_yaw(yaw_deg)

    def get_lidar_pos(self):
        return self._local.get_lidar_pos()

    def set_lidar_pos(self, x, y, z):
        self._local.set_lidar_pos(x, y, z)

    def get_lidar_yaw(self):
        return self._local.get_lidar_yaw()

    def set_lidar_yaw(self, yaw_deg):
        self._local.set_lidar_yaw(yaw_deg)

    def get_robot_dims(self):
        return self._local.get_robot_dims()

    def set_robot_dims(self, length, width, height):
        self._local.set_robot_dims(length, width, height)

    def get_robot_half_width(self):
        return self._local.get_robot_half_width()

    def get_target_dims(self):
        return self._local.get_target_dims()

    def set_target_dims(self, height, width):
        self._local.set_target_dims(height, width)

    def get_ref_point(self):
        return self._local.get_ref_point()

    def set_ref_point(self, point):
        self._local.set_ref_point(point)

    def get_sensor_offset(self, source):
        return self._local.get_sensor_offset(source)

    def get_map_param(self, key):
        return self._local.get_map_param(key)

    def set_map_param(self, key, value):
        self._local.set_map_param(key, value)

    def get_map_params(self):
        return self._local.get_map_params()

    def get_nav_param(self, key):
        return self._local.get_nav_param(key)

    def set_nav_param(self, key, value):
        self._local.set_nav_param(key, value)

    def get_nav_params(self):
        return self._local.get_nav_params()

    def get_detect_expiry(self):
        return self._local.get_detect_expiry()

    def set_detect_expiry(self, val):
        self._local.set_detect_expiry(val)

    def get_detect_interval(self):
        return self._local.get_detect_interval()

    def set_detect_interval(self, val):
        self._local.set_detect_interval(val)

    def get_confidence_threshold(self):
        return self._local.get_confidence_threshold()

    def set_confidence_threshold(self, val):
        self._local.set_confidence_threshold(val)

    def get_filter_max_aspect(self):
        return self._local.get_filter_max_aspect()

    def set_filter_max_aspect(self, val):
        self._local.set_filter_max_aspect(val)

    def get_filter_max_box_pct(self):
        return self._local.get_filter_max_box_pct()

    def set_filter_max_box_pct(self, val):
        self._local.set_filter_max_box_pct(val)

    def get_filter_size_tolerance(self):
        return self._local.get_filter_size_tolerance()

    def set_filter_size_tolerance(self, val):
        self._local.set_filter_size_tolerance(val)

    def get_filter_tracker_hits(self):
        return self._local.get_filter_tracker_hits()

    def set_filter_tracker_hits(self, val):
        self._local.set_filter_tracker_hits(val)

    def get_filter_tracker_dist(self):
        return self._local.get_filter_tracker_dist()

    def set_filter_tracker_dist(self, val):
        self._local.set_filter_tracker_dist(val)

    def get_filter_bbox_window(self):
        return self._local.get_filter_bbox_window()

    def set_filter_bbox_window(self, val):
        self._local.set_filter_bbox_window(val)

    def get_filter_bbox_factor(self):
        return self._local.get_filter_bbox_factor()

    def set_filter_bbox_factor(self, val):
        self._local.set_filter_bbox_factor(val)

    def request_bbox_reset(self):
        self._local.request_bbox_reset()

    def check_and_clear_bbox_reset(self):
        return self._local.check_and_clear_bbox_reset()

    def get_stream_mode(self):
        return self._local.get_stream_mode()

    def set_stream_mode(self, enabled):
        self._local.set_stream_mode(enabled)

    def set_detector_mode(self, mode):
        self._local.set_detector_mode(mode)

    def compute_expected_box_height(self, distance_m):
        return self._local.compute_expected_box_height(distance_m)

    def save_calibration(self):
        """Save calibration to disk and notify ROS + Camera processes to reload.

        Returns:
            True on success, False on failure.
        """
        result = self._local.save_calibration()
        # Notify both consumer processes to reload
        self._hub.settings_changed_ros.set()
        self._hub.settings_changed_cam.set()
        return result


class IPCSharedState:
    """GUI-process SharedState replacement backed by IPCHub.

    Provides the same interface as SharedState:
        .sensors  — IPCSensorStore
        .detection — IPCDetectionStore
        .nav — IPCNavStore
        .running, .request_shutdown(), .add_error(), .get_errors()
    """

    def __init__(self, hub):
        """Initialize GUI-process shared state backed by IPCHub.

        Args:
            hub: IPCHub instance created by the main process.
        """
        self._hub = hub
        self._ros_node = None  # Not available in GUI process

        # Local detection store for settings (calibration, map/nav params)
        from .detection_store import DetectionStore
        self._local_detection = DetectionStore()

        self.sensors = IPCSensorStore(hub)
        self.detection = IPCDetectionStore(hub, self._local_detection)
        self.nav = IPCNavStore(hub)

        self._accumulated_errors = []

    @property
    def running(self):
        return not self._hub.shutdown.is_set()

    def request_shutdown(self):
        self._hub.shutdown.set()

    def add_error(self, source, error):
        self._hub.add_error(source, error)

    def get_errors(self):
        new = self._hub.get_errors()
        self._accumulated_errors.extend(new)
        if len(self._accumulated_errors) > 10:
            self._accumulated_errors = self._accumulated_errors[-10:]
        return self._accumulated_errors[:]

    def clear_errors(self):
        self._hub.get_errors()  # drain
        self._accumulated_errors.clear()

    def send_ros_command(self, cmd_dict):
        """Send a command to the ROS process via IPC queue.

        Used by settings_window for motor tests, arduino commands, etc.
        """
        try:
            self._hub.cmd_queue.put_nowait(cmd_dict)
        except Exception:
            pass

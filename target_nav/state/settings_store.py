"""Unified settings store for all target_nav persisted and runtime state.

SettingsStore holds all detection, calibration, geometry, navigation, and
map display state behind a single RLock.

Architecture:
    SharedState owns one SettingsStore instance used by the GUI. Nav and
    Camera processes each create their own SettingsStore for local settings;
    setting changes are communicated via CmdRingBuffer (struct SHM) for Nav
    and /settings_changed ROS2 topic for Camera.

Thread safety:
    A single RLock protects all state.  ``_try_lock()`` uses a 0.1 s timeout
    so the GTK main thread is never blocked indefinitely.  Operations that
    span multiple data groups (e.g. ``set_target_dims`` + ``mark_dirty``)
    are guarded by ``_config_lock`` for atomicity.

Calibration workflow:
    1. Distance calibration (Settings > Calibration): record bounding-box
       height at a known distance, or set reference values manually.
    2. Sensor mounting (Settings > Sensors): camera/LiDAR position and yaw
       relative to base_link, affecting TF2 transforms and distance fusion.
    3. Navigation tuning (Settings > Navigation): approach distance, speeds,
       obstacle thresholds, blind approach and spiral search parameters.
    4. Map display (Settings > Map): rotation, overlay toggles, marker sizes.

Settings file format (calibration.json)::

    {
        "ref_box_height": 180.0,
        "ref_distance": 1.0,
        "camera_fov": 60.0,
        "camera_pos": {"x": 0.12, "y": 0.0, "z": 0.10},
        "camera_yaw": 0.0,
        "lidar": {"x": 0.0, "y": 0.0, "z": 0.15},
        "lidar_yaw": 180.0,
        "robot": {"length": 0.25, "width": 0.26, "height": 0.15},
        "target": {"height": 0.25, "width": 0.07},
        "ref_point": "front",
        "nav_params": { ... },
        "map_params": { ... },
        "drpai_params": { "conf_threshold": 0.70, "nms_threshold": 0.65, ... }
    }
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any

import mmap as mmap_mod
import struct

from target_nav.config import (
    CALIBRATION_FILE, PARAM_RANGES, validate_param,
    DEFAULT_CAMERA,
    DEFAULT_CAMERA_FOV,
    DEFAULT_CAMERA_YAW,
    DEFAULT_CAMERA_FLIP_H,
    DEFAULT_DETECT_PARAMS,
    DEFAULT_DRPAI_PARAMS,
    DEFAULT_FRAME_H,
    DEFAULT_FRAME_W,
    DEFAULT_LIDAR,
    DEFAULT_LIDAR_YAW,
    DEFAULT_MAP_PARAMS,
    DEFAULT_NAV_PARAMS,
    DEFAULT_REF_BOX_HEIGHT,
    DEFAULT_REF_DISTANCE,
    DEFAULT_REF_POINT,
    DEFAULT_ROBOT,
    DEFAULT_TARGET,
    DRPAI_CONFIG_INI_PATH,
    LIDAR_MIN_RANGE,
    REF_POINTS,
    SHM_SETTINGS_MAX_SIZE,
    SHM_SETTINGS_PATH,
    STORE_LOCK_TIMEOUT,
)
from target_nav.utils.log import get_logger

logger = get_logger('state.settings_store')


class SettingsShmWriter:
    """Write serialized settings JSON to /dev/shm/v2n_settings for cross-process sync.

    Layout: [uint32 seq][uint32 json_len][json_bytes...]

    The GUI process writes here after save_calibration().  Nav and Camera
    processes read from this SHM on /settings_changed topic notification,
    avoiding the 100-150ms disk round-trip through calibration.json.
    """

    def __init__(self):
        self._fd = -1
        self._mmap = None
        self._seq = 0

    def write(self, json_str: str) -> bool:
        """Write settings JSON to shared memory.

        Args:
            json_str: JSON-encoded settings string.

        Returns:
            True on success, False if the data exceeds SHM size or I/O fails.
        """
        try:
            data = json_str.encode('utf-8')
            total = 8 + len(data)
            if total > SHM_SETTINGS_MAX_SIZE:
                logger.warning("Settings JSON too large for SHM: %d bytes", total)
                return False
            if self._mmap is None:
                self._fd = os.open(SHM_SETTINGS_PATH,
                                   os.O_CREAT | os.O_RDWR, 0o666)
                os.ftruncate(self._fd, SHM_SETTINGS_MAX_SIZE)
                self._mmap = mmap_mod.mmap(self._fd, SHM_SETTINGS_MAX_SIZE)
            self._seq += 1
            header = struct.pack('<II', self._seq, len(data))
            self._mmap.seek(0)
            self._mmap.write(header + data)
            self._mmap.flush()
            return True
        except Exception as e:
            logger.warning("SHM settings write failed: %s", e)
            return False

    def close(self):
        """Release shared memory resources."""
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
            self._mmap = None
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = -1


class SettingsShmReader:
    """Read settings JSON from /dev/shm/v2n_settings (cross-process sync).

    Used by Nav and Camera processes to pick up settings changes written
    by the GUI process with ~1ms latency instead of 100-150ms disk I/O.
    Falls back gracefully if SHM is not available (e.g., first boot).
    """

    def __init__(self):
        self._fd = -1
        self._mmap = None
        self._last_seq = 0

    def read(self) -> 'dict | None':
        """Read settings dict if the sequence number has advanced.

        Returns:
            Parsed dict from JSON, or None if no new data / not available.
        """
        try:
            if self._mmap is None:
                if not os.path.exists(SHM_SETTINGS_PATH):
                    return None
                self._fd = os.open(SHM_SETTINGS_PATH, os.O_RDONLY)
                size = os.fstat(self._fd).st_size
                if size < 8:
                    return None
                self._mmap = mmap_mod.mmap(self._fd, size,
                                           access=mmap_mod.ACCESS_READ)
            self._mmap.seek(0)
            header = self._mmap.read(8)
            seq, json_len = struct.unpack('<II', header)
            if seq == 0 or seq == self._last_seq:
                return None  # No new data
            if json_len == 0 or json_len > SHM_SETTINGS_MAX_SIZE - 8:
                return None
            data = self._mmap.read(json_len)

            # Double-read validation: re-read seq to detect torn write
            self._mmap.seek(0)
            seq_check = struct.unpack('<I', self._mmap.read(4))[0]
            if seq_check != seq:
                return None  # Writer was mid-update

            self._last_seq = seq
            return json.loads(data.decode('utf-8'))
        except Exception as e:
            logger.debug("SHM settings read failed: %s", e)
            return None

    def close(self):
        """Release shared memory resources."""
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
            self._mmap = None
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = -1


class SettingsStore:
    """Unified store for all detection, calibration, and tuning state.

    Holds all runtime and persisted state behind a single RLock.  The public
    Used by GUI panels, Navigator, and camera worker for all detection,
    calibration, and tuning state.

    Thread safety:
        A single RLock with 0.1 s timeout protects all internal state.
        ``_config_lock`` guards multi-section mutations that must be atomic.
    """

    LOCK_TIMEOUT = STORE_LOCK_TIMEOUT

    def __init__(self) -> None:
        """Initialize all state to defaults and load from disk."""
        self._lock = threading.RLock()
        self._config_lock = threading.Lock()

        # -- Camera data (real-time, not persisted) --
        self._camera_frame: Any = None
        self._detections: list = []
        self._detection_info: str = "Initializing..."
        self._detection_time: float = 0.0

        # -- Detection filter parameters --
        d = DEFAULT_DETECT_PARAMS
        self._detect_interval: float = d['detect_interval']
        self._detect_expiry: float = d['detect_expiry']
        self._confidence_threshold: float = d['confidence_threshold']
        self._filter_max_aspect: float = d['filter_max_aspect']
        self._filter_max_box_pct: float = d['filter_max_box_pct']
        self._filter_size_tolerance: float = 2.0   # compat only
        self._filter_tracker_hits: int = 2          # compat only
        self._filter_tracker_dist: int = 80         # compat only
        self._filter_bbox_window: int = d['filter_bbox_window']
        self._filter_bbox_factor: float = d['filter_bbox_factor']
        self._bbox_reset_requested: bool = False

        # -- Distance calibration --
        self._ref_box_height: float = DEFAULT_REF_BOX_HEIGHT
        self._ref_distance: float = DEFAULT_REF_DISTANCE
        self._calibration_dirty: bool = False

        # -- Robot geometry --
        self._robot: dict = dict(DEFAULT_ROBOT)
        self._lidar: dict = dict(DEFAULT_LIDAR)
        self._lidar_yaw: float = DEFAULT_LIDAR_YAW
        self._camera: dict = dict(DEFAULT_CAMERA)
        self._camera_yaw: float = DEFAULT_CAMERA_YAW
        self._camera_flip_h: bool = DEFAULT_CAMERA_FLIP_H 
        self._target: dict = dict(DEFAULT_TARGET)
        self._ref_point: str = DEFAULT_REF_POINT

        # -- Camera optics --
        self._camera_fov: float = DEFAULT_CAMERA_FOV
        self._frame_w: int = DEFAULT_FRAME_W
        self._frame_h: int = DEFAULT_FRAME_H

        # -- Map display --
        self._map_params: dict = dict(DEFAULT_MAP_PARAMS)

        # -- Navigation parameters --
        self._nav_params: dict = dict(DEFAULT_NAV_PARAMS)

        # -- DRP-AI C++ binary parameters (persisted, written to config.ini) --
        self._drpai_params: dict = dict(DEFAULT_DRPAI_PARAMS)
        self._drpai_restart_requested: bool = False

        # -- Detector status (not persisted) --
        self._stream_mode: bool = False
        self._detector_mode: str = "Initializing"

        # Load persisted settings from disk
        self._load_calibration()

    # -- Lock helpers --

    def _try_lock(self, timeout: float | None = None) -> bool:
        """Attempt to acquire the lock within *timeout* seconds.

        Args:
            timeout: Override for LOCK_TIMEOUT.  If None, uses 0.1 s.

        Returns:
            True if the lock was acquired, False on timeout.
        """
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self) -> None:
        """Release the lock, silently ignoring RuntimeError if not held."""
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # ===================================================================
    # Camera data (real-time frame + detections)
    # ===================================================================

    def set_camera(self, frame: Any, detections: list,
                   info: str, fresh_detection: bool = False) -> bool:
        """Store latest camera frame and detection results.

        Args:
            frame: RGB numpy array (H, W, 3) uint8, or None.
            detections: List of Det objects from the detector.
            info: Human-readable detection info string.
            fresh_detection: If True, records the current time as
                the detection timestamp (used for staleness checks).

        Returns:
            True on success, False if the lock timed out.
        """
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

    def get_camera(self) -> tuple[Any, list, str, float]:
        """Return (frame_copy, detections_copy, info_str, det_age_seconds).

        Returns a copy of the frame to prevent concurrent mutation.
        ``det_age`` is seconds since the last fresh detection, or 999.0
        if no detection has been recorded yet.
        """
        if not self._try_lock():
            return None, [], "Lock timeout", 0.0
        try:
            frame = self._camera_frame.copy() if self._camera_frame is not None else None
            det_age = time.time() - self._detection_time if self._detection_time > 0 else 999.0
            return frame, self._detections[:], self._detection_info, det_age
        finally:
            self._release()

    # ===================================================================
    # Detection filters
    # ===================================================================

    def set_detect_expiry(self, val: float) -> None:
        """Set detection expiry time in seconds (minimum 0.3 s)."""
        if self._try_lock():
            try:
                self._detect_expiry = max(0.3, float(val))
            finally:
                self._release()

    def get_detect_expiry(self) -> float:
        """Return detection expiry time in seconds."""
        if self._try_lock():
            try:
                return self._detect_expiry
            finally:
                self._release()
        return 1.0

    def set_confidence_threshold(self, val: float) -> None:
        """Set minimum confidence score (clamped to [0.05, 0.95])."""
        if self._try_lock():
            try:
                self._confidence_threshold = max(0.05, min(0.95, float(val)))
            finally:
                self._release()

    def get_confidence_threshold(self) -> float:
        """Return minimum confidence score threshold."""
        if self._try_lock():
            try:
                return self._confidence_threshold
            finally:
                self._release()
        return 0.30

    def request_bbox_reset(self) -> None:
        """Signal the camera worker to reset its bbox stability tracker."""
        if self._try_lock():
            try:
                self._bbox_reset_requested = True
            finally:
                self._release()

    def check_and_clear_bbox_reset(self) -> bool:
        """Check and atomically clear the bbox reset flag.

        Returns:
            True if a reset was requested since the last check.
        """
        if self._try_lock():
            try:
                val = self._bbox_reset_requested
                self._bbox_reset_requested = False
                return val
            finally:
                self._release()
        return False

    # ===================================================================
    # Distance calibration
    # ===================================================================

    def set_calibration(self, box_height: float, distance: float) -> None:
        """Set vision distance calibration reference values and mark dirty.

        Args:
            box_height: Bounding-box height in pixels at the reference distance.
            distance: Reference distance in meters.
        """
        if self._try_lock():
            try:
                self._ref_box_height = box_height
                self._ref_distance = distance
                self._calibration_dirty = True
            finally:
                self._release()

    def get_calibration(self) -> tuple[float, float]:
        """Return (ref_box_height, ref_distance) calibration pair."""
        if self._try_lock():
            try:
                return self._ref_box_height, self._ref_distance
            finally:
                self._release()
        return DEFAULT_REF_BOX_HEIGHT, DEFAULT_REF_DISTANCE

    def is_calibration_dirty(self) -> bool:
        """Check and clear the calibration dirty flag.

        Returns:
            True if calibration was modified since the last check.
        """
        if self._try_lock():
            try:
                dirty = self._calibration_dirty
                self._calibration_dirty = False
                return dirty
            finally:
                self._release()
        return False

    def mark_calibration_dirty(self) -> None:
        """Mark calibration as dirty so consumers know to reload.

        Used by ROS and Camera processes after reloading calibration
        from disk to signal that the DistanceEstimator needs updating.
        """
        if self._try_lock():
            try:
                self._calibration_dirty = True
            finally:
                self._release()

    # ===================================================================
    # Robot geometry
    # ===================================================================

    def set_robot_dims(self, length: float, width: float, height: float) -> None:
        """Set robot body dimensions in meters."""
        if self._try_lock():
            try:
                self._robot = {'length': length, 'width': width, 'height': height}
            finally:
                self._release()

    def get_robot_dims(self) -> dict:
        """Return dict with 'length', 'width', 'height' in meters."""
        if self._try_lock():
            try:
                return dict(self._robot)
            finally:
                self._release()
        return dict(DEFAULT_ROBOT)

    def get_robot_half_width(self) -> float:
        """Return half the robot width in meters (for obstacle clearance)."""
        if self._try_lock():
            try:
                return self._robot['width'] / 2.0
            finally:
                self._release()
        return DEFAULT_ROBOT['width'] / 2.0

    def set_lidar_pos(self, x: float, y: float, z: float) -> None:
        """Set LiDAR mounting position relative to base_link center (meters)."""
        if self._try_lock():
            try:
                self._lidar = {'x': x, 'y': y, 'z': z}
            finally:
                self._release()

    def get_lidar_pos(self) -> dict:
        """Return LiDAR position dict with 'x', 'y', 'z' in meters."""
        if self._try_lock():
            try:
                return dict(self._lidar)
            finally:
                self._release()
        return dict(DEFAULT_LIDAR)

    def set_lidar_yaw(self, yaw_deg: float) -> None:
        """Set LiDAR mounting yaw in degrees relative to robot forward."""
        if self._try_lock():
            try:
                self._lidar_yaw = yaw_deg
            finally:
                self._release()

    def get_lidar_yaw(self) -> float:
        """Return LiDAR mounting yaw in degrees."""
        if self._try_lock():
            try:
                return self._lidar_yaw
            finally:
                self._release()
        return DEFAULT_LIDAR_YAW

    def set_camera_pos(self, x: float, y: float, z: float) -> None:
        """Set camera mounting position relative to base_link center (meters)."""
        if self._try_lock():
            try:
                self._camera = {'x': x, 'y': y, 'z': z}
            finally:
                self._release()

    def get_camera_pos(self) -> dict:
        """Return camera position dict with 'x', 'y', 'z' in meters."""
        if self._try_lock():
            try:
                return dict(self._camera)
            finally:
                self._release()
        return dict(DEFAULT_CAMERA)

    def set_camera_yaw(self, yaw_deg: float) -> None:
        """Set camera mounting yaw in degrees relative to robot forward."""
        if self._try_lock():
            try:
                self._camera_yaw = yaw_deg
            finally:
                self._release()

    def get_camera_yaw(self) -> float:
        """Return camera mounting yaw in degrees."""
        if self._try_lock():
            try:
                return self._camera_yaw
            finally:
                self._release()
        return DEFAULT_CAMERA_YAW
    def set_camera_flip_h(self, flip: bool) -> None:
        with self._lock:
            self._camera_flip_h = bool(flip)

    def get_camera_flip_h(self) -> bool:
        with self._lock:
            return self._camera_flip_h
    def set_target_dims(self, height: float, width: float) -> None:
        """Set real-world target dimensions and mark calibration dirty.

        Acquires ``_config_lock`` to atomically update geometry and
        invalidate the calibration.

        Args:
            height: Target real height in meters.
            width: Target real width (diameter) in meters.
        """
        with self._config_lock:
            if self._try_lock():
                try:
                    self._target = {'height': height, 'width': width}
                    self._calibration_dirty = True
                finally:
                    self._release()

    def get_target_dims(self) -> dict:
        """Return dict with target 'height' and 'width' in meters."""
        if self._try_lock():
            try:
                return dict(self._target)
            finally:
                self._release()
        return dict(DEFAULT_TARGET)

    def set_ref_point(self, point: str) -> None:
        """Set the distance reference point ('center', 'front', 'camera', 'lidar')."""
        if point in REF_POINTS and self._try_lock():
            try:
                self._ref_point = point
            finally:
                self._release()

    def get_ref_point(self) -> str:
        """Return the active distance reference point string."""
        if self._try_lock():
            try:
                return self._ref_point
            finally:
                self._release()
        return DEFAULT_REF_POINT

    def get_sensor_offset(self, source: str) -> float:
        """Return X offset to convert sensor distance to reference-point distance.

        The sensor measures distance from its own position.  The reference
        point (center, front edge, camera, or LiDAR) is at a different X
        coordinate.  This offset converts between the two:

            distance_from_ref = sensor_distance + get_sensor_offset(source)

        Derivation (target on the X axis for clarity):
            target_x  = sensor_x + sensor_distance
            dist_ref  = target_x - ref_x
                      = sensor_distance + (sensor_x - ref_x)

        So offset = sensor_x - ref_x.  When the sensor is AHEAD of the
        reference point the offset is positive and the reference-point
        distance is larger than the sensor distance (the ref is farther
        from the target).

        Args:
            source: 'camera' or 'lidar'.

        Returns:
            Signed X offset in meters (positive = sensor ahead of ref).
        """
        if self._try_lock():
            try:
                cam_x = self._camera['x']
                lidar_x = self._lidar['x']
                robot_len = self._robot['length']
                ref = self._ref_point
            finally:
                self._release()
        else:
            cam_x = DEFAULT_CAMERA['x']
            lidar_x = DEFAULT_LIDAR['x']
            robot_len = DEFAULT_ROBOT['length']
            ref = DEFAULT_REF_POINT

        if ref == 'center':
            ref_x = 0.0
        elif ref == 'front':
            ref_x = robot_len / 2.0
        elif ref == 'camera':
            ref_x = cam_x
        elif ref == 'lidar':
            ref_x = lidar_x
        else:
            ref_x = 0.0

        sensor_x = cam_x if source == 'camera' else lidar_x
        return sensor_x - ref_x

    def get_lidar_blind_distance(self) -> float:
        """Minimum distance from robot front edge that LiDAR can detect.

        Accounts for RPLidar A1 hardware min range (0.15m) and LiDAR
        mounting offset.  Objects closer than this to the front edge
        are invisible to LiDAR -- Navigator switches to blind approach.
        """
        if self._try_lock():
            try:
                front_edge_x = self._robot['length'] / 2.0
                lidar_x = self._lidar['x']
            finally:
                self._release()
        else:
            front_edge_x = DEFAULT_ROBOT['length'] / 2.0
            lidar_x = DEFAULT_LIDAR['x']
        return max(0.0, LIDAR_MIN_RANGE - (front_edge_x - lidar_x))

    # ===================================================================
    # Camera optics
    # ===================================================================

    def set_camera_fov(self, fov_deg: float) -> None:
        """Set horizontal FOV and mark calibration dirty.

        Acquires ``_config_lock`` to atomically update optics and
        invalidate calibration.

        Args:
            fov_deg: Horizontal field of view in degrees.
        """
        with self._config_lock:
            if self._try_lock():
                try:
                    self._camera_fov = fov_deg
                    self._calibration_dirty = True
                finally:
                    self._release()

    def get_camera_fov(self) -> float:
        """Return horizontal field of view in degrees."""
        if self._try_lock():
            try:
                return self._camera_fov
            finally:
                self._release()
        return DEFAULT_CAMERA_FOV

    def compute_expected_box_height(self, distance_m: float) -> float:
        """Predict bounding-box height in pixels at a given distance.

        Uses current FOV, frame dimensions, and target height via the
        pinhole camera model.  Acquires ``_config_lock`` for a consistent
        cross-section read.

        Args:
            distance_m: Distance to target in meters.

        Returns:
            Expected bounding-box height in pixels, or 0.0 if distance <= 0.
        """
        with self._config_lock:
            if self._try_lock():
                try:
                    fov = self._camera_fov
                    fh = self._frame_h
                    fw = self._frame_w
                    real_h = self._target.get('height', DEFAULT_TARGET['height'])
                finally:
                    self._release()
            else:
                fov = DEFAULT_CAMERA_FOV
                fh = DEFAULT_FRAME_H
                fw = DEFAULT_FRAME_W
                real_h = DEFAULT_TARGET['height']

        if distance_m <= 0:
            return 0.0
        vfov_rad = math.radians(fov) * (fh / fw)
        focal_px = (fh / 2.0) / math.tan(vfov_rad / 2.0)
        return (real_h * focal_px) / distance_m

    # ===================================================================
    # Map display
    # ===================================================================

    def set_map_param(self, key: str, value: object) -> None:
        """Set a single map display parameter by key.  Ignores unknown keys."""
        if key in DEFAULT_MAP_PARAMS and self._try_lock():
            try:
                self._map_params[key] = value
            finally:
                self._release()

    def get_map_param(self, key: str) -> object:
        """Return a single map display parameter value, or the default."""
        if self._try_lock():
            try:
                return self._map_params.get(key, DEFAULT_MAP_PARAMS.get(key))
            finally:
                self._release()
        return DEFAULT_MAP_PARAMS.get(key)

    def get_map_params(self) -> dict:
        """Return a copy of all map display parameters as a dict."""
        if self._try_lock():
            try:
                return dict(self._map_params)
            finally:
                self._release()
        return dict(DEFAULT_MAP_PARAMS)

    # ===================================================================
    # Navigation parameters
    # ===================================================================

    def set_nav_param(self, key: str, value: object) -> None:
        """Set a single navigation tuning parameter by key.  Ignores unknown keys."""
        if key in DEFAULT_NAV_PARAMS and self._try_lock():
            try:
                self._nav_params[key] = value
            finally:
                self._release()

    def get_nav_param(self, key: str) -> object:
        """Return a single navigation parameter value, or the default."""
        if self._try_lock():
            try:
                return self._nav_params.get(key, DEFAULT_NAV_PARAMS.get(key))
            finally:
                self._release()
        return DEFAULT_NAV_PARAMS.get(key)

    def get_nav_params(self) -> dict:
        """Return a copy of all navigation parameters as a dict."""
        if self._try_lock():
            try:
                return dict(self._nav_params)
            finally:
                self._release()
        return dict(DEFAULT_NAV_PARAMS)

    # ===================================================================
    # Detector status (not persisted)
    # ===================================================================

    def set_stream_mode(self, enabled: bool) -> None:
        """Set whether the DRP-AI detector is in stream mode."""
        if self._try_lock():
            try:
                self._stream_mode = enabled
            finally:
                self._release()

    def get_stream_mode(self) -> bool:
        """Return True if the DRP-AI detector is in stream mode."""
        if self._try_lock():
            try:
                return self._stream_mode
            finally:
                self._release()
        return False

    def set_detector_mode(self, mode: str) -> None:
        """Set detector mode string (e.g. 'Stream', 'Pipe', 'Initializing')."""
        if self._try_lock():
            try:
                self._detector_mode = mode
            finally:
                self._release()

    def get_detector_mode(self) -> str:
        """Return current detector mode string."""
        if self._try_lock():
            try:
                return self._detector_mode
            finally:
                self._release()
        return "Unknown"

    # ===================================================================
    # DRP-AI C++ binary parameters
    # ===================================================================

    def get_drpai_params(self) -> dict:
        """Return a copy of all DRP-AI C++ binary parameters."""
        if self._try_lock():
            try:
                return dict(self._drpai_params)
            finally:
                self._release()
        return dict(DEFAULT_DRPAI_PARAMS)

    def get_drpai_param(self, key: str) -> object:
        """Return a single DRP-AI parameter value, or the default."""
        if self._try_lock():
            try:
                return self._drpai_params.get(key, DEFAULT_DRPAI_PARAMS.get(key))
            finally:
                self._release()
        return DEFAULT_DRPAI_PARAMS.get(key)

    def set_drpai_param(self, key: str, value: object) -> None:
        """Set a single DRP-AI parameter by key.  Ignores unknown keys."""
        if key in DEFAULT_DRPAI_PARAMS and self._try_lock():
            try:
                self._drpai_params[key] = type(DEFAULT_DRPAI_PARAMS[key])(value)
            finally:
                self._release()

    def request_drpai_restart(self) -> None:
        """Signal the camera worker to restart the DRP-AI subprocess.

        Call this after writing updated DRP-AI params to config.ini so
        the C++ binary picks up the new values on restart.
        """
        if self._try_lock():
            try:
                self._drpai_restart_requested = True
            finally:
                self._release()

    def check_and_clear_drpai_restart(self) -> bool:
        """Check and atomically clear the DRP-AI restart flag.

        Returns:
            True if a restart was requested since the last check.
        """
        if self._try_lock():
            try:
                val = self._drpai_restart_requested
                self._drpai_restart_requested = False
                return val
            finally:
                self._release()
        return False

    def write_drpai_config_ini(self) -> bool:
        """Write current DRP-AI params to the C++ binary's config.ini.

        Reads the existing config.ini, updates only the keys we manage
        (conf_threshold, nms_threshold, max_detections, drp_max_freq,
        drpai_freq), and writes it back.  Comments and unknown keys are
        preserved.

        Returns:
            True on success, False if the file could not be written.
        """
        params = self.get_drpai_params()
        ini_path = DRPAI_CONFIG_INI_PATH
        try:
            # Read existing file (preserve comments and unknown keys)
            lines = []
            if os.path.isfile(ini_path):
                with open(ini_path, 'r') as f:
                    lines = f.readlines()

            # Track which keys we've written so we can append missing ones
            written_keys = set()
            new_lines = []
            for line in lines:
                stripped = line.strip()
                # Skip blank or comment lines -- keep as-is
                if not stripped or stripped.startswith('#'):
                    new_lines.append(line)
                    continue
                # Parse "key = value" or "key=value"
                if '=' in stripped:
                    key_part = stripped.split('=', 1)[0].strip()
                    if key_part in params:
                        new_lines.append(f"{key_part} = {params[key_part]}\n")
                        written_keys.add(key_part)
                        continue
                new_lines.append(line)

            # Append any params not already in the file
            for key, value in params.items():
                if key not in written_keys:
                    new_lines.append(f"{key} = {value}\n")

            # Atomic write via tmp + replace
            tmp_path = ini_path + '.tmp'
            with open(tmp_path, 'w') as f:
                f.writelines(new_lines)
            os.replace(tmp_path, ini_path)
            logger.info("Wrote DRP-AI config: %s", ini_path)
            return True
        except Exception as e:
            logger.error("Failed to write DRP-AI config.ini: %s", e)
            return False

    # ===================================================================
    # Persistence
    # ===================================================================

    def save_calibration(self) -> bool:
        """Persist all settings to calibration.json (atomic write).

        Acquires ``_config_lock`` while reading state to get a consistent
        snapshot, then releases the lock before performing disk I/O.
        Uses write-to-tmp + ``os.replace`` for crash safety.

        Returns:
            True on success, False on failure.
        """
        try:
            if self._config_lock:
                if not self._config_lock.acquire(timeout=1.0):
                    logger.error("Save failed: config_lock acquire timed out")
                    return False
            try:
                if not self._try_lock():
                    logger.error("Save failed: could not acquire lock")
                    return False
                try:
                    data = {
                        'ref_box_height': self._ref_box_height,
                        'ref_distance': self._ref_distance,
                        'confidence_threshold': self._confidence_threshold,
                        'detect_interval': self._detect_interval,
                        'detect_expiry': self._detect_expiry,
                        'filter_max_aspect': self._filter_max_aspect,
                        'filter_max_box_pct': self._filter_max_box_pct,
                        'filter_size_tolerance': self._filter_size_tolerance,
                        'filter_tracker_hits': self._filter_tracker_hits,
                        'filter_tracker_dist': self._filter_tracker_dist,
                        'filter_bbox_window': self._filter_bbox_window,
                        'filter_bbox_factor': self._filter_bbox_factor,
                        'robot': dict(self._robot),
                        'lidar': dict(self._lidar),
                        'lidar_yaw': self._lidar_yaw,
                        'camera_pos': dict(self._camera),
                        'camera_yaw': self._camera_yaw,
                        'camera_flip_h': self._camera_flip_h,
                        'camera_fov': self._camera_fov,
                        'target': dict(self._target),
                        'ref_point': self._ref_point,
                        'nav_params': dict(self._nav_params),
                        'map_params': dict(self._map_params),
                        'drpai_params': dict(self._drpai_params),
                    }
                finally:
                    self._release()
            finally:
                if self._config_lock:
                    self._config_lock.release()

            cal_dir = os.path.dirname(CALIBRATION_FILE)
            os.makedirs(cal_dir, exist_ok=True)
            tmp_file = CALIBRATION_FILE + '.tmp'
            with open(tmp_file, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_file, CALIBRATION_FILE)
            logger.info("Saved to %s", CALIBRATION_FILE)

            # Also write to shared memory for fast cross-process sync.
            # Nav/Camera processes read from SHM (~1ms) instead of disk (~100ms).
            if not hasattr(self, '_shm_writer'):
                self._shm_writer = SettingsShmWriter()
            self._shm_writer.write(json.dumps(data))

            return True
        except Exception as e:
            logger.error("Save failed: %s", e)
            return False

    def _load_calibration(self) -> None:
        """Load all settings from calibration.json on disk.

        Reads the JSON file, validates and clamps all values, and updates
        internal state.  If the file does not exist or contains invalid
        JSON, logs an error and returns without modifying any state.
        Unknown keys are silently ignored.
        """
        try:
            if not os.path.exists(CALIBRATION_FILE):
                return
            with open(CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
            self._apply_calibration_dict(data)
            logger.info("Loaded from disk: ref=%spx@%sm",
                        self._ref_box_height, self._ref_distance)
        except Exception as e:
            logger.error("Load from disk failed: %s", e)

    def load_from_shm(self, reader: 'SettingsShmReader') -> bool:
        """Try to load settings from shared memory (faster than disk).

        Used by Nav and Camera processes on /settings_changed notification
        to pick up GUI changes with ~1ms latency instead of disk I/O.

        Args:
            reader: SettingsShmReader instance to read from.

        Returns:
            True if SHM had new data and it was applied, False otherwise.
        """
        data = reader.read()
        if data is None:
            return False
        try:
            self._apply_calibration_dict(data)
            logger.info("Loaded from SHM: ref=%spx@%sm",
                        self._ref_box_height, self._ref_distance)
            return True
        except Exception as e:
            logger.error("Load from SHM failed: %s", e)
            return False

    def _apply_calibration_dict(self, data: dict) -> None:
        """Apply a parsed settings dict to internal state.

        Validates and clamps all values.  Called by both _load_calibration()
        (from disk) and load_from_shm() (from shared memory).

        Args:
            data: Parsed settings dict (same format as calibration.json).
        """
        vp = validate_param  # shorthand
        if not self._try_lock():
            logger.error("Apply calibration failed: could not acquire lock")
            return
        try:
            # -- Calibration (validated against PARAM_RANGES) --
            self._ref_box_height = vp('ref_box_height',
                data.get('ref_box_height'), DEFAULT_REF_BOX_HEIGHT)
            self._ref_distance = vp('ref_distance',
                data.get('ref_distance'), DEFAULT_REF_DISTANCE)
            self._calibration_dirty = False

            # -- Detection filters --
            if 'confidence_threshold' in data:
                self._confidence_threshold = vp('confidence_threshold',
                    data['confidence_threshold'], self._confidence_threshold)
            if 'detect_interval' in data:
                self._detect_interval = vp('detect_interval',
                    data['detect_interval'], self._detect_interval)
            if 'detect_expiry' in data:
                self._detect_expiry = vp('detect_expiry',
                    data['detect_expiry'], self._detect_expiry)
            if 'filter_max_aspect' in data:
                self._filter_max_aspect = vp('filter_max_aspect',
                    data['filter_max_aspect'], self._filter_max_aspect)
            if 'filter_max_box_pct' in data:
                self._filter_max_box_pct = vp('filter_max_box_pct',
                    data['filter_max_box_pct'], self._filter_max_box_pct)
            if 'filter_bbox_window' in data:
                self._filter_bbox_window = vp('filter_bbox_window',
                    data['filter_bbox_window'], self._filter_bbox_window)
            if 'filter_bbox_factor' in data:
                self._filter_bbox_factor = vp('filter_bbox_factor',
                    data['filter_bbox_factor'], self._filter_bbox_factor)

            # -- Robot geometry --
            if 'robot' in data:
                r = data['robot']
                self._robot = {
                    'length': vp('robot_length', r.get('length'), DEFAULT_ROBOT['length']),
                    'width':  vp('robot_width',  r.get('width'),  DEFAULT_ROBOT['width']),
                    'height': vp('robot_height', r.get('height'), DEFAULT_ROBOT['height']),
                }
            if 'lidar' in data:
                lp = data['lidar']
                self._lidar = {
                    'x': vp('lidar_x', lp.get('x'), DEFAULT_LIDAR['x']),
                    'y': vp('lidar_y', lp.get('y'), DEFAULT_LIDAR['y']),
                    'z': vp('lidar_z', lp.get('z'), DEFAULT_LIDAR['z']),
                }
            if 'lidar_yaw' in data:
                self._lidar_yaw = vp('lidar_yaw',
                    data['lidar_yaw'], DEFAULT_LIDAR_YAW)
            if 'camera_pos' in data:
                cp = data['camera_pos']
                self._camera = {
                    'x': vp('camera_x', cp.get('x'), DEFAULT_CAMERA['x']),
                    'y': vp('camera_y', cp.get('y'), DEFAULT_CAMERA['y']),
                    'z': vp('camera_z', cp.get('z'), DEFAULT_CAMERA['z']),
                }
            if 'camera_yaw' in data:
                self._camera_yaw = vp('camera_yaw',
                    data['camera_yaw'], DEFAULT_CAMERA_YAW)
            if 'camera_flip_h' in data:                                        
                self._camera_flip_h = bool(data['camera_flip_h'])                   
            if 'target' in data:
                t = data['target']
                self._target = {
                    'height': max(0.01, float(t.get('height', DEFAULT_TARGET['height']))),
                    'width':  max(0.01, float(t.get('width', DEFAULT_TARGET['width']))),
                }
            if data.get('ref_point') in REF_POINTS:
                self._ref_point = data['ref_point']

            # -- Camera optics --
            if 'camera_fov' in data:
                self._camera_fov = vp('camera_fov',
                    data['camera_fov'], DEFAULT_CAMERA_FOV)

            # -- Navigation params (each validated against PARAM_RANGES) --
            if 'nav_params' in data:
                for k, v in data['nav_params'].items():
                    if k in DEFAULT_NAV_PARAMS:
                        self._nav_params[k] = vp(k, v, DEFAULT_NAV_PARAMS[k])

            # -- Map display params --
            if 'map_params' in data:
                for k, v in data['map_params'].items():
                    if k in DEFAULT_MAP_PARAMS:
                        self._map_params[k] = type(DEFAULT_MAP_PARAMS[k])(v)

            # -- DRP-AI C++ binary params --
            if 'drpai_params' in data:
                dp = data['drpai_params']
                for k, v in dp.items():
                    if k in DEFAULT_DRPAI_PARAMS:
                        range_key = f'drpai_{k}'
                        self._drpai_params[k] = vp(range_key, v,
                            DEFAULT_DRPAI_PARAMS[k])

        finally:
            self._release()

    @staticmethod
    def _clamp_positive(val: float, minimum: float = 0.001) -> float:
        """Clamp a value to be at least *minimum* (prevents division by zero)."""
        try:
            v = float(val)
            return v if v >= minimum else minimum
        except (TypeError, ValueError):
            return minimum


# Backward compatibility alias
DetectionStore = SettingsStore

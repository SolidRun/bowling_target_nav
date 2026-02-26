"""Thread-safe storage for map, robot pose, and LiDAR data."""

import threading
import time

import numpy as np


class SensorStore:
    """Thread-safe container for SLAM map, robot pose, and LiDAR scan data."""

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
        self._map_img = None
        self._map_info = None
        self._map_count = 0
        self._map_time = 0.0
        self._map_hz = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._last_pose_time = 0.0
        self._laser_points = np.empty((0, 2), dtype=np.float32)
        self._laser_time = 0.0
        self._scan_count = 0
        self._scan_hz = 0.0
        self._raw_scan = None
        self._raw_scan_time = 0.0

    def _try_lock(self, timeout=None):
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # -- Map --
    def set_map(self, img, info):
        if not self._try_lock():
            return False
        try:
            now = time.time()
            if self._map_time > 0:
                dt = now - self._map_time
                if 0 < dt < 30.0:
                    self._map_hz = 0.3 * (1.0 / dt) + 0.7 * self._map_hz
            self._map_img = img
            self._map_info = info
            self._map_count += 1
            self._map_time = now
            return True
        finally:
            self._release()

    def get_map(self):
        if not self._try_lock():
            return None, None, 0
        try:
            if self._map_img is not None:
                return self._map_img.copy(), self._map_info, self._map_count
            return None, None, 0
        finally:
            self._release()

    # -- Robot pose --
    def set_robot_pose(self, x, y, theta):
        if not self._try_lock():
            return False
        try:
            self._robot_x = x
            self._robot_y = y
            self._robot_theta = theta
            self._last_pose_time = time.time()
            return True
        finally:
            self._release()

    def get_robot_pose(self):
        if not self._try_lock():
            return 0.0, 0.0, 0.0
        try:
            return self._robot_x, self._robot_y, self._robot_theta
        finally:
            self._release()

    # -- Laser --
    def set_laser(self, points):
        if not self._try_lock():
            return False
        try:
            now = time.time()
            if self._laser_time > 0:
                dt = now - self._laser_time
                if 0 < dt < 2.0:
                    self._scan_hz = 0.3 * (1.0 / dt) + 0.7 * self._scan_hz
            self._laser_points = points
            self._laser_time = now
            self._scan_count += 1
            return True
        finally:
            self._release()

    def get_laser(self):
        if not self._try_lock():
            return np.empty((0, 2), dtype=np.float32), 0, 0.0
        try:
            return self._laser_points.copy(), self._scan_count, self._laser_time
        finally:
            self._release()

    def set_raw_scan(self, scan_msg):
        if not self._try_lock():
            return False
        try:
            self._raw_scan = scan_msg
            self._raw_scan_time = time.time()
            return True
        finally:
            self._release()

    def get_diagnostics(self):
        """Return diagnostic snapshot: rates, ages, counts."""
        if not self._try_lock():
            return {}
        try:
            now = time.time()
            return {
                'scan_hz': self._scan_hz,
                'scan_count': self._scan_count,
                'scan_age': now - self._laser_time if self._laser_time > 0 else -1,
                'map_hz': self._map_hz,
                'map_count': self._map_count,
                'map_age': now - self._map_time if self._map_time > 0 else -1,
                'tf_age': now - self._last_pose_time if self._last_pose_time > 0 else -1,
                'n_points': len(self._laser_points),
            }
        finally:
            self._release()

    def get_lidar_distance_at_angle(self, angle_rad, window=0.15):
        """Look up LiDAR range at a specific angle for distance fusion."""
        if not self._try_lock():
            return float('inf')
        try:
            scan = self._raw_scan
            if scan is None or time.time() - self._raw_scan_time > 0.5:
                return float('inf')

            angle_min_q = angle_rad - window
            angle_max_q = angle_rad + window
            start_idx = int((angle_min_q - scan.angle_min) / scan.angle_increment)
            end_idx = int((angle_max_q - scan.angle_min) / scan.angle_increment)
            num_ranges = len(scan.ranges)
            start_idx = max(0, min(num_ranges - 1, start_idx))
            end_idx = max(0, min(num_ranges - 1, end_idx))

            if start_idx > end_idx:
                return float('inf')

            valid = [
                r for r in scan.ranges[start_idx:end_idx + 1]
                if scan.range_min < r < scan.range_max
            ]
            return min(valid) if valid else float('inf')
        finally:
            self._release()

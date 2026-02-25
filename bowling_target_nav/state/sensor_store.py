"""Thread-safe storage for map, robot pose, and LiDAR data."""

import threading
import time


class SensorStore:
    """Thread-safe container for SLAM map, robot pose, and LiDAR scan data."""

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
        self._map_img = None
        self._map_info = None
        self._map_count = 0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._laser_points = []
        self._scan_count = 0
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
            self._map_img = img
            self._map_info = info
            self._map_count += 1
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
            self._laser_points = points
            self._scan_count += 1
            return True
        finally:
            self._release()

    def get_laser(self):
        if not self._try_lock():
            return [], 0
        try:
            return self._laser_points[:], self._scan_count
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

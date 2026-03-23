"""Thread-safe storage for robot pose and LiDAR scan data.

SensorStore holds the robot pose (x, y, theta) from TF, laser scan
points in Cartesian form, raw LaserScan messages for angle-based
distance lookup, and smoothed rate/age diagnostics.  The ROS thread
writes all data; the GTK main loop reads it for laser rendering.

Architecture:
    SharedState owns one SensorStore instance. The GUI bridge node
    subscribes to ROS2 topics and writes data into this store; the GTK
    main loop reads it for laser rendering.

Key class:
    SensorStore -- RLock-protected container with 0.1 s timeout.

Thread safety:
    All public methods acquire ``_lock`` (RLock) with a 0.1 s timeout.
    On timeout, getters return safe defaults (empty arrays, zeros).
"""

import threading
import time

import numpy as np

from target_nav.config import STORE_LOCK_TIMEOUT


class SensorStore:
    """Thread-safe container for robot pose and LiDAR scan data.

    Owns the robot pose from TF, Cartesian laser points, raw LaserScan
    message, and smoothed rate / count diagnostics.  Written by the ROS
    thread; read by the GTK main loop for map_panel and status bar
    rendering.

    Thread safety:
        All access is protected by an RLock with STORE_LOCK_TIMEOUT.
        On timeout, getters return safe defaults.
    """

    LOCK_TIMEOUT = STORE_LOCK_TIMEOUT

    def __init__(self):
        """Initialize sensor fields to empty/zero defaults."""
        self._lock = threading.RLock()
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

    # -- Robot pose --
    def set_robot_pose(self, x, y, theta):
        """Update the robot pose from TF.

        Args:
            x: X position in odom frame (meters).
            y: Y position in odom frame (meters).
            theta: Heading angle (radians).

        Returns:
            True on success, False if the lock timed out.
        """
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
        """Return (x, y, theta) or (0.0, 0.0, 0.0) on lock timeout."""
        if not self._try_lock():
            return 0.0, 0.0, 0.0
        try:
            return self._robot_x, self._robot_y, self._robot_theta
        finally:
            self._release()

    # -- Laser --
    def set_laser(self, points):
        """Store new laser scan points and update scan rate.

        Args:
            points: numpy float32 array of shape (N, 2) with (x, y) pairs
                in base_link frame.

        Returns:
            True on success, False if the lock timed out.
        """
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
        """Return (points_copy, scan_count, laser_time)."""
        if not self._try_lock():
            return np.empty((0, 2), dtype=np.float32), 0, 0.0
        try:
            return self._laser_points.copy(), self._scan_count, self._laser_time
        finally:
            self._release()

    def get_laser_pose(self):
        """Return pose at time of last scan (same as get_robot_pose in threading mode)."""
        return self.get_robot_pose()

    def set_raw_scan(self, scan_msg):
        """Store raw LaserScan message for angle-based distance lookup.

        Args:
            scan_msg: ROS sensor_msgs/LaserScan message.

        Returns:
            True on success, False if the lock timed out.
        """
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
                'tf_age': now - self._last_pose_time if self._last_pose_time > 0 else -1,
                'n_points': len(self._laser_points),
            }
        finally:
            self._release()

    def get_lidar_distance_at_angle(self, angle_rad, window=0.15):
        """Look up LiDAR range at a specific angle for distance fusion.

        Handles angle wrapping for LiDAR mounted at non-zero yaw (e.g., 180°).
        The query angle is in the laser frame.  A 360° scan that starts at
        -179° and ends at +180° has a gap at ±180°; this function correctly
        searches both ends of the scan array when the query window crosses
        that boundary.

        Args:
            angle_rad: Target angle in radians (in LiDAR frame).
            window: Angular half-window (radians) to search around angle_rad.

        Returns:
            Minimum valid range within the window, or float('inf') if none.
        """
        if not self._try_lock():
            return float('inf')
        try:
            scan = self._raw_scan
            if scan is None or time.time() - self._raw_scan_time > 0.5:
                return float('inf')

            import math
            num_ranges = len(scan.ranges)
            if num_ranges == 0:
                return float('inf')

            # Normalize angle to [-pi, pi]
            a = math.atan2(math.sin(angle_rad), math.cos(angle_rad))

            # Compute raw (unclamped) indices for the query window
            inc = scan.angle_increment
            start_raw = int((a - window - scan.angle_min) / inc)
            end_raw = int((a + window - scan.angle_min) / inc)

            # Collect valid ranges, handling wrap-around at both ends
            valid = []
            rmin = scan.range_min
            rmax = scan.range_max
            ranges = scan.ranges
            for i in range(start_raw, end_raw + 1):
                # Wrap index into [0, num_ranges) for 360° scans
                idx = i % num_ranges
                r = ranges[idx]
                if rmin < r < rmax:
                    valid.append(r)

            return min(valid) if valid else float('inf')
        finally:
            self._release()

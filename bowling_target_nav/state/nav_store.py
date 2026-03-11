"""Thread-safe storage for navigation state, commands, and obstacles.

Used in threading mode.  The navigator writes state/target/obstacle data;
the GUI reads via get_gui_snapshot() (single lock acquisition per frame).
GO/STOP commands flow from GUI to navigator through request_go/request_stop
which set boolean flags polled by the nav loop.
"""

import threading
import time


class NavStore:
    """Thread-safe container for navigation state and control commands."""

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
        self._nav_state = "IDLE"
        self._nav_target = None  # (x, y, distance) of current target
        self._go_requested = False
        self._stop_requested = False
        self._last_target_time = 0.0
        self._search_start_time = 0.0
        self._obstacle_ahead = False
        self._obstacle_dist = float('inf')
        self._nav_target_map = None  # (x, y) in map frame for GUI overlay
        self._current_cmd_vel = (0.0, 0.0, 0.0)  # (vx, vy, wz)
        self._lidar_at_target = float('inf')  # LiDAR distance at target angle

    def _try_lock(self, timeout=None):
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # -- Nav state --
    def set_nav_state(self, nav_state, target=None):
        """Update the navigation state and optionally the current target.

        Args:
            nav_state: State string (e.g. 'IDLE', 'NAVIGATING', 'SEARCHING').
            target: (x, y, distance) tuple in robot frame, or None.

        Returns:
            True on success, False if the lock timed out.
        """
        if not self._try_lock():
            return False
        try:
            self._nav_state = nav_state
            if target is not None:
                self._nav_target = target
            return True
        finally:
            self._release()

    def get_nav_state(self):
        """Return (state_str, nav_target) or ('UNKNOWN', None) on timeout."""
        if not self._try_lock():
            return "UNKNOWN", None
        try:
            return self._nav_state, self._nav_target
        finally:
            self._release()

    # -- GO / STOP commands --
    def request_go(self):
        """Set the GO flag, consumed by the navigator via check_and_clear_go()."""
        if self._try_lock():
            try: self._go_requested = True
            finally: self._release()

    def request_stop(self):
        """Set the STOP flag, consumed by the navigator via check_and_clear_stop()."""
        if self._try_lock():
            try: self._stop_requested = True
            finally: self._release()

    def check_and_clear_go(self):
        """Return True and clear the GO flag if it was set, else False."""
        if not self._try_lock():
            return False
        try:
            val = self._go_requested
            self._go_requested = False
            return val
        finally:
            self._release()

    def check_and_clear_stop(self):
        """Return True and clear the STOP flag if it was set, else False."""
        if not self._try_lock():
            return False
        try:
            val = self._stop_requested
            self._stop_requested = False
            return val
        finally:
            self._release()

    # -- Target timing --
    def update_target_seen(self):
        """Record that the target was detected in the current frame."""
        if self._try_lock():
            try: self._last_target_time = time.time()
            finally: self._release()

    def get_time_since_target(self):
        """Return seconds since last target detection, or 999.0 if never seen."""
        if not self._try_lock():
            return 999.0
        try:
            if self._last_target_time == 0:
                return 999.0
            return time.time() - self._last_target_time
        finally:
            self._release()

    # -- Search --
    def start_search(self):
        """Begin the SEARCHING state and record the search start time."""
        if self._try_lock():
            try:
                self._search_start_time = time.time()
                self._nav_state = "SEARCHING"
            finally: self._release()

    def get_search_time(self):
        """Return seconds spent in the current search phase, or 0.0."""
        if not self._try_lock():
            return 0.0
        try:
            if self._search_start_time == 0:
                return 0.0
            return time.time() - self._search_start_time
        finally:
            self._release()

    # -- Obstacles --
    def set_obstacle(self, ahead, dist):
        """Update obstacle status from the navigator's scan check.

        Args:
            ahead: True if an obstacle blocks the forward path.
            dist: Distance to the nearest obstacle (meters).
        """
        if self._try_lock():
            try:
                self._obstacle_ahead = ahead
                self._obstacle_dist = dist
            finally: self._release()

    def get_obstacle(self):
        """Return (obstacle_ahead, obstacle_dist)."""
        if not self._try_lock():
            return False, float('inf')
        try:
            return self._obstacle_ahead, self._obstacle_dist
        finally:
            self._release()

    # -- Map-frame target (for GUI overlay) --
    def set_nav_target_map(self, x, y):
        """Store the target position in map frame for GUI overlay drawing.

        Args:
            x: Target X in map frame (meters).
            y: Target Y in map frame (meters).
        """
        if self._try_lock():
            try: self._nav_target_map = (x, y)
            finally: self._release()

    def clear_nav_target_map(self):
        """Remove the map-frame target (e.g. when navigation stops)."""
        if self._try_lock():
            try: self._nav_target_map = None
            finally: self._release()

    def get_nav_target_map(self):
        """Return (x, y) in map frame, or None if no active target."""
        if self._try_lock():
            try: return self._nav_target_map
            finally: self._release()
        return None

    # -- LiDAR at target angle (for GUI comparison) --
    def set_lidar_at_target(self, dist):
        """Store LiDAR distance at current target angle for GUI display."""
        if self._try_lock():
            try: self._lidar_at_target = dist
            finally: self._release()

    def get_lidar_at_target(self):
        """Return LiDAR distance at target angle, or inf."""
        if self._try_lock():
            try: return self._lidar_at_target
            finally: self._release()
        return float('inf')

    # -- Current cmd_vel (for GUI display) --
    def set_current_cmd_vel(self, vx, vy, wz):
        """Store the last published cmd_vel for GUI display.

        Args:
            vx: Linear X velocity (m/s).
            vy: Linear Y velocity (m/s, mecanum only).
            wz: Angular Z velocity (rad/s).
        """
        if self._try_lock():
            try: self._current_cmd_vel = (vx, vy, wz)
            finally: self._release()

    def get_current_cmd_vel(self):
        """Return (vx, vy, wz) or (0.0, 0.0, 0.0) on timeout."""
        if self._try_lock():
            try: return self._current_cmd_vel
            finally: self._release()
        return (0.0, 0.0, 0.0)

    # -- Bulk snapshot for GUI (single lock instead of 6 per frame) --
    def get_gui_snapshot(self):
        """Return all GUI-needed data in one lock acquisition.

        Returns dict with: nav_state, nav_target, time_since_target,
        search_time, obstacle_ahead, obstacle_dist, cmd_vel, nav_target_map.
        """
        now = time.time()
        if not self._try_lock():
            return {
                'nav_state': 'UNKNOWN', 'nav_target': None,
                'time_since_target': 999.0, 'search_time': 0.0,
                'obstacle_ahead': False, 'obstacle_dist': float('inf'),
                'cmd_vel': (0.0, 0.0, 0.0), 'nav_target_map': None,
                'lidar_at_target': float('inf'),
            }
        try:
            tst = (now - self._last_target_time) if self._last_target_time > 0 else 999.0
            st = (now - self._search_start_time) if self._search_start_time > 0 else 0.0
            return {
                'nav_state': self._nav_state,
                'nav_target': self._nav_target,
                'time_since_target': tst,
                'search_time': st,
                'obstacle_ahead': self._obstacle_ahead,
                'obstacle_dist': self._obstacle_dist,
                'cmd_vel': self._current_cmd_vel,
                'nav_target_map': self._nav_target_map,
                'lidar_at_target': self._lidar_at_target,
            }
        finally:
            self._release()

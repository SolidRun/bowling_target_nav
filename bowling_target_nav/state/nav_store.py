"""Thread-safe storage for navigation state, commands, and obstacles."""

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

    def _try_lock(self, timeout=None):
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # -- Nav state --
    def set_nav_state(self, nav_state, target=None):
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
        if not self._try_lock():
            return "UNKNOWN", None
        try:
            return self._nav_state, self._nav_target
        finally:
            self._release()

    # -- GO / STOP commands --
    def request_go(self):
        if self._try_lock():
            try: self._go_requested = True
            finally: self._release()

    def request_stop(self):
        if self._try_lock():
            try: self._stop_requested = True
            finally: self._release()

    def check_and_clear_go(self):
        if not self._try_lock():
            return False
        try:
            val = self._go_requested
            self._go_requested = False
            return val
        finally:
            self._release()

    def check_and_clear_stop(self):
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
        if self._try_lock():
            try: self._last_target_time = time.time()
            finally: self._release()

    def get_time_since_target(self):
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
        if self._try_lock():
            try:
                self._search_start_time = time.time()
                self._nav_state = "SEARCHING"
            finally: self._release()

    def get_search_time(self):
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
        if self._try_lock():
            try:
                self._obstacle_ahead = ahead
                self._obstacle_dist = dist
            finally: self._release()

    def get_obstacle(self):
        if not self._try_lock():
            return False, float('inf')
        try:
            return self._obstacle_ahead, self._obstacle_dist
        finally:
            self._release()

    # -- Map-frame target (for GUI overlay) --
    def set_nav_target_map(self, x, y):
        if self._try_lock():
            try: self._nav_target_map = (x, y)
            finally: self._release()

    def clear_nav_target_map(self):
        if self._try_lock():
            try: self._nav_target_map = None
            finally: self._release()

    def get_nav_target_map(self):
        if self._try_lock():
            try: return self._nav_target_map
            finally: self._release()
        return None

    # -- Current cmd_vel (for GUI display) --
    def set_current_cmd_vel(self, vx, vy, wz):
        if self._try_lock():
            try: self._current_cmd_vel = (vx, vy, wz)
            finally: self._release()

    def get_current_cmd_vel(self):
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
            }
        finally:
            self._release()

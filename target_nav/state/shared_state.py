"""Top-level shared state facade for the GUI process.

SharedState composes three domain stores -- SensorStore (map, pose, laser),
SettingsStore (camera frames, calibration, filters, nav/map params), and
NavStore (navigation state machine, GO/STOP commands, obstacles) -- plus
lifecycle primitives (shutdown event, error log) and a ROS-node reference.

Ownership model / delegation:
    SharedState itself holds NO robot data. It delegates to three sub-stores
    accessed as public attributes:

        self.sensors   (SensorStore)   -- Written by: SHM poll thread.
            Robot pose (x, y, theta), LiDAR laser scan points, diagnostics.

        self.detection (SettingsStore) -- Written by: camera worker, GUI settings.
            Camera frames, detection list, calibration params, nav params,
            map display params, sensor mounting config, target dimensions.

        self.nav       (NavStore)      -- Written by: Navigator (ROS process).
            Navigation state (IDLE/NAVIGATING/SEARCHING/...), cmd_vel,
            GO/STOP request flags, target position, obstacle info.

    GUI code reads all three stores. The Navigator reads sensors + detection
    and writes nav. The camera worker writes detection (frames/detections).

Architecture (3-process, lock-free struct SHM):
    The GUI process (Core 0) owns one SharedState singleton created in
    ``state/__init__.py``.  Each domain store uses its own RLock with a
    0.1 s timeout so the GUI thread never blocks indefinitely.  Nav and
    Camera processes run on separate cores and communicate via lock-free
    SPSC struct SHM channels (no Bridge process, no JSON for real-time
    data).  A lightweight SHM poll daemon thread reads struct SHM from
    Nav (NavShmReader, LaserShmReader) and Camera (DetShmReader) and
    populates SharedState for the GTK panels to read.  GUI commands are
    written to a CmdRingBuffer in SHM for the Nav process to consume.

Runs in: GUI process (Process 0, Core 0). Created once at startup.

Related modules:
    state/sensor_store.py   -- SensorStore implementation.
    state/settings_store.py -- SettingsStore implementation.
    state/nav_store.py      -- NavStore implementation.
    state/command_dispatcher.py -- routes send_ros_command() calls.
    ipc/shm_struct.py       -- struct-based SHM readers/writers.

Thread safety:
    ``_shutdown_event`` is a threading.Event (inherently thread-safe).
    ``_errors`` is guarded by ``_error_lock`` (RLock, 0.1 s timeout).
    Domain stores manage their own locks internally.
"""

import threading

from .sensor_store import SensorStore
from .settings_store import SettingsStore
from .nav_store import NavStore


class SharedState:
    """Thin facade over domain stores plus lifecycle management.

    All three threads (ROS, Camera, GTK) import the singleton from
    ``state/__init__.py``.  Provides ``running`` / ``request_shutdown()``
    for lifecycle, ``add_error()`` / ``get_errors()`` for cross-thread
    error reporting, and ``send_ros_command()`` for dispatching commands
    to the ROS node (threading mode only).
    """

    def __init__(self):
        """Create domain stores and lifecycle primitives."""
        self._shutdown_event = threading.Event()
        self.sensors = SensorStore()
        self.detection = SettingsStore()
        self.nav = NavStore()

        # ROS node reference (set by ros_thread, used by SettingsWindow)
        self._ros_node = None

        # Settings change publisher (set by main_multiprocess via CmdRingBuffer)
        self._settings_pub = None

        # Error tracking
        self._error_lock = threading.RLock()
        self._errors = []

    @property
    def running(self):
        """True while the application has not been asked to shut down."""
        return not self._shutdown_event.is_set()

    def request_shutdown(self):
        """Signal all threads to stop."""
        self._shutdown_event.set()

    def add_error(self, source, error):
        """Record an error from any thread.  Keeps at most 10 entries.

        Args:
            source: Short identifier for the error origin (e.g. 'ros', 'camera').
            error: Human-readable error description.
        """
        if self._error_lock.acquire(timeout=0.1):
            try:
                self._errors.append(f"{source}: {error}")
                if len(self._errors) > 10:
                    self._errors = self._errors[-10:]
            finally:
                self._error_lock.release()

    def get_errors(self):
        """Return a copy of the current error list (up to 10 entries)."""
        if self._error_lock.acquire(timeout=0.1):
            try:
                return self._errors[:]
            finally:
                self._error_lock.release()
        return []

    def clear_errors(self):
        """Clear the accumulated error list."""
        if self._error_lock.acquire(timeout=0.1):
            try:
                self._errors.clear()
            finally:
                self._error_lock.release()

    def close(self):
        """Release all resources held by domain stores.

        Call during application shutdown to close SHM file descriptors
        and mmap objects that were opened lazily during save_calibration().
        Safe to call multiple times.
        """
        self.request_shutdown()
        if hasattr(self.detection, '_shm_writer'):
            self.detection._shm_writer.close()

    def send_ros_command(self, cmd_dict):
        """Execute a ROS command directly (threading mode -- same process).

        When run via the GUI, commands are sent through the
        /nav_command ROS2 topic instead.

        Args:
            cmd_dict: Command dictionary with a 'type' key and
                command-specific payload (see command_dispatcher.py).
        """
        from .command_dispatcher import dispatch_command
        dispatch_command(cmd_dict, self, self._ros_node)

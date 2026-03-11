"""SharedState facade composing domain-specific stores.

Used in single-process (threading) mode.  All three threads (ROS, Camera,
GTK main loop) share one SharedState instance via the ``state`` module
singleton.  Each domain store (SensorStore, DetectionStore, NavStore) uses
its own RLock with a timeout to prevent GUI hangs.

In multiprocess mode, the GUI uses IPCSharedState (see ipc_state.py) which
provides the same public API but reads/writes through IPCHub shared memory.
"""

import threading

from .sensor_store import SensorStore
from .detection_store import DetectionStore
from .nav_store import NavStore


class SharedState:
    """Thin facade over domain stores + lifecycle management.

    All three threads (ROS, Camera, GTK) import the singleton from state/__init__.py.
    """

    def __init__(self):
        """Create domain stores and lifecycle primitives."""
        self._shutdown_event = threading.Event()
        self.sensors = SensorStore()
        self.detection = DetectionStore()
        self.nav = NavStore()

        # ROS node reference (set by ros_thread, used by SettingsWindow)
        self._ros_node = None

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

    def send_ros_command(self, cmd_dict):
        """Execute a ROS command directly (threading mode — same process).

        In multiprocess mode, IPCSharedState overrides this to send via queue.
        """
        cmd_type = cmd_dict.get('type', '')
        node = self._ros_node

        if cmd_type == 'go':
            self.nav.request_go()
        elif cmd_type == 'stop':
            self.nav.request_stop()
        elif cmd_type == 'stop_robot':
            if node and hasattr(node, 'cmd_vel_pub'):
                from geometry_msgs.msg import Twist
                node.cmd_vel_pub.publish(Twist())
        elif cmd_type == 'set_nav_param':
            attr = cmd_dict.get('attr', '')
            value = cmd_dict.get('value')
            self.detection.set_nav_param(attr, value)
            if node and hasattr(node, 'navigator') and hasattr(node.navigator, attr):
                setattr(node.navigator, attr, value)
        elif cmd_type == 'set_robot_half_width':
            value = cmd_dict.get('value', 0.13)
            if node and hasattr(node, 'navigator'):
                node.navigator.robot_half_width = value
        elif cmd_type == 'reset_nav_params':
            from .detection_store import DEFAULT_NAV_PARAMS
            for k, v in DEFAULT_NAV_PARAMS.items():
                self.detection.set_nav_param(k, v)
            if node and hasattr(node, 'navigator'):
                for k, v in DEFAULT_NAV_PARAMS.items():
                    if hasattr(node.navigator, k):
                        setattr(node.navigator, k, v)
        elif cmd_type == 'arduino_cmd':
            data = cmd_dict.get('data', '')
            if node and hasattr(node, 'arduino_cmd_pub'):
                from std_msgs.msg import String
                msg = String()
                msg.data = data
                node.arduino_cmd_pub.publish(msg)
        elif cmd_type == 'reset_odom':
            if node and hasattr(node, 'reset_odom_pub'):
                from std_msgs.msg import Empty
                node.reset_odom_pub.publish(Empty())
        elif cmd_type == 'test_motor':
            if node and hasattr(node, 'cmd_vel_pub'):
                from geometry_msgs.msg import Twist
                twist = Twist()
                direction = cmd_dict.get('direction', '')
                speed = cmd_dict.get('speed', 0.1)
                if direction == 'forward':
                    twist.linear.x = speed
                elif direction == 'backward':
                    twist.linear.x = -speed
                elif direction == 'left':
                    twist.linear.y = speed
                elif direction == 'right':
                    twist.linear.y = -speed
                elif direction == 'rotate_left':
                    twist.angular.z = speed
                elif direction == 'rotate_right':
                    twist.angular.z = -speed
                node.cmd_vel_pub.publish(twist)

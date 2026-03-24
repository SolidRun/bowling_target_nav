"""Self-contained ROS2 navigation node.

Provides NavNode, a standalone ROS2 node that:
  - Reads detections from struct SHM (DetShmReader) via a 20 Hz poll timer,
    storing them in local state for the Navigator.
  - Subscribes to /scan (LaserScan) for LiDAR data.
  - Reads GUI commands from struct SHM (CmdRingBuffer) via a 20 Hz poll timer.
  - Subscribes to /settings_changed (String) to reload calibration from disk.
  - Looks up TF transforms (map->base_link, fallback odom->base_link).
  - Publishes /cmd_vel (Twist) for navigation commands.
  - Publishes /arduino/cmd (String) for direct motor commands.
  - Publishes /reset_odom (Empty) for odometry resets.
  - Writes nav state to struct SHM (NavShmWriter) at 10 Hz as the PRIMARY
    path; also publishes /nav_state (String) as a ROS topic backup.
  - Writes LiDAR points to struct SHM (LaserShmWriter) on each scan.
  - Broadcasts static TF for camera_link and laser frames.
  - Runs a 20 Hz control_loop that delegates to Navigator and NavController.

Data flow (all struct SHM):
    camera_process --DetShmWriter--> NavNode (DetShmReader)
    NavNode --NavShmWriter--> GUI (NavShmReader)
    NavNode --LaserShmWriter--> GUI (LaserShmReader)
    GUI --CmdRingBuffer.push()--> NavNode (CmdRingBuffer.pop())

main() is the standalone entry point: initializes rclpy, creates the node,
pins to CPU core 1, and spins.  ros_thread() is kept for threading mode
compatibility.

Related modules:
    app/nav_controller.py  -- state machine called from control_loop().
    app/target_tracker.py  -- Kalman tracking owned by NavController.
    nav/navigator.py       -- low-level motion algorithms.
    app/main.py            -- spawns this as Process 1 (multiprocess mode).
"""

import json
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty, String
import tf2_ros
from tf2_ros import TransformException
import numpy as np

from target_nav.state import state
from target_nav.nav.navigator import Navigator
from target_nav.app.nav_controller import NavController
from target_nav.detectors.detection import Det
from target_nav.state.command_dispatcher import dispatch_command
from target_nav.utils.log import get_logger
from target_nav.ipc.shm_struct import (
    NavShmWriter, LaserShmWriter, DetShmReader, CmdRingBuffer, NAV_STATES,
    CMD_GO, CMD_STOP, CMD_SET_PARAM, CMD_TEST_MOTOR,
)

logger = get_logger('app.nav_node')


class NavNode(Node):
    """Self-contained ROS2 navigation node -- receives detections and commands
    via struct SHM, publishes nav state for GUI consumption.

    Struct SHM channels (primary data path):
        DetShmReader   -- reads detections from camera process (20 Hz poll).
        NavShmWriter   -- writes nav state snapshot for GUI (10 Hz).
        LaserShmWriter -- writes transformed LiDAR points for GUI (per scan).
        CmdRingBuffer  -- reads commands from GUI (20 Hz poll).

    Subscriptions:
        /scan (LaserScan) -- LiDAR scan data.
        /settings_changed (String) -- triggers calibration reload from disk.

    Publishers:
        /cmd_vel (Twist) -- velocity commands for the robot.
        /arduino/cmd (String) -- raw commands for the Arduino motor driver.
        /reset_odom (Empty) -- odometry reset trigger.
        /nav_state (String) -- JSON-encoded nav state snapshot (backup).

    TF:
        Static broadcaster for camera_link and laser frames (from settings).
        Listener for map->base_link (or odom->base_link fallback) pose.

    Timers:
        20 Hz control_loop -- navigation state machine tick.
        20 Hz _poll_detections -- reads detections from struct SHM.
        20 Hz _poll_commands -- reads commands from struct SHM.
        10 Hz nav_state_publisher -- publishes nav state snapshot.
        2 Hz sensor TF check -- republishes static TFs when settings change.
    """

    def __init__(self):
        """Initialize subscriptions, publishers, TF, Navigator, and timers."""
        super().__init__('nav_node')

        # TF2 for pose lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Subscriptions (LiDAR via ROS topic; detections & commands via SHM)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.create_subscription(
            String, '/settings_changed', self.settings_changed_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino/cmd', 1)
        self.reset_odom_pub = self.create_publisher(Empty, '/reset_odom', 1)
        self._nav_state_pub = self.create_publisher(String, '/nav_state', 10)

        # Navigator (owns all navigation algorithms)
        self.navigator = Navigator(
            state=state,
            cmd_vel_pub=self.cmd_vel_pub,
            tf_buffer=self.tf_buffer,
            logger=self.get_logger())

        # Navigation state machine controller
        self._nav_controller = NavController(
            navigator=self.navigator,
            logger=self.get_logger(),
            tf_buffer=self.tf_buffer)

        # Static TF broadcaster for sensor frames
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._last_cam_yaw = None
        self._last_lidar_yaw = None
        self._last_cam_pos = None
        self._last_lidar_pos = None
        self.create_timer(0.5, self._publish_sensor_tfs)   # 2 Hz

        # Struct SHM writers (nav -> GUI, lock-free SPSC)
        try:
            self._nav_shm = NavShmWriter()
            self._laser_shm = LaserShmWriter()
        except Exception as e:
            self.get_logger().warning(f'Failed to create SHM writers: {e}')
            self._nav_shm = None
            self._laser_shm = None

        # Struct SHM reader for detections (camera -> nav)
        try:
            self._det_reader = DetShmReader()
        except Exception as e:
            self.get_logger().warning(f'Failed to create DetShmReader: {e}')
            self._det_reader = None

        # Command ring buffer (GUI -> nav, lock-free SPSC)
        try:
            self._cmd_ring = CmdRingBuffer.open()
        except Exception as e:
            self.get_logger().warning(f'Failed to open CmdRingBuffer: {e}')
            self._cmd_ring = None

        # Timers
        self.create_timer(0.05, self.control_loop)          # 20 Hz
        self.create_timer(0.1, self._publish_nav_state)     # 10 Hz
        self.create_timer(0.05, self._poll_commands)         # 20 Hz
        self.create_timer(0.05, self._poll_detections)       # 20 Hz

        # Ensure clean init — no stale target from previous session
        state.nav.clear_nav_target_map()
        state.nav.set_lidar_at_target(float('inf'))
        state.nav.set_nav_state("IDLE", None)

        self.get_logger().info('NavNode started')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    _scan_logged = False

    def scan_cb(self, msg):
        """Process /scan LaserScan: convert to cartesian, rotate to base_link, look up pose.

        Converts valid range readings to (x, y) points in base_link frame
        (applying LiDAR mounting yaw rotation), stores them for obstacle
        avoidance, and looks up the robot pose via TF2 (map->base_link,
        falling back to odom->base_link).

        Args:
            msg: sensor_msgs.msg.LaserScan from the RPLidar.
        """
        if not state.running:
            return
        if not self._scan_logged:
            self.get_logger().info(f'First /scan received: {len(msg.ranges)} ranges')
            self._scan_logged = True
        try:
            ranges = np.array(msg.ranges, dtype=np.float32)
            n = len(ranges)
            angles = np.linspace(
                msg.angle_min,
                msg.angle_min + (n - 1) * msg.angle_increment,
                n, dtype=np.float32)
            valid = (ranges > msg.range_min) & (ranges < msg.range_max)
            r_v = ranges[valid]
            a_v = angles[valid]
            if len(r_v) > 0:
                # Convert polar (range, angle) to Cartesian in laser frame
                x_l = r_v * np.cos(a_v)
                y_l = r_v * np.sin(a_v)

                # Transform from laser frame to base_link frame:
                # 1. Rotate by LiDAR mounting yaw (e.g., 180° if mounted backward)
                # 2. Translate by LiDAR mounting position (e.g., x=0.12m forward)
                lidar_yaw = math.radians(state.detection.get_lidar_yaw())
                lidar_pos = state.detection.get_lidar_pos()
                if lidar_yaw != 0.0:
                    cos_ly = math.cos(lidar_yaw)
                    sin_ly = math.sin(lidar_yaw)
                    x_b = x_l * cos_ly - y_l * sin_ly + lidar_pos['x']
                    y_b = x_l * sin_ly + y_l * cos_ly + lidar_pos['y']
                else:
                    x_b = x_l + lidar_pos['x']
                    y_b = y_l + lidar_pos['y']
                points = np.column_stack((x_b, y_b))
            else:
                points = np.empty((0, 2), dtype=np.float32)
            state.sensors.set_raw_scan(msg)

            # Pose lookup from odometry TF
            t = None
            try:
                t = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', rclpy.time.Time())
            except TransformException:
                pass
            if t is not None:
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                theta = math.atan2(siny, cosy)
                state.sensors.set_robot_pose(x, y, theta)

            state.sensors.set_laser(points)

            # Write to struct SHM for direct GUI consumption
            if self._laser_shm is not None:
                try:
                    pose = state.sensors.get_robot_pose()
                    px = pose[0] if pose else 0.0
                    py = pose[1] if pose else 0.0
                    pt = pose[2] if pose else 0.0
                    scan_hz = 1.0 / msg.scan_time if msg.scan_time > 0 else 0.0
                    # Downsample evenly to fit SHM limit (400 pts)
                    # while preserving full 360° coverage.
                    # Without this, sequential truncation only shows ~135°.
                    n_pts = len(points)
                    if n_pts > 400:
                        idx = np.linspace(0, n_pts - 1, 400, dtype=int)
                        pts_ds = points[idx]
                    else:
                        pts_ds = points
                    pts_list = [(float(p[0]), float(p[1])) for p in pts_ds]
                    self._laser_shm.write(
                        pts_list, px, py, pt, scan_hz, n)
                except Exception as e:
                    self.get_logger().warning(
                        f'SHM laser write failed: {e}', throttle_duration_sec=5.0)
        except Exception as e:
            state.add_error('ros_scan', str(e))

    def settings_changed_cb(self, msg):
        """Process /settings_changed: reload calibration from SHM or disk.

        Triggered when the GUI saves new calibration/sensor settings.
        Tries shared memory first (~1ms latency) for real-time sync,
        falls back to calibration.json on disk (~100ms) if SHM unavailable.

        Args:
            msg: std_msgs.msg.String (payload content is ignored).
        """
        try:
            from target_nav.state.settings_store import SettingsShmReader
            if not hasattr(self, '_shm_reader'):
                self._shm_reader = SettingsShmReader()
            if not state.detection.load_from_shm(self._shm_reader):
                state.detection._load_calibration()
                self.get_logger().info('Calibration reloaded from disk (SHM unavailable)')
            else:
                self.get_logger().info('Calibration reloaded from SHM')
            state.detection.mark_calibration_dirty()
        except Exception as e:
            self.get_logger().warning(f'Failed to reload calibration: {e}')

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_nav_state(self):
        """Publish navigation state snapshot as JSON on /nav_state at 10 Hz.

        Collects the nav state from NavStore.get_gui_snapshot() and publishes
        it as a JSON-encoded String so the GUI can display navigation status
        without needing shared memory.

        Published fields:
            nav_state (str): Current state (IDLE, NAVIGATING, SEARCHING, etc.).
            nav_target (list|null): [class_name, angle, distance] or null.
            nav_target_map (list|null): [x, y] map coordinates or null.
            cmd_vel (list): [vx, vy, wz] current velocity command.
            obstacle_dist (float): Distance to nearest obstacle in meters.
            obstacle_ahead (bool): True if obstacle within clearance zone.
            time_since_target (float): Seconds since last target detection.
            search_time (float): Seconds spent in current search.
            lidar_at_target (float): LiDAR distance at target angle.
        """
        try:
            snap = state.nav.get_gui_snapshot()
            # Convert tuples/non-serializable types for JSON
            nav_target = snap['nav_target']
            if nav_target is not None:
                nav_target = list(nav_target)
            nav_target_map = snap['nav_target_map']
            if nav_target_map is not None:
                nav_target_map = list(nav_target_map)
            cmd_vel = list(snap['cmd_vel'])

            # Convert all numeric values to Python float (numpy float32 is
            # not JSON serializable)
            def _f(v):
                """Convert to Python float, handling inf/nan."""
                try:
                    v = float(v)
                    return v if math.isfinite(v) else 999.0
                except (TypeError, ValueError):
                    return 0.0

            target_robot_xy = state.nav.get_target_robot_xy()
            if target_robot_xy is not None:
                target_robot_xy = [_f(target_robot_xy[0]), _f(target_robot_xy[1])]

            payload = {
                'nav_state': snap['nav_state'],
                'nav_target': [_f(x) for x in nav_target] if nav_target else None,
                'nav_target_map': [_f(x) for x in nav_target_map] if nav_target_map else None,
                'target_robot_xy': target_robot_xy,
                'cmd_vel': [_f(x) for x in cmd_vel],
                'obstacle_dist': _f(snap['obstacle_dist']),
                'obstacle_ahead': bool(snap['obstacle_ahead']),
                'time_since_target': _f(snap['time_since_target']),
                'search_time': _f(snap['search_time']),
                'lidar_at_target': _f(snap.get('lidar_at_target', float('inf'))),
            }
            msg = String()
            msg.data = json.dumps(payload, default=lambda o: float(o))
            self._nav_state_pub.publish(msg)

            # Also write to struct SHM for direct GUI consumption
            if self._nav_shm is not None:
                try:
                    nav_state_name = snap['nav_state']
                    nav_state_idx = (
                        NAV_STATES.index(nav_state_name)
                        if nav_state_name in NAV_STATES else 6  # ERROR
                    )
                    trxy = target_robot_xy or [0.0, 0.0]
                    ntm = nav_target_map or [0.0, 0.0]
                    self._nav_shm.write(
                        timestamp=time.time(),
                        nav_state=nav_state_idx,
                        lidar_at_target=_f(snap.get('lidar_at_target', float('inf'))),
                        time_since_target=_f(snap['time_since_target']),
                        target_robot_x=_f(trxy[0]),
                        target_robot_y=_f(trxy[1]),
                        nav_target_map_x=_f(ntm[0]),
                        nav_target_map_y=_f(ntm[1]),
                        vx=_f(cmd_vel[0]),
                        vy=_f(cmd_vel[1]),
                        wz=_f(cmd_vel[2]),
                        obstacle_ahead=bool(snap['obstacle_ahead']),
                        obstacle_dist=_f(snap['obstacle_dist']),
                        search_time=_f(snap['search_time']),
                        detector_mode=state.detection.get_detector_mode()
                            if hasattr(state.detection, 'get_detector_mode') else '',
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f'SHM nav write failed: {e}', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warning(f'Failed to publish nav state: {e}')

    # ------------------------------------------------------------------
    # SHM detection polling
    # ------------------------------------------------------------------

    def _poll_detections(self):
        """Poll DetShmReader at 20 Hz and update detection state.

        Reads Detection dataclass objects from struct SHM, converts them
        to Det objects, and stores them via state.detection.set_camera()
        so the Navigator can use them.
        """
        if self._det_reader is None:
            return

        try:
            result = self._det_reader.read()
        except Exception as e:
            self.get_logger().warning(
                f'DetShmReader read failed: {e}', throttle_duration_sec=5.0)
            return

        if result is None:
            return  # No new data or torn read

        timestamp, det_objects, info = result

        dets = []
        for d in det_objects:
            dets.append(Det(
                class_name=d.class_name,
                confidence=d.confidence,
                bbox=(d.bbox_x1, d.bbox_y1, d.bbox_x2, d.bbox_y2),
                distance=d.distance,
                angle=d.angle,
                bbox_clipped=d.bbox_clipped,
            ))

        det_age = time.time() - timestamp if timestamp > 0 else 999.0
        state.detection.set_camera(
            None, dets, info, fresh_detection=(det_age < 2.0))

    # ------------------------------------------------------------------
    # SHM command polling
    # ------------------------------------------------------------------

    def _poll_commands(self):
        """Poll the CmdRingBuffer at 20 Hz and dispatch any pending commands.

        Maps ring buffer command types to the same dict format used by
        dispatch_command(), so both ROS topic and SHM command paths share
        identical handling logic.
        """
        if self._cmd_ring is None:
            # Try to open lazily (GUI may start after nav)
            try:
                self._cmd_ring = CmdRingBuffer.open()
            except Exception:
                pass
            if self._cmd_ring is None:
                return

        # Drain all pending commands (up to slot count to avoid infinite loop)
        for _ in range(8):
            try:
                result = self._cmd_ring.pop()
            except Exception as e:
                self.get_logger().warning(
                    f'CmdRingBuffer pop failed: {e}', throttle_duration_sec=5.0)
                self._cmd_ring = None
                return
            if result is None:
                break

            cmd_type, payload_raw = result
            try:
                if cmd_type == CMD_GO:
                    cmd_dict = {'type': 'go'}
                elif cmd_type == CMD_STOP:
                    cmd_dict = {'type': 'stop'}
                elif cmd_type == CMD_SET_PARAM:
                    # Payload is JSON-encoded param dict
                    payload_str = payload_raw.rstrip(b'\x00').decode('utf-8')
                    cmd_dict = json.loads(payload_str)
                    cmd_dict.setdefault('type', 'set_nav_param')
                elif cmd_type == CMD_TEST_MOTOR:
                    payload_str = payload_raw.rstrip(b'\x00').decode('utf-8')
                    cmd_dict = json.loads(payload_str) if payload_str else {}
                    cmd_dict.setdefault('type', 'test_motor')
                else:
                    continue

                dispatch_command(cmd_dict, state, self)
            except Exception as e:
                self.get_logger().warning(
                    f'SHM command dispatch failed (type={cmd_type}): {e}')

    # ------------------------------------------------------------------
    # Static TF
    # ------------------------------------------------------------------

    def _publish_sensor_tfs(self):
        """Publish static TF frames for camera_link and laser from settings store.

        Called at 2 Hz but only republishes when mounting parameters change.
        Uses StaticTransformBroadcaster (not dynamic) because:
          - Static TFs never go stale between publishes.
          - A dynamic TF at 2 Hz can expire, causing TF consumers to fall
            back to the URDF's identity TF -- which garbles transforms
            when lidar_yaw is non-zero (e.g., LiDAR mounted at 180 deg).

        The TF chain is: odom -> base_link -> camera_link / laser.
        """
        cam = state.detection.get_camera_pos()
        cam_yaw = state.detection.get_camera_yaw()
        lidar = state.detection.get_lidar_pos()
        lidar_yaw_deg = state.detection.get_lidar_yaw()

        # Skip if nothing changed
        if (cam_yaw == self._last_cam_yaw and
                lidar_yaw_deg == self._last_lidar_yaw and
                cam == self._last_cam_pos and
                lidar == self._last_lidar_pos):
            return
        self._last_cam_yaw = cam_yaw
        self._last_lidar_yaw = lidar_yaw_deg
        self._last_cam_pos = dict(cam) if cam else None
        self._last_lidar_pos = dict(lidar) if lidar else None

        now = self.get_clock().now().to_msg()

        # Camera frame
        cam_yaw_rad = math.radians(cam_yaw)
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = cam['x']
        t_cam.transform.translation.y = cam['y']
        t_cam.transform.translation.z = cam['z']
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.0
        t_cam.transform.rotation.z = math.sin(cam_yaw_rad / 2.0)
        t_cam.transform.rotation.w = math.cos(cam_yaw_rad / 2.0)

        # LiDAR frame
        lidar_yaw = math.radians(lidar_yaw_deg)
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'laser'
        t_lidar.transform.translation.x = lidar['x']
        t_lidar.transform.translation.y = lidar['y']
        t_lidar.transform.translation.z = lidar['z']
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = math.sin(lidar_yaw / 2.0)
        t_lidar.transform.rotation.w = math.cos(lidar_yaw / 2.0)

        self._tf_broadcaster.sendTransform([t_cam, t_lidar])

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        """Main 20 Hz control loop timer callback -- delegates to NavController.

        Called by a rclpy timer every 50ms. Runs three things in order:

        1. Parameter refresh (throttled to every 2s internally): re-reads
           tunable parameters from SettingsStore so Settings GUI changes
           take effect without restarting navigation.
        2. NavController.step(): one tick of the navigation state machine.
        3. cmd_vel watchdog: stops motors if Navigator hasn't published
           a velocity command for 0.5s, preventing runaway on crash/hang.

        On any unhandled exception, publishes a zero Twist as a safety stop.
        """
        try:
            self.navigator.refresh_params()
            self._nav_controller.step()
        except Exception as e:
            self.get_logger().error(f"NavController error: {e}")
            self.cmd_vel_pub.publish(Twist())

        # Safety watchdog: stop motors if no cmd published recently
        self.navigator.check_watchdog()


def _close_shm(node):
    """Close SHM writers/readers/ring buffer on a NavNode instance."""
    for attr in ('_nav_shm', '_laser_shm', '_det_reader', '_cmd_ring'):
        writer = getattr(node, attr, None)
        if writer is not None:
            try:
                writer.close()
            except Exception:
                pass


def ros_thread():
    """ROS2 spin thread entry point for threading mode compatibility.

    Initializes rclpy, creates the NavNode, and spins (processing
    callbacks at ~50 Hz) until shared_state.running becomes False.
    On exit, destroys the node and shuts down rclpy.

    Note:
        This function runs in a dedicated thread started by the GUI process.
        It is the only thread that calls rclpy.spin_once().
    """
    node = None
    logger.info("Starting...")
    try:
        rclpy.init()
        node = NavNode()
        state._ros_node = node

        while state.running:
            try:
                rclpy.spin_once(node, timeout_sec=0.02)
            except Exception as e:
                if state.running:
                    import traceback
                    logger.error("spin_once error: %s", e)
                    traceback.print_exc()
                    state.add_error('ros_spin', str(e))
                break
    except Exception as e:
        import traceback
        logger.critical("Fatal error: %s", e)
        traceback.print_exc()
        state.add_error('ros_thread', str(e))
    finally:
        logger.info("Cleaning up...")
        if node:
            try:
                _close_shm(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        logger.info("Stopped")


def main():
    """Standalone entry point: initialize rclpy, pin to core 1, and spin.

    Intended for running the navigation node as an independent process
    (e.g. via ros2 run or a launch file) without needing the GUI or
    the GUI. Pins the process to CPU core 1 and ignores SIGINT (expects
    the parent or launch system to handle shutdown via rclpy).

    The node runs until rclpy is shut down (e.g. via Ctrl+C or
    ros2 lifecycle).
    """
    # Pin to CPU core 1 with elevated priority so DRP-AI can't starve it
    try:
        os.sched_setaffinity(0, {1})
        os.nice(-5)  # higher priority than default (0)
        logger.info("Pinned to core 1 (nice=-5)")
    except Exception:
        logger.warning("Could not pin to core 1 or set priority")

    # Ignore SIGINT in child -- parent/launch handles it
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    rclpy.init()
    node = NavNode()
    state._ros_node = node

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _close_shm(node)
        node.destroy_node()
        rclpy.shutdown()
        logger.info("Stopped")


if __name__ == '__main__':
    main()

"""Navigation control state machine.

NavController encapsulates the 20 Hz control loop logic called by
NavNode.control_loop(). It manages the navigation state machine,
uses TargetTracker for validated/smoothed target positions, and
delegates low-level motion to the Navigator.

State machine diagram::

    IDLE ──(GO)──> NAVIGATING ──(arrived)──> ARRIVED
      ^                |   |                    |
      |                |   └──(close+lost)──> BLIND_APPROACH
      |                |                        |
      |                └──(lost>timeout)──> SEARCHING
      |                                       |
      |                                 SPIRAL_SEARCH
      |                                       |
      └───────────(STOP / exhausted)──────────┘

State machine transitions::

    IDLE --(GO)--> NAVIGATING
    NAVIGATING --(target visible)--> navigate_to_target()
    NAVIGATING --(target lost briefly, < lost_timeout)--> keep navigating last target
    NAVIGATING --(target lost, close)--> BLIND_APPROACH (dead-reckoning)
    NAVIGATING --(target lost, far)--> SEARCHING (360-degree scan)
    BLIND_APPROACH --(target re-detected)--> NAVIGATING
    BLIND_APPROACH --(arrived / timeout / obstacle)--> ARRIVED or SEARCHING
    SEARCHING --(360 deg done, spiral enabled)--> SPIRAL_SEARCH
    SEARCHING --(360 deg done, spiral disabled)--> IDLE
    SEARCHING --(safety timeout)--> IDLE
    SPIRAL_SEARCH --(exhausted)--> IDLE
    any state --(STOP)--> IDLE
    NAVIGATING/BLIND_APPROACH --(arrival confirmed)--> ARRIVED
    ARRIVED --(GO)--> NAVIGATING (user must press GO again)

Thread safety: step() is called only from the ROS spin thread's timer
callback. All shared state access goes through locked stores.

Related modules:
    app/nav_node.py       -- owns NavController, calls step() at 20 Hz.
    app/target_tracker.py -- TargetTracker used for validated target positions.
    nav/navigator.py      -- Navigator handles low-level motion commands.
"""

import math
import time

from target_nav.state import state
from target_nav.app.target_tracker import TargetTracker


class NavController:
    """Navigation state machine controller.

    Uses TargetTracker for size-gated, LiDAR-confirmed, Kalman-filtered
    target tracking. The tracker replaces the old find_best_target() +
    _update_target_preview() pipeline with a single update() call.

    Args:
        navigator: Navigator instance (owns motion algorithms).
        logger: ROS2 logger for info/warn messages.
        tf_buffer: tf2_ros.Buffer for TF lookups.
    """

    def __init__(self, navigator, logger, tf_buffer):
        """Initialize state machine and target tracker."""
        self.navigator = navigator
        self.logger = logger
        self.tf_buffer = tf_buffer

        # Navigation state
        self.is_active = False
        self.last_target = None  # last TrackedTarget for grace period

        # Target tracker (Kalman filter + size gate + LiDAR matching)
        self.tracker = TargetTracker()

    def step(self):
        """Run one tick of the navigation state machine.

        Called at 20 Hz from NavNode.control_loop(). Pipeline:
            1. Handle STOP/GO commands.
            2. Get detections + LiDAR points.
            3. Run tracker.update() → TrackedTarget or None.
            4. Update nav_store (target_robot_xy, lidar_at_target, map goal).
            5. Dispatch to navigation behavior based on state.
        """
        nav = self.navigator

        # --- STOP ---
        if state.nav.check_and_clear_stop():
            nav.stop_robot()
            self.is_active = False
            nav.blind_approach_active = False
            self.last_target = None
            self.tracker.reset()
            state.nav.set_nav_state("IDLE")
            state.nav.clear_nav_target_map()
            state.nav.clear_target_robot_xy()
            state.nav.set_lidar_at_target(float('inf'))
            self.logger.info("Navigation stopped")
            return

        # --- GO ---
        if state.nav.check_and_clear_go():
            self.is_active = True
            nav.blind_approach_active = False
            nav._arrival_first_seen = 0.0
            self.last_target = None
            self.tracker.reset()
            state.detection.request_bbox_reset()
            state.nav.clear_nav_target_map()
            state.nav.clear_target_robot_xy()
            state.nav.set_lidar_at_target(float('inf'))
            state.nav.set_nav_state("IDLE", None)
            self.logger.info("Navigation activated")

        # --- Get inputs ---
        _, detections, _, det_age = state.detection.get_camera()
        if det_age > state.detection.get_detect_expiry():
            detections = []

        lpoints, _, _ = state.sensors.get_laser()
        camera_yaw_rad = math.radians(state.detection.get_camera_yaw())

        # Refresh tracker calibration periodically
        self.tracker.refresh_config(state.detection)

        # --- Run tracker ---
        tracked = self.tracker.update(detections, lpoints, camera_yaw_rad)

        # --- Update shared state for GUI (radar display + info bar) ---
        if tracked:
            state.nav.set_target_robot_xy(tracked.x_bl, tracked.y_bl)
            lidar_dist = self.tracker.get_lidar_dist()
            state.nav.set_lidar_at_target(lidar_dist)

            # Update odom-frame goal for navigation
            # tracked.angle is in base_link frame; convert back to camera
            # frame: cam_angle = angle_bl - camera_yaw_rad
            nav._update_map_goal(
                tracked.distance, tracked.angle - camera_yaw_rad,
                camera_yaw_rad)

            self.last_target = tracked
        else:
            state.nav.clear_target_robot_xy()
            # Only reset lidar_at_target when target is truly lost,
            # not on every tick without a detection (prevents flickering)
            tst = state.nav.get_time_since_target()
            if tst > 3.0:
                state.nav.set_lidar_at_target(float('inf'))

        # Update target timing even when idle (for GUI display)
        if tracked and tracked.age_ticks == 0:
            state.nav.update_target_seen()

        # --- IDLE: no navigation active ---
        if not self.is_active:
            if tracked is None:
                state.nav.clear_nav_target_map()
            return

        # Arrival detection is handled ONLY by Navigator._check_arrival()
        # (arrival.py) which uses multi-signal scoring. No duplicate check here.

        nav_state, _ = state.nav.get_nav_state()

        # --- TARGET VISIBLE (tracker has a target) ---
        if tracked and tracked.age_ticks == 0:
            # Fresh detection this tick
            state.nav.update_target_seen()

            if nav.blind_approach_active:
                self.logger.info("Target re-detected during BLIND_APPROACH")
                nav.blind_approach_active = False
                state.nav.set_nav_state("NAVIGATING")

            if nav_state in ("SEARCHING", "BLIND_APPROACH", "SPIRAL_SEARCH"):
                if nav._spiral_active:
                    nav._spiral_active = False
                    nav._spiral_radius = 0.0
                    nav._spiral_start_time = 0.0
                nav._search_initialized = False
                self.logger.info("Target found!")

            if nav_state == "ARRIVED":
                nav.stop_robot()
                self.is_active = False
                return

            nav.navigate_to_target(tracked)

        elif tracked and tracked.age_ticks > 0:
            # Tracker is predicting (no fresh detection but track still alive)
            if nav_state == "NAVIGATING":
                nav.navigate_to_target(tracked)

        else:
            # --- TARGET LOST (tracker returned None) ---
            time_since = state.nav.get_time_since_target()

            if nav.blind_approach_active:
                nav.blind_approach_step()
                return

            # Grace period with last known target
            if (nav_state == "NAVIGATING" and time_since <= nav.lost_timeout
                    and self.last_target is not None):
                nav.navigate_to_target(self.last_target)
                return

            # Lost timeout expired
            if nav_state == "NAVIGATING" and time_since > nav.lost_timeout:
                _, nav_target = state.nav.get_nav_state()
                last_dist = nav_target[2] if nav_target else float('inf')

                if last_dist < nav.blind_approach_entry_distance:
                    if nav.enter_blind_approach():
                        nav.blind_approach_step()
                        return

                odom_goal = state.nav.get_nav_target_map()
                if odom_goal is not None and self.last_target is not None:
                    nav.navigate_to_target(self.last_target)
                    return

                self.logger.warning(
                    f"Target lost for {time_since:.1f}s - searching...")
                state.nav.start_search()
                nav.search_rotate()

            elif nav_state == "SEARCHING":
                search_time = state.nav.get_search_time()
                if search_time > nav.search_timeout:
                    self.logger.warning(
                        f"Search timeout ({search_time:.1f}s)")
                    nav.stop_robot()
                    self.is_active = False
                    state.nav.set_nav_state("IDLE", None)
                elif nav.search_rotate():
                    if nav.spiral_search_enabled:
                        self.logger.info(
                            "360° scan done - starting spiral search")
                        nav.start_spiral_search()
                    else:
                        self.logger.warning(
                            "360° scan complete - target not found")
                        self.is_active = False
                        state.nav.set_nav_state("IDLE", None)

            elif nav_state == "SPIRAL_SEARCH":
                if nav.spiral_search_step():
                    self.logger.warning("Spiral search exhausted")
                    nav.stop_robot()
                    self.is_active = False
                    state.nav.set_nav_state("IDLE", None)

            elif nav_state == "IDLE" and self.is_active:
                self.logger.info("No target visible - searching...")
                state.nav.start_search()
                nav.search_rotate()

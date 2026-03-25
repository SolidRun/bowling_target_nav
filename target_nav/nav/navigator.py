"""Mecanum holonomic navigation with obstacle avoidance and blind approach.

This module implements the main navigation pipeline for a 4-wheel mecanum
robot.  The Navigator class is the central controller, composed from four
mixins that each handle one aspect of navigation:

Algorithm phases (in order of priority):

1. **Map-goal navigation** -- camera detects target, TF2 converts to odom
   frame, robot navigates to the stored goal using odometry + LiDAR.
   Camera re-detections refine the goal continuously.
2. **VFH obstacle avoidance** (``ObstacleAvoidanceMixin``) -- Vector Field
   Histogram gap-finding lets the robot strafe around obstacles while
   keeping heading toward the map goal.
3. **Blind approach** (``BlindApproachMixin``) -- dead-reckon to the last
   known odom-frame target position when the target drops out of view at
   close range (below camera FOV or LiDAR minimum range).
4. **Tiered search** (``SearchMixin``) -- 360-degree rotation scan (Tier 1)
   followed by an expanding Archimedean spiral (Tier 2) when the target
   is lost after ``lost_timeout`` seconds.
5. **Temporal arrival confirmation** (``ArrivalMixin``) -- requires multiple
   sensor signals to agree for a sustained period before declaring arrival.

Distance fusion: calibrated vision distance is used by default; LiDAR
distance is substituted when the bounding box is clipped (target partly
out of frame, making vision distance unreliable).

All velocity commands are published through ``_publish_cmd`` which sanitizes
NaN/Inf and enforces speed caps.

Runs in: Nav process (Process 1, Core 1) -- ROS2 control_loop timer
callback (single-threaded).

Cross-references:
  - ``nav/obstacle_avoidance.py`` -- VFH mixin
  - ``nav/blind_approach.py``     -- dead-reckoning mixin
  - ``nav/search.py``             -- 360-deg scan + spiral mixin
  - ``nav/arrival.py``            -- temporal arrival mixin
  - ``app/nav_controller.py``     -- orchestrates Navigator from the ROS node
  - ``config.py``                 -- DEFAULT_NAV_PARAMS (user-tunable defaults)
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist, PointStamped

# ---------------------------------------------------------------------------
# Algorithm constants (internal tuning -- NOT user-configurable)
# ---------------------------------------------------------------------------
# These control the internal behavior of VFH obstacle avoidance, blind
# approach, arrival scoring, and spiral search.  They are tuned for the
# mecanum platform and should rarely need changing.
#
# User-tunable parameters (approach_distance, speeds, timeouts, etc.)
# are defined in config.py DEFAULT_NAV_PARAMS and loaded at
# runtime via state.detection.get_nav_params().  Users change these
# through the Settings GUI.
# ---------------------------------------------------------------------------

# -- Speed scaling factors --
# Multipliers applied to the user-configured linear_speed or min_linear_speed.
# Example: at APPROACH_CREEP_FACTOR=0.5 with min_speed=0.10, creep speed = 0.05 m/s.
# Tuning: increase if robot is too cautious, decrease if it overshoots.
APPROACH_CREEP_FACTOR = 0.5       # arrival creep: speed = min_speed * 0.5
VFH_AVOIDANCE_SPEED_FACTOR = 0.6  # VFH gap traversal: speed = linear_speed * 0.6
EMERGENCY_BACKUP_FACTOR = 0.4     # emergency reverse: speed = min_speed * 0.4
VFH_BACKUP_SPEED_FACTOR = 0.3     # no-gap reverse: speed = min_speed * 0.3
SLOWDOWN_MIN_FACTOR = 0.3         # obstacle zone floor: speed >= linear_speed * 0.3
BLIND_OBSTACLE_SPEED_FACTOR = 0.3 # blind approach near obstacle: speed * 0.3
SPIRAL_AVOIDANCE_SPEED_FACTOR = 0.5  # spiral near obstacle: speed * 0.5

# -- Angular control --
# Proportional steering: angular_vel = GAIN * angle_error_rad, clamped to ±CLAMP.
# Higher GAIN → faster correction but risk of oscillation.
# Lower GAIN → smoother but may not track fast-moving targets.
# Tuned empirically for 0.20 m/s at 20 Hz control rate.
STEERING_ANGULAR_GAIN = 0.4       # normal navigation: wz = 0.4 * error_rad
STEERING_ANGULAR_CLAMP = 0.2      # max |wz| during normal navigation (rad/s)
ARRIVAL_ANGULAR_GAIN = 0.3        # gentler gain during arrival creep
ARRIVAL_ANGULAR_CLAMP = 0.15      # max |wz| during arrival creep (rad/s)
VFH_FUSED_ANGULAR_FACTOR = 0.4    # angular speed factor when target visible during VFH
VFH_FUSED_ANGULAR_GAIN = 0.5      # proportional gain for fused VFH steering
VFH_ROTATE_ANGULAR_FACTOR = 0.3   # angular speed factor when rotating toward gap
VFH_ROTATE_ANGULAR_GAIN = 0.3     # proportional gain when rotating toward gap
CLEAR_PATH_ANGULAR_FACTOR = 0.4   # angular speed factor on clear path
CLEAR_PATH_ANGULAR_GAIN = 0.5     # proportional gain on clear path
BLIND_ANGULAR_CLAMP = 0.2         # max abs angular vel in blind approach
BLIND_ANGULAR_GAIN = 0.5          # proportional gain in blind approach

# -- Timing constants --
# Durations and intervals that control temporal behavior.
# ARRIVAL_CONFIRM_SECS: at 0.15 m/s the robot crosses the 22cm arrival zone
# in ~1.5s. A 0.3s confirmation catches it; 0.8s+ would miss it.
ARRIVAL_CONFIRM_SECS = 0.3        # seconds distance must stay < threshold to confirm arrival
ARRIVAL_HYSTERESIS_FACTOR = 1.5   # reset timer only when dist > threshold * 1.5 (prevents flicker)
VFH_AVOID_COMMIT_SECS = 1.5      # seconds to commit to avoidance direction
VFH_COMMIT_TARGET_SECS = 0.5     # shorter commit when target is visible
VFH_STUCK_TIMEOUT_SECS = 3.0     # stuck backing up timeout
LIDAR_STALE_TIMEOUT_SECS = 2.0   # LiDAR data staleness threshold
LOG_INTERVAL_SECS = 2.0          # minimum interval between repeated log messages
OBS_LOG_INTERVAL_SECS = 1.0      # minimum interval between obstacle log messages

# -- Geometric / threshold constants --
# Distances, margins, and bin counts for VFH histogram, corridor checks,
# and blind approach safety limits.
VFH_NUM_BINS = 36                 # polar histogram bins: 360°/36 = 10° per bin
AVOID_CLEAR_TICKS = 5             # consecutive clear ticks before resetting avoidance
LIDAR_MIN_POINT_DIST = 0.03      # ignore LiDAR points closer than this (noise floor)
VFH_CLEARANCE_MARGIN = 0.10      # extra margin added to obstacle_distance for gap check
VFH_BODY_MARGIN = 0.03           # extra margin added to robot_half_width for body angle
FRONT_CORRIDOR_MARGIN = 0.05     # extra width added to front corridor check
SIDE_CHECK_DEPTH = 0.4           # lateral depth for side obstacle checks
TARGET_NEAR_FACTOR = 1.5         # target visible slowdown factor
BLIND_LIDAR_ARRIVAL_FACTOR = 3   # blind approach LiDAR arrival factor
BLIND_MAX_HEADING_DEG = 45       # max heading divergence before aborting blind approach
BLIND_SPEED_RAMP_DIST = 0.30     # distance over which blind approach ramps speed

# -- Spiral search constants --
# Parameters for the Archimedean spiral (Tier 2 search). The spiral
# equation is r(theta) = initial_radius + growth_rate * theta / (2*pi),
# so the radius increases by growth_rate meters per revolution.
SPIRAL_DT_CLAMP = 0.2            # max dt per tick to prevent jumps after pauses
SPIRAL_MIN_RADIUS = 0.05         # minimum radius before using fallback angular rate
SPIRAL_MIN_ANGULAR_RATE = 2.0    # angular rate (rad/s) when radius is very small
SPIRAL_WAYPOINT_THRESHOLD = 0.02 # distance threshold to consider waypoint reached
SPIRAL_SWEEP_FREQ = 0.8          # camera sweep oscillation frequency (Hz)
REPULSION_MIN_MAG = 0.01         # minimum repulsive magnitude to act on


def _clamp_twist(cmd):
    """Sanitize a Twist message by replacing NaN/Inf values with zero.

    Args:
        cmd: geometry_msgs.msg.Twist to sanitize (modified in-place).

    Returns:
        True if all values were already finite, False if any were replaced.
    """
    valid = True
    for attr in ('x', 'y', 'z'):
        v = getattr(cmd.linear, attr)
        if not math.isfinite(v):
            setattr(cmd.linear, attr, 0.0)
            valid = False
        v = getattr(cmd.angular, attr)
        if not math.isfinite(v):
            setattr(cmd.angular, attr, 0.0)
            valid = False
    return valid


def _cap_speed(cmd, max_linear, max_angular):
    """Clamp total linear velocity magnitude and angular velocity.

    Scales linear.x and linear.y proportionally so the resultant speed
    does not exceed max_linear. Clamps angular.z independently.

    Args:
        cmd: geometry_msgs.msg.Twist to clamp (modified in-place).
        max_linear: Maximum resultant linear speed in m/s.
        max_angular: Maximum absolute angular velocity in rad/s.
    """
    total = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
    if total > max_linear and total > 0:
        scale = max_linear / total
        cmd.linear.x *= scale
        cmd.linear.y *= scale
    cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))


from .obstacle_avoidance import ObstacleAvoidanceMixin
from .blind_approach import BlindApproachMixin
from .search import SearchMixin
from .arrival import ArrivalMixin


class Navigator(ObstacleAvoidanceMixin, BlindApproachMixin, SearchMixin, ArrivalMixin):
    """Central navigation controller for the mecanum robot, composed from mixins.

    Lifecycle:
      1. Created once by NavController when navigation starts.
      2. ``navigate_to_target()`` is called each control-loop tick when a
         target is visible.  It updates the odom-frame goal, checks arrival,
         and delegates to VFH / blind approach as needed.
      3. ``refresh_params()`` is called periodically to pick up live Settings
         GUI changes.
      4. ``stop_robot()`` is called on navigation end or mode switch.

    Mixin composition (MRO):
      - ``ObstacleAvoidanceMixin`` -- VFH gap-finding + obstacle corridor checks
      - ``BlindApproachMixin``     -- dead-reckoning when camera loses target
      - ``SearchMixin``            -- 360-deg scan + Archimedean spiral
      - ``ArrivalMixin``           -- multi-signal temporal arrival confirmation

    Thread-safety: This class is only called from the ROS control_loop timer
    (single-threaded). All shared state access goes through locked stores.

    Args:
        state: SharedState instance for sensor/nav data access.
        cmd_vel_pub: ROS2 Twist publisher for /cmd_vel.
        tf_buffer: TF2 buffer for coordinate transforms.
        logger: ROS2 logger instance.
    """

    def __init__(self, state, cmd_vel_pub, tf_buffer, logger):
        """Initialize Navigator with ROS2 interfaces and load tunable params.

        Composes ObstacleAvoidanceMixin (VFH), BlindApproachMixin,
        SearchMixin (360-deg scan + spiral), and ArrivalMixin (temporal
        confirmation). All navigation parameters are read from
        state.detection and can be changed live via the Settings GUI.

        Args:
            state: SharedState -- locked access to sensor, nav, and detection stores.
            cmd_vel_pub: ROS2 Publisher[Twist] for /cmd_vel.
            tf_buffer: tf2_ros.Buffer for camera_link -> map transforms.
            logger: rclpy logger for navigation log messages.
        """
        self._state = state
        self._cmd_vel_pub = cmd_vel_pub
        self._tf_buffer = tf_buffer
        self._logger = logger

        # LiDAR hardware minimum range (RPLidar A1)
        from target_nav.config import LIDAR_MIN_RANGE
        self.lidar_min_range = LIDAR_MIN_RANGE

        # Navigation parameters (tunable via SettingsWindow, persisted in config)
        params = state.detection.get_nav_params()
        self.approach_distance = params['approach_distance']
        self.linear_speed = params['linear_speed']
        self.min_linear_speed = params['min_linear_speed']
        self.angular_speed = params['angular_speed']
        self.obstacle_distance = params['obstacle_distance']
        self.obstacle_slowdown_distance = params['obstacle_slowdown_distance']
        self.robot_half_width = state.detection.get_robot_half_width()
        self.search_angular_speed = params['search_angular_speed']
        self.lost_timeout = params['lost_timeout']
        self.search_timeout = params['search_timeout']

        # Blind approach parameters (auto-derived from simple settings)
        self.blind_approach_active = False
        self.blind_approach_start_time = 0.0
        self.blind_approach_start_pose = (0.0, 0.0, 0.0)
        self.blind_approach_target_map = (0.0, 0.0)
        self.blind_approach_target_distance = 0.0
        self.blind_approach_entry_distance = params.get('blind_approach_entry_distance',
                                                         self.obstacle_slowdown_distance)
        self.blind_approach_speed = params.get('blind_approach_speed',
                                                self.min_linear_speed)
        self.blind_approach_timeout = params.get('blind_approach_timeout', 10.0)
        self.blind_approach_lidar_stop = params.get('blind_approach_lidar_stop',
                                                     self.lidar_min_range)
        self.blind_approach_arrival_margin = params.get('blind_approach_arrival_margin',
                                                         self.approach_distance)

        # Tracking
        self.last_target_angle = 0.0
        self._last_nav_log = 0.0
        self._last_obs_log = 0.0
        self._stuck_start = 0.0  # For stuck detection

        # VFH obstacle avoidance state
        self._avoid_direction = 0     # -1=right, +1=left, 0=none
        self._avoid_start_time = 0.0
        self._avoid_min_commit = VFH_AVOID_COMMIT_SECS
        self._avoid_clear_count = 0   # consecutive obstacle-free ticks

        # 360° search scan state (Tier 1)
        self._search_start_yaw = 0.0
        self._search_last_yaw = 0.0
        self._search_total_rotated = 0.0
        self._search_initialized = False

        # Spiral search state (Tier 2: after 360° scan fails)
        self._spiral_active = False
        self._spiral_start_time = 0.0
        self._spiral_start_x = 0.0
        self._spiral_start_y = 0.0
        self._spiral_angle = 0.0
        self._spiral_radius = 0.0
        self._spiral_last_time = 0.0

        # Spiral search tunable parameters
        self.spiral_search_enabled = params['spiral_search_enabled']
        self.spiral_initial_radius = params['spiral_initial_radius']
        self.spiral_max_radius = params['spiral_max_radius']
        self.spiral_growth_rate = params['spiral_growth_rate']
        self.spiral_linear_speed = params['spiral_linear_speed']
        self.spiral_angular_speed = params['spiral_angular_speed']
        self.spiral_timeout = params['spiral_timeout']

        # Arrival confirmation (temporal filter)
        self._arrival_first_seen = 0.0   # timestamp when first "close" reading
        self._arrival_confirm_time = ARRIVAL_CONFIRM_SECS
        self._arrival_hysteresis = ARRIVAL_HYSTERESIS_FACTOR

        # cmd_vel watchdog: if _publish_cmd() isn't called for 0.5s, stop motors.
        # Prevents runaway if the control loop crashes or hangs.
        # _watchdog_fired prevents continuous zero-Twist publishing that would
        # interfere with firmware position commands via /arduino/cmd.
        self._last_cmd_time = time.time()
        self._watchdog_fired = False

        # Throttle param refresh to avoid lock contention every tick
        self._last_param_refresh = 0.0

    def refresh_params(self):
        """Re-read tunable parameters from SettingsStore (call periodically).

        Without this, parameters changed in the Settings GUI while navigating
        would be ignored until the next navigation session. Called every 2s
        from the control loop to pick up live changes.
        """
        now = time.time()
        if now - self._last_param_refresh < 2.0:
            return  # Only refresh every 2 seconds
        self._last_param_refresh = now

        params = self._state.detection.get_nav_params()
        self.approach_distance = params['approach_distance']
        self.linear_speed = params['linear_speed']
        self.min_linear_speed = params['min_linear_speed']
        self.angular_speed = params['angular_speed']
        self.obstacle_distance = params['obstacle_distance']
        self.obstacle_slowdown_distance = params['obstacle_slowdown_distance']
        self.robot_half_width = self._state.detection.get_robot_half_width()
        self.search_angular_speed = params['search_angular_speed']
        self.lost_timeout = params['lost_timeout']
        self.search_timeout = params['search_timeout']
        self.blind_approach_entry_distance = params.get(
            'blind_approach_entry_distance', self.obstacle_slowdown_distance)
        self.blind_approach_speed = params.get(
            'blind_approach_speed', self.min_linear_speed)
        self.blind_approach_timeout = params.get('blind_approach_timeout', 10.0)
        self.blind_approach_lidar_stop = params.get(
            'blind_approach_lidar_stop', self.lidar_min_range)
        self.blind_approach_arrival_margin = params.get(
            'blind_approach_arrival_margin', self.approach_distance)
        self.spiral_search_enabled = params['spiral_search_enabled']
        self.spiral_initial_radius = params['spiral_initial_radius']
        self.spiral_max_radius = params['spiral_max_radius']
        self.spiral_growth_rate = params['spiral_growth_rate']
        self.spiral_linear_speed = params['spiral_linear_speed']
        self.spiral_angular_speed = params['spiral_angular_speed']
        self.spiral_timeout = params['spiral_timeout']


    def _base_to_laser_angle(self, base_angle):
        """Convert a base_link-frame angle to the laser frame for raw scan lookup.

        Subtracts the LiDAR mounting yaw so the angle can be used as an index
        into the raw LaserScan data, which is in the laser frame.

        Args:
            base_angle: Angle in base_link frame [rad].

        Returns:
            Corresponding angle in laser frame [rad].
        """
        lidar_yaw = math.radians(self._state.detection.get_lidar_yaw())
        return base_angle - lidar_yaw

    def _speed_for_distance(self, distance):
        """Linear speed ramp: full speed when far, creep speed when close.

        Interpolates linearly between min_linear_speed (at approach_distance)
        and linear_speed (at obstacle_slowdown_distance). Below approach_distance,
        returns min_speed * APPROACH_CREEP_FACTOR.

        Args:
            distance: Distance to target in meters.

        Returns:
            Speed in m/s.
        """
        if distance >= self.obstacle_slowdown_distance:
            return self.linear_speed
        if distance <= self.approach_distance:
            return self.min_linear_speed * APPROACH_CREEP_FACTOR
        # Linear ramp between min and max
        ratio = (distance - self.approach_distance) / max(
            self.obstacle_slowdown_distance - self.approach_distance, 0.01)
        return self.min_linear_speed + ratio * (self.linear_speed - self.min_linear_speed)

    def _publish_cmd(self, cmd):
        """Sanitize and publish a Twist command.

        All velocity commands must go through this method. It replaces
        NaN/Inf with zero, enforces speed caps, publishes to /cmd_vel,
        and updates the shared state with the current command.

        Args:
            cmd: geometry_msgs.msg.Twist to publish.
        """
        if not _clamp_twist(cmd):
            self._logger.warning("NaN/Inf in Twist command - clamped to zero")
        _cap_speed(cmd, self.linear_speed, self.angular_speed)
        self._cmd_vel_pub.publish(cmd)
        self._state.nav.set_current_cmd_vel(cmd.linear.x, cmd.linear.y, cmd.angular.z)
        self._last_cmd_time = time.time()
        self._watchdog_fired = False  # Reset so watchdog can fire again if needed

    def check_watchdog(self):
        """Stop motors if no command was published for 0.5s (safety watchdog).

        Called from the control loop every tick. If the navigation logic
        crashed or hung, this ensures motors don't keep running with
        the last velocity forever.

        Publishes zero Twist ONCE on timeout, then sets _watchdog_fired to
        prevent continuous publishing. This avoids interfering with firmware
        position commands (FWD, TMOTOR, CALIB) sent via /arduino/cmd — those
        set _vel_mode=False in arduino_node, but a continuous zero Twist
        from this watchdog would flip _vel_mode back to True.

        Reset when a new velocity command is published via _publish_cmd().

        Returns:
            True if watchdog triggered (motors stopped), False if OK.
        """
        if time.time() - self._last_cmd_time > 0.5:
            if not self._watchdog_fired:
                cmd = Twist()
                self._cmd_vel_pub.publish(cmd)
                self._state.nav.set_current_cmd_vel(0.0, 0.0, 0.0)
                self._watchdog_fired = True
            return True
        return False

    def _update_map_goal(self, vision_distance, cam_angle, camera_yaw_rad):
        """Compute and store the target position in odom frame.

        Primary: TF2 transform from camera_link to map/odom.
        Fallback: manual computation using odometry + camera mounting offsets.

        Args:
            vision_distance: Raw vision distance in meters.
            cam_angle: Angle in camera frame (radians).
            camera_yaw_rad: Camera mounting yaw in radians.

        Returns:
            (map_x, map_y) tuple, or None if computation failed.
        """
        try:
            pt = PointStamped()
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.header.frame_id = 'camera_link'
            pt.point.x = vision_distance * math.cos(cam_angle)
            pt.point.y = vision_distance * math.sin(cam_angle)
            pt.point.z = 0.0
            from tf2_geometry_msgs import do_transform_point
            tf = self._tf_buffer.lookup_transform('odom', 'camera_link',
                                                   rclpy.time.Time())
            map_pt = do_transform_point(pt, tf)
            self._state.nav.set_nav_target_map(map_pt.point.x, map_pt.point.y)
            return (map_pt.point.x, map_pt.point.y)
        except Exception:
            pass

        # Fallback: manual computation if TF not available
        try:
            robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()
            cam_pos = self._state.detection.get_camera_pos()
            det_x = vision_distance * math.cos(cam_angle)
            det_y = vision_distance * math.sin(cam_angle)
            cos_cy = math.cos(camera_yaw_rad)
            sin_cy = math.sin(camera_yaw_rad)
            body_x = cam_pos['x'] + det_x * cos_cy - det_y * sin_cy
            body_y = cam_pos['y'] + det_x * sin_cy + det_y * cos_cy
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            map_x = robot_x + body_x * cos_t - body_y * sin_t
            map_y = robot_y + body_x * sin_t + body_y * cos_t
            self._state.nav.set_nav_target_map(map_x, map_y)
            return (map_x, map_y)
        except Exception:
            return None

    def _map_goal_to_base_link(self):
        """Convert stored odom-frame goal to base_link coordinates.

        Uses current odometry pose to compute the displacement from the
        robot to the stored goal, rotated into the robot's body frame.

        Returns:
            (target_x_bl, target_y_bl, distance) in base_link frame,
            or None if no map goal or pose is unavailable.
        """
        map_goal = self._state.nav.get_nav_target_map()
        if map_goal is None:
            return None
        robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()
        dx = map_goal[0] - robot_x
        dy = map_goal[1] - robot_y
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)
        target_x = dx * cos_t + dy * sin_t
        target_y = -dx * sin_t + dy * cos_t
        distance = math.sqrt(dx * dx + dy * dy)
        return target_x, target_y, distance

    def navigate_to_target(self, target):
        """Navigate toward a detected target using odom-goal approach.

        1. Camera detection → compute odom-frame goal via TF2.
        2. Convert stored goal → base_link using odometry.
        3. Navigate toward the stable goal (not the jittery camera pixel).
        4. Camera re-detections continuously refine the goal.

        Args:
            target: TrackedTarget or Det object with distance, angle, and
                bbox_clipped fields. TrackedTarget provides pre-fused
                distance and base_link angle from the Kalman tracker.
                Raw Det objects are converted inline (legacy path).
        """
        # TrackedTarget: angle is already in base_link frame, distance is
        # already LiDAR-fused by the Kalman tracker. Skip conversion.
        from target_nav.app.target_tracker import TrackedTarget
        camera_yaw_rad = math.radians(self._state.detection.get_camera_yaw())

        if isinstance(target, TrackedTarget):
            sensor_distance = target.distance
            angle_bl = target.angle
            cam_angle = angle_bl - camera_yaw_rad  # reverse: bl to camera frame
            bbox_clipped = target.bbox_clipped
            source = "tracker"
        else:
            # Legacy path: raw Det object from grace period
            cam_angle = -target.angle  # Camera convention (+=right) to ROS
            angle_bl = cam_angle + camera_yaw_rad
            bbox_clipped = target.bbox_clipped

            cam_offset = self._state.detection.get_sensor_offset('camera')
            lidar_offset = self._state.detection.get_sensor_offset('lidar')
            vision_distance_ref = target.distance + cam_offset

            if bbox_clipped:
                lidar_distance = self._state.sensors.get_lidar_distance_at_angle(
                    self._base_to_laser_angle(angle_bl))
                if lidar_distance < float('inf'):
                    sensor_distance = lidar_distance + lidar_offset
                    source = "lidar"
                else:
                    sensor_distance = vision_distance_ref
                    source = "vision"
            else:
                sensor_distance = vision_distance_ref
                source = "vision"

        self.last_target_angle = angle_bl

        if not math.isfinite(sensor_distance) or sensor_distance <= 0:
            return

        # Step 1: Update the odom-frame goal from this camera detection
        self._update_map_goal(sensor_distance, cam_angle, camera_yaw_rad)

        # Step 2: Convert stored odom goal → base_link for navigation
        # This is the key improvement: we navigate to the stable odom goal,
        # not the raw camera detection which jitters frame-to-frame.
        bl = self._map_goal_to_base_link()
        if bl is not None:
            target_x, target_y, distance = bl
            angle = math.atan2(target_y, target_x)
        else:
            # Fallback: use sensor-based navigation if odom goal unavailable
            distance = sensor_distance
            angle = angle_bl
            target_x = distance * math.cos(angle)
            target_y = distance * math.sin(angle)

        if not math.isfinite(distance) or distance <= 0:
            return

        prev_state, _ = self._state.nav.get_nav_state()
        self._state.nav.set_nav_state("NAVIGATING", (target_x, target_y, distance))

        if prev_state != "NAVIGATING" or time.time() - self._last_nav_log > LOG_INTERVAL_SECS:
            clip_tag = " [CLIPPED]" if bbox_clipped else ""
            self._logger.info(
                f"Target at {distance:.2f}m (map-goal, src={source}), "
                f"{math.degrees(angle):.1f}deg{clip_tag}")
            self._last_nav_log = time.time()

        # --- Arrival check (multi-signal with temporal confirmation) ---
        if self._check_arrival(distance, angle, bbox_clipped):
            self._arrive()
            return

        # --- Close approach: within obstacle avoidance zone ---
        if distance < self.obstacle_slowdown_distance:
            if distance < self.blind_approach_entry_distance:
                if self.enter_blind_approach():
                    self.blind_approach_step()
                    return

            cmd = Twist()
            speed = self._speed_for_distance(distance)
            cmd.linear.x = speed * math.cos(angle)
            cmd.linear.y = speed * math.sin(angle)
            cmd.angular.z = max(-STEERING_ANGULAR_CLAMP, min(STEERING_ANGULAR_CLAMP, angle * STEERING_ANGULAR_GAIN))
            self._publish_cmd(cmd)
            return

        # --- Bbox clipped + vision-only: unreliable distance ---
        if bbox_clipped and source == "vision" and sensor_distance < self.blind_approach_entry_distance:
            self._logger.info(
                f"Bbox clipped + vision-only at {sensor_distance:.2f}m -> early blind approach")
            if self.enter_blind_approach():
                self.blind_approach_step()
                return

        # --- During arrival confirmation: slow creep ---
        if self._arrival_first_seen != 0.0:
            cmd = Twist()
            creep_speed = self.min_linear_speed * APPROACH_CREEP_FACTOR
            cmd.linear.x = creep_speed * math.cos(angle)
            cmd.linear.y = creep_speed * math.sin(angle)
            cmd.angular.z = max(-ARRIVAL_ANGULAR_CLAMP, min(ARRIVAL_ANGULAR_CLAMP, angle * ARRIVAL_ANGULAR_GAIN))
            self._publish_cmd(cmd)
            return

        self.direct_navigate(target_x, target_y, distance, target_visible=True)

    def stop_robot(self):
        """Stop all robot motion and reset navigation state.

        Publishes a zero-velocity Twist and resets all internal state:
        stuck timer, arrival timer, avoidance direction, and search/spiral flags.
        """
        cmd = Twist()
        self._publish_cmd(cmd)
        self._stuck_start = 0.0
        self._arrival_first_seen = 0.0
        self._avoid_direction = 0
        self._avoid_clear_count = 0
        self._search_initialized = False
        self._spiral_active = False


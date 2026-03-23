"""Unit tests for Navigator class -- fully mocked, no ROS2 or hardware required.

Tests cover:
- _speed_for_distance: speed ramp algorithm
- check_obstacles: LiDAR front/side corridor analysis
- _find_best_gap: VFH polar histogram gap finding
- _check_arrival: multi-signal arrival scoring with temporal confirmation
- navigate_to_target: main navigation pipeline
- _compute_repulsion: potential field repulsive vector
"""

import math
import time
import sys
from unittest.mock import MagicMock, patch

from target_nav.config import (
    DEFAULT_NAV_PARAMS, DEFAULT_LIDAR_YAW, DEFAULT_CAMERA_YAW, DEFAULT_CAMERA,
)

import pytest

# ---------------------------------------------------------------------------
# Block ROS2 imports so tests run without a ROS2 installation.
# Navigator imports rclpy and geometry_msgs at module level.
# ---------------------------------------------------------------------------
_mock_twist_instance = None


class _FakeTwist:
    """Minimal stand-in for geometry_msgs.msg.Twist."""
    def __init__(self):
        self.linear = _FakeVec()
        self.angular = _FakeVec()


class _FakeVec:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakePointStamped:
    def __init__(self):
        self.header = MagicMock()
        self.point = _FakeVec()


# Patch modules before importing Navigator
_geom_mod = MagicMock()
_geom_mod.Twist = _FakeTwist
_geom_mod.PointStamped = _FakePointStamped

_rclpy_mod = MagicMock()
_rclpy_time = MagicMock()
_rclpy_time.Time.return_value.to_msg.return_value = MagicMock()
_rclpy_mod.time = _rclpy_time

sys.modules.setdefault('rclpy', _rclpy_mod)
sys.modules.setdefault('rclpy.time', _rclpy_time)
sys.modules.setdefault('geometry_msgs', MagicMock())
sys.modules.setdefault('geometry_msgs.msg', _geom_mod)
sys.modules.setdefault('tf2_geometry_msgs', MagicMock())

from target_nav.detectors.detection import Det
from target_nav.nav.navigator import (
    Navigator,
    APPROACH_CREEP_FACTOR,
    LIDAR_STALE_TIMEOUT_SECS,
    ARRIVAL_CONFIRM_SECS,
    ARRIVAL_HYSTERESIS_FACTOR,
    LIDAR_MIN_POINT_DIST,
    VFH_CLEARANCE_MARGIN,
    VFH_BODY_MARGIN,
    VFH_NUM_BINS,
    FRONT_CORRIDOR_MARGIN,
    SIDE_CHECK_DEPTH,
    BBOX_CLIPPED_SCORE,
    BBOX_BLIND_SCORE,
    MAP_CLOSE_SCORE,
    MAP_VERY_CLOSE_SCORE,
    SENSOR_CLOSE_SCORE,
    MAP_ARRIVAL_FACTOR,
    BBOX_CLIPPED_DIST_FACTOR,
)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def _make_navigator(laser_points=None, robot_pose=(0, 0, 0),
                    laser_time=None, lidar_dist_at_angle=float('inf'),
                    nav_target_map=None):
    """Build a Navigator with fully mocked dependencies."""
    state = MagicMock()

    # Detection store
    state.detection.get_nav_params.return_value = dict(DEFAULT_NAV_PARAMS)
    state.detection.get_robot_half_width.return_value = 0.13
    state.detection.get_lidar_yaw.return_value = DEFAULT_LIDAR_YAW
    state.detection.get_camera_yaw.return_value = DEFAULT_CAMERA_YAW
    state.detection.get_camera_pos.return_value = dict(DEFAULT_CAMERA)
    state.detection.get_sensor_offset.return_value = 0.005
    state.detection.get_detect_expiry.return_value = 1.5
    state.detection.get_camera.return_value = (None, [], "", 999.0)

    # Sensor store
    if laser_points is None:
        laser_points = []
    lt = laser_time if laser_time is not None else time.time()
    state.sensors.get_laser.return_value = (laser_points, 0, lt)
    state.sensors.get_robot_pose.return_value = robot_pose
    state.sensors.get_lidar_distance_at_angle.return_value = lidar_dist_at_angle

    # Nav store
    state.nav.get_nav_state.return_value = ("IDLE", None)
    state.nav.get_nav_target_map.return_value = nav_target_map

    cmd_vel_pub = MagicMock()
    tf_buffer = MagicMock()
    # Make TF lookup raise so fallback path is used
    tf_buffer.lookup_transform.side_effect = Exception("no TF")
    logger = MagicMock()

    # Patch LIDAR_MIN_RANGE import inside Navigator.__init__
    with patch('target_nav.state.detection_store.LIDAR_MIN_RANGE', 0.15):
        nav = Navigator(state, cmd_vel_pub, tf_buffer, None, logger)

    return nav, cmd_vel_pub, state


def _make_target(**kwargs):
    """Create a Det with sensible defaults, overridable by kwargs."""
    defaults = dict(
        class_name='bowling-pin', confidence=0.9,
        bbox=(280, 100, 360, 400),
        distance=1.5, angle=0.1, bbox_clipped=False,
    )
    defaults.update(kwargs)
    return Det(**defaults)


# ===================================================================
# TestSpeedForDistance
# ===================================================================

class TestSpeedForDistance:
    """Tests for Navigator._speed_for_distance (pure math speed ramp)."""

    def test_full_speed_when_far(self):
        nav, _, _ = _make_navigator()
        # Beyond slowdown zone -> full speed
        speed = nav._speed_for_distance(5.0)
        assert speed == pytest.approx(nav.linear_speed)

    def test_full_speed_at_slowdown_boundary(self):
        nav, _, _ = _make_navigator()
        speed = nav._speed_for_distance(nav.obstacle_slowdown_distance)
        assert speed == pytest.approx(nav.linear_speed)

    def test_min_speed_at_approach_distance(self):
        nav, _, _ = _make_navigator()
        speed = nav._speed_for_distance(nav.approach_distance)
        assert speed == pytest.approx(nav.min_linear_speed * APPROACH_CREEP_FACTOR)

    def test_min_speed_below_approach(self):
        nav, _, _ = _make_navigator()
        speed = nav._speed_for_distance(0.0)
        assert speed == pytest.approx(nav.min_linear_speed * APPROACH_CREEP_FACTOR)

    def test_min_speed_negative_distance(self):
        nav, _, _ = _make_navigator()
        speed = nav._speed_for_distance(-1.0)
        assert speed == pytest.approx(nav.min_linear_speed * APPROACH_CREEP_FACTOR)

    def test_linear_ramp_midpoint(self):
        nav, _, _ = _make_navigator()
        mid = (nav.approach_distance + nav.obstacle_slowdown_distance) / 2.0
        speed = nav._speed_for_distance(mid)
        expected_min = nav.min_linear_speed * APPROACH_CREEP_FACTOR
        expected_max = nav.linear_speed
        # Midpoint should be between min and max
        assert expected_min < speed < expected_max

    def test_linear_ramp_is_monotonic(self):
        nav, _, _ = _make_navigator()
        distances = [nav.approach_distance + i * 0.01
                     for i in range(int((nav.obstacle_slowdown_distance - nav.approach_distance) / 0.01) + 1)]
        speeds = [nav._speed_for_distance(d) for d in distances]
        for i in range(1, len(speeds)):
            assert speeds[i] >= speeds[i - 1] - 1e-9


# ===================================================================
# TestCheckObstacles
# ===================================================================

class TestCheckObstacles:
    """Tests for Navigator.check_obstacles (LiDAR corridor analysis)."""

    def test_clear_path_no_points_nearby(self):
        # Points far away in front corridor
        points = [(3.0, 0.0), (2.5, 0.05)]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == float('inf')
        assert left_free is True
        assert right_free is True

    def test_front_obstacle(self):
        # Point directly ahead within slowdown distance and within corridor width
        points = [(0.15, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == pytest.approx(0.15)
        assert left_free is True
        assert right_free is True

    def test_left_obstacle(self):
        hw = 0.13  # robot_half_width
        # Point to the left side: x in (-hw, slowdown), y in [hw, hw+SIDE_CHECK_DEPTH)
        points = [(0.1, hw + 0.1)]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == float('inf')
        assert left_free is False
        assert right_free is True

    def test_right_obstacle(self):
        hw = 0.13
        points = [(0.1, -(hw + 0.1))]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == float('inf')
        assert left_free is True
        assert right_free is False

    def test_stale_laser_returns_fail_safe(self):
        old_time = time.time() - LIDAR_STALE_TIMEOUT_SECS - 1.0
        points = [(0.5, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points, laser_time=old_time)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == 0.0
        assert left_free is False
        assert right_free is False

    def test_empty_points_returns_fail_safe(self):
        nav, _, _ = _make_navigator(laser_points=[])
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == 0.0
        assert left_free is False
        assert right_free is False

    def test_nan_points_ignored(self):
        points = [(float('nan'), 0.0), (float('nan'), float('nan'))]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        # NaN points ignored, so effectively empty -- but laser_time is fresh,
        # so no fail-safe; the corridor just has no valid points.
        assert min_front == float('inf')
        assert left_free is True
        assert right_free is True

    def test_point_behind_robot_ignored(self):
        # x <= 0 should not count as front obstacle
        points = [(-0.3, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, left_free, right_free = nav.check_obstacles()
        assert min_front == float('inf')

    def test_multiple_front_points_returns_closest(self):
        points = [(0.35, 0.0), (0.20, 0.0), (0.30, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        min_front, _, _ = nav.check_obstacles()
        assert min_front == pytest.approx(0.20)


# ===================================================================
# TestFindBestGap
# ===================================================================

class TestFindBestGap:
    """Tests for Navigator._find_best_gap (VFH polar histogram)."""

    def test_no_obstacles_returns_target_angle(self):
        # All bins are clear -> best gap should be close to target
        # Provide distant points so histogram has data but nothing close
        points = [(5.0, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        gap = nav._find_best_gap(0.0)
        # Gap should be near 0 (straight ahead)
        assert gap is not None
        assert abs(gap) < math.radians(10)

    def test_obstacle_blocking_target_returns_side_gap(self):
        # Place a dense wall of obstacles straight ahead
        obs_dist = 0.15
        points = [(obs_dist, y * 0.01) for y in range(-5, 6)]
        # Add clear points far to the right
        points += [(3.0, -1.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        gap = nav._find_best_gap(0.0)
        # Should find a gap to the side (not straight ahead)
        assert gap is not None
        assert abs(gap) > math.radians(5)

    def test_obstacle_on_left_returns_right_gap(self):
        # Block left half, leave right clear
        points = []
        for i in range(1, 20):
            ang = math.radians(i * 5)  # 5 to 95 degrees (left side)
            d = 0.15
            points.append((d * math.cos(ang), d * math.sin(ang)))
        # Add a far point on the right so histogram has data
        points.append((3.0, -0.5))
        nav, _, _ = _make_navigator(laser_points=points)
        gap = nav._find_best_gap(0.0)
        assert gap is not None
        # Gap should be on the right (negative angle)
        assert gap < 0

    def test_no_passable_gap_returns_none(self):
        # Dense wall of obstacles across entire front hemisphere
        points = []
        for i in range(-90, 91, 2):
            ang = math.radians(i)
            d = 0.10  # Very close, within obstacle + clearance distance
            points.append((d * math.cos(ang), d * math.sin(ang)))
        nav, _, _ = _make_navigator(laser_points=points)
        gap = nav._find_best_gap(0.0)
        assert gap is None

    def test_narrow_corridor_too_small_returns_none(self):
        # Create a tiny gap in the middle of a wall (smaller than robot body)
        points = []
        clear_dist = 0.20 + VFH_CLEARANCE_MARGIN  # obstacle_distance + margin
        body_angle = 2.0 * math.atan2(0.13 + VFH_BODY_MARGIN, clear_dist)
        body_bins = max(1, int(math.ceil(body_angle / (math.pi / VFH_NUM_BINS))))

        # Fill all bins with close obstacles
        for i in range(-90, 91):
            ang = math.radians(i)
            d = 0.10
            points.append((d * math.cos(ang), d * math.sin(ang)))
        nav, _, _ = _make_navigator(laser_points=points)

        # Even if we remove ONE bin worth of obstacles, gap is too narrow
        gap = nav._find_best_gap(0.0)
        assert gap is None

    def test_empty_points_returns_none(self):
        nav, _, _ = _make_navigator(laser_points=[])
        gap = nav._find_best_gap(0.0)
        assert gap is None

    def test_target_angle_preference(self):
        # Two gaps available, one near target, one far -- should pick nearer
        points = []
        # Wall from -30 to +30 degrees, gaps at ~+60 and ~-60
        for i in range(-30, 31):
            ang = math.radians(i)
            d = 0.10
            points.append((d * math.cos(ang), d * math.sin(ang)))
        nav, _, _ = _make_navigator(laser_points=points)

        # Target at +60 degrees -> should prefer the +60 gap
        target = math.radians(60)
        gap = nav._find_best_gap(target)
        assert gap is not None
        assert gap > 0  # Should be on the left (positive) side


# ===================================================================
# TestCheckArrival
# ===================================================================

class TestCheckArrival:
    """Tests for Navigator._check_arrival (multi-signal arrival scoring)."""

    def test_far_away_returns_false(self):
        nav, _, _ = _make_navigator()
        result = nav._check_arrival(distance=5.0, angle=0.0, bbox_clipped=False)
        assert result is False

    def test_sensor_close_starts_timer_then_confirms(self):
        # Sensor distance within threshold: score = SENSOR_CLOSE_SCORE = 1.0
        nav, _, state = _make_navigator(
            laser_points=[(3.0, 0.0)],  # far away so check_obstacles doesn't add score
            lidar_dist_at_angle=float('inf'),
        )
        # First call: starts timer, returns False
        result1 = nav._check_arrival(distance=0.05, angle=0.0, bbox_clipped=False)
        assert result1 is False
        assert nav._arrival_first_seen != 0.0

        # After confirmation period: returns True
        nav._arrival_first_seen = time.time() - ARRIVAL_CONFIRM_SECS - 0.01
        result2 = nav._check_arrival(distance=0.05, angle=0.0, bbox_clipped=False)
        assert result2 is True

    def test_map_close_returns_true_after_confirmation(self):
        # Map distance within 2x threshold: score = MAP_CLOSE_SCORE = 1.0
        nav, _, state = _make_navigator(
            laser_points=[(3.0, 0.0)],
            robot_pose=(1.0, 1.0, 0.0),
            nav_target_map=(1.05, 1.05),
            lidar_dist_at_angle=float('inf'),
        )
        # Map dist ~= 0.07, threshold = 0.10, 2x threshold = 0.20
        # score: MAP_CLOSE_SCORE(1.0) + MAP_VERY_CLOSE_SCORE(0.5) = 1.5
        result1 = nav._check_arrival(distance=5.0, angle=0.0, bbox_clipped=False)
        assert result1 is False
        assert nav._arrival_first_seen != 0.0

        nav._arrival_first_seen = time.time() - ARRIVAL_CONFIRM_SECS - 0.01
        result2 = nav._check_arrival(distance=5.0, angle=0.0, bbox_clipped=False)
        assert result2 is True

    def test_bbox_clipped_plus_sensor_blind_gives_high_score(self):
        nav, _, _ = _make_navigator(
            laser_points=[(3.0, 0.0)],
            lidar_dist_at_angle=float('inf'),
        )
        # bbox_clipped=True, laser_at_target=inf -> BBOX_BLIND_SCORE=0.7
        # distance < threshold * BBOX_CLIPPED_DIST_FACTOR -> BBOX_CLIPPED_SCORE=0.3
        # Total = 1.0 (just enough)
        threshold = nav.approach_distance  # 0.10
        dist = threshold * BBOX_CLIPPED_DIST_FACTOR - 0.01  # 0.29
        result = nav._check_arrival(distance=dist, angle=0.0, bbox_clipped=True)
        assert result is False  # First call starts timer
        assert nav._arrival_first_seen != 0.0

    def test_score_resets_when_distance_increases(self):
        nav, _, _ = _make_navigator(
            laser_points=[(3.0, 0.0)],
            lidar_dist_at_angle=float('inf'),
        )
        # Start arrival timer with sensor close
        nav._check_arrival(distance=0.05, angle=0.0, bbox_clipped=False)
        assert nav._arrival_first_seen != 0.0

        # Distance increases beyond hysteresis threshold
        # best_dist > threshold * ARRIVAL_HYSTERESIS_FACTOR -> timer resets
        hysteresis_dist = nav.approach_distance * ARRIVAL_HYSTERESIS_FACTOR + 0.5
        nav._check_arrival(distance=hysteresis_dist, angle=0.0, bbox_clipped=False)
        assert nav._arrival_first_seen == 0.0

    def test_insufficient_score_does_not_start_timer(self):
        nav, _, _ = _make_navigator(
            laser_points=[(3.0, 0.0)],
            lidar_dist_at_angle=float('inf'),
        )
        # Distance slightly above threshold: no signals fire
        dist = nav.approach_distance + 0.5
        nav._check_arrival(distance=dist, angle=0.0, bbox_clipped=False)
        assert nav._arrival_first_seen == 0.0


# ===================================================================
# TestNavigateToTarget
# ===================================================================

class TestNavigateToTarget:
    """Tests for Navigator.navigate_to_target (main navigation pipeline)."""

    def test_publishes_cmd_vel_toward_target(self):
        # Target ahead, no obstacles -> should publish forward velocity
        nav, cmd_vel_pub, state = _make_navigator(
            laser_points=[(3.0, 0.0)],
        )
        target = _make_target(distance=1.5, angle=0.0, bbox_clipped=False)
        nav.navigate_to_target(target)
        assert cmd_vel_pub.publish.called
        twist = cmd_vel_pub.publish.call_args[0][0]
        assert twist.linear.x > 0  # Moving forward

    def test_close_target_drives_straight_not_avoiding(self):
        # Target within slowdown zone: should drive toward it, not VFH avoid
        nav, cmd_vel_pub, state = _make_navigator(
            laser_points=[(0.25, 0.0)],  # obstacle near = target itself
        )
        target = _make_target(distance=0.30, angle=0.0, bbox_clipped=False)
        nav.navigate_to_target(target)
        assert cmd_vel_pub.publish.called
        twist = cmd_vel_pub.publish.call_args[0][0]
        # Should be moving forward (toward the target)
        assert twist.linear.x > 0

    def test_invalid_distance_skipped(self):
        # After sensor offset (+0.005), distance must still be <= 0 to be rejected
        nav, cmd_vel_pub, _ = _make_navigator()
        target = _make_target(distance=-0.01, angle=0.0)
        nav.navigate_to_target(target)
        # Negative distance after offset (-0.01 + 0.005 = -0.005) is rejected
        assert not cmd_vel_pub.publish.called

    def test_negative_distance_skipped(self):
        nav, cmd_vel_pub, _ = _make_navigator()
        target = _make_target(distance=-1.0, angle=0.0)
        nav.navigate_to_target(target)
        assert not cmd_vel_pub.publish.called

    def test_nan_distance_skipped(self):
        nav, cmd_vel_pub, _ = _make_navigator()
        target = _make_target(distance=float('nan'), angle=0.0)
        nav.navigate_to_target(target)
        assert not cmd_vel_pub.publish.called

    def test_bbox_clipped_uses_laser_distance(self):
        # Bbox clipped -> should query laser distance at target angle
        laser_d = 0.35
        nav, cmd_vel_pub, state = _make_navigator(
            laser_points=[(3.0, 0.0)],
            lidar_dist_at_angle=laser_d,
        )
        target = _make_target(distance=2.0, angle=0.1, bbox_clipped=True)
        nav.navigate_to_target(target)
        # Should have queried laser scan distance
        state.sensors.get_lidar_distance_at_angle.assert_called()

    def test_angle_convention_inversion(self):
        # Camera angle is inverted (cam +=right -> ROS +=left)
        nav, cmd_vel_pub, state = _make_navigator(
            laser_points=[(3.0, 0.0)],
        )
        target = _make_target(distance=1.5, angle=0.3, bbox_clipped=False)
        nav.navigate_to_target(target)
        # After inversion, angle should be -0.3 in base_link
        # So last_target_angle should be negative
        assert nav.last_target_angle == pytest.approx(-0.3)


# ===================================================================
# TestComputeRepulsion
# ===================================================================

class TestComputeRepulsion:
    """Tests for Navigator._compute_repulsion (potential field)."""

    def test_no_points_returns_zero(self):
        nav, _, _ = _make_navigator(laser_points=[])
        rx, ry = nav._compute_repulsion(1.0)
        assert rx == 0.0
        assert ry == 0.0

    def test_obstacle_ahead_repels_backward(self):
        # Single obstacle directly ahead
        points = [(0.3, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        # Repulsion should push backward (negative x)
        assert rx < 0
        assert abs(ry) < abs(rx)  # Mostly in x direction

    def test_obstacle_on_left_repels_right(self):
        points = [(0.0, 0.3)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        # Repulsion should push right (negative y)
        assert ry < 0

    def test_obstacle_beyond_influence_ignored(self):
        points = [(2.0, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        assert rx == 0.0
        assert ry == 0.0

    def test_very_close_point_ignored(self):
        # Points closer than LIDAR_MIN_POINT_DIST are skipped
        points = [(0.01, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        assert rx == 0.0
        assert ry == 0.0

    def test_symmetric_obstacles_cancel_out(self):
        # Two obstacles symmetrically placed -> repulsion should cancel in y
        points = [(0.3, 0.2), (0.3, -0.2)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        assert ry == pytest.approx(0.0, abs=1e-9)
        assert rx < 0  # Still repels backward

    def test_nan_points_ignored(self):
        points = [(float('nan'), 0.0), (0.3, 0.0)]
        nav, _, _ = _make_navigator(laser_points=points)
        rx, ry = nav._compute_repulsion(1.0)
        # Only the valid point contributes
        assert rx < 0

    def test_closer_obstacle_produces_stronger_repulsion(self):
        nav1, _, _ = _make_navigator(laser_points=[(0.2, 0.0)])
        rx1, _ = nav1._compute_repulsion(1.0)

        nav2, _, _ = _make_navigator(laser_points=[(0.5, 0.0)])
        rx2, _ = nav2._compute_repulsion(1.0)

        # Closer obstacle -> stronger (more negative) repulsion
        assert abs(rx1) > abs(rx2)

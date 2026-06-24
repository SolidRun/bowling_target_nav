"""Navigation and search settings tabs for the SettingsWindow.

Provides two main tab builders:

    _build_nav_page: Navigate tab with sub-tabs Speed, Target, Approach.
    _build_search_page: Search tab with sub-tabs Scan, Spiral.

All parameter changes are immediately sent to the running Navigator via
``send_ros_command({'type': 'set_nav_param', ...})`` and persisted to
SettingsStore via auto-save.

Runs in: GUI process (Process 0) on the GTK main thread.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from target_nav.config import DEFAULT_NAV_PARAMS
from target_nav.utils.log import get_logger

logger = get_logger('gui.settings.nav')


class NavTabMixin:
    """Navigation and search settings tabs with sub-tab layout."""

    # ---- Navigator param helpers ----

    def _nav_callback(self, attr):
        """Return a callback that updates a navigator parameter."""
        def cb(v):
            self._state.send_ros_command({
                'type': 'set_nav_param', 'attr': attr, 'value': v})
            self._state.detection.set_nav_param(attr, v)
            logger.info("nav.%s = %s", attr, v)
            self._schedule_save()
        return cb

    def _add_nav_slider(self, box, label, min_v, max_v, step, default, attr):
        """Create a slider row bound to a navigator parameter."""
        stored = self._state.detection.get_nav_param(attr)
        initial = stored if stored is not None else default
        box.pack_start(self._make_slider_row(
            f"nav.{attr}", label, min_v, max_v, step, initial,
            self._nav_callback(attr)), False, False, 0)

    def _reset_nav_params(self, keys):
        """Reset navigator parameters to DEFAULT_NAV_PARAMS and sync widgets."""
        for k in keys:
            v = DEFAULT_NAV_PARAMS[k]
            self._state.detection.set_nav_param(k, v)
            self._state.send_ros_command({
                'type': 'set_nav_param', 'attr': k, 'value': v})
            self._set_widget_value(f'nav.{k}', v)
        self._schedule_save()

    # ---- Navigate tab (3 sub-tabs) ----

    def _build_nav_page(self):
        """Build the Navigate tab with sub-tabs: Speed, Target, Approach.

        Speed sub-tab:
            linear_speed        -- Forward cruising speed (m/s).
            angular_speed       -- In-place rotation speed (rad/s).
            min_linear_speed    -- Minimum forward speed when slowing down (m/s).

        Target sub-tab:
            approach_distance   -- Distance at which robot declares "arrived" (m).
            obstacle_distance   -- Minimum obstacle clearance before hard stop (m).
            obstacle_slowdown_distance -- Distance at which robot starts to slow (m).
            lost_timeout        -- Seconds without detection before entering search (s).

        Approach sub-tab (blind approach = dead-reckoning final approach):
            blind_approach_entry_distance -- Camera distance to switch from
                visual tracking to blind dead-reckoning (m).
            blind_approach_speed    -- Forward speed during blind approach (m/s).
            blind_approach_timeout  -- Max time before giving up (s).
            blind_approach_lidar_stop -- LiDAR reading that triggers stop (m).
            blind_approach_arrival_margin -- Odometry distance that counts
                as "arrived" during blind approach (m).
        """

        # -- Speed sub-tab --
        speed_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        speed_box.set_margin_top(4)
        self._add_nav_slider(speed_box, "Forward Speed (m/s)", 0.05, 0.40, 0.01,
                             DEFAULT_NAV_PARAMS['linear_speed'], 'linear_speed')
        self._add_nav_slider(speed_box, "Rotation Speed (rad/s)", 0.1, 1.0, 0.05,
                             DEFAULT_NAV_PARAMS['angular_speed'], 'angular_speed')
        self._add_nav_slider(speed_box, "Min Speed (m/s)", 0.03, 0.20, 0.01,
                             DEFAULT_NAV_PARAMS['min_linear_speed'], 'min_linear_speed')
        speed_box.pack_start(self._make_reset_row(
            lambda: self._reset_nav_params(
                ['linear_speed', 'angular_speed', 'min_linear_speed'])),
            False, False, 0)

        # -- Target sub-tab --
        target_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        target_box.set_margin_top(4)
        self._add_nav_slider(target_box, "Stop Distance (m)", 0.03, 0.50, 0.01,
                             DEFAULT_NAV_PARAMS['approach_distance'], 'approach_distance')
        self._add_nav_slider(target_box, "Obstacle Distance (m)", 0.10, 0.50, 0.01,
                             DEFAULT_NAV_PARAMS['obstacle_distance'], 'obstacle_distance')
        self._add_nav_slider(target_box, "Slowdown Zone (m)", 0.20, 1.00, 0.05,
                             DEFAULT_NAV_PARAMS['obstacle_slowdown_distance'],
                             'obstacle_slowdown_distance')
        self._add_nav_slider(target_box, "Target Lost Timeout (s)", 1.0, 15.0, 0.5,
                             DEFAULT_NAV_PARAMS['lost_timeout'], 'lost_timeout')
        target_box.pack_start(self._make_reset_row(
            lambda: self._reset_nav_params(
                ['approach_distance', 'obstacle_distance',
                 'obstacle_slowdown_distance', 'lost_timeout'])),
            False, False, 0)

        # -- Approach sub-tab --
        approach_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        approach_box.set_margin_top(4)
        self._add_nav_slider(approach_box, "Entry Distance (m)", 0.15, 1.0, 0.01,
                             DEFAULT_NAV_PARAMS['blind_approach_entry_distance'],
                             'blind_approach_entry_distance')
        self._add_nav_slider(approach_box, "Approach Speed (m/s)", 0.03, 0.20, 0.01,
                             DEFAULT_NAV_PARAMS['blind_approach_speed'],
                             'blind_approach_speed')
        self._add_nav_slider(approach_box, "Timeout (s)", 3.0, 30.0, 1.0,
                             DEFAULT_NAV_PARAMS['blind_approach_timeout'],
                             'blind_approach_timeout')
        self._add_nav_slider(approach_box, "LiDAR Stop (m)", 0.05, 0.30, 0.01,
                             DEFAULT_NAV_PARAMS['blind_approach_lidar_stop'],
                             'blind_approach_lidar_stop')
        self._add_nav_slider(approach_box, "Arrival Margin (m)", 0.03, 0.60, 0.01,
                             DEFAULT_NAV_PARAMS['blind_approach_arrival_margin'],
                             'blind_approach_arrival_margin')
        approach_box.pack_start(self._make_reset_row(
            lambda: self._reset_nav_params(
                ['blind_approach_entry_distance', 'blind_approach_speed',
                 'blind_approach_timeout', 'blind_approach_lidar_stop',
                 'blind_approach_arrival_margin'])),
            False, False, 0)

        return self._make_sub_notebook([
            ("Speed", speed_box),
            ("Target", target_box),
            ("Approach", approach_box),
        ])

    # ---- Search tab (2 sub-tabs) ----

    def _build_search_page(self):
        """Build the Search tab with sub-tabs: Scan, Spiral.

        Scan sub-tab (360-degree in-place rotation when target is lost):
            search_angular_speed -- Rotation speed during scan (rad/s).
            search_timeout       -- Max scan duration before giving up (s).

        Spiral sub-tab (outward spiral drive if scan fails):
            spiral_search_enabled -- Enable/disable spiral after scan timeout.
            spiral_initial_radius -- Starting radius of the spiral (m).
            spiral_max_radius     -- Maximum radius before giving up (m).
            spiral_growth_rate    -- How much radius grows per revolution (m/rev).
            spiral_linear_speed   -- Forward speed during spiral (m/s).
            spiral_angular_speed  -- Turn rate during spiral (rad/s).
            spiral_timeout        -- Max spiral duration (s).
        """

        # -- Scan sub-tab --
        scan_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        scan_box.set_margin_top(4)
        scan_box.pack_start(self._make_section("360 Scan (when target lost)"),
                            False, False, 0)
        self._add_nav_slider(scan_box, "Scan Speed (rad/s)", 0.05, 1.0, 0.05,
                             DEFAULT_NAV_PARAMS['search_angular_speed'],
                             'search_angular_speed')
        self._add_nav_slider(scan_box, "Scan Timeout (s)", 10.0, 90.0, 5.0,
                             DEFAULT_NAV_PARAMS['search_timeout'], 'search_timeout')
        scan_box.pack_start(self._make_reset_row(
            lambda: self._reset_nav_params(
                ['search_angular_speed', 'search_timeout'])),
            False, False, 0)

        # -- Spiral sub-tab --
        spiral_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        spiral_box.set_margin_top(4)

        val = self._state.detection.get_nav_param('spiral_search_enabled')
        spiral_box.pack_start(self._make_switch_row(
            'nav.spiral_search_enabled', "Enable Spiral Search",
            val if val is not None else DEFAULT_NAV_PARAMS['spiral_search_enabled'],
            self._nav_callback('spiral_search_enabled')), False, False, 0)

        self._add_nav_slider(spiral_box, "Start Radius (m)", 0.1, 1.0, 0.05,
                             DEFAULT_NAV_PARAMS['spiral_initial_radius'],
                             'spiral_initial_radius')
        self._add_nav_slider(spiral_box, "Max Radius (m)", 0.5, 5.0, 0.5,
                             DEFAULT_NAV_PARAMS['spiral_max_radius'],
                             'spiral_max_radius')
        self._add_nav_slider(spiral_box, "Growth Rate (m/rev)", 0.05, 0.50, 0.05,
                             DEFAULT_NAV_PARAMS['spiral_growth_rate'],
                             'spiral_growth_rate')
        self._add_nav_slider(spiral_box, "Spiral Speed (m/s)", 0.05, 0.30, 0.01,
                             DEFAULT_NAV_PARAMS['spiral_linear_speed'],
                             'spiral_linear_speed')
        self._add_nav_slider(spiral_box, "Turn Speed (rad/s)", 0.1, 0.5, 0.05,
                             DEFAULT_NAV_PARAMS['spiral_angular_speed'],
                             'spiral_angular_speed')
        self._add_nav_slider(spiral_box, "Spiral Timeout (s)", 10.0, 120.0, 5.0,
                             DEFAULT_NAV_PARAMS['spiral_timeout'], 'spiral_timeout')
        spiral_box.pack_start(self._make_reset_row(
            lambda: self._reset_nav_params(
                ['spiral_search_enabled', 'spiral_initial_radius',
                 'spiral_max_radius', 'spiral_growth_rate',
                 'spiral_linear_speed', 'spiral_angular_speed',
                 'spiral_timeout'])),
            False, False, 0)

        return self._make_sub_notebook([
            ("Scan", scan_box),
            ("Spiral", spiral_box),
        ])

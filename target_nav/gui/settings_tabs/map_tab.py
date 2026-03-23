"""Radar display settings tab for the SettingsWindow.

Controls the LiDAR radar view appearance: range, point size, grid,
nav target visibility, robot marker size.

Runs in: GUI process (Process 0) on the GTK main thread.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from target_nav.utils.log import get_logger

logger = get_logger('gui.settings.map')


class MapTabMixin:
    """Radar display settings tab."""

    def _map_callback(self, key):
        """Return a callback that sets a radar display parameter."""
        def cb(v):
            from target_nav.config import DEFAULT_MAP_PARAMS
            if isinstance(DEFAULT_MAP_PARAMS.get(key), bool):
                v = bool(v)
            elif isinstance(DEFAULT_MAP_PARAMS.get(key), int):
                v = int(v)
            self._state.detection.set_map_param(key, v)
            logger.info("radar.%s = %s", key, v)
            self._schedule_save()
        return cb

    def _build_map_page(self):
        """Build the Radar settings page.

        Sections:
            Radar View  -- radar_range (1-10m), laser_point_size (1-5px).
            Robot Marker -- robot_arrow_len (10-40px).
            Overlays    -- show_grid (distance circles), show_nav_target (diamond).
        """
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.set_margin_top(4)
        mp = self._state.detection.get_map_params()

        # Radar range
        box.pack_start(self._make_section("Radar View"), False, False, 0)
        box.pack_start(self._make_slider_row(
            'map.radar_range', "Range (m)", 1, 10, 1,
            mp.get('radar_range', 5),
            self._map_callback('radar_range')), False, False, 0)

        # Point appearance
        box.pack_start(self._make_slider_row(
            'map.laser_point_size', "Point Size (px)", 1, 5, 1,
            mp.get('laser_point_size', 2),
            self._map_callback('laser_point_size')), False, False, 0)

        # Robot marker
        box.pack_start(self._make_section("Robot Marker"), False, False, 0)
        box.pack_start(self._make_slider_row(
            'map.robot_arrow_len', "Arrow Length (px)", 10, 40, 2,
            mp.get('robot_arrow_len', 20),
            self._map_callback('robot_arrow_len')), False, False, 0)

        # Overlays
        box.pack_start(self._make_section("Overlays"), False, False, 0)
        box.pack_start(self._make_switch_row(
            'map.show_grid', "Distance Circles",
            mp.get('show_grid', True),
            self._map_callback('show_grid')), False, False, 0)
        box.pack_start(self._make_switch_row(
            'map.show_nav_target', "Show Nav Target",
            mp.get('show_nav_target', True),
            self._map_callback('show_nav_target')), False, False, 0)

        # Reset
        box.pack_start(self._make_reset_row(self._on_reset_map), False, False, 0)
        return box

    def _on_reset_map(self, _btn=None):
        """Reset all radar settings to defaults."""
        from target_nav.config import DEFAULT_MAP_PARAMS
        for k, v in DEFAULT_MAP_PARAMS.items():
            self._state.detection.set_map_param(k, v)
            self._set_widget_value(f'map.{k}', v)
        self._schedule_save()

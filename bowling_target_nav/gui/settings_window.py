"""Settings window with parameter sliders and calibration tools.

Lazy-constructed on first SETTINGS click from MainGUI. Uses Gtk.Notebook
with left-side tabs optimized for the V2N 1024x600 touchscreen. Each tab
is a scrollable page of Scale sliders, Switch toggles, and action buttons.

Tabs and their purposes:
  - Nav:      Core navigation parameters (linear/angular speed, arrival
              distance, approach gains, speed ramp, lost timeout).
  - Search:   Search-mode parameters (rotation speed, search timeout,
              spiral enable/radius/step/speed).
  - Blind:    Blind-approach parameters (entry distance, max duration/
              distance, speed, and obstacle abort threshold).
  - Detect:   Detection tuning (confidence threshold, detection expiry,
              calibration reference height/distance, camera FOV, and
              detection filter parameters for aspect ratio, box size,
              tracker consistency).
  - Obstacle: Obstacle avoidance settings (enable, distances, avoidance
              gains, scan angle range).
  - Map:      Map display options (rotation, grid, laser visibility,
              robot/arrow size, laser point size, nav target overlay).
  - Setup:    Hardware setup -- camera and LiDAR mounting position (x/y/z)
              and yaw angle, published as static TF frames.
  - Tools:    Diagnostic tools (motor test, reset odometry, save/load
              calibration, and manual ROS command publishing).

All Scale/Switch widgets are tracked in _widgets dict so reset can
update their positions without rebuilding pages.  The _updating flag
suppresses value-changed callbacks during programmatic slider updates.
"""

import math
import traceback

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

# ROS message imports — optional, not needed in multiprocess mode
try:
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Empty, String
    _HAS_ROS_MSGS = True
except ImportError:
    _HAS_ROS_MSGS = False

_LOG_TAG = "[Settings]"


def _log(msg):
    print(f"{_LOG_TAG} {msg}", flush=True)


class SettingsWindow(Gtk.Window):
    """Fullscreen settings window with Notebook tabs for runtime parameter tuning.

    Reads current values from shared_state on construction and writes changes
    back immediately via slider callbacks. SAVE ALL persists to calibration.json.

    Tabs: Nav, Search, Blind, Detect, Obstacle, Map, Setup, Tools.
    """

    def __init__(self, shared_state):
        """Initialize the settings window and build all notebook tabs.

        Args:
            shared_state: SharedState or IPCState facade for reading/writing
                navigation, detection, and sensor parameters.
        """
        _log("__init__ start")
        super().__init__(title="Robot Settings")
        self._state = shared_state
        self._parent_window = None
        self._motor_test_timer = None
        self._save_timer = None

        # Track all widgets by key so reset can update them
        # key -> Gtk.Scale or Gtk.Switch
        self._widgets = {}
        # Suppress callbacks while programmatically updating sliders
        self._updating = False

        # Auto-detect screen size
        display = Gdk.Display.get_default()
        if display and display.get_n_monitors() > 0:
            geo = display.get_monitor(0).get_geometry()
            _log(f"Screen: {geo.width}x{geo.height}")
            self.set_default_size(geo.width, geo.height)
        else:
            _log("Screen: fallback 1024x600")
            self.set_default_size(1024, 600)

        self.get_style_context().add_class('settings-window')
        self.connect('delete-event', self._on_delete)
        self.connect('key-press-event', self._on_key)

        # Main layout: header bar + notebook
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(vbox)

        # Header with BACK button
        header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        header.set_margin_start(10)
        header.set_margin_end(10)
        header.set_margin_top(6)
        header.set_margin_bottom(4)

        back_btn = Gtk.Button(label="\u25c0  BACK")
        back_btn.set_size_request(120, 44)
        back_btn.connect('clicked', self._on_back)
        header.pack_start(back_btn, False, False, 0)

        title_label = Gtk.Label()
        title_label.set_markup(
            "<span font_weight='bold' size='16000' foreground='#58a6ff'>"
            "Robot Settings</span>")
        header.pack_start(title_label, True, True, 0)

        save_btn = Gtk.Button(label="SAVE ALL")
        save_btn.set_size_request(120, 44)
        save_btn.get_style_context().add_class('suggested-action')
        save_btn.connect('clicked', self._on_save_all)
        header.pack_end(save_btn, False, False, 0)

        self._header_status = Gtk.Label(label="")
        self._header_status.set_xalign(1)
        header.pack_end(self._header_status, False, False, 10)

        vbox.pack_start(header, False, False, 0)
        vbox.pack_start(Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL),
                        False, False, 0)

        # Notebook tabs
        self._notebook = Gtk.Notebook()
        self._notebook.set_tab_pos(Gtk.PositionType.LEFT)
        self._notebook.set_scrollable(True)

        pages = [
            ("Nav", self._build_nav_page),
            ("Search", self._build_search_page),
            ("Blind", self._build_blind_page),
            ("Detect", self._build_detection_page),
            ("Obstacle", self._build_obstacle_page),
            ("Map", self._build_map_page),
            ("Setup", self._build_setup_page),
            ("Tools", self._build_tools_page),
        ]

        for title, builder in pages:
            try:
                page = builder()
                tab_label = Gtk.Label(label=title)
                tab_label.set_size_request(80, 36)
                self._notebook.append_page(page, tab_label)
                _log(f"Tab '{title}' built OK")
            except Exception as e:
                _log(f"Tab '{title}' FAILED: {e}")
                traceback.print_exc()
                err = Gtk.Label(label=f"Error: {e}")
                self._notebook.append_page(err, Gtk.Label(label=title))

        vbox.pack_start(self._notebook, True, True, 0)
        _log("__init__ done")

    # ---- Navigation ----

    def show_all(self):
        """Override to clear stale status message each time window is shown."""
        self._header_status.set_text("")
        super().show_all()

    def _on_back(self, _button):
        _log("BACK clicked")
        self._stop_motor_test()
        self.hide()
        if self._parent_window:
            self._parent_window.return_from_settings()

    def _on_delete(self, _widget, _event):
        self._on_back(None)
        return True

    def _on_key(self, _widget, event):
        key = Gdk.keyval_name(event.keyval)
        if key and key.lower() == 'escape':
            self._on_back(None)
            return True
        return False

    # ---- Auto-save ----

    def _schedule_save(self):
        if self._save_timer:
            GLib.source_remove(self._save_timer)
        self._save_timer = GLib.timeout_add(2000, self._do_save)

    def _do_save(self):
        try:
            ok = self._state.detection.save_calibration()
            _log(f"Auto-saved: {'OK' if ok else 'FAILED'}")
        except Exception as e:
            _log(f"Auto-save error: {e}")
        self._save_timer = None
        return False

    def _on_save_all(self, _button):
        _log("SAVE ALL clicked")
        try:
            # Log current values before saving
            np = self._state.detection.get_nav_params()
            mp = self._state.detection.get_map_params()
            _log(f"  nav_params: {np}")
            _log(f"  map_params: {mp}")
            ok = self._state.detection.save_calibration()
            if ok:
                self._header_status.set_markup(
                    "<span foreground='#50fa7b'>Saved to device!</span>")
                _log("Save OK")
            else:
                self._header_status.set_markup(
                    "<span foreground='#ff6b6b'>Save failed!</span>")
                _log("Save returned False")
        except Exception as e:
            _log(f"Save error: {e}")
            self._header_status.set_markup(
                f"<span foreground='#ff6b6b'>Error: {e}</span>")

    # ---- Motor test cleanup ----

    def _stop_motor_test(self):
        if self._motor_test_timer:
            GLib.source_remove(self._motor_test_timer)
            self._motor_test_timer = None
            self._state.send_ros_command({'type': 'stop_robot'})

    # ---- Widget helpers ----

    def _make_section(self, text):
        label = Gtk.Label()
        label.set_markup(
            f"<span foreground='#58a6ff' font_weight='bold' size='13000'>"
            f"{text}</span>")
        label.set_xalign(0)
        label.set_margin_start(12)
        label.set_margin_top(14)
        label.set_margin_bottom(6)
        return label

    def _make_slider_row(self, widget_key, label_text, min_val, max_val,
                         step, initial, callback):
        """Slider row. Registers Scale in self._widgets[widget_key]."""
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_margin_start(12)
        row.set_margin_end(12)
        row.set_margin_top(4)
        row.set_margin_bottom(4)

        label = Gtk.Label(label=label_text)
        label.set_xalign(0)
        label.set_size_request(200, -1)
        row.pack_start(label, False, False, 0)

        digits = 2 if step < 0.1 else 1
        adj = Gtk.Adjustment(value=initial, lower=min_val, upper=max_val,
                             step_increment=step, page_increment=step * 5)
        scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adj)
        scale.set_digits(digits)
        scale.set_value_pos(Gtk.PositionType.RIGHT)
        row.pack_start(scale, True, True, 0)

        val_label = Gtk.Label(label=f"{initial:.{digits}f}")
        val_label.set_size_request(55, -1)
        row.pack_start(val_label, False, False, 0)

        def on_change(s):
            if self._updating:
                return
            v = s.get_value()
            val_label.set_text(f"{v:.{digits}f}")
            callback(v)

        scale.connect('value-changed', on_change)
        # Also update val_label when we programmatically set slider
        scale._val_label = val_label
        scale._digits = digits

        self._widgets[widget_key] = scale
        return row

    def _set_widget_value(self, key, value):
        """Update a tracked widget's value without triggering callbacks."""
        w = self._widgets.get(key)
        if w is None:
            return
        self._updating = True
        try:
            if isinstance(w, Gtk.Scale):
                w.set_value(value)
                if hasattr(w, '_val_label'):
                    w._val_label.set_text(f"{value:.{w._digits}f}")
            elif isinstance(w, Gtk.SpinButton):
                w.set_value(value)
            elif isinstance(w, Gtk.Switch):
                w.set_active(bool(value))
        finally:
            self._updating = False

    def _make_switch_row(self, widget_key, label_text, initial, callback):
        """Toggle switch row. Registers Switch in self._widgets[widget_key]."""
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_margin_start(12)
        row.set_margin_top(4)
        lbl = Gtk.Label(label=label_text)
        lbl.set_xalign(0)
        lbl.set_size_request(200, -1)
        row.pack_start(lbl, False, False, 0)
        sw = Gtk.Switch()
        sw.set_active(bool(initial))

        def on_toggle(switch, _pspec):
            if self._updating:
                return
            callback(switch.get_active())

        sw.connect('notify::active', on_toggle)
        row.pack_start(sw, False, False, 0)
        self._widgets[widget_key] = sw
        return row

    def _make_spin_row(self, label_text, min_v, max_v, step, default,
                       digits=1, unit=""):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        row.set_margin_start(12)
        row.set_margin_top(3)
        lbl = Gtk.Label(label=label_text)
        lbl.set_xalign(0)
        lbl.set_size_request(90, -1)
        row.pack_start(lbl, False, False, 0)
        spin = Gtk.SpinButton.new_with_range(min_v, max_v, step)
        spin.set_value(default)
        spin.set_digits(digits)
        row.pack_start(spin, False, False, 0)
        if unit:
            row.pack_start(Gtk.Label(label=unit), False, False, 4)
        return row, spin

    def _scrolled_page(self, box):
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.add(box)
        return scroll

    def _make_angle_row(self, widget_key, label_text, initial, callback):
        """Angle input: spin button (-360..360, step 1) + preset buttons."""
        row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        row.set_margin_start(12)
        row.set_margin_end(12)
        row.set_margin_top(4)
        row.set_margin_bottom(4)

        # Top: label + spin
        top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        lbl = Gtk.Label(label=label_text)
        lbl.set_xalign(0)
        lbl.set_size_request(160, -1)
        top.pack_start(lbl, False, False, 0)

        spin = Gtk.SpinButton.new_with_range(-360, 360, 1)
        spin.set_value(initial)
        spin.set_digits(1)
        spin.set_size_request(100, -1)
        top.pack_start(spin, False, False, 0)
        top.pack_start(Gtk.Label(label="deg"), False, False, 4)
        row.pack_start(top, False, False, 0)

        # Bottom: preset angle buttons
        presets = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        presets.set_margin_start(160)
        for angle in [0, 45, 90, 135, 180, -45, -90, -135, -180]:
            sign = "+" if angle > 0 else ""
            btn = Gtk.Button(label=f"{sign}{angle}" if angle != 0 else "0")
            btn.set_size_request(48, 30)

            def on_preset(_b, a=angle):
                spin.set_value(a)
            btn.connect('clicked', on_preset)
            presets.pack_start(btn, False, False, 0)
        row.pack_start(presets, False, False, 0)

        def on_change(_s):
            if self._updating:
                return
            v = spin.get_value()
            callback(v)
        spin.connect('value-changed', on_change)

        # Register for reset support
        self._widgets[widget_key] = spin
        return row

    # ---- Navigator param helpers ----

    def _nav_callback(self, attr):
        def cb(v):
            self._state.send_ros_command({
                'type': 'set_nav_param', 'attr': attr, 'value': v})
            self._state.detection.set_nav_param(attr, v)
            _log(f"nav.{attr} = {v}")
            self._schedule_save()
        return cb

    def _add_nav_slider(self, box, label, min_v, max_v, step, default, attr):
        """Add a nav-linked slider. Uses attr as widget key."""
        stored = self._state.detection.get_nav_param(attr)
        initial = stored if stored is not None else default
        box.pack_start(self._make_slider_row(
            f"nav.{attr}", label, min_v, max_v, step, initial,
            self._nav_callback(attr)), False, False, 0)

    # ---- Map param helper ----

    def _map_callback(self, key):
        def cb(v):
            from bowling_target_nav.state.detection_store import DEFAULT_MAP_PARAMS
            if isinstance(DEFAULT_MAP_PARAMS.get(key), int):
                v = int(v)
            elif isinstance(DEFAULT_MAP_PARAMS.get(key), bool):
                v = bool(v)
            self._state.detection.set_map_param(key, v)
            _log(f"map.{key} = {v}")
            self._schedule_save()
            # Reset cartographer map when rotation changes so map rebuilds cleanly
            if key == 'rotation':
                import threading
                def _reset():
                    try:
                        from bowling_target_nav.nodes.main_gui import _reset_cartographer_map
                        _reset_cartographer_map()
                    except Exception as e:
                        print(f"[Settings] Map reset on rotation failed: {e}", flush=True)
                threading.Thread(target=_reset, daemon=True).start()
        return cb

    # ==================================================================
    # PAGE BUILDERS
    # ==================================================================

    def _build_nav_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.pack_start(self._make_section("Navigation"), False, False, 0)
        self._add_nav_slider(box, "Linear Speed (m/s)", 0.05, 0.40, 0.01,
                             0.15, 'linear_speed')
        self._add_nav_slider(box, "Min Speed (m/s)", 0.05, 0.25, 0.01,
                             0.10, 'min_linear_speed')
        self._add_nav_slider(box, "Angular Speed (rad/s)", 0.1, 1.0, 0.05,
                             0.4, 'angular_speed')
        self._add_nav_slider(box, "Approach Distance (m)", 0.03, 0.50, 0.01,
                             0.08, 'approach_distance')
        self._add_nav_slider(box, "Lost Timeout (s)", 1.0, 10.0, 0.5,
                             3.0, 'lost_timeout')
        return self._scrolled_page(box)

    def _build_search_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.pack_start(self._make_section("360\u00b0 Scan (Tier 1)"), False, False, 0)
        self._add_nav_slider(box, "Scan Speed (rad/s)", 0.05, 1.0, 0.05,
                             0.35, 'search_angular_speed')
        self._add_nav_slider(box, "Scan Timeout (s)", 10.0, 60.0, 5.0,
                             30.0, 'search_timeout')

        box.pack_start(self._make_section("Spiral Search (Tier 2)"), False, False, 0)
        self._add_nav_slider(box, "Initial Radius (m)", 0.1, 1.0, 0.05,
                             0.3, 'spiral_initial_radius')
        self._add_nav_slider(box, "Max Radius (m)", 0.5, 5.0, 0.5,
                             2.0, 'spiral_max_radius')
        self._add_nav_slider(box, "Growth Rate (m/rev)", 0.05, 0.50, 0.05,
                             0.15, 'spiral_growth_rate')
        self._add_nav_slider(box, "Spiral Speed (m/s)", 0.05, 0.30, 0.01,
                             0.10, 'spiral_linear_speed')
        self._add_nav_slider(box, "Sweep Speed (rad/s)", 0.1, 0.8, 0.05,
                             0.25, 'spiral_angular_speed')
        self._add_nav_slider(box, "Spiral Timeout (s)", 15.0, 90.0, 5.0,
                             45.0, 'spiral_timeout')

        val = self._state.detection.get_nav_param('spiral_search_enabled')
        box.pack_start(self._make_switch_row(
            'nav.spiral_search_enabled', "Spiral Search Enabled",
            val if val is not None else True,
            self._nav_callback('spiral_search_enabled')), False, False, 0)

        return self._scrolled_page(box)

    def _build_blind_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.pack_start(self._make_section("Blind Approach"), False, False, 0)
        self._add_nav_slider(box, "Entry Distance (m)", 0.20, 1.50, 0.05,
                             0.37, 'blind_approach_entry_distance')
        self._add_nav_slider(box, "Approach Speed (m/s)", 0.05, 0.30, 0.01,
                             0.12, 'blind_approach_speed')
        self._add_nav_slider(box, "Timeout (s)", 3.0, 15.0, 0.5,
                             10.0, 'blind_approach_timeout')
        self._add_nav_slider(box, "LiDAR Stop (m)", 0.15, 0.40, 0.01,
                             0.15, 'blind_approach_lidar_stop')
        self._add_nav_slider(box, "Arrival Margin (m)", 0.05, 0.30, 0.01,
                             0.10, 'blind_approach_arrival_margin')
        return self._scrolled_page(box)

    def _build_detection_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.pack_start(self._make_section("Post-Processing"), False, False, 0)

        det = self._state.detection

        def _det_cb(setter):
            def cb(v):
                setter(v)
                self._schedule_save()
            return cb

        box.pack_start(self._make_slider_row(
            'det.confidence', "Confidence", 0.05, 0.95, 0.05,
            det.get_confidence_threshold(),
            _det_cb(det.set_confidence_threshold)), False, False, 0)

        box.pack_start(self._make_slider_row(
            'det.expiry', "Detection Memory (s)", 0.3, 5.0, 0.1,
            det.get_detect_expiry(),
            _det_cb(det.set_detect_expiry)), False, False, 0)

        info = Gtk.Label()
        info.set_markup(
            "<span foreground='#8899aa' size='small'>"
            "Confidence: lower = detect more (but more false positives). "
            "Memory: how long to remember a lost target.</span>")
        info.set_xalign(0)
        info.set_margin_start(12)
        info.set_margin_top(10)
        info.set_line_wrap(True)
        box.pack_start(info, False, False, 0)

        return self._scrolled_page(box)

    def _build_obstacle_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.pack_start(self._make_section("Obstacle Avoidance"), False, False, 0)
        self._add_nav_slider(box, "Stop Distance (m)", 0.10, 0.50, 0.01,
                             0.20, 'obstacle_distance')
        self._add_nav_slider(box, "Slowdown (m)", 0.20, 1.00, 0.05,
                             0.40, 'obstacle_slowdown_distance')

        hw = self._state.detection.get_robot_half_width()
        info = Gtk.Label()
        info.set_markup(
            f"<span foreground='#8899aa'>Robot half-width: {hw:.3f}m "
            f"(change in Setup tab)</span>")
        info.set_xalign(0)
        info.set_margin_start(12)
        info.set_margin_top(10)
        box.pack_start(info, False, False, 0)
        return self._scrolled_page(box)

    def _build_map_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        mp = self._state.detection.get_map_params()

        box.pack_start(self._make_section("Map Display"), False, False, 0)
        box.pack_start(self._make_angle_row(
            'map.rotation', "Map Rotation",
            mp.get('rotation', 0.0),
            self._map_callback('rotation')), False, False, 0)

        box.pack_start(self._make_section("Robot Symbol"), False, False, 0)
        box.pack_start(self._make_slider_row(
            'map.robot_size', "Robot Size (px)", 4, 30, 1,
            mp.get('robot_size', 12),
            self._map_callback('robot_size')), False, False, 0)
        box.pack_start(self._make_slider_row(
            'map.robot_arrow_len', "Arrow Length (px)", 8, 50, 2,
            mp.get('robot_arrow_len', 20),
            self._map_callback('robot_arrow_len')), False, False, 0)

        box.pack_start(self._make_section("Laser"), False, False, 0)
        box.pack_start(self._make_slider_row(
            'map.laser_point_size', "Point Size", 0, 2, 1,
            mp.get('laser_point_size', 1),
            self._map_callback('laser_point_size')), False, False, 0)

        box.pack_start(self._make_section("Visibility"), False, False, 0)
        for label, key, default in [
            ("Show Grid", 'show_grid', True),
            ("Show Laser", 'show_laser', True),
            ("Show Nav Target", 'show_nav_target', True),
        ]:
            box.pack_start(self._make_switch_row(
                f'map.{key}', label, mp.get(key, default),
                self._map_callback(key)), False, False, 0)

        # Reset button
        reset_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        reset_row.set_margin_start(12)
        reset_row.set_margin_top(14)
        reset_btn = Gtk.Button(label="RESET MAP DEFAULTS")
        reset_btn.set_size_request(180, 36)
        self._map_reset_label = Gtk.Label(label="")
        reset_btn.connect('clicked', self._on_reset_map)
        reset_row.pack_start(reset_btn, False, False, 0)
        reset_row.pack_start(self._map_reset_label, True, True, 0)
        box.pack_start(reset_row, False, False, 0)

        return self._scrolled_page(box)

    def _on_reset_map(self, _btn):
        _log("RESET MAP clicked")
        from bowling_target_nav.state.detection_store import DEFAULT_MAP_PARAMS
        for k, v in DEFAULT_MAP_PARAMS.items():
            self._state.detection.set_map_param(k, v)
            self._set_widget_value(f'map.{k}', v)
        self._schedule_save()
        self._map_reset_label.set_text("Reset to defaults")

    # ==================================================================
    # SETUP PAGE
    # ==================================================================

    def _build_setup_page(self):
        det = self._state.detection
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        # Robot diagram
        self._robot_da = Gtk.DrawingArea()
        self._robot_da.set_size_request(-1, 180)
        self._robot_da.connect('draw', self._draw_robot_diagram)
        box.pack_start(self._robot_da, False, False, 4)

        howto = Gtk.Label()
        howto.set_markup(
            "<span size='small' foreground='#888'>"
            "1. Measure robot  2. Set sensor positions  "
            "3. Place target at 1m, CALIBRATE  4. SAVE</span>")
        howto.set_xalign(0)
        howto.set_margin_start(12)
        box.pack_start(howto, False, False, 2)

        # Robot dims
        box.pack_start(self._make_section("Robot Body (cm)"), False, False, 0)
        robot = det.get_robot_dims()
        dims_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        dims_row.set_margin_start(12)
        r1, self._sp_robot_len = self._make_spin_row(
            "Length", 10, 60, 1, robot['length'] * 100, 0, "cm")
        r2, self._sp_robot_wid = self._make_spin_row(
            "Width", 10, 60, 1, robot['width'] * 100, 0, "cm")
        r3, self._sp_robot_hgt = self._make_spin_row(
            "Height", 5, 40, 1, robot['height'] * 100, 0, "cm")
        for r in (r1, r2, r3):
            dims_row.pack_start(r, False, False, 0)
        box.pack_start(dims_row, False, False, 2)
        for sp in (self._sp_robot_len, self._sp_robot_wid, self._sp_robot_hgt):
            sp.connect('value-changed', self._on_robot_dims_changed)

        # LiDAR
        box.pack_start(self._make_section("LiDAR Position (cm)"), False, False, 0)
        lidar = det.get_lidar_pos()
        lidar_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        lidar_row.set_margin_start(12)
        r1, self._sp_lidar_x = self._make_spin_row(
            "Forward", -20, 20, 0.5, lidar['x'] * 100, 1, "cm")
        r2, self._sp_lidar_y = self._make_spin_row(
            "Lateral", -20, 20, 0.5, lidar['y'] * 100, 1, "cm")
        r3, self._sp_lidar_z = self._make_spin_row(
            "Height", 0, 30, 0.5, lidar['z'] * 100, 1, "cm")
        for r in (r1, r2, r3):
            lidar_row.pack_start(r, False, False, 0)
        box.pack_start(lidar_row, False, False, 2)
        for sp in (self._sp_lidar_x, self._sp_lidar_y, self._sp_lidar_z):
            sp.connect('value-changed', self._on_sensor_pos_changed)

        # LiDAR Heading (yaw) — for LiDAR mounted rotated (e.g. 180°)
        box.pack_start(self._make_angle_row(
            'lidar_yaw', "LiDAR Yaw",
            det.get_lidar_yaw(),
            self._on_lidar_yaw_changed), False, False, 0)

        # Camera
        box.pack_start(self._make_section("Camera Position (cm)"), False, False, 0)
        cam = det.get_camera_pos()
        cam_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        cam_row.set_margin_start(12)
        r1, self._sp_cam_x = self._make_spin_row(
            "Forward", -20, 20, 0.5, cam['x'] * 100, 1, "cm")
        r2, self._sp_cam_y = self._make_spin_row(
            "Lateral", -20, 20, 0.5, cam['y'] * 100, 1, "cm")
        r3, self._sp_cam_z = self._make_spin_row(
            "Height", 0, 30, 0.5, cam['z'] * 100, 1, "cm")
        for r in (r1, r2, r3):
            cam_row.pack_start(r, False, False, 0)
        box.pack_start(cam_row, False, False, 2)
        for sp in (self._sp_cam_x, self._sp_cam_y, self._sp_cam_z):
            sp.connect('value-changed', self._on_sensor_pos_changed)

        # Camera Heading (yaw)
        box.pack_start(self._make_section("Camera Heading"), False, False, 0)
        box.pack_start(self._make_angle_row(
            'camera_yaw', "Camera Yaw",
            det.get_camera_yaw(),
            self._on_camera_yaw_changed), False, False, 0)

        # FOV
        fov_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        fov_row.set_margin_start(12)
        r1, self._sp_cam_fov = self._make_spin_row(
            "Camera FOV", 30, 120, 1, det.get_camera_fov(), 0, "deg")
        fov_row.pack_start(r1, False, False, 0)
        box.pack_start(fov_row, False, False, 2)
        self._sp_cam_fov.connect('value-changed', self._on_fov_changed)

        # Target
        box.pack_start(self._make_section("Target Object (cm)"), False, False, 0)
        target = det.get_target_dims()
        tgt_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        tgt_row.set_margin_start(12)
        r1, self._sp_tgt_h = self._make_spin_row(
            "Height", 5, 100, 1, target['height'] * 100, 0, "cm")
        r2, self._sp_tgt_w = self._make_spin_row(
            "Width", 2, 50, 1, target['width'] * 100, 0, "cm")
        for r in (r1, r2):
            tgt_row.pack_start(r, False, False, 0)
        box.pack_start(tgt_row, False, False, 2)
        for sp in (self._sp_tgt_h, self._sp_tgt_w):
            sp.connect('value-changed', self._on_target_changed)

        # Reference point
        box.pack_start(self._make_section("Distance Reference"), False, False, 0)
        from bowling_target_nav.state.detection_store import REF_POINTS
        ref_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        ref_row.set_margin_start(12)
        ref_row.pack_start(Gtk.Label(label="Measure from:"), False, False, 0)
        self._ref_combo = Gtk.ComboBoxText()
        ref_labels = {'center': "Center", 'front': "Front edge",
                      'camera': "Camera", 'lidar': "LiDAR"}
        current_ref = det.get_ref_point()
        for i, rp in enumerate(REF_POINTS):
            self._ref_combo.append_text(ref_labels.get(rp, rp))
            if rp == current_ref:
                self._ref_combo.set_active(i)
        self._ref_combo.connect('changed', self._on_ref_point_changed)
        ref_row.pack_start(self._ref_combo, False, False, 0)
        self._ref_info_label = Gtk.Label()
        self._ref_info_label.set_xalign(0)
        ref_row.pack_start(self._ref_info_label, True, True, 8)
        box.pack_start(ref_row, False, False, 4)
        self._update_ref_info()

        # Calibration
        box.pack_start(self._make_section("Distance Calibration"), False, False, 0)
        ref_height, ref_dist = det.get_calibration()
        self._cal_label = Gtk.Label(
            label=f"Current: {ref_height:.0f}px at {ref_dist:.2f}m")
        self._cal_label.set_xalign(0)
        self._cal_label.set_margin_start(12)
        box.pack_start(self._cal_label, False, False, 2)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        cal_row.set_margin_start(12)
        cal_row.set_margin_end(12)
        cal_row.pack_start(Gtk.Label(label="Known dist (m):"), False, False, 0)
        self._cal_dist_spin = Gtk.SpinButton.new_with_range(0.1, 5.0, 0.05)
        self._cal_dist_spin.set_value(1.0)
        self._cal_dist_spin.set_digits(2)
        cal_row.pack_start(self._cal_dist_spin, False, False, 0)
        cal_btn = Gtk.Button(label="CALIBRATE")
        cal_btn.connect('clicked', self._on_calibrate)
        cal_row.pack_start(cal_btn, False, False, 8)
        self._cal_result = Gtk.Label(label="")
        cal_row.pack_start(self._cal_result, True, True, 0)
        box.pack_start(cal_row, False, False, 4)

        # Manual entry
        manual_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        manual_row.set_margin_start(12)
        manual_row.set_margin_end(12)
        manual_row.pack_start(Gtk.Label(label="Manual: H"), False, False, 0)
        self._manual_h_spin = Gtk.SpinButton.new_with_range(10, 480, 1)
        self._manual_h_spin.set_value(ref_height)
        manual_row.pack_start(self._manual_h_spin, False, False, 0)
        manual_row.pack_start(Gtk.Label(label="px @"), False, False, 4)
        self._manual_d_spin = Gtk.SpinButton.new_with_range(0.1, 5.0, 0.05)
        self._manual_d_spin.set_value(ref_dist)
        self._manual_d_spin.set_digits(2)
        manual_row.pack_start(self._manual_d_spin, False, False, 0)
        manual_row.pack_start(Gtk.Label(label="m"), False, False, 2)
        apply_btn = Gtk.Button(label="APPLY")
        apply_btn.connect('clicked', self._on_manual_cal)
        manual_row.pack_start(apply_btn, False, False, 6)
        box.pack_start(manual_row, False, False, 4)

        # Save/Reset
        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        btn_row.set_margin_start(12)
        btn_row.set_margin_end(12)
        btn_row.set_margin_top(8)
        save_btn = Gtk.Button(label="SAVE TO DEVICE")
        save_btn.set_size_request(140, 36)
        save_btn.connect('clicked', self._on_save_all)
        reset_btn = Gtk.Button(label="RESET ALL DEFAULTS")
        reset_btn.set_size_request(160, 36)
        reset_btn.connect('clicked', self._on_reset_defaults)
        self._setup_status = Gtk.Label(label="")
        btn_row.pack_start(save_btn, False, False, 0)
        btn_row.pack_start(reset_btn, False, False, 0)
        btn_row.pack_start(self._setup_status, True, True, 0)
        box.pack_start(btn_row, False, False, 4)

        return self._scrolled_page(box)

    # ---- Robot diagram ----

    def _draw_robot_diagram(self, widget, cr):
        try:
            alloc = widget.get_allocation()
            W, H = alloc.width, alloc.height
            cr.set_source_rgb(0.12, 0.12, 0.14)
            cr.paint()

            rlen = self._sp_robot_len.get_value()
            rwid = self._sp_robot_wid.get_value()
            lx = self._sp_lidar_x.get_value()
            ly = self._sp_lidar_y.get_value()
            cx = self._sp_cam_x.get_value()
            cy = self._sp_cam_y.get_value()

            max_dim = max(rlen, rwid, 30)
            scale = min((W - 80) / (max_dim * 1.5),
                        (H - 60) / (max_dim * 1.5))
            ox, oy = W / 2, H / 2 + 10

            def to_px(xcm, ycm):
                return ox + ycm * scale, oy - xcm * scale

            cr.set_source_rgba(0.5, 0.8, 1.0, 0.6)
            cr.set_font_size(11)
            cr.move_to(ox - 16, oy - (rlen / 2) * scale - 12)
            cr.show_text("FRONT")

            bx, by = to_px(rlen / 2, rwid / 2)
            bw, bh = rwid * scale, rlen * scale
            cr.set_source_rgba(0.2, 0.3, 0.6, 0.4)
            cr.rectangle(bx, by, bw, bh)
            cr.fill()
            cr.set_source_rgba(0.3, 0.5, 0.9, 0.8)
            cr.set_line_width(1.5)
            cr.rectangle(bx, by, bw, bh)
            cr.stroke()

            for wx, wy in [(rlen/2-2, rwid/2-1), (rlen/2-2, -rwid/2+1),
                           (-rlen/2+2, rwid/2-1), (-rlen/2+2, -rwid/2+1)]:
                wpx, wpy = to_px(wx, wy)
                cr.set_source_rgba(0.4, 0.4, 0.4, 0.9)
                cr.rectangle(wpx-2, wpy-4, 4, 8)
                cr.fill()

            cpx, cpy = to_px(0, 0)
            cr.set_source_rgba(1, 1, 1, 0.4)
            cr.set_line_width(1)
            cr.move_to(cpx-8, cpy); cr.line_to(cpx+8, cpy)
            cr.move_to(cpx, cpy-8); cr.line_to(cpx, cpy+8)
            cr.stroke()

            lpx, lpy = to_px(lx, ly)
            cr.set_source_rgb(1, 0.2, 0.2)
            cr.arc(lpx, lpy, 6, 0, 2*math.pi); cr.fill()
            cr.set_source_rgba(1, 0.2, 0.2, 0.2)
            cr.arc(lpx, lpy, 20, 0, 2*math.pi); cr.stroke()
            cr.set_font_size(10)
            cr.set_source_rgb(1, 0.4, 0.4)
            cr.move_to(lpx+9, lpy+4); cr.show_text("LiDAR")

            camx, camy = to_px(cx, cy)
            cr.set_source_rgb(0.2, 0.9, 0.3)
            cr.move_to(camx, camy-7)
            cr.line_to(camx-5, camy+5)
            cr.line_to(camx+5, camy+5)
            cr.close_path(); cr.fill()
            fov = self._state.detection.get_camera_fov()
            fov_half = math.radians(fov / 2)
            cr.set_source_rgba(0.2, 0.9, 0.3, 0.15)
            cr.move_to(camx, camy)
            cr.line_to(camx - 30*math.sin(fov_half), camy - 30*math.cos(fov_half))
            cr.line_to(camx + 30*math.sin(fov_half), camy - 30*math.cos(fov_half))
            cr.close_path(); cr.fill()
            cr.set_font_size(10)
            cr.set_source_rgb(0.3, 1, 0.4)
            cr.move_to(camx+9, camy+4); cr.show_text("Cam")

            cr.set_source_rgba(0.7, 0.7, 0.7, 0.7)
            cr.set_font_size(9)
            cr.move_to(bx + bw + 8, by + bh/2 + 3)
            cr.show_text(f"{rlen:.0f}cm")
            cr.move_to(bx + bw/2 - 10, by + bh + 14)
            cr.show_text(f"{rwid:.0f}cm")
        except Exception as e:
            _log(f"Robot diagram error: {e}")
        return False

    # ---- Setup handlers ----

    def _on_robot_dims_changed(self, _spin):
        l_m = self._sp_robot_len.get_value() / 100.0
        w_m = self._sp_robot_wid.get_value() / 100.0
        h_m = self._sp_robot_hgt.get_value() / 100.0
        self._state.detection.set_robot_dims(l_m, w_m, h_m)
        self._state.send_ros_command({
            'type': 'set_robot_half_width', 'value': w_m / 2.0})
        if hasattr(self, '_ref_info_label'):
            self._update_ref_info()
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_sensor_pos_changed(self, _spin):
        self._state.detection.set_lidar_pos(
            self._sp_lidar_x.get_value() / 100.0,
            self._sp_lidar_y.get_value() / 100.0,
            self._sp_lidar_z.get_value() / 100.0)
        self._state.detection.set_camera_pos(
            self._sp_cam_x.get_value() / 100.0,
            self._sp_cam_y.get_value() / 100.0,
            self._sp_cam_z.get_value() / 100.0)
        if hasattr(self, '_ref_info_label'):
            self._update_ref_info()
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_ref_point_changed(self, combo):
        from bowling_target_nav.state.detection_store import REF_POINTS
        idx = combo.get_active()
        if 0 <= idx < len(REF_POINTS):
            self._state.detection.set_ref_point(REF_POINTS[idx])
            self._update_ref_info()
            self._schedule_save()

    def _update_ref_info(self):
        try:
            cam_off = self._state.detection.get_sensor_offset('camera')
            lidar_off = self._state.detection.get_sensor_offset('lidar')
            self._ref_info_label.set_markup(
                f"<span size='small' foreground='#aaa'>"
                f"cam: {cam_off*100:+.1f}cm  "
                f"lidar: {lidar_off*100:+.1f}cm</span>")
        except Exception as e:
            _log(f"ref info error: {e}")

    def _on_camera_yaw_changed(self, value):
        self._state.detection.set_camera_yaw(value)
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_lidar_yaw_changed(self, value):
        self._state.detection.set_lidar_yaw(value)
        self._robot_da.queue_draw()
        self._schedule_save()
        # Reset cartographer map — old trajectory used wrong TF orientation
        import threading
        def _reset():
            try:
                from bowling_target_nav.nodes.main_gui import _reset_cartographer_map
                _reset_cartographer_map()
            except Exception as e:
                print(f"[Settings] Map reset failed: {e}", flush=True)
        threading.Thread(target=_reset, daemon=True).start()

    def _on_fov_changed(self, _spin):
        self._state.detection.set_camera_fov(self._sp_cam_fov.get_value())
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_target_changed(self, _spin):
        self._state.detection.set_target_dims(
            self._sp_tgt_h.get_value() / 100.0,
            self._sp_tgt_w.get_value() / 100.0)
        self._schedule_save()

    def _on_calibrate(self, _button):
        _log("CALIBRATE clicked")
        _, detections, _, det_age = self._state.detection.get_camera()
        if not detections or det_age > 1.0:
            self._cal_result.set_text("No detection visible!")
            return
        best = max(detections, key=lambda d: (d['bbox'][3] - d['bbox'][1]))
        box_h = best['bbox'][3] - best['bbox'][1]
        if box_h <= 0:
            self._cal_result.set_text("Invalid bbox")
            return
        dist = self._cal_dist_spin.get_value()
        self._state.detection.set_calibration(float(box_h), dist)
        self._cal_label.set_text(f"Current: {box_h:.0f}px at {dist:.2f}m")
        self._manual_h_spin.set_value(box_h)
        self._manual_d_spin.set_value(dist)
        self._cal_result.set_text(f"OK: {box_h:.0f}px @ {dist:.2f}m")

    def _on_manual_cal(self, _button):
        box_h = self._manual_h_spin.get_value()
        dist = self._manual_d_spin.get_value()
        self._state.detection.set_calibration(float(box_h), dist)
        self._cal_label.set_text(f"Current: {box_h:.0f}px at {dist:.2f}m")
        self._cal_result.set_text(f"Applied: {box_h:.0f}px @ {dist:.2f}m")

    def _on_reset_defaults(self, _button):
        """Reset ALL parameters and update ALL UI widgets."""
        _log("RESET ALL DEFAULTS clicked")
        from bowling_target_nav.state.detection_store import (
            DEFAULT_ROBOT, DEFAULT_LIDAR, DEFAULT_LIDAR_YAW,
            DEFAULT_CAMERA, DEFAULT_TARGET,
            DEFAULT_CAMERA_FOV, REF_POINTS, DEFAULT_REF_POINT,
            DEFAULT_NAV_PARAMS, DEFAULT_MAP_PARAMS)

        # Update Setup tab spin buttons
        self._sp_robot_len.set_value(DEFAULT_ROBOT['length'] * 100)
        self._sp_robot_wid.set_value(DEFAULT_ROBOT['width'] * 100)
        self._sp_robot_hgt.set_value(DEFAULT_ROBOT['height'] * 100)
        self._sp_lidar_x.set_value(DEFAULT_LIDAR['x'] * 100)
        self._sp_lidar_y.set_value(DEFAULT_LIDAR['y'] * 100)
        self._sp_lidar_z.set_value(DEFAULT_LIDAR['z'] * 100)
        self._state.detection.set_lidar_yaw(DEFAULT_LIDAR_YAW)
        self._sp_cam_x.set_value(DEFAULT_CAMERA['x'] * 100)
        self._sp_cam_y.set_value(DEFAULT_CAMERA['y'] * 100)
        self._sp_cam_z.set_value(DEFAULT_CAMERA['z'] * 100)
        self._sp_cam_fov.set_value(DEFAULT_CAMERA_FOV)
        self._sp_tgt_h.set_value(DEFAULT_TARGET['height'] * 100)
        self._sp_tgt_w.set_value(DEFAULT_TARGET['width'] * 100)

        # Calibration
        self._state.detection.set_calibration(180.0, 1.0)
        self._cal_label.set_text("Current: 180px at 1.00m")
        self._manual_h_spin.set_value(180)
        self._manual_d_spin.set_value(1.0)

        # Reference point
        self._state.detection.set_ref_point(DEFAULT_REF_POINT)
        self._ref_combo.set_active(REF_POINTS.index(DEFAULT_REF_POINT))
        self._update_ref_info()

        # Nav params -> update store + sliders
        for k, v in DEFAULT_NAV_PARAMS.items():
            self._state.detection.set_nav_param(k, v)
            self._set_widget_value(f'nav.{k}', v)
        # Push to navigator in ROS process
        self._state.send_ros_command({'type': 'reset_nav_params'})

        # Map params -> update store + sliders
        for k, v in DEFAULT_MAP_PARAMS.items():
            self._state.detection.set_map_param(k, v)
            self._set_widget_value(f'map.{k}', v)

        # Detection params
        self._state.detection.set_confidence_threshold(0.30)
        self._set_widget_value('det.confidence', 0.30)
        self._state.detection.set_detect_expiry(1.0)
        self._set_widget_value('det.expiry', 1.0)

        self._schedule_save()
        self._setup_status.set_text("All reset to defaults")
        _log("All defaults restored")

    # ==================================================================
    # TOOLS PAGE
    # ==================================================================

    def _build_tools_page(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        box.pack_start(self._make_section("Motor Calibration"), False, False, 0)
        cal_info = Gtk.Label()
        cal_info.set_markup(
            "<span size='small' foreground='#888'>"
            "CALIBRATE: ~15s motor test. SYNC: reset controller. "
            "Place robot on blocks first.</span>")
        cal_info.set_xalign(0)
        cal_info.set_margin_start(12)
        cal_info.set_line_wrap(True)
        box.pack_start(cal_info, False, False, 2)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        cal_row.set_margin_start(12)
        cal_row.set_margin_end(12)
        cal_row.set_margin_top(4)
        for label, cmd, w in [("CALIBRATE", "CALIB", 120), ("SYNC", "SYNC", 80),
                               ("READ", "READ", 80), ("RESET ENC", "RESET", 100)]:
            btn = Gtk.Button(label=label)
            btn.set_size_request(w, 40)
            btn.connect('clicked', self._on_arduino_cmd, cmd)
            cal_row.pack_start(btn, False, False, 0)
        box.pack_start(cal_row, False, False, 2)

        self._arduino_status = Gtk.Label(label="")
        self._arduino_status.set_xalign(0)
        self._arduino_status.set_margin_start(12)
        box.pack_start(self._arduino_status, False, False, 4)

        box.pack_start(self._make_section("Motor Test"), False, False, 0)
        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        speed_row.set_margin_start(12)
        speed_row.pack_start(Gtk.Label(label="Speed (m/s):"), False, False, 0)
        self._test_speed = Gtk.SpinButton.new_with_range(0.05, 0.30, 0.01)
        self._test_speed.set_value(0.10)
        self._test_speed.set_digits(2)
        speed_row.pack_start(self._test_speed, False, False, 0)
        box.pack_start(speed_row, False, False, 4)

        btn_grid = Gtk.Grid()
        btn_grid.set_column_spacing(6)
        btn_grid.set_row_spacing(6)
        btn_grid.set_margin_start(12)
        btn_grid.set_margin_end(12)
        for label, row, col in [("FORWARD", 0, 0), ("BACKWARD", 0, 1),
                                 ("LEFT", 1, 0), ("RIGHT", 1, 1),
                                 ("TURN L", 2, 0), ("TURN R", 2, 1)]:
            btn = Gtk.Button(label=label)
            btn.set_size_request(120, 40)
            btn.connect('clicked', self._on_motor_test, label)
            btn_grid.attach(btn, col, row, 1, 1)
        box.pack_start(btn_grid, False, False, 4)

        self._motor_status = Gtk.Label(label="")
        self._motor_status.set_xalign(0)
        self._motor_status.set_margin_start(12)
        box.pack_start(self._motor_status, False, False, 4)

        box.pack_start(self._make_section("Odometry"), False, False, 0)
        odom_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        odom_row.set_margin_start(12)
        odom_btn = Gtk.Button(label="RESET ODOMETRY")
        odom_btn.set_size_request(160, 40)
        odom_btn.connect('clicked', self._on_reset_odom)
        odom_row.pack_start(odom_btn, False, False, 0)
        self._odom_status = Gtk.Label(label="")
        odom_row.pack_start(self._odom_status, True, True, 0)
        box.pack_start(odom_row, False, False, 4)

        return self._scrolled_page(box)

    # ---- Tool handlers ----

    def _on_arduino_cmd(self, _button, cmd):
        _log(f"Arduino: {cmd}")
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': cmd})
        if cmd == 'CALIB':
            self._arduino_status.set_markup(
                "<span foreground='#ffaa00'>Calibrating... ~15s</span>")
        else:
            self._arduino_status.set_text(f"Sent: {cmd}")

    def _on_motor_test(self, _button, direction):
        _log(f"Motor test: {direction}")
        speed = self._test_speed.get_value()
        direction_map = {
            "FORWARD": "forward", "BACKWARD": "backward",
            "LEFT": "left", "RIGHT": "right",
            "TURN L": "rotate_left", "TURN R": "rotate_right",
        }
        self._state.send_ros_command({
            'type': 'test_motor',
            'direction': direction_map.get(direction, ''),
            'speed': speed})
        self._motor_status.set_text(f"Testing {direction}...")
        if self._motor_test_timer:
            GLib.source_remove(self._motor_test_timer)
        def stop():
            self._state.send_ros_command({'type': 'stop_robot'})
            self._motor_status.set_text(f"{direction} done")
            self._motor_test_timer = None
            return False
        self._motor_test_timer = GLib.timeout_add(1000, stop)

    def _on_reset_odom(self, _button):
        _log("Reset odom")
        self._state.send_ros_command({'type': 'reset_odom'})
        self._odom_status.set_text("Odometry reset sent")

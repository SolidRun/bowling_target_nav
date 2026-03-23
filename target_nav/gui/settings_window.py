"""Settings window with parameter sliders and calibration tools.

Lazy-constructed on first SETTINGS click from MainGUI. Uses Gtk.Notebook
with top tabs for the V2N 1024x600 touchscreen. Each main tab may contain
sub-tabs (inner Gtk.Notebook) to split content into focused pages that
each fit without scrolling.

Tabs (5 main, with sub-tabs inside):
  - Navigate:  Sub-tabs: Speed, Target, Approach.
  - Search:    Sub-tabs: Scan, Spiral.
  - Sensors:   Sub-tabs: Detection, Calibration, Mounting.
  - Map:       Flat page (rotation, overlays, sizes).
  - Tools:     Sub-tabs: Drive (hold-to-move), Motors, System.

Design principles for 1024x600 embedded touchscreen:
  - No scrolling — each sub-page fits in ~440px height.
  - No expanders — everything visible flat on its sub-page.
  - Sub-tabs on top for quick switching between related settings.
  - Compact widgets with reduced margins.

All Scale/Switch widgets are tracked in _widgets dict so reset can
update their positions without rebuilding pages.  The _updating flag
suppresses value-changed callbacks during programmatic slider updates.

Tab-specific code lives in settings_tabs/ as mixin classes:
  NavTabMixin, DetectTabMixin, MapTabMixin, SetupTabMixin, ToolsTabMixin.
"""

import traceback

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from target_nav.gui.settings_tabs import (
    NavTabMixin, DetectTabMixin, MapTabMixin, SetupTabMixin, ToolsTabMixin)
from target_nav.utils.log import get_logger

logger = get_logger('gui.settings')


class SettingsWindow(NavTabMixin, DetectTabMixin, MapTabMixin,
                     SetupTabMixin, ToolsTabMixin, Gtk.Window):
    """Fullscreen settings window with Notebook tabs for runtime parameter tuning.

    Reads current values from shared_state on construction and writes changes
    back immediately via slider callbacks. SAVE ALL persists to calibration.json.

    Tabs: Navigate, Search, Sensors, Map, Tools.
    """

    def __init__(self, shared_state):
        """Initialize the settings window and build all notebook tabs.

        Args:
            shared_state: SharedState or IPCState facade for reading/writing
                navigation, detection, and sensor parameters.
        """
        logger.info("__init__ start")
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
            logger.info("Screen: %dx%d", geo.width, geo.height)
            self.set_default_size(geo.width, geo.height)
        else:
            logger.info("Screen: fallback 1024x600")
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
        header.set_margin_top(2)
        header.set_margin_bottom(2)

        back_btn = Gtk.Button(label="<  BACK")
        back_btn.set_size_request(100, 34)
        back_btn.connect('clicked', self._on_back)
        header.pack_start(back_btn, False, False, 0)

        title_label = Gtk.Label()
        title_label.set_markup(
            "<span font_weight='bold' size='14000' foreground='#58a6ff'>"
            "Robot Settings</span>")
        header.pack_start(title_label, True, True, 0)

        save_btn = Gtk.Button(label="SAVE ALL")
        save_btn.set_size_request(100, 34)
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
        self._notebook.set_tab_pos(Gtk.PositionType.TOP)
        self._notebook.set_scrollable(True)

        pages = [
            ("Navigate", self._build_nav_page),
            ("Search", self._build_search_page),
            ("Sensors", self._build_sensors_page),
            ("Radar", self._build_map_page),
            ("Tools", self._build_tools_page),
        ]

        for title, builder in pages:
            try:
                page = builder()
                tab_label = Gtk.Label(label=title)
                tab_label.set_size_request(-1, 30)
                self._notebook.append_page(page, tab_label)
                logger.info("Tab '%s' built OK", title)
            except Exception as e:
                logger.error("Tab '%s' FAILED: %s", title, e)
                traceback.print_exc()
                err = Gtk.Label(label=f"Error: {e}")
                self._notebook.append_page(err, Gtk.Label(label=title))

        vbox.pack_start(self._notebook, True, True, 0)
        logger.info("__init__ done")

    # ---- Window lifecycle ----

    def show_all(self):
        """Override to clear stale status message each time window is shown."""
        self._header_status.set_text("")
        super().show_all()

    def _on_back(self, _button):
        """Return to the main window -- stop any motor test and hide settings."""
        logger.info("BACK clicked")
        self._stop_motor_test()
        self.hide()
        if self._parent_window:
            self._parent_window.return_from_settings()

    def _on_delete(self, _widget, _event):
        """Intercept window close to hide instead of destroy."""
        self._on_back(None)
        return True

    def _on_key(self, _widget, event):
        """Handle Escape key to return to main window."""
        key = Gdk.keyval_name(event.keyval)
        if key and key.lower() == 'escape':
            self._on_back(None)
            return True
        return False

    # ---- Auto-save ----

    def _schedule_save(self):
        """Schedule an auto-save after 2 seconds, debounced (resets on repeat calls)."""
        if self._save_timer:
            GLib.source_remove(self._save_timer)
        self._save_timer = GLib.timeout_add(2000, self._do_save)

    def _do_save(self):
        """Execute the debounced auto-save (called by GLib timer)."""
        try:
            ok = self._state.detection.save_calibration()
            logger.info("Auto-saved: %s", 'OK' if ok else 'FAILED')
            if ok:
                self._notify_settings_changed()
        except Exception as e:
            logger.error("Auto-save error: %s", e)
        self._save_timer = None
        return False

    def _on_save_all(self, _button):
        """Persist all settings to calibration.json on the device."""
        logger.info("SAVE ALL clicked")
        try:
            # Log current values before saving
            nav_p = self._state.detection.get_nav_params()
            map_p = self._state.detection.get_map_params()
            logger.debug("  nav_params: %s", nav_p)
            logger.debug("  map_params: %s", map_p)
            ok = self._state.detection.save_calibration()
            if ok:
                self._notify_settings_changed()
                self._header_status.set_markup(
                    "<span foreground='#50fa7b'>Saved to device!</span>")
                logger.info("Save OK")
            else:
                self._header_status.set_markup(
                    "<span foreground='#ff6b6b'>Save failed!</span>")
                logger.warning("Save returned False")
        except Exception as e:
            logger.error("Save error: %s", e)
            self._header_status.set_markup(
                f"<span foreground='#ff6b6b'>Error: {e}</span>")

    # ---- Settings notification ----

    def _notify_settings_changed(self):
        """Publish /settings_changed so nav_node and camera_node reload calibration.

        In threading mode, nodes share the same SettingsStore instance so
        changes are immediately visible — this is a no-op.
        In multiprocess mode, each node has its own SettingsStore loaded from
        disk, so they need a signal to reload calibration.json.
        """
        pub = getattr(self._state, '_settings_pub', None)
        if pub is not None:
            try:
                from std_msgs.msg import String
                import json, time
                msg = String()
                msg.data = json.dumps({'timestamp': time.time()})
                pub.publish(msg)
                logger.info("Published /settings_changed")
            except Exception as e:
                logger.warning("Failed to publish /settings_changed: %s", e)

    # ---- Motor test cleanup ----

    def _stop_motor_test(self):
        """Cancel any running motor/drive timers and send a stop command."""
        # Cancel motor test timer
        if getattr(self, '_motor_test_timer', None) is not None:
            GLib.source_remove(self._motor_test_timer)
            self._motor_test_timer = None
        # Cancel drive timers (repeat + auto-stop)
        for attr in ('_drive_repeat_timer', '_drive_stop_timer'):
            timer_id = getattr(self, attr, None)
            if timer_id is not None:
                GLib.source_remove(timer_id)
                setattr(self, attr, None)
        self._state.send_ros_command({'type': 'stop_robot'})

    # ---- Widget helpers (shared by all tabs) ----

    def _make_sub_notebook(self, pages):
        """Create an inner Gtk.Notebook for sub-tabs inside a main tab.

        Args:
            pages: List of (title, Gtk.Widget) tuples.

        Returns:
            Gtk.Notebook with compact styling.
        """
        nb = Gtk.Notebook()
        nb.set_tab_pos(Gtk.PositionType.TOP)
        for title, widget in pages:
            tab_label = Gtk.Label(label=title)
            tab_label.set_size_request(-1, 26)
            nb.append_page(widget, tab_label)
        return nb

    def _make_section(self, text):
        """Create a styled section header label.

        Args:
            text: Section title text.

        Returns:
            Gtk.Label with blue bold Pango markup.
        """
        label = Gtk.Label()
        from gi.repository import GLib
        safe = GLib.markup_escape_text(text)
        label.set_markup(
            f"<span foreground='#58a6ff' font_weight='bold' size='12000'>"
            f"{safe}</span>")
        label.set_xalign(0)
        label.set_margin_start(10)
        label.set_margin_top(8)
        label.set_margin_bottom(4)
        return label

    def _make_slider_row(self, widget_key, label_text, min_val, max_val,
                         step, initial, callback):
        """Create a horizontal row with label + Gtk.Scale + value display.

        The Scale widget is registered in ``self._widgets[widget_key]``
        for programmatic reset. The ``_updating`` flag is checked in the
        change callback to suppress feedback loops.

        Args:
            widget_key: Key for the ``_widgets`` registry.
            label_text: Human-readable label (200px wide).
            min_val: Slider minimum.
            max_val: Slider maximum.
            step: Slider step increment.
            initial: Initial slider value.
            callback: Called with the new float value on user change.

        Returns:
            Gtk.Box row ready to pack into a page.
        """
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        row.set_margin_start(10)
        row.set_margin_end(10)
        row.set_margin_top(2)
        row.set_margin_bottom(2)

        label = Gtk.Label()
        label.set_markup(f"<span size='10000'>{label_text}</span>")
        label.set_xalign(0)
        label.set_size_request(140, -1)
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
        """Create a horizontal row with label + Gtk.Switch toggle.

        Args:
            widget_key: Key for the ``_widgets`` registry.
            label_text: Human-readable label (200px wide).
            initial: Initial boolean state.
            callback: Called with True/False on user toggle.

        Returns:
            Gtk.Box row ready to pack into a page.
        """
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        row.set_margin_start(10)
        row.set_margin_top(2)
        lbl = Gtk.Label()
        lbl.set_markup(f"<span size='10000'>{label_text}</span>")
        lbl.set_xalign(0)
        lbl.set_size_request(140, -1)
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
        """Create a horizontal row with label + Gtk.SpinButton + optional unit label.

        Args:
            label_text: Label shown to the left.
            min_v: Minimum spin value.
            max_v: Maximum spin value.
            step: Step increment.
            default: Initial value.
            digits: Decimal places to display.
            unit: Optional unit suffix (e.g. "cm", "deg").

        Returns:
            Tuple of (Gtk.Box row, Gtk.SpinButton).
        """
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        row.set_margin_start(10)
        row.set_margin_top(2)
        lbl = Gtk.Label()
        lbl.set_markup(f"<span size='10000'>{label_text}</span>")
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
        """Wrap a content box in a ScrolledWindow (fallback for complex pages).

        Args:
            box: Gtk.Box containing the page content.

        Returns:
            Gtk.ScrolledWindow ready to add as a Notebook page.
        """
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.add(box)
        return scroll

    def _make_angle_row(self, widget_key, label_text, initial, callback):
        """Create an angle input with SpinButton (-360..360) and preset buttons.

        Preset buttons (0, +/-45, +/-90, +/-135, +/-180) set the spin
        value with a single tap -- useful on the V2N touchscreen.

        Args:
            widget_key: Key for the ``_widgets`` registry.
            label_text: Human-readable label.
            initial: Initial angle in degrees.
            callback: Called with the new float angle on change.

        Returns:
            Gtk.Box with top (label+spin) and bottom (presets) rows.
        """
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

    # ---- Per-page reset button ----

    def _make_reset_row(self, callback):
        """Create a RESET DEFAULTS button row with status feedback label.

        Args:
            callback: Zero-argument callable that resets the page's parameters.

        Returns:
            Gtk.Box row with button and status label.
        """
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_margin_start(12)
        row.set_margin_end(12)
        row.set_margin_top(14)
        row.set_margin_bottom(6)
        btn = Gtk.Button(label="RESET DEFAULTS")
        btn.set_size_request(160, 36)
        status = Gtk.Label(label="")
        status.set_xalign(0)

        def on_click(_b):
            callback()
            status.set_markup(
                "<span foreground='#50fa7b'>Reset to defaults</span>")
            self._schedule_save()

        btn.connect('clicked', on_click)
        row.pack_start(btn, False, False, 0)
        row.pack_start(status, True, True, 8)
        return row

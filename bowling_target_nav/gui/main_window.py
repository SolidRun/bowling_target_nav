"""Main fullscreen GTK3 window for V2N Robot Control GUI.

Provides the MainGUI class which renders a split-panel display:
  - Left panel: SLAM map with robot pose, laser scan, and navigation target
  - Right panel: Camera feed with detection overlays

The window includes a bottom control bar with GO/STOP buttons, a live
navigation status label (Pango markup), and buttons for settings and quit.
Rendering is delegated to panel functions in gui/panels/. All robot state
is accessed through the shared_state facade (works in both threading and
multiprocess modes).

Keyboard shortcuts: G = go, S/Space = stop, Q/Escape = quit.
Refresh rate: ~30 FPS via GLib.timeout_add.
"""

import math
import sys
import traceback

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from .panels.map_panel import draw_map_panel
from .panels.camera_panel import draw_camera_panel

_LOG_TAG = "[MainWindow]"


def _log(msg):
    print(f"{_LOG_TAG} {msg}", flush=True)


class MainGUI(Gtk.Window):
    """Fullscreen GUI with SLAM map, camera panel, and control buttons.

    All rendering delegates to panel functions. All state access goes
    through the shared_state facade.
    """

    def __init__(self, shared_state):
        """Initialize the fullscreen main window.

        Args:
            shared_state: SharedState or IPCState facade providing access to
                navigation, detection, and sensor data across threads/processes.
        """
        _log("__init__ start")
        from bowling_target_nav.core.config import get_config
        self._display_name = get_config().detection.target.display_name
        super().__init__(title=f"V2N {self._display_name} Control")
        self._state = shared_state
        self._settings_window = None
        self._quitting = False
        self._timer_id = None
        self.fullscreen()

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(vbox)

        # Drawing area for map + camera panels
        self.da = Gtk.DrawingArea()
        self.da.connect('draw', self.on_draw)
        vbox.pack_start(self.da, True, True, 0)

        # Control bar
        control_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        control_box.set_margin_start(20)
        control_box.set_margin_end(20)
        control_box.set_margin_top(10)
        control_box.set_margin_bottom(10)

        # GO button
        self.go_btn = Gtk.Button(label="GO TO TARGET")
        self.go_btn.set_size_request(200, 60)
        self.go_btn.get_style_context().add_class('suggested-action')
        self.go_btn.connect('clicked', self._on_go)
        control_box.pack_start(self.go_btn, False, False, 0)

        # STOP button
        self.stop_btn = Gtk.Button(label="STOP")
        self.stop_btn.set_size_request(200, 60)
        self.stop_btn.get_style_context().add_class('destructive-action')
        self.stop_btn.connect('clicked', self._on_stop)
        control_box.pack_start(self.stop_btn, False, False, 0)

        # Status label
        self.status_label = Gtk.Label(label="Status: IDLE")
        self.status_label.set_xalign(0)
        self.status_label.get_style_context().add_class('status-label')
        control_box.pack_start(self.status_label, True, True, 20)

        # Settings button
        settings_btn = Gtk.Button(label="SETTINGS")
        settings_btn.set_size_request(120, 60)
        settings_btn.get_style_context().add_class('settings-btn')
        settings_btn.connect('clicked', self._on_settings)
        control_box.pack_end(settings_btn, False, False, 0)

        # Quit button
        quit_btn = Gtk.Button(label="QUIT")
        quit_btn.set_size_request(100, 60)
        quit_btn.get_style_context().add_class('quit-btn')
        quit_btn.connect('clicked', self.on_quit)
        control_box.pack_end(quit_btn, False, False, 0)

        vbox.pack_start(control_box, False, False, 0)

        self.connect('key-press-event', self._on_key)
        self.connect('destroy', self.on_quit)

        # 30 FPS refresh
        self._timer_id = GLib.timeout_add(33, self._on_tick)
        _log("__init__ done")

    # ---- Lazy settings window creation ----

    def _ensure_settings_window(self):
        """Create the settings window on first use (not during __init__)."""
        if self._settings_window is not None:
            return True
        try:
            _log("Creating SettingsWindow (lazy)...")
            from .settings_window import SettingsWindow
            self._settings_window = SettingsWindow(self._state)
            _log("SettingsWindow created OK")
            return True
        except Exception as e:
            _log(f"FAILED to create SettingsWindow: {e}")
            traceback.print_exc()
            return False

    # ---- Button handlers ----

    def _on_go(self, button):
        """Handle GO button click -- request navigation start."""
        _log("GO clicked")
        self._state.nav.request_go()

    def _on_stop(self, button):
        """Handle STOP button click -- request navigation halt."""
        _log("STOP clicked")
        self._state.nav.request_stop()

    def _on_settings(self, button):
        """Open the settings window, hiding the main window while it is shown."""
        _log("SETTINGS clicked")
        try:
            if not self._ensure_settings_window():
                _log("Cannot open settings - creation failed")
                return

            self._settings_window._parent_window = self
            _log("Hiding main window...")
            self.hide()
            _log("Main window hidden. Showing settings...")
            self._settings_window.show_all()
            _log("Settings show_all done. Fullscreening...")
            self._settings_window.fullscreen()
            _log("Settings window opened OK")
        except Exception as e:
            _log(f"Settings error: {e}")
            traceback.print_exc()
            # Recover: show main window again
            self.show_all()
            self.fullscreen()

    def return_from_settings(self):
        """Called by SettingsWindow to return to this window."""
        _log("Returning from settings")
        if self._settings_window:
            self._settings_window.hide()
        self.show_all()
        self.fullscreen()
        _log("Main window restored")

    # ---- Tick / draw ----

    def _on_tick(self):
        """30 FPS timer callback -- update status label and request a redraw.

        Returns:
            True to keep the timer running, False to stop it.
        """
        if self._quitting:
            return False
        if not self._state.running:
            self.on_quit(None)
            return False

        self._update_status()
        self.da.queue_draw()
        return True

    def _update_status(self):
        """Update the status bar label with current navigation state.

        Reads a snapshot from the nav state facade and renders Pango markup
        showing nav state, target distance, speed, obstacle warnings, and
        search timers.
        """
        try:
            snap = self._state.nav.get_gui_snapshot()
        except Exception as e:
            _log(f"get_gui_snapshot error: {e}")
            return

        nav_state = snap.get('nav_state', 'UNKNOWN')
        nav_target = snap.get('nav_target')
        time_since_target = snap.get('time_since_target', 999.0)
        search_time = snap.get('search_time', 0.0)
        cmd_vel = snap.get('cmd_vel', (0.0, 0.0, 0.0))
        vx, vy, wz = cmd_vel
        speed = math.sqrt(vx * vx + vy * vy)

        obs_str = (f" <span foreground='#ff6b6b' weight='bold'>\u26a0 OBS {snap.get('obstacle_dist', 999):.2f}m</span>"
                   if snap.get('obstacle_ahead') else "")
        spd_str = (f" <span foreground='#79c0ff'>\u2192 {speed:.2f} m/s</span>"
                   if speed > 0.01 else "")

        if nav_state == "SEARCHING":
            markup = (f"<span foreground='#f0d050' weight='bold' size='large'>"
                      f"\u25cf SEARCHING</span>  "
                      f"<span foreground='#b0b8c2'>{search_time:.0f}s elapsed</span>"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{obs_str}")
        elif nav_state == "SPIRAL_SEARCH":
            markup = (f"<span foreground='#e09040' weight='bold' size='large'>"
                      f"\u25cf SPIRAL SEARCH</span>  "
                      f"<span foreground='#b0b8c2'>{search_time:.0f}s elapsed</span>"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{spd_str}{obs_str}")
        elif nav_state == "BLIND_APPROACH":
            markup = (f"<span foreground='#ffb347' weight='bold' size='large'>"
                      f"\u25cf BLIND APPROACH</span>  "
                      f"<span foreground='#b0b8c2'>Dead-reckoning</span>"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{spd_str}{obs_str}")
        elif nav_state == "NAVIGATING" and nav_target:
            markup = (f"<span foreground='#50fa7b' weight='bold' size='large'>"
                      f"\u25cf NAVIGATING</span>  "
                      f"<span foreground='#e0e6ed'>Target: "
                      f"<b>{nav_target[2]:.2f}m</b></span>"
                      f"{spd_str}{obs_str}")
        elif nav_state == "ARRIVED":
            markup = (f"<span foreground='#69b4ff' weight='bold' size='large'>"
                      f"\u2714 ARRIVED</span>  "
                      f"<span foreground='#c0c8d2'>at target!</span>")
        elif nav_state == "ERROR":
            markup = (f"<span foreground='#ff6b6b' weight='bold' size='large'>"
                      f"\u2716 ERROR</span>")
        else:
            markup = (f"<span foreground='#6e7681' size='large'>"
                      f"\u25cb IDLE</span>  "
                      f"<span foreground='#8899aa'>Press <b>GO</b> to start</span>")

        self.status_label.set_markup(markup)

    def on_draw(self, widget, cr):
        """Cairo draw handler -- render title bar then delegate to panel renderers.

        Args:
            widget: The DrawingArea widget.
            cr: Cairo context for drawing.
        """
        try:
            alloc = widget.get_allocation()
            W, H = alloc.width, alloc.height

            # Background
            cr.set_source_rgb(0.05, 0.07, 0.09)
            cr.paint()

            margin = 10
            title_h = 50
            panel_w = (W - margin * 3) // 2
            panel_h = H - title_h - margin

            map_x, map_y = margin, title_h
            cam_x, cam_y = margin * 2 + panel_w, title_h

            # Title bar
            cr.set_source_rgb(0.345, 0.651, 1.0)
            cr.set_font_size(20)
            cr.move_to(margin, 32)
            cr.show_text(f"V2N {self._display_name} Control")

            cr.set_font_size(14)
            det_mode = self._state.detection.get_detector_mode()
            if "DRP-AI" in det_mode:
                cr.set_source_rgb(0.0, 0.8, 0.3)
                subtitle = f"SLAM + {det_mode} + Navigation"
            elif "ONNX" in det_mode:
                cr.set_source_rgb(0.85, 0.7, 0.0)
                subtitle = f"SLAM + {det_mode} + Navigation"
            else:
                cr.set_source_rgb(0.55, 0.59, 0.63)
                subtitle = "SLAM + Camera + Navigation"
            cr.move_to(margin + 280, 36)
            cr.show_text(subtitle)

            # Delegate to panel renderers
            draw_map_panel(cr, map_x, map_y, panel_w, panel_h, self._state)
            draw_camera_panel(cr, cam_x, cam_y, panel_w, panel_h, self._state)

        except Exception as e:
            _log(f"Draw error: {e}")

        return False

    # ---- Keyboard ----

    def _on_key(self, widget, event):
        """Handle keyboard shortcuts (G=go, S/Space=stop, Q/Escape=quit)."""
        key = Gdk.keyval_name(event.keyval)
        if key is None:
            return False
        key = key.lower()
        if key in ('q', 'escape'):
            self.on_quit(None)
        elif key == 'g':
            self._state.nav.request_go()
        elif key in ('s', 'space'):
            self._state.nav.request_stop()
        return True

    # ---- Quit ----

    def on_quit(self, widget):
        """Shut down the application: stop state, destroy settings, remove timer."""
        if self._quitting:
            return
        self._quitting = True
        _log("Quit requested")
        self._state.request_shutdown()
        # Cleanup settings window
        if self._settings_window:
            try:
                self._settings_window.destroy()
            except Exception:
                pass
            self._settings_window = None
        # Remove timer safely
        if self._timer_id is not None:
            try:
                GLib.source_remove(self._timer_id)
            except Exception:
                pass
            self._timer_id = None
        Gtk.main_quit()

"""Main fullscreen GTK3 window for V2N Robot Control GUI.

Provides the MainGUI class which renders a split-panel display:
  - Left panel: LiDAR map with robot pose, laser scan, and navigation target
  - Right panel: Camera feed with detection overlays

The window includes a bottom control bar with GO/STOP buttons, a live
navigation status label (Pango markup), and buttons for settings and quit.
Rendering is delegated to panel functions in ``gui/panels/``. All robot
state is accessed through the shared_state facade (works in both
SharedState populated by a ROS2 bridge node).

Runs in: GUI process (Process 0) on the GTK main thread.

Keyboard shortcuts: G = go, S/Space = stop, Q/Escape = quit.
Refresh rate: 10 FPS via GLib.timeout_add(100ms) — saves CPU on embedded.

Key class:
    MainGUI -- the fullscreen Gtk.Window.
"""

import math
import traceback

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from .panels.map_panel import draw_map_panel
from .panels.camera_panel import draw_camera_panel
from target_nav.utils.log import get_logger

logger = get_logger('gui.main')


class MainGUI(Gtk.Window):
    """Fullscreen GUI with LiDAR map, camera panel, and control buttons.

    All rendering delegates to panel functions. All state access goes
    through the shared_state facade.
    """

    def __init__(self, shared_state):
        """Initialize the fullscreen main window.

        Args:
            shared_state: SharedState or IPCState facade providing access to
                navigation, detection, and sensor data across threads/processes.
        """
        logger.info("__init__ start")
        from target_nav.config import TARGET_DISPLAY_NAME
        self._display_name = TARGET_DISPLAY_NAME
        super().__init__(title=f"V2N {self._display_name} Control")
        self._state = shared_state
        self._settings_window = None
        self._in_settings = False
        self._quitting = False
        self._timer_id = None
        self._draw_count = 0
        self.set_default_size(1024, 600)
        self.fullscreen()

        # Stack for switching between main view and settings
        self._stack = Gtk.Stack()
        self._stack.set_transition_type(Gtk.StackTransitionType.NONE)
        self.add(self._stack)

        # --- Main view (radar + camera + controls) ---
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        self.da = Gtk.DrawingArea()
        self.da.connect('draw', self.on_draw)
        self.da.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.da.connect('button-press-event', self._on_da_click)
        # DA expands to fill available space ABOVE the control bar
        main_box.pack_start(self.da, True, True, 0)

        # Control bar — fixed height, always visible at bottom.
        # pack_start with expand=False ensures it doesn't get pushed off-screen.
        control_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        control_box.set_margin_start(20)
        control_box.set_margin_end(20)
        control_box.set_margin_top(5)
        control_box.set_margin_bottom(5)
        control_box.set_size_request(-1, 50)  # minimum 50px height

        self.go_btn = Gtk.Button(label="GO TO TARGET")
        self.go_btn.set_size_request(180, 45)
        self.go_btn.get_style_context().add_class('suggested-action')
        self.go_btn.connect('clicked', self._on_go)
        control_box.pack_start(self.go_btn, False, False, 0)

        self.stop_btn = Gtk.Button(label="STOP")
        self.stop_btn.set_size_request(180, 45)
        self.stop_btn.get_style_context().add_class('destructive-action')
        self.stop_btn.connect('clicked', self._on_stop)
        control_box.pack_start(self.stop_btn, False, False, 0)

        self.capture_btn = Gtk.Button(label="CAP")
        self.capture_btn.set_size_request(60, 45)
        self.capture_btn.connect('clicked', self._on_capture)
        control_box.pack_start(self.capture_btn, False, False, 0)

        self.status_label = Gtk.Label(label="Status: IDLE")
        self.status_label.set_xalign(0)
        self.status_label.get_style_context().add_class('status-label')
        control_box.pack_start(self.status_label, True, True, 20)

        settings_btn = Gtk.Button(label="SETTINGS")
        settings_btn.set_size_request(120, 45)
        settings_btn.get_style_context().add_class('settings-btn')
        settings_btn.connect('clicked', self._on_settings)
        control_box.pack_end(settings_btn, False, False, 0)

        quit_btn = Gtk.Button(label="QUIT")
        quit_btn.set_size_request(100, 45)
        quit_btn.get_style_context().add_class('quit-btn')
        quit_btn.connect('clicked', self.on_quit)
        control_box.pack_end(quit_btn, False, False, 0)

        main_box.pack_start(control_box, False, False, 0)
        self._stack.add_named(main_box, "main")

        self.connect('key-press-event', self._on_key)
        self.connect('destroy', self.on_quit)

        # 15 FPS refresh — smooth radar view while saving CPU on embedded
        # Single timer at 100ms (10 FPS). SHM polling is done by external
        # _gui_poll_shm callback added in main_multiprocess(). Only ONE timer
        # runs in GTK main loop to leave time for input event processing.
        self._timer_id = GLib.timeout_add(100, self._on_tick)
        logger.info("__init__ done")

    # ---- Lazy settings window creation ----

    def _ensure_settings_window(self):
        """Create the settings window on first use (not during __init__).

        Returns:
            True if the settings window is available, False on error.
        """
        if self._settings_window is not None:
            return True
        try:
            logger.info("Creating SettingsWindow (lazy)...")
            from .settings_window import SettingsWindow
            self._settings_window = SettingsWindow(self._state)
            logger.info("SettingsWindow created OK")
            return True
        except Exception as e:
            logger.error("FAILED to create SettingsWindow: %s", e)
            traceback.print_exc()
            return False

    # ---- Button handlers ----

    def _on_capture(self, button):
        """Capture raw camera frame for model training. Saves to /root/captures/.

        Button turns green on success, red on failure, then resets after 1.5s.
        """
        import os
        import time as _time

        def _reset_btn():
            self.capture_btn.set_label("CAP")
            ctx = self.capture_btn.get_style_context()
            ctx.remove_class('suggested-action')
            ctx.remove_class('destructive-action')
            return False  # don't repeat

        frame, _, _, _ = self._state.detection.get_camera()
        if frame is None:
            self.capture_btn.set_label("FAIL")
            self.capture_btn.get_style_context().add_class('destructive-action')
            GLib.timeout_add(1500, _reset_btn)
            return

        capture_dir = '/root/captures'
        os.makedirs(capture_dir, exist_ok=True)
        timestamp = _time.strftime('%Y%m%d_%H%M%S')
        filename = f"{capture_dir}/cap_{timestamp}.png"
        try:
            import struct as _struct
            import zlib as _zlib
            h, w = frame.shape[:2]
            rgb = frame[:, :, ::-1].copy()
            raw = b''
            for r in range(h):
                raw += b'\x00' + rgb[r].tobytes()
            comp = _zlib.compress(raw)
            def _c(ct, d):
                c = ct + d
                return _struct.pack('>I', len(d)) + c + _struct.pack('>I', _zlib.crc32(c) & 0xFFFFFFFF)
            with open(filename, 'wb') as f:
                f.write(b'\x89PNG\r\n\x1a\n')
                f.write(_c(b'IHDR', _struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
                f.write(_c(b'IDAT', comp))
                f.write(_c(b'IEND', b''))
            self.capture_btn.set_label("OK")
            self.capture_btn.get_style_context().add_class('suggested-action')
            GLib.timeout_add(1500, _reset_btn)
        except Exception as e:
            self.capture_btn.set_label("FAIL")
            self.capture_btn.get_style_context().add_class('destructive-action')
            GLib.timeout_add(1500, _reset_btn)

    def _on_go(self, button):
        """Handle GO button click -- request navigation start."""
        logger.info("GO clicked")
        self._state.send_ros_command({'type': 'go'})

    def _on_stop(self, button):
        """Handle STOP button click -- request navigation halt."""
        logger.info("STOP clicked")
        self._state.send_ros_command({'type': 'stop'})

    def _on_settings(self, button):
        """Switch to settings view using Gtk.Stack (same window, no crash)."""
        logger.info("SETTINGS clicked")
        try:
            if not self._ensure_settings_window():
                logger.warning("Cannot open settings - creation failed")
                return

            self._settings_window._parent_window = self

            # Add settings content to stack if not already there
            if self._stack.get_child_by_name("settings") is None:
                settings_content = self._settings_window.get_child()
                if settings_content:
                    old_parent = settings_content.get_parent()
                    if old_parent:
                        old_parent.remove(settings_content)
                    self._stack.add_named(settings_content, "settings")

            self._stack.set_visible_child_name("settings")
            self._stack.get_child_by_name("settings").show_all()
            self._in_settings = True
            logger.info("Settings opened")
        except Exception as e:
            logger.error("Settings error: %s", e)
            traceback.print_exc()

    def return_from_settings(self):
        """Switch back to main view."""
        logger.info("Returning from settings")
        self._stack.set_visible_child_name("main")
        self._in_settings = False
        logger.info("Main view restored")

    # ---- Tick / draw ----

    def _on_tick(self):
        """10 FPS timer callback -- update status label and request a redraw.

        Called every 100ms via GLib.timeout_add. Checks if the application
        is still running, updates the status bar Pango markup, and queues
        a GTK redraw of the DrawingArea (which triggers on_draw).

        Returns:
            True to keep the timer running, False to stop it.
        """
        if self._quitting:
            return False
        if not self._state.running:
            self.on_quit(None)
            return False

        if not self._in_settings:
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
            logger.error("get_gui_snapshot error: %s", e)
            return

        nav_state = snap.get('nav_state', 'UNKNOWN')
        nav_target = snap.get('nav_target')
        time_since_target = snap.get('time_since_target', 999.0)
        search_time = snap.get('search_time', 0.0)
        cmd_vel = snap.get('cmd_vel', (0.0, 0.0, 0.0))
        vx, vy, wz = cmd_vel
        speed = math.sqrt(vx * vx + vy * vy)

        obs_str = (f" <span foreground='#ff6b6b' weight='bold'>! OBS {snap.get('obstacle_dist', 999):.2f}m</span>"
                   if snap.get('obstacle_ahead') else "")
        spd_str = (f" <span foreground='#79c0ff'> {speed:.2f} m/s</span>"
                   if speed > 0.01 else "")

        # Odom-frame distance from robot to target
        map_dist_str = ""
        nav_target_map = snap.get('nav_target_map')
        if nav_target_map is not None:
            rx, ry, _ = self._state.sensors.get_robot_pose()
            dx = nav_target_map[0] - rx
            dy = nav_target_map[1] - ry
            map_d = math.sqrt(dx * dx + dy * dy)
            color = '#50fa7b' if map_d < 0.20 else '#f0d050' if map_d < 0.50 else '#79c0ff'
            map_dist_str = f"  <span foreground='{color}'><b>Dist: {map_d:.2f}m</b></span>"

        if nav_state == "SEARCHING":
            markup = (f"<span foreground='#f0d050' weight='bold' size='large'>"
                      f"SEARCHING</span>  "
                      f"<span foreground='#b0b8c2'>{search_time:.0f}s elapsed</span>"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{map_dist_str}{obs_str}")
        elif nav_state == "SPIRAL_SEARCH":
            markup = (f"<span foreground='#e09040' weight='bold' size='large'>"
                      f"SPIRAL SEARCH</span>  "
                      f"<span foreground='#b0b8c2'>{search_time:.0f}s elapsed</span>"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{map_dist_str}{spd_str}{obs_str}")
        elif nav_state == "BLIND_APPROACH":
            markup = (f"<span foreground='#ffb347' weight='bold' size='large'>"
                      f"BLIND APPROACH</span>  "
                      f"<span foreground='#b0b8c2'>Dead-reckoning</span>"
                      f"{map_dist_str}"
                      f"  <span foreground='#8899aa'>Lost {time_since_target:.0f}s</span>"
                      f"{spd_str}{obs_str}")
        elif nav_state == "NAVIGATING" and nav_target:
            markup = (f"<span foreground='#50fa7b' weight='bold' size='large'>"
                      f"NAVIGATING</span>  "
                      f"<span foreground='#e0e6ed'>Sensor: "
                      f"<b>{nav_target[2]:.2f}m</b></span>"
                      f"{map_dist_str}{spd_str}{obs_str}")
        elif nav_state == "ARRIVED":
            markup = (f"<span foreground='#69b4ff' weight='bold' size='large'>"
                      f"ARRIVED</span>  "
                      f"<span foreground='#c0c8d2'>at target!</span>"
                      f"{map_dist_str}")
        elif nav_state == "ERROR":
            markup = (f"<span foreground='#ff6b6b' weight='bold' size='large'>"
                      f"ERROR</span>")
        else:
            markup = (f"<span foreground='#6e7681' size='large'>"
                      f"IDLE</span>  "
                      f"<span foreground='#8899aa'>Press <b>GO</b> to start</span>")

        self.status_label.set_markup(markup)

    def on_draw(self, widget, cr):
        """Cairo draw handler -- render title bar then delegate to panel renderers.

        Layout: the DrawingArea is split 50/50 horizontally with a 10px margin
        between panels. Left half = LiDAR radar (draw_map_panel), right half =
        camera feed (draw_camera_panel). A 50px title bar sits above both panels.

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
                subtitle = f"{det_mode} + LiDAR + Navigation"
            else:
                cr.set_source_rgb(0.55, 0.59, 0.63)
                subtitle = "DRP-AI + LiDAR + Navigation"
            cr.move_to(margin + 280, 36)
            cr.show_text(subtitle)

            # Delegate to panel renderers
            draw_map_panel(cr, map_x, map_y, panel_w, panel_h, self._state)
            draw_camera_panel(cr, cam_x, cam_y, panel_w, panel_h, self._state)
            self._draw_count += 1

        except Exception as e:
            logger.error("Draw error: %s", e)

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
            self._state.send_ros_command({'type': 'go'})
        elif key in ('s', 'space'):
            self._state.send_ros_command({'type': 'stop'})
        elif key in ('plus', 'equal', 'kp_add'):
            self._zoom_radar(-1)  # zoom in = reduce range
        elif key in ('minus', 'kp_subtract'):
            self._zoom_radar(1)   # zoom out = increase range
        return True

    def _on_da_click(self, widget, event):
        """Handle clicks on the drawing area (radar zoom buttons)."""
        w = widget.get_allocated_width()
        h = widget.get_allocated_height()
        margin = 20
        panel_w = (w - margin * 3) // 2
        # Zoom buttons are in top-right of left panel (radar)
        btn_size = 28
        btn_x = margin + panel_w - btn_size - 8
        btn_y1 = 85  # matches map_panel content_y + 4
        btn_y2 = btn_y1 + btn_size + 4
        ex, ey = event.x, event.y
        if btn_x <= ex <= btn_x + btn_size:
            if btn_y1 <= ey <= btn_y1 + btn_size:
                self._zoom_radar(-1)  # + = zoom in
            elif btn_y2 <= ey <= btn_y2 + btn_size:
                self._zoom_radar(1)   # - = zoom out

    def _zoom_radar(self, direction):
        """Adjust radar range. direction: -1 = zoom in, +1 = zoom out."""
        current = self._state.detection.get_map_param('radar_range')
        if current is None:
            current = 5
        steps = [1, 2, 3, 5, 8, 10, 15, 20]
        try:
            idx = steps.index(current)
        except ValueError:
            idx = 3  # default to 5m
        idx = max(0, min(len(steps) - 1, idx + direction))
        self._state.detection.set_map_param('radar_range', steps[idx])

    # ---- Quit ----

    def on_quit(self, widget):
        """Clean shutdown: stop motors, kill processes, release resources."""
        if self._quitting:
            return
        self._quitting = True
        logger.info("Quit requested — stopping motors and cleaning up")

        # 1. STOP MOTORS IMMEDIATELY (before anything else)
        try:
            self._state.send_ros_command({'type': 'stop_robot'})
            self._state.send_ros_command({'type': 'arduino_cmd', 'data': 'STOP'})
        except Exception:
            pass

        # 2. Signal all threads/processes to stop
        self._state.request_shutdown()

        # 3. Cleanup settings window
        if self._settings_window:
            try:
                self._settings_window.destroy()
            except Exception:
                pass
            self._settings_window = None

        # 4. Remove GTK timer
        if self._timer_id is not None:
            try:
                GLib.source_remove(self._timer_id)
            except Exception:
                pass
            self._timer_id = None

        # 5. Exit GTK main loop → triggers finally block in main_multiprocess
        #    which handles: SHM poll thread join, SHM cleanup, child process kill
        Gtk.main_quit()

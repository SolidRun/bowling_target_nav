"""Main fullscreen GTK3 window for V2N Robot Control GUI."""

import math

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from .settings_window import SettingsWindow
from .panels.map_panel import draw_map_panel
from .panels.camera_panel import draw_camera_panel


class MainGUI(Gtk.Window):
    """Fullscreen GUI with SLAM map, camera panel, and control buttons.

    All rendering delegates to panel functions. All state access goes
    through the shared_state facade.
    """

    def __init__(self, shared_state):
        super().__init__(title="V2N Robot Control")
        self._state = shared_state
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
        self.settings_window = SettingsWindow(shared_state)
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

    # ---- Button handlers ----

    def _on_go(self, button):
        self._state.nav.request_go()

    def _on_stop(self, button):
        self._state.nav.request_stop()

    def _on_settings(self, button):
        self.settings_window.show_all()
        self.settings_window.present()

    # ---- Tick / draw ----

    def _on_tick(self):
        if not self._state.running:
            self.on_quit(None)
            return False

        self._update_status()
        self.da.queue_draw()
        return True

    def _update_status(self):
        # Single lock acquisition instead of 6 separate ones
        snap = self._state.nav.get_gui_snapshot()
        nav_state = snap['nav_state']
        nav_target = snap['nav_target']
        time_since_target = snap['time_since_target']
        search_time = snap['search_time']
        vx, vy, wz = snap['cmd_vel']
        speed = math.sqrt(vx * vx + vy * vy)

        obs_str = (f" | <span foreground='#f85149'>OBS {snap['obstacle_dist']:.2f}m</span>"
                   if snap['obstacle_ahead'] else "")
        spd_str = f" | {speed:.2f} m/s" if speed > 0.01 else ""

        if nav_state == "SEARCHING":
            markup = (f"<span foreground='#e3b341'>\u25cf SEARCHING</span> "
                      f"({search_time:.0f}s) | Lost {time_since_target:.0f}s{obs_str}")
        elif nav_state == "BLIND_APPROACH":
            markup = (f"<span foreground='#ffa657'>\u25cf BLIND APPROACH</span> "
                      f"| Dead-reckoning | Lost {time_since_target:.0f}s{spd_str}{obs_str}")
        elif nav_state == "NAVIGATING" and nav_target:
            markup = (f"<span foreground='#3fb950'>\u25cf NAVIGATING</span> "
                      f"| Target: {nav_target[2]:.2f}m{spd_str}{obs_str}")
        elif nav_state == "ARRIVED":
            markup = "<span foreground='#58a6ff'>\u25cf ARRIVED</span> at target!"
        elif nav_state == "ERROR":
            markup = "<span foreground='#f85149'>\u25cf ERROR</span>"
        else:
            markup = "<span foreground='#8b949e'>\u25cb IDLE</span> | Press GO to start"

        self.status_label.set_markup(markup)

    def on_draw(self, widget, cr):
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
            cr.set_font_size(26)
            cr.move_to(margin, 36)
            cr.show_text("V2N Robot Control")

            cr.set_source_rgb(0.55, 0.59, 0.63)
            cr.set_font_size(14)
            cr.move_to(margin + 280, 36)
            cr.show_text("SLAM + Camera + Navigation")

            # Delegate to panel renderers
            draw_map_panel(cr, map_x, map_y, panel_w, panel_h, self._state)
            draw_camera_panel(cr, cam_x, cam_y, panel_w, panel_h, self._state)

        except Exception as e:
            print(f"[GUI] Draw error: {e}", flush=True)

        return False

    # ---- Keyboard ----

    def _on_key(self, widget, event):
        key = Gdk.keyval_name(event.keyval).lower()
        if key in ('q', 'escape'):
            self.on_quit(None)
        elif key == 'g':
            self._state.nav.request_go()
        elif key in ('s', 'space'):
            self._state.nav.request_stop()
        return True

    # ---- Quit ----

    def on_quit(self, widget):
        print("[GUI] Quit requested", flush=True)
        self._state.request_shutdown()
        if hasattr(self, '_timer_id'):
            GLib.source_remove(self._timer_id)
        Gtk.main_quit()

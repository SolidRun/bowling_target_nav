#!/usr/bin/env python3
"""Floating desktop launcher for Bowling Target Nav GUI."""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk, Pango
import subprocess
import os
import signal

CSS = b"""
window {
    background: linear-gradient(180deg, #1a2332, #0f1720);
    border-radius: 18px;
}
.title-label {
    color: #e8ecf1;
    font-size: 22px;
    font-weight: bold;
}
.subtitle-label {
    color: #7a8ea3;
    font-size: 12px;
}
.start-btn {
    background: linear-gradient(180deg, #28a745, #1e7e34);
    color: white;
    font-size: 18px;
    font-weight: bold;
    border-radius: 14px;
    border: none;
    padding: 14px 30px;
    min-height: 56px;
}
.start-btn:hover {
    background: linear-gradient(180deg, #34c759, #28a745);
}
.start-btn:active {
    background: linear-gradient(180deg, #1e7e34, #155d27);
}
.stop-btn {
    background: linear-gradient(180deg, #dc3545, #b02a37);
    color: white;
    font-size: 18px;
    font-weight: bold;
    border-radius: 14px;
    border: none;
    padding: 14px 30px;
    min-height: 56px;
}
.stop-btn:hover {
    background: linear-gradient(180deg, #e8444f, #dc3545);
}
.stop-btn:active {
    background: linear-gradient(180deg, #b02a37, #8b2131);
}
.status-running {
    color: #28a745;
    font-size: 13px;
    font-weight: bold;
}
.status-stopped {
    color: #8899aa;
    font-size: 13px;
}
.pin-icon {
    color: #f0c040;
    font-size: 48px;
}
"""


def _kill_gui():
    """Kill all GUI-related processes and clean up."""
    subprocess.run(["pkill", "-f", "main_gui"], capture_output=True)
    subprocess.run(["pkill", "-f", "app_yolo_cam"], capture_output=True)
    # Clean shared memory from DRP-AI stream mode
    try:
        os.remove("/dev/shm/v2n_camera")
    except OSError:
        pass
    # Free camera device
    subprocess.run(["fuser", "-k", "/dev/video0"],
                   capture_output=True, timeout=3)


class BowlingLauncher(Gtk.Window):
    def __init__(self):
        super().__init__(title="Bowling Nav")
        self.set_default_size(320, 260)
        self.set_resizable(False)
        self.set_decorated(False)
        self.set_app_paintable(True)
        self.set_position(Gtk.WindowPosition.CENTER)

        # Load CSS
        provider = Gtk.CssProvider()
        provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        self.gui_proc = None
        self._gui_was_running = False

        # Main container
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        box.set_margin_top(24)
        box.set_margin_bottom(24)
        box.set_margin_start(28)
        box.set_margin_end(28)
        self.add(box)

        # Pin icon
        pin = Gtk.Label(label="\U0001F3B3")
        pin.get_style_context().add_class("pin-icon")
        box.pack_start(pin, False, False, 0)

        # Title
        title = Gtk.Label(label="Bowling Target Nav")
        title.get_style_context().add_class("title-label")
        box.pack_start(title, False, False, 0)

        # Subtitle
        sub = Gtk.Label(label="RZ/V2N Robot Control")
        sub.get_style_context().add_class("subtitle-label")
        box.pack_start(sub, False, False, 4)

        # Start/Stop button
        self.btn = Gtk.Button(label="\u25B6  Start GUI")
        self.btn.get_style_context().add_class("start-btn")
        self.btn.connect("clicked", self.on_btn_clicked)
        box.pack_start(self.btn, False, False, 8)

        # Status
        self.status = Gtk.Label(label="Stopped")
        self.status.get_style_context().add_class("status-stopped")
        box.pack_start(self.status, False, False, 0)

        # Allow dragging the window
        self.drag_x = 0
        self.drag_y = 0
        self.add_events(Gdk.EventMask.BUTTON_PRESS_MASK |
                        Gdk.EventMask.BUTTON_RELEASE_MASK |
                        Gdk.EventMask.POINTER_MOTION_MASK)
        self.connect("button-press-event", self.on_drag_begin)
        self.connect("motion-notify-event", self.on_drag_motion)

        GLib.timeout_add(2000, self.check_status)
        self.connect("destroy", lambda w: Gtk.main_quit())

    def on_drag_begin(self, widget, event):
        if event.button == 1:
            self.drag_x = event.x
            self.drag_y = event.y

    def on_drag_motion(self, widget, event):
        if event.state & Gdk.ModifierType.BUTTON1_MASK:
            x, y = self.get_position()
            self.move(int(x + event.x - self.drag_x),
                      int(y + event.y - self.drag_y))

    def on_btn_clicked(self, widget):
        if self._is_gui_running():
            # Stop GUI: kill all related processes
            _kill_gui()
            self.gui_proc = None
            self._set_stopped()
        else:
            # Start GUI: launch via script, then hide launcher
            env = os.environ.copy()
            env['HOME'] = '/root'
            self.gui_proc = subprocess.Popen(
                ["/bin/bash", "/root/bowling_gui.sh"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self._set_running()
            # Hide launcher so it doesn't cover the fullscreen GUI
            GLib.timeout_add(1500, self._hide_for_gui)

    def _hide_for_gui(self):
        """Hide launcher after GUI starts (so it doesn't cover fullscreen GUI)."""
        if self._is_gui_running():
            self.hide()
        return False  # one-shot timer

    def _is_gui_running(self):
        try:
            result = subprocess.run(["pgrep", "-f", "main_gui"],
                                     capture_output=True)
            return result.returncode == 0
        except Exception:
            return False

    def _set_running(self):
        self.btn.set_label("\u25A0  Stop GUI")
        ctx = self.btn.get_style_context()
        ctx.remove_class("start-btn")
        ctx.add_class("stop-btn")
        self.status.set_text("\u25CF  Running")
        sctx = self.status.get_style_context()
        sctx.remove_class("status-stopped")
        sctx.add_class("status-running")

    def _set_stopped(self):
        self.btn.set_label("\u25B6  Start GUI")
        ctx = self.btn.get_style_context()
        ctx.remove_class("stop-btn")
        ctx.add_class("start-btn")
        self.status.set_text("Stopped")
        sctx = self.status.get_style_context()
        sctx.remove_class("status-running")
        sctx.add_class("status-stopped")

    def check_status(self):
        running = self._is_gui_running()

        if running:
            self._set_running()
            self._gui_was_running = True
        else:
            self._set_stopped()
            self.gui_proc = None
            # GUI just stopped -> show launcher again
            if self._gui_was_running:
                self._gui_was_running = False
                # Re-center on screen
                screen = Gdk.Screen.get_default()
                sw, sh = screen.get_width(), screen.get_height()
                ww, wh = self.get_size()
                self.move((sw - ww) // 2, (sh - wh) // 2)
                self.show()
                self.present()
        return True


def main():
    import time
    for _ in range(30):
        for d in ['/run/user/996', '/run/user/0', '/tmp']:
            for name in ['wayland-0', 'wayland-1']:
                path = os.path.join(d, name)
                if os.path.exists(path):
                    os.environ['XDG_RUNTIME_DIR'] = d
                    os.environ['WAYLAND_DISPLAY'] = name
                    break
            if 'WAYLAND_DISPLAY' in os.environ:
                break
        if 'WAYLAND_DISPLAY' in os.environ:
            break
        time.sleep(1)

    os.environ.setdefault('HOME', '/root')
    win = BowlingLauncher()
    win.show_all()
    # Center on screen (set_position doesn't work on Wayland)
    screen = Gdk.Screen.get_default()
    sw, sh = screen.get_width(), screen.get_height()
    ww, wh = win.get_size()
    win.move((sw - ww) // 2, (sh - wh) // 2)
    Gtk.main()


if __name__ == '__main__':
    main()

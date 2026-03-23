#!/usr/bin/env python3
"""Fullscreen launcher for V2N Robot Control.

Displays a dark fullscreen window with a centered Start/Stop button.
Hides the Weston panel (task bar) for a clean embedded look.
When the user presses Start, launches gui.sh and hides the launcher.
When the GUI exits, the launcher reappears.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, Gdk
import subprocess
import os
import signal

CSS = b"""
window {
    background-color: #0a0e14;
}
.title-label {
    color: #c9d1d9;
    font-size: 28px;
    font-weight: bold;
}
.subtitle-label {
    color: #58687a;
    font-size: 14px;
}
.start-btn {
    background-color: #238636;
    background-image: none;
    color: white;
    font-size: 22px;
    font-weight: bold;
    border-radius: 12px;
    border: 2px solid #2ea043;
    padding: 16px 50px;
    min-height: 64px;
    min-width: 280px;
}
.start-btn:hover {
    background-color: #2ea043;
    background-image: none;
}
.stop-btn {
    background-color: #da3633;
    background-image: none;
    color: white;
    font-size: 22px;
    font-weight: bold;
    border-radius: 12px;
    border: 2px solid #f85149;
    padding: 16px 50px;
    min-height: 64px;
    min-width: 280px;
}
.stop-btn:hover {
    background-color: #f85149;
    background-image: none;
}
.status-running {
    color: #2ea043;
    font-size: 15px;
    font-weight: bold;
}
.status-stopped {
    color: #484f58;
    font-size: 15px;
}
"""


def _kill_gui():
    """Kill all GUI-related processes and clean up shared memory."""
    subprocess.run(["pkill", "-9", "-f", "main_gui"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "app_yolo_cam"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "nav_node"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "camera_node"], capture_output=True)
    for shm in ['v2n_camera', 'v2n_detections', 'v2n_calibration']:
        try:
            os.remove(f"/dev/shm/{shm}")
        except OSError:
            pass


def _hide_weston_panel():
    """No-op: killing weston-desktop-shell crashes Weston on the V2N.

    The fullscreen window covers the panel anyway, so hiding it is
    unnecessary. Keep this function as a no-op so callers don't break.
    """
    pass


class RobotLauncher(Gtk.Window):
    """Fullscreen launcher with a single Start/Stop button."""

    def __init__(self):
        super().__init__(title="V2N Robot")
        self.fullscreen()
        self.set_decorated(False)

        # Load CSS
        provider = Gtk.CssProvider()
        provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        self.gui_proc = None
        self._gui_was_running = False

        # Center everything vertically and horizontally
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        outer.set_valign(Gtk.Align.CENTER)
        outer.set_halign(Gtk.Align.CENTER)
        self.add(outer)

        # SolidRun logo -- check installed package then source tree
        _pkg_dir = os.path.dirname(os.path.abspath(__file__))
        logo_candidates = [
            '/root/ros2_ws/install/target_nav/lib/python3.12/site-packages/target_nav/gui/solidrun_logo.png',
            os.path.join(_pkg_dir, '..', 'target_nav', 'gui', 'solidrun_logo.png'),
        ]
        logo_path = None
        for p in logo_candidates:
            if os.path.exists(p):
                logo_path = p
                break
        if logo_path and os.path.exists(logo_path):
            try:
                from gi.repository import GdkPixbuf
                pixbuf = GdkPixbuf.Pixbuf.new_from_file(logo_path)
                # Scale to max 200px wide, keep aspect ratio
                max_w = 200
                if pixbuf.get_width() > max_w:
                    scale = max_w / pixbuf.get_width()
                    new_w = int(pixbuf.get_width() * scale)
                    new_h = int(pixbuf.get_height() * scale)
                    pixbuf = pixbuf.scale_simple(
                        new_w, new_h, GdkPixbuf.InterpType.BILINEAR)
                logo = Gtk.Image.new_from_pixbuf(pixbuf)
                outer.pack_start(logo, False, False, 0)
                outer.pack_start(Gtk.Box(), False, False, 12)
            except Exception:
                pass

        # Title
        title = Gtk.Label(label="V2N Robot Control")
        title.get_style_context().add_class("title-label")
        outer.pack_start(title, False, False, 0)

        # Subtitle
        sub = Gtk.Label(label="Vision-Guided Target Navigation")
        sub.get_style_context().add_class("subtitle-label")
        outer.pack_start(sub, False, False, 8)

        # Spacer
        outer.pack_start(Gtk.Box(), False, False, 20)

        # Start/Stop button
        self.btn = Gtk.Button(label="Start Robot")
        self.btn.get_style_context().add_class("start-btn")
        self.btn.connect("clicked", self._on_btn)
        outer.pack_start(self.btn, False, False, 0)

        # Capture button (small, below start)
        self.capture_btn = Gtk.Button(label="Capture Image")
        self.capture_btn.set_size_request(180, 40)
        self.capture_btn.connect("clicked", self._on_capture)
        outer.pack_start(self.capture_btn, False, False, 8)

        # Status label
        self.status = Gtk.Label(label="Ready")
        self.status.get_style_context().add_class("status-stopped")
        outer.pack_start(self.status, False, False, 12)

        # Poll GUI status every 2 seconds
        GLib.timeout_add(2000, self._check_status)
        self.connect("destroy", lambda w: Gtk.main_quit())
        self.connect("key-press-event", self._on_key)

    def _on_key(self, widget, event):
        """Handle keyboard: Enter to start, Escape to quit."""
        key = Gdk.keyval_name(event.keyval)
        if key == 'Return' or key == 'KP_Enter':
            self._on_btn(None)
        elif key == 'Escape':
            Gtk.main_quit()

    def _on_btn(self, widget):
        """Start or stop the GUI."""
        if self._is_gui_running():
            _kill_gui()
            self.gui_proc = None
            self._set_stopped()
        else:
            env = os.environ.copy()
            env['HOME'] = '/root'
            self.gui_proc = subprocess.Popen(
                ["/bin/bash", "/root/gui.sh"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            self._set_running()
            GLib.timeout_add(1500, self._hide_for_gui)

    def _hide_for_gui(self):
        """Hide launcher after GUI starts."""
        if self._is_gui_running():
            self.hide()
        return False

    def _is_gui_running(self):
        """Check if main_gui process is running."""
        try:
            r = subprocess.run(["pgrep", "-f", "main_gui"], capture_output=True)
            return r.returncode == 0
        except Exception:
            return False

    def _set_running(self):
        """Update button and status to Running state."""
        self.btn.set_label("Stop Robot")
        ctx = self.btn.get_style_context()
        ctx.remove_class("start-btn")
        ctx.add_class("stop-btn")
        self.status.set_text("Running")
        sctx = self.status.get_style_context()
        sctx.remove_class("status-stopped")
        sctx.add_class("status-running")

    def _set_stopped(self):
        """Update button and status to Stopped state."""
        self.btn.set_label("Start Robot")
        ctx = self.btn.get_style_context()
        ctx.remove_class("stop-btn")
        ctx.add_class("start-btn")
        self.status.set_text("Ready")
        sctx = self.status.get_style_context()
        sctx.remove_class("status-running")
        sctx.add_class("status-stopped")

    def _check_status(self):
        """Periodically check if GUI is running and update UI."""
        running = self._is_gui_running()
        if running:
            self._set_running()
            self._gui_was_running = True
        else:
            self._set_stopped()
            self.gui_proc = None
            if self._gui_was_running:
                self._gui_was_running = False
                self.show()
                self.present()
                self.fullscreen()
        return True


    def _read_camera_shm(self):
        """Read raw BGR frame from DRP-AI shared memory.

        Returns (width, height, channels, frame_bytes) or None on failure.
        """
        import struct as _struct
        import mmap as _mmap

        shm_path = '/dev/shm/v2n_camera'
        if not os.path.exists(shm_path):
            return None
        try:
            fd = os.open(shm_path, os.O_RDONLY)
            fsize = os.fstat(fd).st_size
            mm = _mmap.mmap(fd, fsize, access=_mmap.ACCESS_READ)
            seq, w, h, ch = _struct.unpack_from('<IIII', mm, 0)
            if fsize < 16 + w * h * ch or w == 0 or h == 0:
                mm.close()
                os.close(fd)
                return None
            data = bytes(mm[16:16 + w * h * ch])
            mm.close()
            os.close(fd)
            return w, h, ch, data
        except Exception:
            return None

    def _bgr_to_pixbuf(self, w, h, bgr_data):
        """Convert BGR bytes to GdkPixbuf (RGB) for GTK display."""
        from gi.repository import GdkPixbuf, GLib as _GLib
        rgb = bytearray(w * h * 3)
        for i in range(w * h):
            rgb[i * 3] = bgr_data[i * 3 + 2]      # R
            rgb[i * 3 + 1] = bgr_data[i * 3 + 1]  # G
            rgb[i * 3 + 2] = bgr_data[i * 3]       # B
        return GdkPixbuf.Pixbuf.new_from_data(
            bytes(rgb), GdkPixbuf.Colorspace.RGB, False, 8, w, h, w * 3)

    def _save_frame_png(self, w, h, bgr_data, filename):
        """Save BGR frame as PNG (pure Python, no OpenCV)."""
        import struct as _struct
        import zlib as _zlib

        raw_rows = b''
        for row in range(h):
            off = row * w * 3
            row_bgr = bgr_data[off:off + w * 3]
            row_rgb = bytearray(w * 3)
            for px in range(w):
                row_rgb[px * 3] = row_bgr[px * 3 + 2]
                row_rgb[px * 3 + 1] = row_bgr[px * 3 + 1]
                row_rgb[px * 3 + 2] = row_bgr[px * 3]
            raw_rows += b'\x00' + bytes(row_rgb)

        compressed = _zlib.compress(raw_rows)

        def _chunk(ctype, data):
            c = ctype + data
            return _struct.pack('>I', len(data)) + c + _struct.pack('>I', _zlib.crc32(c) & 0xFFFFFFFF)

        with open(filename, 'wb') as f:
            f.write(b'\x89PNG\r\n\x1a\n')
            f.write(_chunk(b'IHDR', _struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
            f.write(_chunk(b'IDAT', compressed))
            f.write(_chunk(b'IEND', b''))

    def _on_capture(self, widget):
        """Show camera preview window with Save/Discard buttons."""
        frame = self._read_camera_shm()
        if frame is None:
            self.status.set_text("No camera - start robot first")
            return

        w, h, ch, bgr_data = frame
        pixbuf = self._bgr_to_pixbuf(w, h, bgr_data)

        # Scale preview to fit screen
        from gi.repository import GdkPixbuf
        max_w, max_h = 640, 400
        scale = min(max_w / w, max_h / h)
        preview = pixbuf.scale_simple(
            int(w * scale), int(h * scale), GdkPixbuf.InterpType.BILINEAR)

        # Preview dialog
        dlg = Gtk.Dialog(title="Capture Preview", parent=self)
        dlg.set_modal(True)
        dlg.set_default_size(int(w * scale) + 20, int(h * scale) + 80)

        content = dlg.get_content_area()
        content.set_spacing(8)

        img = Gtk.Image.new_from_pixbuf(preview)
        content.pack_start(img, True, True, 0)

        info_label = Gtk.Label(label=f"{w}x{h} - Ready to save")
        content.pack_start(info_label, False, False, 0)

        btn_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        btn_box.set_halign(Gtk.Align.CENTER)

        save_btn = Gtk.Button(label="Save")
        save_btn.set_size_request(120, 40)
        discard_btn = Gtk.Button(label="Discard")
        discard_btn.set_size_request(120, 40)

        def _on_save(b):
            import time as _time
            capture_dir = '/root/captures'
            os.makedirs(capture_dir, exist_ok=True)
            timestamp = _time.strftime('%Y%m%d_%H%M%S')
            filename = f"{capture_dir}/capture_{timestamp}.png"
            try:
                self._save_frame_png(w, h, bgr_data, filename)
                self.status.set_text(f"Saved: {os.path.basename(filename)}")
            except Exception as e:
                self.status.set_text(f"Save failed: {e}")
            dlg.destroy()

        def _on_discard(b):
            self.status.set_text("Capture discarded")
            dlg.destroy()

        save_btn.connect("clicked", _on_save)
        discard_btn.connect("clicked", _on_discard)
        btn_box.pack_start(save_btn, False, False, 0)
        btn_box.pack_start(discard_btn, False, False, 0)
        content.pack_start(btn_box, False, False, 0)

        dlg.show_all()
        dlg.run()
        dlg.destroy()


def main():
    """Wait for Wayland display, hide panel, then show launcher."""
    import time

    # Wait for Wayland compositor
    for _ in range(30):
        for d in ['/run/user/996', '/run/user/0', '/tmp']:
            for name in ['wayland-0', 'wayland-1']:
                if os.path.exists(os.path.join(d, name)):
                    os.environ['XDG_RUNTIME_DIR'] = d
                    os.environ['WAYLAND_DISPLAY'] = name
                    break
            if 'WAYLAND_DISPLAY' in os.environ:
                break
        if 'WAYLAND_DISPLAY' in os.environ:
            break
        time.sleep(1)

    os.environ.setdefault('HOME', '/root')

    _hide_weston_panel()

    win = RobotLauncher()
    win.show_all()
    Gtk.main()


if __name__ == '__main__':
    main()

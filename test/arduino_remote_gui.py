#!/usr/bin/env python3
"""
Arduino Remote Control GUI
==========================

Professional GUI for remote Arduino motor control.
Connect to arduino_remote_server.py running on V2N.

Features:
- Visual motor status indicators
- Keyboard controls (WASD + QE)
- Speed control with presets
- Connection status with auto-reconnect
- Command log with timestamps
- Encoder display
- Direct command panel (Move, VEL, Raw)

Usage:
    python3 arduino_remote_gui.py --host <v2n-ip>
    python3 arduino_remote_gui.py --host 192.168.50.1
"""

import sys
import socket
import json
import time
import argparse
import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, Pango


# =============================================================================
# Custom CSS Styling
# =============================================================================

CSS = b"""
/* Main window */
window {
    background-color: #1e1e2e;
}

/* Headers */
.title-label {
    font-size: 24px;
    font-weight: bold;
    color: #cdd6f4;
}

.subtitle-label {
    font-size: 12px;
    color: #6c7086;
}

/* Status indicators */
.status-connected {
    color: #a6e3a1;
    font-weight: bold;
}

.status-disconnected {
    color: #f38ba8;
    font-weight: bold;
}

.status-connecting {
    color: #f9e2af;
    font-weight: bold;
}

/* Frames */
.control-frame {
    background-color: #313244;
    border-radius: 12px;
    padding: 15px;
}

.control-frame > label {
    color: #cdd6f4;
    font-weight: bold;
    font-size: 14px;
}

/* Motor buttons */
.motor-btn {
    background: linear-gradient(180deg, #45475a 0%, #313244 100%);
    border: 2px solid #585b70;
    border-radius: 8px;
    color: #cdd6f4;
    font-size: 14px;
    font-weight: bold;
    min-width: 80px;
    min-height: 50px;
    transition: all 0.2s ease;
}

.motor-btn:hover {
    background: linear-gradient(180deg, #585b70 0%, #45475a 100%);
    border-color: #89b4fa;
}

.motor-btn:active, .motor-btn-active {
    background: linear-gradient(180deg, #89b4fa 0%, #74c7ec 100%);
    border-color: #89b4fa;
    color: #1e1e2e;
}

.stop-btn {
    background: linear-gradient(180deg, #f38ba8 0%, #eb6f92 100%);
    border: 2px solid #f38ba8;
    color: #1e1e2e;
    font-size: 16px;
}

.stop-btn:hover {
    background: linear-gradient(180deg, #eb6f92 0%, #f38ba8 100%);
}

/* Connect button */
.connect-btn {
    background: linear-gradient(180deg, #a6e3a1 0%, #94e2d5 100%);
    border: none;
    border-radius: 8px;
    color: #1e1e2e;
    font-weight: bold;
    padding: 10px 30px;
}

.connect-btn:hover {
    background: linear-gradient(180deg, #94e2d5 0%, #a6e3a1 100%);
}

.disconnect-btn {
    background: linear-gradient(180deg, #f38ba8 0%, #eb6f92 100%);
}

.disconnect-btn:hover {
    background: linear-gradient(180deg, #eb6f92 0%, #f38ba8 100%);
}

/* Speed controls */
.speed-scale trough {
    background-color: #45475a;
    border-radius: 4px;
    min-height: 8px;
}

.speed-scale highlight {
    background: linear-gradient(90deg, #89b4fa 0%, #74c7ec 100%);
    border-radius: 4px;
}

.speed-scale slider {
    background-color: #cdd6f4;
    border-radius: 50%;
    min-width: 20px;
    min-height: 20px;
}

.speed-preset {
    background-color: #45475a;
    border: 1px solid #585b70;
    border-radius: 6px;
    color: #cdd6f4;
    padding: 5px 15px;
    font-size: 12px;
}

.speed-preset:hover {
    background-color: #585b70;
}

.speed-preset:checked {
    background-color: #89b4fa;
    color: #1e1e2e;
}

/* Log view */
.log-view {
    background-color: #11111b;
    color: #a6adc8;
    font-family: monospace;
    font-size: 11px;
    border-radius: 8px;
    padding: 10px;
}

/* Command buttons */
.cmd-btn {
    background-color: #45475a;
    border: 1px solid #585b70;
    border-radius: 6px;
    color: #cdd6f4;
    padding: 8px 20px;
}

.cmd-btn:hover {
    background-color: #585b70;
    border-color: #89b4fa;
}

/* Motor indicators */
.motor-indicator {
    background-color: #45475a;
    border-radius: 50%;
    min-width: 30px;
    min-height: 30px;
}

.motor-indicator-active {
    background-color: #a6e3a1;
}

/* Keyboard hint */
.kbd {
    background-color: #45475a;
    border: 1px solid #585b70;
    border-radius: 4px;
    color: #cdd6f4;
    font-family: monospace;
    font-size: 11px;
    padding: 2px 6px;
}

/* Info labels */
.info-label {
    color: #6c7086;
    font-size: 11px;
}

.value-label {
    color: #cdd6f4;
    font-weight: bold;
}

/* Encoder display */
.encoder-frame {
    background-color: #313244;
    border-radius: 8px;
    padding: 10px;
}

.encoder-value {
    font-family: monospace;
    font-size: 14px;
    color: #89b4fa;
}

/* Tuning panel */
.tuning-section-label {
    color: #89b4fa;
    font-weight: bold;
    font-size: 13px;
}

.tuning-entry {
    background-color: #45475a;
    border: 1px solid #585b70;
    border-radius: 4px;
    color: #cdd6f4;
    font-family: monospace;
    font-size: 12px;
    padding: 4px 8px;
    min-width: 80px;
}

.tuning-entry:focus {
    border-color: #89b4fa;
}

.tuning-name {
    color: #cdd6f4;
    font-family: monospace;
    font-size: 12px;
}

.tuning-comment {
    color: #6c7086;
    font-size: 10px;
}

.save-btn {
    background: linear-gradient(180deg, #a6e3a1 0%, #94e2d5 100%);
    border: none;
    border-radius: 8px;
    color: #1e1e2e;
    font-weight: bold;
    padding: 10px 30px;
}

.save-btn:hover {
    background: linear-gradient(180deg, #94e2d5 0%, #a6e3a1 100%);
}

.build-btn {
    background: linear-gradient(180deg, #f9e2af 0%, #fab387 100%);
    border: none;
    border-radius: 8px;
    color: #1e1e2e;
    font-weight: bold;
    padding: 10px 30px;
}

.build-btn:hover {
    background: linear-gradient(180deg, #fab387 0%, #f9e2af 100%);
}

.changed-entry {
    border-color: #f9e2af;
    border-width: 2px;
}

.tab-label {
    color: #cdd6f4;
    font-weight: bold;
    font-size: 14px;
    padding: 5px 15px;
}

notebook header tabs tab {
    background-color: #313244;
    border-radius: 8px 8px 0 0;
    padding: 8px 20px;
}

notebook header tabs tab:checked {
    background-color: #45475a;
}

notebook header {
    background-color: #1e1e2e;
}

notebook > stack {
    background-color: #1e1e2e;
}
"""


# =============================================================================
# Available directions for move commands
# =============================================================================

MOVE_DIRECTIONS = [
    "FWD", "BWD", "LEFT", "RIGHT",
    "TURN", "DIAGFL", "DIAGFR", "DIAGBL", "DIAGBR",
]


# =============================================================================
# Network Client
# =============================================================================

class ArduinoClient:
    """TCP client for remote Arduino control."""

    def __init__(self, host: str, port: int = 5555):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.lock = threading.Lock()

    def connect(self) -> tuple:
        """Connect to remote server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(2.0)
            self.connected = True

            # Ping to verify
            response = self._send({"action": "ping"})
            if response and response.get("status") == "ok":
                return True, "Connected successfully"
            return False, "Server not responding"

        except socket.timeout:
            return False, "Connection timeout"
        except ConnectionRefusedError:
            return False, "Connection refused - is server running?"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        """Disconnect from server."""
        if self.socket:
            try:
                self._send({"action": "motor", "cmd": "STOP"})
                self.socket.close()
            except:
                pass
        self.socket = None
        self.connected = False

    def _send(self, data: dict) -> dict:
        """Send command and get response."""
        if not self.socket:
            return {"error": "Not connected"}
        with self.lock:
            try:
                self.socket.send(json.dumps(data).encode('utf-8'))
                response = self.socket.recv(1024)
                return json.loads(response.decode('utf-8'))
            except Exception as e:
                return {"error": str(e)}

    def motor(self, cmd: str, speed: int = 100, ticks: int = 0) -> dict:
        """Send motor command. ticks=0 means hold-to-move (large ticks)."""
        return self._send({"action": "motor", "cmd": cmd, "speed": speed, "ticks": ticks})

    def command(self, cmd: str) -> dict:
        """Send general command."""
        return self._send({"action": "command", "cmd": cmd})

    def get_encoders(self) -> dict:
        """Get latest encoder values from firmware stream."""
        return self._send({"action": "get_encoders"})


# =============================================================================
# Main GUI
# =============================================================================

class ArduinoRemoteGUI(Gtk.Window):
    """Professional Arduino Remote Control GUI with direct command panel."""

    MOTOR_COMMANDS = {
        'w': 'FWD', 's': 'BWD', 'a': 'LEFT', 'd': 'RIGHT',
        'q': 'TURNL', 'e': 'TURNR', 'space': 'STOP',
        'z': 'DIAGFL', 'c': 'DIAGFR', 'x': 'DIAGBL', 'v': 'DIAGBR',
    }

    def __init__(self, host: str, port: int = 5555):
        super().__init__(title="V2N Arduino Remote Control")
        self.client = ArduinoClient(host, port)
        self.host = host
        self.port = port
        self.speed = 100
        self.ticks = 0  # 0 = hold-to-move (large ticks), >0 = finite move
        self.active_key = None
        self.active_command = None
        self.encoder_baseline = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self.encoder_raw = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self._enc_poll_id = None  # GLib timer ID for encoder polling

        # Apply CSS
        self._apply_css()

        # Window setup
        self.set_default_size(800, 800)
        self.set_border_width(15)
        self.set_resizable(True)

        # Build UI
        self._build_ui()

        # Events
        self.connect("destroy", self._on_close)
        self.connect("key-press-event", self._on_key_press)
        self.connect("key-release-event", self._on_key_release)

        self._log("Ready to connect to V2N")
        self._log(f"Target: {host}:{port}")

    def _apply_css(self):
        """Apply custom CSS styling."""
        provider = Gtk.CssProvider()
        provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    def _build_ui(self):
        """Build the user interface with tabs."""
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.add(main_box)

        # Header
        main_box.pack_start(self._create_header(), False, False, 0)

        # Notebook (tabs)
        self.notebook = Gtk.Notebook()
        self.notebook.set_tab_pos(Gtk.PositionType.TOP)
        main_box.pack_start(self.notebook, True, True, 0)

        # Tab 1: Control
        control_page = self._create_control_page()
        control_label = Gtk.Label(label="Control")
        control_label.get_style_context().add_class("tab-label")
        self.notebook.append_page(control_page, control_label)

        # Tab 2: Tuning
        tuning_page = self._create_tuning_page()
        tuning_label = Gtk.Label(label="Tuning")
        tuning_label.get_style_context().add_class("tab-label")
        self.notebook.append_page(tuning_page, tuning_label)

        # Keyboard hints (always visible)
        main_box.pack_start(self._create_keyboard_hints(), False, False, 0)

    def _create_control_page(self) -> Gtk.Box:
        """Create the control tab content."""
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        page.set_margin_top(10)

        content = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        page.pack_start(content, True, True, 0)

        # Left column - Controls
        left_col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        left_col.pack_start(self._create_motor_controls(), False, False, 0)
        left_col.pack_start(self._create_speed_control(), False, False, 0)
        left_col.pack_start(self._create_commands(), False, False, 0)
        content.pack_start(left_col, False, False, 0)

        # Right column - Log and status
        right_col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        right_col.pack_start(self._create_encoder_display(), False, False, 0)
        right_col.pack_start(self._create_log(), True, True, 0)
        content.pack_start(right_col, True, True, 0)

        return page

    def _create_header(self) -> Gtk.Box:
        """Create header section."""
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=15)

        # Title
        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)

        title = Gtk.Label(label="V2N Arduino Control")
        title.get_style_context().add_class("title-label")
        title.set_halign(Gtk.Align.START)
        title_box.pack_start(title, False, False, 0)

        subtitle = Gtk.Label(label=f"Remote: {self.host}:{self.port}")
        subtitle.get_style_context().add_class("subtitle-label")
        subtitle.set_halign(Gtk.Align.START)
        title_box.pack_start(subtitle, False, False, 0)

        box.pack_start(title_box, True, True, 0)

        # Connection controls
        conn_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)

        self.status_label = Gtk.Label(label="Disconnected")
        self.status_label.get_style_context().add_class("status-disconnected")
        conn_box.pack_start(self.status_label, False, False, 0)

        self.connect_btn = Gtk.Button(label="Connect")
        self.connect_btn.get_style_context().add_class("connect-btn")
        self.connect_btn.connect("clicked", self._on_connect)
        conn_box.pack_start(self.connect_btn, False, False, 0)

        box.pack_start(conn_box, False, False, 0)

        return box

    def _create_motor_controls(self) -> Gtk.Frame:
        """Create motor control buttons."""
        frame = Gtk.Frame(label="  Motor Controls  ")
        frame.get_style_context().add_class("control-frame")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        box.set_margin_top(15)
        box.set_margin_bottom(15)
        box.set_margin_start(15)
        box.set_margin_end(15)
        frame.add(box)

        # Direction grid
        grid = Gtk.Grid()
        grid.set_row_spacing(8)
        grid.set_column_spacing(8)
        grid.set_halign(Gtk.Align.CENTER)

        buttons = [
            ("\u21b0\nDIAG FL", "DIAGFL", 0, 0, "motor-btn"),
            ("\u25b2\nFWD", "FWD", 0, 1, "motor-btn"),
            ("\u21b1\nDIAG FR", "DIAGFR", 0, 2, "motor-btn"),
            ("\u25c0\nLEFT", "LEFT", 1, 0, "motor-btn"),
            ("\u25a0\nSTOP", "STOP", 1, 1, "stop-btn"),
            ("\u25b6\nRIGHT", "RIGHT", 1, 2, "motor-btn"),
            ("\u21b2\nDIAG BL", "DIAGBL", 2, 0, "motor-btn"),
            ("\u25bc\nBWD", "BWD", 2, 1, "motor-btn"),
            ("\u21b3\nDIAG BR", "DIAGBR", 2, 2, "motor-btn"),
            ("\u21ba\nTURN L", "TURNL", 0, 4, "motor-btn"),
            ("\u21bb\nTURN R", "TURNR", 2, 4, "motor-btn"),
        ]

        self.motor_buttons = {}
        for label, cmd, row, col, style in buttons:
            btn = Gtk.Button(label=label)
            btn.get_style_context().add_class(style)
            btn.set_size_request(80, 60)
            btn.connect("pressed", self._on_motor_pressed, cmd)
            btn.connect("released", self._on_motor_released)
            grid.attach(btn, col, row, 1, 1)
            self.motor_buttons[cmd] = btn

        spacer = Gtk.Label(label="")
        spacer.set_size_request(20, 1)
        grid.attach(spacer, 3, 0, 1, 3)

        box.pack_start(grid, False, False, 0)

        return frame

    def _create_speed_control(self) -> Gtk.Frame:
        """Create speed and ticks control section."""
        frame = Gtk.Frame(label="  Speed & Ticks  ")
        frame.get_style_context().add_class("control-frame")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(15)
        box.set_margin_end(15)
        frame.add(box)

        # Speed slider
        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        speed_title = Gtk.Label(label="Speed:")
        speed_title.get_style_context().add_class("tuning-name")
        speed_title.set_size_request(50, -1)
        speed_row.pack_start(speed_title, False, False, 0)

        self.speed_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 10, 255, 5)
        self.speed_scale.set_value(100)
        self.speed_scale.set_hexpand(True)
        self.speed_scale.get_style_context().add_class("speed-scale")
        self.speed_scale.connect("value-changed", self._on_speed_changed)
        speed_row.pack_start(self.speed_scale, True, True, 0)

        self.speed_label = Gtk.Label(label="100")
        self.speed_label.get_style_context().add_class("value-label")
        self.speed_label.set_size_request(40, -1)
        speed_row.pack_start(self.speed_label, False, False, 0)

        box.pack_start(speed_row, False, False, 0)

        # Speed presets
        preset_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        preset_box.set_halign(Gtk.Align.CENTER)

        for preset, value in [("Slow", 50), ("Medium", 100), ("Fast", 150), ("Max", 255)]:
            btn = Gtk.Button(label=preset)
            btn.get_style_context().add_class("speed-preset")
            btn.connect("clicked", self._on_speed_preset, value)
            preset_box.pack_start(btn, False, False, 0)

        box.pack_start(preset_box, False, False, 0)

        # Ticks control
        ticks_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        ticks_title = Gtk.Label(label="Ticks:")
        ticks_title.get_style_context().add_class("tuning-name")
        ticks_title.set_size_request(50, -1)
        ticks_row.pack_start(ticks_title, False, False, 0)

        self.ticks_spin = Gtk.SpinButton.new_with_range(0, 999999, 100)
        self.ticks_spin.set_value(0)
        self.ticks_spin.get_style_context().add_class("tuning-entry")
        self.ticks_spin.connect("value-changed", self._on_ticks_changed)
        ticks_row.pack_start(self.ticks_spin, False, False, 0)

        ticks_hint = Gtk.Label(label="0 = hold-to-move, >0 = finite move (4320/rev)")
        ticks_hint.get_style_context().add_class("tuning-comment")
        ticks_row.pack_start(ticks_hint, False, False, 0)

        box.pack_start(ticks_row, False, False, 0)

        # Ticks presets
        ticks_preset_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        ticks_preset_box.set_halign(Gtk.Align.CENTER)

        for preset, value in [("Hold", 0), ("1cm", 172), ("5cm", 860), ("10cm", 1720), ("1rev", 4320)]:
            btn = Gtk.Button(label=preset)
            btn.get_style_context().add_class("speed-preset")
            btn.connect("clicked", self._on_ticks_preset, value)
            ticks_preset_box.pack_start(btn, False, False, 0)

        box.pack_start(ticks_preset_box, False, False, 0)

        return frame

    def _create_commands(self) -> Gtk.Frame:
        """Create command buttons."""
        frame = Gtk.Frame(label="  Commands  ")
        frame.get_style_context().add_class("control-frame")

        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(15)
        box.set_margin_end(15)
        box.set_halign(Gtk.Align.CENTER)
        frame.add(box)

        for cmd in ["SYNC", "READ", "RESET"]:
            btn = Gtk.Button(label=cmd)
            btn.get_style_context().add_class("cmd-btn")
            btn.connect("clicked", self._on_command, cmd)
            box.pack_start(btn, False, False, 0)

        return frame

    def _create_encoder_display(self) -> Gtk.Frame:
        """Create encoder display with relative values and reset."""
        frame = Gtk.Frame(label="  Encoders (relative)  ")
        frame.get_style_context().add_class("control-frame")

        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        outer.set_margin_top(10)
        outer.set_margin_bottom(10)
        outer.set_margin_start(15)
        outer.set_margin_end(15)
        frame.add(outer)

        grid = Gtk.Grid()
        grid.set_row_spacing(8)
        grid.set_column_spacing(15)

        self.encoder_labels = {}
        positions = [("FL", 0, 0), ("FR", 0, 1), ("RL", 1, 0), ("RR", 1, 1)]

        for name, row, col in positions:
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)

            label = Gtk.Label(label=name)
            label.get_style_context().add_class("info-label")
            box.pack_start(label, False, False, 0)

            value = Gtk.Label(label="----")
            value.get_style_context().add_class("encoder-value")
            box.pack_start(value, False, False, 0)

            self.encoder_labels[name] = value
            grid.attach(box, col, row, 1, 1)

        outer.pack_start(grid, False, False, 0)

        # Reset and Read buttons
        btn_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        btn_box.set_halign(Gtk.Align.CENTER)

        reset_btn = Gtk.Button(label="Reset Zero")
        reset_btn.get_style_context().add_class("cmd-btn")
        reset_btn.connect("clicked", self._on_reset_encoders)
        btn_box.pack_start(reset_btn, False, False, 0)

        read_btn = Gtk.Button(label="Read")
        read_btn.get_style_context().add_class("cmd-btn")
        read_btn.connect("clicked", self._on_command, "READ")
        btn_box.pack_start(read_btn, False, False, 0)

        outer.pack_start(btn_box, False, False, 0)

        return frame

    def _on_reset_encoders(self, button):
        """Reset encoder baseline to current values."""
        for label in ["FL", "FR", "RL", "RR"]:
            self.encoder_baseline[label] = self.encoder_raw[label]
            self.encoder_labels[label].set_text("0")
        self._log("Encoders zeroed")

    def _parse_encoder_response(self, resp: str):
        """Parse encoder values and display relative to baseline.

        Firmware format: ENC,FL:1234,FR:5678,RL:9012,RR:3456,t_us:...
        """
        try:
            labels = ["FL", "FR", "RL", "RR"]
            for part in resp.split(","):
                part = part.strip()
                for label in labels:
                    if part.startswith(f"{label}:"):
                        raw = int(part.split(":")[1])
                        self.encoder_raw[label] = raw
                        relative = raw - self.encoder_baseline[label]
                        GLib.idle_add(
                            self.encoder_labels[label].set_text,
                            str(relative)
                        )
        except Exception:
            pass

    def _start_encoder_polling(self):
        """Start polling encoder values from the server (firmware streams at 20Hz)."""
        if self._enc_poll_id is None:
            self._enc_poll_id = GLib.timeout_add(200, self._poll_encoders)

    def _stop_encoder_polling(self):
        """Stop encoder polling."""
        if self._enc_poll_id is not None:
            GLib.source_remove(self._enc_poll_id)
            self._enc_poll_id = None

    def _poll_encoders(self) -> bool:
        """Fetch latest encoder values from server and update display."""
        if not self.client.connected:
            self._enc_poll_id = None
            return False  # Stop timer

        def fetch():
            resp = self.client.get_encoders()
            enc = resp.get("encoders")
            if enc:
                GLib.idle_add(self._update_encoder_display, enc)

        threading.Thread(target=fetch, daemon=True).start()
        return True  # Keep timer running

    def _update_encoder_display(self, enc: dict):
        """Update encoder labels with relative values."""
        for label in ["FL", "FR", "RL", "RR"]:
            if label in enc:
                self.encoder_raw[label] = enc[label]
                relative = enc[label] - self.encoder_baseline[label]
                self.encoder_labels[label].set_text(str(relative))

    def _create_log(self) -> Gtk.Frame:
        """Create log display."""
        frame = Gtk.Frame(label="  Activity Log  ")
        frame.get_style_context().add_class("control-frame")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        box.set_margin_top(10)
        box.set_margin_bottom(10)
        box.set_margin_start(10)
        box.set_margin_end(10)
        frame.add(box)

        scroll = Gtk.ScrolledWindow()
        scroll.set_min_content_height(200)
        scroll.set_hexpand(True)
        scroll.set_vexpand(True)

        self.log_view = Gtk.TextView()
        self.log_view.set_editable(False)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD)
        self.log_view.get_style_context().add_class("log-view")
        self.log_buffer = self.log_view.get_buffer()

        scroll.add(self.log_view)
        box.pack_start(scroll, True, True, 0)

        clear_btn = Gtk.Button(label="Clear Log")
        clear_btn.get_style_context().add_class("cmd-btn")
        clear_btn.connect("clicked", lambda w: self.log_buffer.set_text(""))
        box.pack_start(clear_btn, False, False, 0)

        return frame

    # =========================================================================
    # Direct Command Page
    # =========================================================================

    def _create_tuning_page(self) -> Gtk.Box:
        """Create the direct command tab - send commands with custom parameters."""
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        page.set_margin_top(10)
        page.set_margin_start(10)
        page.set_margin_end(10)

        # --- Move Command Section ---
        move_frame = Gtk.Frame(label="  Move Command (DIR,speed,ticks)  ")
        move_frame.get_style_context().add_class("control-frame")

        move_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        move_box.set_margin_top(12)
        move_box.set_margin_bottom(12)
        move_box.set_margin_start(15)
        move_box.set_margin_end(15)
        move_frame.add(move_box)

        # Direction selector
        dir_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        dir_label = Gtk.Label(label="Direction:")
        dir_label.get_style_context().add_class("tuning-name")
        dir_label.set_size_request(80, -1)
        dir_row.pack_start(dir_label, False, False, 0)

        self.dir_combo = Gtk.ComboBoxText()
        for d in MOVE_DIRECTIONS:
            self.dir_combo.append_text(d)
        self.dir_combo.set_active(0)  # FWD
        self.dir_combo.get_style_context().add_class("tuning-entry")
        dir_row.pack_start(self.dir_combo, True, True, 0)
        move_box.pack_start(dir_row, False, False, 0)

        # Speed input
        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        speed_label = Gtk.Label(label="Speed:")
        speed_label.get_style_context().add_class("tuning-name")
        speed_label.set_size_request(80, -1)
        speed_row.pack_start(speed_label, False, False, 0)

        self.move_speed_spin = Gtk.SpinButton.new_with_range(20, 255, 5)
        self.move_speed_spin.set_value(100)
        self.move_speed_spin.get_style_context().add_class("tuning-entry")
        speed_row.pack_start(self.move_speed_spin, False, False, 0)

        speed_hint = Gtk.Label(label="(20-255 PWM)")
        speed_hint.get_style_context().add_class("tuning-comment")
        speed_row.pack_start(speed_hint, False, False, 0)
        move_box.pack_start(speed_row, False, False, 0)

        # Ticks input
        ticks_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        ticks_label = Gtk.Label(label="Ticks:")
        ticks_label.get_style_context().add_class("tuning-name")
        ticks_label.set_size_request(80, -1)
        ticks_row.pack_start(ticks_label, False, False, 0)

        self.move_ticks_spin = Gtk.SpinButton.new_with_range(1, 999999, 100)
        self.move_ticks_spin.set_value(5000)
        self.move_ticks_spin.get_style_context().add_class("tuning-entry")
        ticks_row.pack_start(self.move_ticks_spin, False, False, 0)

        ticks_hint = Gtk.Label(label="(4320 = 1 wheel revolution)")
        ticks_hint.get_style_context().add_class("tuning-comment")
        ticks_row.pack_start(ticks_hint, False, False, 0)
        move_box.pack_start(ticks_row, False, False, 0)

        # Send Move button
        send_move_btn = Gtk.Button(label="Send Move")
        send_move_btn.get_style_context().add_class("save-btn")
        send_move_btn.connect("clicked", self._on_send_move)
        move_box.pack_start(send_move_btn, False, False, 0)

        page.pack_start(move_frame, False, False, 0)

        # --- VEL Command Section ---
        vel_frame = Gtk.Frame(label="  Velocity Command (VEL,vx,vy,wz)  ")
        vel_frame.get_style_context().add_class("control-frame")

        vel_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        vel_box.set_margin_top(12)
        vel_box.set_margin_bottom(12)
        vel_box.set_margin_start(15)
        vel_box.set_margin_end(15)
        vel_frame.add(vel_box)

        # VX, VY, WZ spinners in a grid
        vel_grid = Gtk.Grid()
        vel_grid.set_row_spacing(8)
        vel_grid.set_column_spacing(10)

        for i, (name, hint) in enumerate([
            ("vx", "forward/back"),
            ("vy", "left/right strafe"),
            ("wz", "rotation"),
        ]):
            label = Gtk.Label(label=f"{name}:")
            label.get_style_context().add_class("tuning-name")
            label.set_size_request(80, -1)
            vel_grid.attach(label, 0, i, 1, 1)

            spin = Gtk.SpinButton.new_with_range(-255, 255, 5)
            spin.set_value(0)
            spin.get_style_context().add_class("tuning-entry")
            vel_grid.attach(spin, 1, i, 1, 1)

            hint_label = Gtk.Label(label=f"(-255..255, {hint})")
            hint_label.get_style_context().add_class("tuning-comment")
            vel_grid.attach(hint_label, 2, i, 1, 1)

            setattr(self, f'vel_{name}_spin', spin)

        vel_box.pack_start(vel_grid, False, False, 0)

        send_vel_btn = Gtk.Button(label="Send VEL")
        send_vel_btn.get_style_context().add_class("save-btn")
        send_vel_btn.connect("clicked", self._on_send_vel)
        vel_box.pack_start(send_vel_btn, False, False, 0)

        page.pack_start(vel_frame, False, False, 0)

        # --- Raw Command Section ---
        raw_frame = Gtk.Frame(label="  Raw Command  ")
        raw_frame.get_style_context().add_class("control-frame")

        raw_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        raw_box.set_margin_top(12)
        raw_box.set_margin_bottom(12)
        raw_box.set_margin_start(15)
        raw_box.set_margin_end(15)
        raw_frame.add(raw_box)

        self.raw_entry = Gtk.Entry()
        self.raw_entry.get_style_context().add_class("tuning-entry")
        self.raw_entry.set_placeholder_text("e.g. FWD,100,5000 or STOP or READ")
        self.raw_entry.set_hexpand(True)
        self.raw_entry.connect("activate", self._on_send_raw)
        raw_box.pack_start(self.raw_entry, True, True, 0)

        send_raw_btn = Gtk.Button(label="Send")
        send_raw_btn.get_style_context().add_class("save-btn")
        send_raw_btn.connect("clicked", self._on_send_raw)
        raw_box.pack_start(send_raw_btn, False, False, 0)

        page.pack_start(raw_frame, False, False, 0)

        # --- Quick Reference ---
        ref_frame = Gtk.Frame(label="  Quick Reference  ")
        ref_frame.get_style_context().add_class("control-frame")

        ref_label = Gtk.Label()
        ref_label.set_markup(
            '<span font_family="monospace" size="small" color="#a6adc8">'
            'FWD,speed,ticks    BWD,speed,ticks    LEFT,speed,ticks\n'
            'RIGHT,speed,ticks  TURN,speed,ticks   STOP\n'
            'DIAGFL / DIAGFR / DIAGBL / DIAGBR,speed,ticks\n'
            'VEL,vx,vy,wz      READ               SYNC\n'
            '\n'
            'speed: 20-255 PWM    ticks: encoder counts (4320/rev)\n'
            'TURN: +ticks=CCW, -ticks=CW    1cm ~ 172 ticks'
            '</span>'
        )
        ref_label.set_margin_top(10)
        ref_label.set_margin_bottom(10)
        ref_label.set_margin_start(15)
        ref_label.set_margin_end(15)
        ref_frame.add(ref_label)
        page.pack_start(ref_frame, False, False, 0)

        return page

    def _on_send_move(self, button):
        """Send a move command with the specified direction, speed, and ticks."""
        if not self.client.connected:
            self._log("Not connected!")
            return

        direction = self.dir_combo.get_active_text()
        speed = int(self.move_speed_spin.get_value())
        ticks = int(self.move_ticks_spin.get_value())

        self._log(f"Sending: {direction},{speed},{ticks}")

        def do_send():
            response = self.client._send({
                "action": "move",
                "dir": direction,
                "speed": speed,
                "ticks": ticks,
            })
            if "error" in response:
                self._log(f"Error: {response['error']}")
            else:
                self._log(f"Response: {response.get('response', 'OK')}")

        threading.Thread(target=do_send, daemon=True).start()

    def _on_send_vel(self, button):
        """Send a VEL command with custom vx, vy, wz."""
        if not self.client.connected:
            self._log("Not connected!")
            return

        vx = int(self.vel_vx_spin.get_value())
        vy = int(self.vel_vy_spin.get_value())
        wz = int(self.vel_wz_spin.get_value())

        self._log(f"Sending: VEL,{vx},{vy},{wz}")

        def do_send():
            response = self.client._send({
                "action": "command",
                "cmd": f"VEL,{vx},{vy},{wz}",
            })
            if "error" in response:
                self._log(f"Error: {response['error']}")
            else:
                self._log(f"Response: {response.get('response', 'OK')}")

        threading.Thread(target=do_send, daemon=True).start()

    def _on_send_raw(self, widget):
        """Send a raw command string."""
        if not self.client.connected:
            self._log("Not connected!")
            return

        cmd = self.raw_entry.get_text().strip()
        if not cmd:
            return

        self._log(f"Raw: {cmd}")

        def do_send():
            response = self.client._send({
                "action": "command",
                "cmd": cmd,
            })
            if "error" in response:
                self._log(f"Error: {response['error']}")
            else:
                self._log(f"Response: {response.get('response', 'OK')}")

        threading.Thread(target=do_send, daemon=True).start()
        self.raw_entry.set_text("")

    def _create_keyboard_hints(self) -> Gtk.Box:
        """Create keyboard shortcut hints."""
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        box.set_halign(Gtk.Align.CENTER)

        hints = [
            ("W", "Fwd"), ("S", "Bwd"), ("A", "Left"), ("D", "Right"),
            ("Q", "TurnL"), ("E", "TurnR"), ("Z", "DiagFL"), ("C", "DiagFR"),
            ("Space", "Stop"),
        ]

        for key, action in hints:
            hint_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

            key_label = Gtk.Label(label=key)
            key_label.get_style_context().add_class("kbd")
            hint_box.pack_start(key_label, False, False, 0)

            action_label = Gtk.Label(label=action)
            action_label.get_style_context().add_class("info-label")
            hint_box.pack_start(action_label, False, False, 0)

            box.pack_start(hint_box, False, False, 0)

        return box

    # =========================================================================
    # Event Handlers
    # =========================================================================

    def _log(self, message: str):
        """Add message to log."""
        timestamp = time.strftime("%H:%M:%S")
        text = f"[{timestamp}] {message}\n"
        GLib.idle_add(self._append_log, text)

    def _append_log(self, text: str):
        """Append to log buffer (thread-safe)."""
        end_iter = self.log_buffer.get_end_iter()
        self.log_buffer.insert(end_iter, text)
        mark = self.log_buffer.get_insert()
        self.log_view.scroll_to_mark(mark, 0, False, 0, 1)

    def _update_status(self, status: str, style: str):
        """Update connection status."""
        GLib.idle_add(self._set_status, status, style)

    def _set_status(self, status: str, style: str):
        """Set status label (thread-safe)."""
        self.status_label.set_text(status)
        ctx = self.status_label.get_style_context()
        ctx.remove_class("status-connected")
        ctx.remove_class("status-disconnected")
        ctx.remove_class("status-connecting")
        ctx.add_class(style)

    def _on_connect(self, button):
        """Handle connect/disconnect."""
        if not self.client.connected:
            self._update_status("Connecting...", "status-connecting")
            self._log(f"Connecting to {self.host}:{self.port}...")

            def do_connect():
                success, msg = self.client.connect()
                GLib.idle_add(self._connection_result, success, msg)

            threading.Thread(target=do_connect, daemon=True).start()
        else:
            self._stop_encoder_polling()
            self.client.disconnect()
            self._update_status("Disconnected", "status-disconnected")
            self.connect_btn.set_label("Connect")
            ctx = self.connect_btn.get_style_context()
            ctx.remove_class("disconnect-btn")
            ctx.add_class("connect-btn")
            self._log("Disconnected")

    def _connection_result(self, success: bool, msg: str):
        """Handle connection result."""
        if success:
            self._update_status("Connected", "status-connected")
            self.connect_btn.set_label("Disconnect")
            ctx = self.connect_btn.get_style_context()
            ctx.remove_class("connect-btn")
            ctx.add_class("disconnect-btn")
            self._log(f"Connected: {msg}")
            self._start_encoder_polling()
        else:
            self._update_status("Disconnected", "status-disconnected")
            self._log(f"Connection failed: {msg}")

    def _on_speed_changed(self, scale):
        """Handle speed change."""
        self.speed = int(scale.get_value())
        self.speed_label.set_text(str(self.speed))

    def _on_speed_preset(self, button, value: int):
        """Handle speed preset button."""
        self.speed_scale.set_value(value)

    def _on_ticks_changed(self, spin):
        """Handle ticks change."""
        self.ticks = int(spin.get_value())

    def _on_ticks_preset(self, button, value: int):
        """Handle ticks preset button."""
        self.ticks_spin.set_value(value)

    def _on_motor_pressed(self, button, cmd: str):
        """Handle motor button press."""
        if not self.client.connected:
            self._log("Not connected!")
            return

        self.active_command = cmd
        if self.ticks > 0:
            self._log(f"Motor: {cmd} @ speed={self.speed} ticks={self.ticks}")
        else:
            self._log(f"Motor: {cmd} @ speed={self.speed} (hold)")

        button.get_style_context().add_class("motor-btn-active")

        # Send in thread so UI doesn't block
        threading.Thread(
            target=self._send_motor, args=(cmd,), daemon=True
        ).start()

    def _send_motor(self, cmd: str):
        """Send motor command in background thread."""
        response = self.client.motor(cmd, self.speed, self.ticks)
        if "error" in response:
            self._log(f"Error: {response['error']}")

    def _on_motor_released(self, button):
        """Handle motor button release."""
        button.get_style_context().remove_class("motor-btn-active")

        # Only send STOP on release in hold-to-move mode (ticks=0)
        # When ticks>0, let the robot complete the move on its own
        if self.ticks == 0 and self.client.connected and self.active_command and self.active_command != "STOP":
            threading.Thread(
                target=self._send_motor, args=("STOP",), daemon=True
            ).start()
            self._log("Motor: STOP")
        self.active_command = None

    def _on_command(self, button, cmd: str):
        """Handle command button."""
        if not self.client.connected:
            self._log("Not connected!")
            return

        self._log(f"Command: {cmd}")
        response = self.client.command(cmd)

        if "response" in response:
            resp = response["response"]
            self._log(f"Response: {resp}")

            if cmd == "READ" and "ENC" in resp:
                self._parse_encoder_response(resp)
        elif "error" in response:
            self._log(f"Error: {response['error']}")

    def _on_key_press(self, widget, event):
        """Handle key press."""
        if not self.client.connected:
            return False

        # Don't intercept keys when text entries or spin buttons are focused
        focus = self.get_focus()
        if isinstance(focus, (Gtk.Entry, Gtk.SpinButton)):
            return False

        key = Gdk.keyval_name(event.keyval).lower()

        if key == self.active_key:
            return True

        cmd = self.MOTOR_COMMANDS.get(key)
        if cmd:
            self.active_key = key
            self.active_command = cmd

            if cmd in self.motor_buttons:
                self.motor_buttons[cmd].get_style_context().add_class("motor-btn-active")

            self._log(f"Key [{key.upper()}]: {cmd} @ {self.speed}")
            threading.Thread(
                target=self._send_motor, args=(cmd,), daemon=True
            ).start()
            return True

        return False

    def _on_key_release(self, widget, event):
        """Handle key release."""
        key = Gdk.keyval_name(event.keyval).lower()

        if key == self.active_key:
            cmd = self.MOTOR_COMMANDS.get(key)
            if cmd and cmd in self.motor_buttons:
                self.motor_buttons[cmd].get_style_context().remove_class("motor-btn-active")

            self.active_key = None
            # Only send STOP on release in hold-to-move mode (ticks=0)
            if self.ticks == 0 and self.client.connected and self.active_command != "STOP":
                threading.Thread(
                    target=self._send_motor, args=("STOP",), daemon=True
                ).start()
                self._log("Key released: STOP")
            self.active_command = None

        return True

    def _on_close(self, widget):
        """Handle window close."""
        if self.client.connected:
            self.client.motor("STOP")
            self.client.disconnect()
        Gtk.main_quit()


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Arduino Remote Control GUI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 arduino_remote_gui.py --host 192.168.50.1

Make sure arduino_remote_server.py is running on the V2N device!
        """
    )
    parser.add_argument("--host", required=True, help="V2N IP address")
    parser.add_argument("--port", type=int, default=5555, help="Server port (default: 5555)")
    args = parser.parse_args()

    gui = ArduinoRemoteGUI(args.host, args.port)
    gui.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()

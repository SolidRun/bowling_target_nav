"""Hardware Tools tab for SettingsWindow.

Provides three sub-tabs for robot hardware testing and calibration:

    Drive:   8-direction pad + rotation, sends Twist via test_motor_twist
             (arduino_node converts m/s to PWM). Click-to-move with auto-stop.
    Motors:  Test individual motors with raw TMOTOR,id,pwm commands.
    System:  Reset odometry, run calibration, read encoders.

Command path (Drive tab):
    GUI click -> send_ros_command({'type': 'test_motor_twist', ...})
    -> CmdRingBuffer -> nav_node _poll_commands -> dispatch_command
    -> publishes Twist to /cmd_vel
    -> arduino_node converts m/s to PWM using max_linear_speed / max_angular_speed

Command path (Motors / System tabs):
    GUI click -> send_ros_command({'type': 'arduino_cmd', 'data': 'TMOTOR,FL,100'})
    -> CmdRingBuffer -> nav_node -> dispatch_command
    -> publishes String to /arduino/cmd
    -> arduino_node forwards raw command to firmware

Runs in: GUI process (Process 0) on the GTK main thread.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from target_nav.utils.log import get_logger

logger = get_logger('gui.settings.tools')

# Drive tab defaults
DEFAULT_SPEED = 0.20        # m/s (safe default)
MIN_SPEED = 0.05            # m/s
MAX_SPEED = 0.40            # m/s
SPEED_STEP = 0.01           # m/s

DEFAULT_DURATION = 1.5      # seconds
MIN_DURATION = 0.5          # seconds
MAX_DURATION = 5.0          # seconds
DURATION_STEP = 0.1         # seconds

# Motor test tab defaults
DEFAULT_MOTOR_PWM = 100     # safe PWM
MIN_MOTOR_PWM = 0
MAX_MOTOR_PWM = 200         # protect motors (max safe PWM)
MOTOR_PWM_STEP = 10

DEFAULT_MOTOR_DURATION = 2.0  # seconds

# Angular speed multiplier: angular_speed = linear_speed * this factor
# At 0.20 m/s linear -> 0.60 rad/s angular (gentle turn)
ANGULAR_SPEED_FACTOR = 3.0

# Repeat interval for sending velocity commands (firmware 200ms watchdog)
VEL_REPEAT_MS = 150

# Motor IDs matching firmware TMOTOR command
MOTOR_IDS = ['FL', 'FR', 'BL', 'BR']


class ToolsTabMixin:
    """Hardware Tools tab -- Drive, Motors, System."""

    def _build_tools_page(self):
        """Build the Tools tab with 3 sub-tabs."""
        return self._make_sub_notebook([
            ("Drive", self._build_drive_subtab()),
            ("Motors", self._build_motors_subtab()),
            ("System", self._build_system_subtab()),
        ])

    # ================================================================
    # Drive sub-tab
    # ================================================================

    def _build_drive_subtab(self):
        """8-direction pad + rotation with speed/duration sliders."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_margin_top(6)
        box.set_margin_start(6)
        box.set_margin_end(6)

        # -- Timers --
        self._drive_repeat_timer = None
        self._drive_stop_timer = None

        # -- Speed slider --
        box.pack_start(self._make_section("Speed (m/s)"), False, False, 0)
        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        speed_row.set_margin_start(10)
        self._drive_speed_adj = Gtk.Adjustment(
            value=DEFAULT_SPEED, lower=MIN_SPEED, upper=MAX_SPEED,
            step_increment=SPEED_STEP, page_increment=0.05)
        speed_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL,
            adjustment=self._drive_speed_adj)
        speed_scale.set_digits(2)
        speed_scale.set_value_pos(Gtk.PositionType.RIGHT)
        speed_scale.set_hexpand(True)
        speed_row.pack_start(speed_scale, True, True, 0)
        box.pack_start(speed_row, False, False, 0)

        # -- Duration slider --
        box.pack_start(self._make_section("Duration (seconds)"), False, False, 0)
        dur_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        dur_row.set_margin_start(10)
        self._drive_dur_adj = Gtk.Adjustment(
            value=DEFAULT_DURATION, lower=MIN_DURATION, upper=MAX_DURATION,
            step_increment=DURATION_STEP, page_increment=0.5)
        dur_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL,
            adjustment=self._drive_dur_adj)
        dur_scale.set_digits(1)
        dur_scale.set_value_pos(Gtk.PositionType.RIGHT)
        dur_scale.set_hexpand(True)
        dur_row.pack_start(dur_scale, True, True, 0)
        box.pack_start(dur_row, False, False, 0)

        # -- Direction grid --
        box.pack_start(self._make_section("Direction (click to move)"), False, False, 2)
        grid = Gtk.Grid()
        grid.set_column_spacing(4)
        grid.set_row_spacing(4)
        grid.set_margin_start(10)
        grid.set_halign(Gtk.Align.CENTER)

        # Layout: 4 rows x 3 cols
        #   Row 0: FL   FWD   FR
        #   Row 1: LEFT STOP  RIGHT
        #   Row 2: BL   BWD   BR
        #   Row 3: TL   ---   TR
        buttons = [
            ("FL",    0, 0), ("FWD",   0, 1), ("FR",    0, 2),
            ("LEFT",  1, 0), ("STOP",  1, 1), ("RIGHT", 1, 2),
            ("BL",    2, 0), ("BWD",   2, 1), ("BR",    2, 2),
            ("TL",    3, 0),                   ("TR",    3, 2),
        ]
        for label, row, col in buttons:
            btn = Gtk.Button(label=label)
            btn.set_size_request(70, 44)
            if label == "STOP":
                btn.get_style_context().add_class('destructive-action')
                btn.connect('clicked', self._on_drive_stop)
            else:
                btn.connect('clicked', self._on_drive_direction, label)
            grid.attach(btn, col, row, 1, 1)

        box.pack_start(grid, False, False, 2)

        # -- Status label --
        self._drive_status = Gtk.Label(label="")
        self._drive_status.set_xalign(0)
        self._drive_status.set_margin_start(10)
        box.pack_start(self._drive_status, False, False, 4)

        return box

    def _on_drive_direction(self, _btn, direction):
        """Handle direction button click: send velocity for duration then stop."""
        self._cancel_drive_timers()

        speed = self._drive_speed_adj.get_value()
        duration = self._drive_dur_adj.get_value()
        duration_ms = int(duration * 1000)

        # Map direction label to (vx, vy, wz) in m/s and rad/s
        # vx: forward(+) / backward(-)
        # vy: left(+) / right(-)
        # wz: CCW(+) / CW(-)
        ang = speed * ANGULAR_SPEED_FACTOR
        twist_map = {
            'FWD':   ( speed,  0,      0),
            'BWD':   (-speed,  0,      0),
            'LEFT':  ( 0,      speed,  0),
            'RIGHT': ( 0,     -speed,  0),
            'FL':    ( speed,  speed,  0),
            'FR':    ( speed, -speed,  0),
            'BL':    (-speed,  speed,  0),
            'BR':    (-speed, -speed,  0),
            'TL':    ( 0,      0,      ang),
            'TR':    ( 0,      0,     -ang),
        }
        vx, vy, wz = twist_map.get(direction, (0, 0, 0))

        # Send initial command
        self._send_twist(vx, vy, wz)

        # Show status
        self._drive_status.set_markup(
            f"<span foreground='#50fa7b'>{direction} at {speed:.2f} m/s "
            f"for {duration:.1f}s</span>")

        # Repeat every 150ms to keep firmware watchdog alive
        self._drive_repeat_timer = GLib.timeout_add(
            VEL_REPEAT_MS, self._drive_repeat_tick, vx, vy, wz)

        # Auto-stop after duration
        self._drive_stop_timer = GLib.timeout_add(duration_ms, self._drive_auto_stop)

    def _drive_repeat_tick(self, vx, vy, wz):
        """Resend velocity command to keep firmware watchdog alive."""
        self._send_twist(vx, vy, wz)
        return True  # keep repeating

    def _drive_auto_stop(self):
        """Auto-stop after duration elapsed."""
        self._cancel_drive_timers()
        self._state.send_ros_command({'type': 'stop_robot'})
        self._drive_status.set_markup(
            "<span foreground='#888'>Done</span>")
        return False  # do not repeat

    def _on_drive_stop(self, _btn):
        """Emergency stop button."""
        self._cancel_drive_timers()
        self._state.send_ros_command({'type': 'stop_robot'})
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': 'STOP'})
        self._drive_status.set_markup(
            "<span foreground='#ff6b6b'>STOPPED</span>")

    def _cancel_drive_timers(self):
        """Cancel any active drive repeat/stop timers."""
        for attr in ('_drive_repeat_timer', '_drive_stop_timer'):
            timer_id = getattr(self, attr, None)
            if timer_id is not None:
                GLib.source_remove(timer_id)
                setattr(self, attr, None)

    def _send_twist(self, vx, vy, wz):
        """Send a test_motor_twist command with m/s values."""
        self._state.send_ros_command({
            'type': 'test_motor_twist',
            'vx': float(vx),
            'vy': float(vy),
            'wz': float(wz),
        })

    # ================================================================
    # Motors sub-tab
    # ================================================================

    def _build_motors_subtab(self):
        """Test individual motors with TMOTOR,id,pwm command."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_margin_top(6)
        box.set_margin_start(6)
        box.set_margin_end(6)

        self._motor_test_timer = None

        # -- Info --
        box.pack_start(self._make_section("Single Motor Test"), False, False, 0)
        info = Gtk.Label()
        info.set_markup(
            "<span size='small' foreground='#888'>"
            "Sends TMOTOR,id,pwm directly to Arduino firmware.\n"
            "Motor runs for the set duration then auto-stops.\n"
            "PWM 0-200 (safe range). Negative = reverse.</span>")
        info.set_xalign(0)
        info.set_margin_start(10)
        info.set_line_wrap(True)
        box.pack_start(info, False, False, 2)

        # -- PWM slider --
        box.pack_start(self._make_section("PWM"), False, False, 0)
        pwm_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        pwm_row.set_margin_start(10)
        self._motor_pwm_adj = Gtk.Adjustment(
            value=DEFAULT_MOTOR_PWM, lower=MIN_MOTOR_PWM, upper=MAX_MOTOR_PWM,
            step_increment=MOTOR_PWM_STEP, page_increment=50)
        pwm_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL,
            adjustment=self._motor_pwm_adj)
        pwm_scale.set_digits(0)
        pwm_scale.set_value_pos(Gtk.PositionType.RIGHT)
        pwm_scale.set_hexpand(True)
        pwm_row.pack_start(pwm_scale, True, True, 0)
        box.pack_start(pwm_row, False, False, 0)

        # -- Direction toggle --
        box.pack_start(self._make_section("Direction"), False, False, 0)
        dir_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        dir_row.set_margin_start(10)
        self._motor_fwd_btn = Gtk.RadioButton.new_with_label(None, "Forward")
        self._motor_rev_btn = Gtk.RadioButton.new_with_label_from_widget(
            self._motor_fwd_btn, "Reverse")
        self._motor_fwd_btn.set_active(True)
        dir_row.pack_start(self._motor_fwd_btn, False, False, 0)
        dir_row.pack_start(self._motor_rev_btn, False, False, 0)
        box.pack_start(dir_row, False, False, 0)

        # -- Duration --
        box.pack_start(self._make_section("Duration (seconds)"), False, False, 0)
        mdur_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        mdur_row.set_margin_start(10)
        self._motor_dur_spin = Gtk.SpinButton.new_with_range(0.5, 5.0, 0.5)
        self._motor_dur_spin.set_value(DEFAULT_MOTOR_DURATION)
        self._motor_dur_spin.set_digits(1)
        mdur_row.pack_start(self._motor_dur_spin, False, False, 0)
        box.pack_start(mdur_row, False, False, 0)

        # -- Motor buttons --
        box.pack_start(self._make_section("Motor"), False, False, 2)
        motor_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        motor_row.set_margin_start(10)
        for motor_id in MOTOR_IDS:
            btn = Gtk.Button(label=motor_id)
            btn.set_size_request(70, 40)
            btn.connect('clicked', self._on_motor_test, motor_id)
            motor_row.pack_start(btn, False, False, 0)

        stop_btn = Gtk.Button(label="STOP")
        stop_btn.set_size_request(70, 40)
        stop_btn.get_style_context().add_class('destructive-action')
        stop_btn.connect('clicked', self._on_motor_stop)
        motor_row.pack_start(stop_btn, False, False, 0)
        box.pack_start(motor_row, False, False, 2)

        # -- Status --
        self._motor_status = Gtk.Label(label="")
        self._motor_status.set_xalign(0)
        self._motor_status.set_margin_start(10)
        box.pack_start(self._motor_status, False, False, 4)

        return box

    def _on_motor_test(self, _btn, motor_id):
        """Send TMOTOR,id,pwm and schedule auto-stop after duration."""
        # Cancel any previous motor test timer
        self._cancel_motor_test_timer()

        pwm = int(self._motor_pwm_adj.get_value())
        if self._motor_rev_btn.get_active():
            pwm = -pwm
        duration = self._motor_dur_spin.get_value()
        duration_ms = int(duration * 1000)

        cmd = f"TMOTOR,{motor_id},{pwm}"
        logger.info("Motor test: %s for %dms", cmd, duration_ms)
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': cmd})

        self._motor_status.set_markup(
            f"<span foreground='#50fa7b'>{motor_id} at PWM {pwm} "
            f"for {duration:.1f}s</span>")

        # Auto-stop after duration
        self._motor_test_timer = GLib.timeout_add(
            duration_ms, self._motor_test_auto_stop, motor_id)

    def _motor_test_auto_stop(self, motor_id):
        """Stop motor after test duration."""
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': 'STOP'})
        self._motor_status.set_markup(
            f"<span foreground='#888'>{motor_id} done</span>")
        self._motor_test_timer = None
        return False

    def _on_motor_stop(self, _btn):
        """Stop all motors immediately."""
        self._cancel_motor_test_timer()
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': 'STOP'})
        self._motor_status.set_markup(
            "<span foreground='#ff6b6b'>STOPPED</span>")

    def _cancel_motor_test_timer(self):
        """Cancel any running motor test timer."""
        if getattr(self, '_motor_test_timer', None) is not None:
            GLib.source_remove(self._motor_test_timer)
            self._motor_test_timer = None

    # ================================================================
    # System sub-tab
    # ================================================================

    def _build_system_subtab(self):
        """System tools: reset odometry, calibration, read encoders."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_margin_top(6)
        box.set_margin_start(6)
        box.set_margin_end(6)

        # -- Odometry Reset --
        box.pack_start(self._make_section("Odometry"), False, False, 0)
        odom_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        odom_row.set_margin_start(10)
        odom_btn = Gtk.Button(label="RESET ODOMETRY")
        odom_btn.set_size_request(160, 40)
        odom_btn.connect('clicked', self._on_reset_odom)
        odom_row.pack_start(odom_btn, False, False, 0)
        self._sys_odom_status = Gtk.Label(label="")
        odom_row.pack_start(self._sys_odom_status, True, True, 0)
        box.pack_start(odom_row, False, False, 4)

        # -- Encoders --
        box.pack_start(self._make_section("Encoders"), False, False, 0)
        enc_info = Gtk.Label()
        enc_info.set_markup(
            "<span size='small' foreground='#888'>"
            "READ: snapshot of all 4 encoder counters.\n"
            "TENC: stream mode -- spin wheels to see encoder values.</span>")
        enc_info.set_xalign(0)
        enc_info.set_margin_start(10)
        enc_info.set_line_wrap(True)
        box.pack_start(enc_info, False, False, 2)

        enc_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        enc_row.set_margin_start(10)
        read_btn = Gtk.Button(label="READ ENCODERS")
        read_btn.set_size_request(140, 40)
        read_btn.connect('clicked', self._on_sys_arduino_cmd, 'READ')
        enc_row.pack_start(read_btn, False, False, 0)

        tenc_btn = Gtk.Button(label="ENCODER TEST")
        tenc_btn.set_size_request(140, 40)
        tenc_btn.connect('clicked', self._on_sys_arduino_cmd, 'TENC')
        enc_row.pack_start(tenc_btn, False, False, 0)

        enc_stop = Gtk.Button(label="STOP")
        enc_stop.set_size_request(80, 40)
        enc_stop.get_style_context().add_class('destructive-action')
        enc_stop.connect('clicked', self._on_sys_arduino_cmd, 'STOP')
        enc_row.pack_start(enc_stop, False, False, 0)
        box.pack_start(enc_row, False, False, 4)

        self._sys_enc_status = Gtk.Label(label="")
        self._sys_enc_status.set_xalign(0)
        self._sys_enc_status.set_margin_start(10)
        box.pack_start(self._sys_enc_status, False, False, 2)

        # -- Calibration --
        box.pack_start(self._make_section("Motor Calibration"), False, False, 0)
        cal_info = Gtk.Label()
        cal_info.set_markup(
            "<span size='small' foreground='#888'>"
            "Runs ~40s calibration routine:\n"
            "  1. Dead-zone detection (per motor minimum PWM)\n"
            "  2. Forward speed calibration\n"
            "  3. Reverse speed calibration\n"
            "Results saved to Arduino EEPROM.\n"
            "<b>LIFT ROBOT OFF GROUND before starting!</b></span>")
        cal_info.set_xalign(0)
        cal_info.set_margin_start(10)
        cal_info.set_line_wrap(True)
        box.pack_start(cal_info, False, False, 2)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        cal_row.set_margin_start(10)
        calib_btn = Gtk.Button(label="START CALIBRATION")
        calib_btn.set_size_request(160, 40)
        calib_btn.get_style_context().add_class('suggested-action')
        calib_btn.connect('clicked', self._on_sys_arduino_cmd, 'CALIB')
        cal_row.pack_start(calib_btn, False, False, 0)

        abort_btn = Gtk.Button(label="ABORT")
        abort_btn.set_size_request(80, 40)
        abort_btn.get_style_context().add_class('destructive-action')
        abort_btn.connect('clicked', self._on_sys_arduino_cmd, 'STOP')
        cal_row.pack_start(abort_btn, False, False, 0)
        box.pack_start(cal_row, False, False, 4)

        self._sys_cal_status = Gtk.Label(label="")
        self._sys_cal_status.set_xalign(0)
        self._sys_cal_status.set_margin_start(10)
        box.pack_start(self._sys_cal_status, False, False, 4)

        return box

    def _on_reset_odom(self, _btn):
        """Reset wheel odometry to origin."""
        self._state.send_ros_command({'type': 'reset_odom'})
        self._sys_odom_status.set_markup(
            "<span foreground='#50fa7b'>Odometry reset to (0, 0, 0)</span>")

    def _on_sys_arduino_cmd(self, _btn, cmd):
        """Send raw Arduino command and update appropriate status label."""
        logger.info("Arduino cmd: %s", cmd)
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': cmd})

        if cmd == 'CALIB':
            self._sys_cal_status.set_markup(
                "<span foreground='#ffaa00'>Calibrating ~40s... LIFT ROBOT!</span>")
        elif cmd == 'STOP':
            self._cancel_motor_test_timer()
            for attr in ('_sys_cal_status', '_sys_enc_status'):
                lbl = getattr(self, attr, None)
                if lbl:
                    lbl.set_markup(
                        "<span foreground='#ff6b6b'>STOP sent</span>")
        elif cmd == 'READ':
            self._sys_enc_status.set_markup(
                "<span foreground='#888'>Reading encoders...</span>")
        elif cmd == 'TENC':
            self._sys_enc_status.set_markup(
                "<span foreground='#888'>Encoder test mode (spin wheels)</span>")

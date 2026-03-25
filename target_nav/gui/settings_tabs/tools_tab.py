"""Hardware Tools tab for SettingsWindow.

Provides three sub-tabs for robot hardware testing and calibration:

    Drive:   8-direction pad + rotation, sends firmware position commands
             directly (FWD,speed,ticks / TURN,speed,ticks / DIAGFL,speed,ticks).
             User controls PWM speed (20-255) and ticks. No unit conversions.
    Motors:  Test individual motors with raw TMOTOR,id,pwm commands.
    System:  Reset odometry, motor calibration (~40s).

Command path (Drive / Motors / System tabs):
    GUI click -> send_ros_command({'type': 'arduino_cmd', 'data': 'FWD,100,1719'})
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

# Drive tab defaults (firmware units: PWM 20-255, ticks)
DEFAULT_DRIVE_SPEED = 100       # PWM (safe default)
MIN_DRIVE_SPEED = 20            # PWM (below this, motors stall)
MAX_DRIVE_SPEED = 255           # PWM max
DRIVE_SPEED_STEP = 5

DEFAULT_DRIVE_TICKS = 1719      # ~10cm (17.19 ticks/mm)
MIN_DRIVE_TICKS = 100           # ~0.6cm
MAX_DRIVE_TICKS = 17190         # ~100cm
DRIVE_TICKS_STEP = 100

# Motor test tab defaults
DEFAULT_MOTOR_PWM = 100     # safe PWM
MIN_MOTOR_PWM = 0
MAX_MOTOR_PWM = 200         # protect motors (max safe PWM)
MOTOR_PWM_STEP = 10

DEFAULT_MOTOR_DURATION = 2.0  # seconds

# Calibration countdown (estimate with safety margin, firmware timeout is 60s)
CALIB_TIMEOUT_S = 60

# Motor IDs matching firmware TMOTOR command (FL/FR/RL/RR)
MOTOR_IDS = ['FL', 'FR', 'RL', 'RR']


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
        """8-direction pad + rotation with PWM speed and ticks sliders."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_margin_top(6)
        box.set_margin_start(6)
        box.set_margin_end(6)

        # -- Speed slider (PWM 20-255) --
        box.pack_start(self._make_section("Speed (PWM 20-255)"), False, False, 0)
        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        speed_row.set_margin_start(10)
        self._drive_speed_adj = Gtk.Adjustment(
            value=DEFAULT_DRIVE_SPEED, lower=MIN_DRIVE_SPEED, upper=MAX_DRIVE_SPEED,
            step_increment=DRIVE_SPEED_STEP, page_increment=25)
        speed_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL,
            adjustment=self._drive_speed_adj)
        speed_scale.set_digits(0)
        speed_scale.set_value_pos(Gtk.PositionType.RIGHT)
        speed_scale.set_hexpand(True)
        speed_row.pack_start(speed_scale, True, True, 0)
        box.pack_start(speed_row, False, False, 0)

        # -- Ticks slider --
        box.pack_start(self._make_section("Distance (ticks, ~17.19 ticks/mm)"), False, False, 0)
        ticks_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        ticks_row.set_margin_start(10)
        self._drive_ticks_adj = Gtk.Adjustment(
            value=DEFAULT_DRIVE_TICKS, lower=MIN_DRIVE_TICKS, upper=MAX_DRIVE_TICKS,
            step_increment=DRIVE_TICKS_STEP, page_increment=500)
        ticks_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL,
            adjustment=self._drive_ticks_adj)
        ticks_scale.set_digits(0)
        ticks_scale.set_value_pos(Gtk.PositionType.RIGHT)
        ticks_scale.set_hexpand(True)
        ticks_row.pack_start(ticks_scale, True, True, 0)
        box.pack_start(ticks_row, False, False, 0)

        # -- Approx distance label --
        self._drive_dist_label = Gtk.Label()
        self._drive_dist_label.set_xalign(0)
        self._drive_dist_label.set_margin_start(10)
        self._update_dist_label()
        self._drive_ticks_adj.connect('value-changed', lambda _: self._update_dist_label())
        box.pack_start(self._drive_dist_label, False, False, 0)

        # -- Direction grid --
        box.pack_start(self._make_section("Direction (click to move)"), False, False, 2)
        grid = Gtk.Grid()
        grid.set_column_spacing(4)
        grid.set_row_spacing(4)
        grid.set_margin_start(10)
        grid.set_halign(Gtk.Align.CENTER)

        # Layout: 4 rows x 3 cols
        #   Row 0: DIAGFL  FWD     DIAGFR
        #   Row 1: LEFT    STOP    RIGHT
        #   Row 2: DIAGBL  BWD     DIAGBR
        #   Row 3: TL      ---     TR
        # Button labels are short for the GUI, firmware commands are mapped below
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

    def _update_dist_label(self):
        """Update the approximate distance label from current ticks value."""
        ticks = int(self._drive_ticks_adj.get_value())
        mm = ticks / 17.19
        self._drive_dist_label.set_markup(
            f"<span size='small' foreground='#888'>~ {mm:.0f} mm ({mm/10:.1f} cm)</span>")

    def _on_drive_direction(self, _btn, direction):
        """Send firmware position command: CMD,speed,ticks."""
        speed = int(self._drive_speed_adj.get_value())
        ticks = int(self._drive_ticks_adj.get_value())

        # Map GUI button labels to firmware command names
        cmd_map = {
            'FWD':   'FWD',
            'BWD':   'BWD',
            'LEFT':  'LEFT',
            'RIGHT': 'RIGHT',
            'FL':    'DIAGFL',
            'FR':    'DIAGFR',
            'BL':    'DIAGBL',
            'BR':    'DIAGBR',
            'TL':    'TURN',      # TURN,speed,ticks (positive = CCW)
            'TR':    'TURN',      # TURN,speed,-ticks (negative = CW)
        }

        firmware_cmd = cmd_map.get(direction, 'STOP')

        # TURN uses sign of ticks for direction: positive=CCW, negative=CW
        if direction == 'TR':
            ticks = -ticks

        cmd = f"{firmware_cmd},{speed},{ticks}"
        logger.info("Drive cmd: %s", cmd)
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': cmd})

        mm = abs(ticks) / 17.19
        self._drive_status.set_markup(
            f"<span foreground='#50fa7b'>{cmd} (~{mm:.0f}mm)</span>")

    def _on_drive_stop(self, _btn):
        """Emergency stop."""
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': 'STOP'})
        self._drive_status.set_markup(
            "<span foreground='#ff6b6b'>STOPPED</span>")

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
        """System tools: reset odometry, motor calibration."""
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

        # -- Calibration --
        box.pack_start(self._make_section("Motor Calibration"), False, False, 0)
        cal_info = Gtk.Label()
        cal_info.set_markup(
            "<span foreground='#888'>"
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
        """Send raw Arduino command and update calibration status label."""
        logger.info("Arduino cmd: %s", cmd)
        self._state.send_ros_command({'type': 'arduino_cmd', 'data': cmd})

        if cmd == 'CALIB':
            self._cancel_calib_timer()
            self._calib_remaining = CALIB_TIMEOUT_S
            self._calib_timer = GLib.timeout_add(1000, self._calib_tick)
            self._update_calib_label()
        elif cmd == 'STOP':
            self._cancel_motor_test_timer()
            self._cancel_calib_timer()
            self._sys_cal_status.set_markup(
                "<span foreground='#ff6b6b'>Aborted</span>")

    def _update_calib_label(self):
        """Update the calibration countdown label."""
        remaining = self._calib_remaining
        self._sys_cal_status.set_markup(
            f"<span foreground='#ffaa00'>Calibrating... "
            f"{remaining}s remaining</span>")

    def _calib_tick(self):
        """Countdown timer for calibration (1Hz)."""
        self._calib_remaining -= 1

        if self._calib_remaining <= 0:
            self._sys_cal_status.set_markup(
                "<span foreground='#50fa7b'>Calibration should be complete. "
                "Results saved to Arduino EEPROM.</span>")
            self._calib_timer = None
            return False

        self._update_calib_label()
        return True

    def _cancel_calib_timer(self):
        """Cancel calibration countdown timer."""
        timer = getattr(self, '_calib_timer', None)
        if timer is not None:
            GLib.source_remove(timer)
            self._calib_timer = None

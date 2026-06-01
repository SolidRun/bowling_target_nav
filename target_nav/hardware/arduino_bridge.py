#!/usr/bin/env python3
"""
Arduino Serial Bridge for ROS2 Driver Node
===========================================

Serial bridge used by the ArduinoDriverNode for non-blocking communication
with the Arduino mecanum motor controller firmware. Provides:

  - Background read thread: parses telemetry (ODOM, ENC) and command
    responses (READY, OK, DONE, BUSY, ERROR) via on_response callback.
  - Background reconnect thread: auto-reconnects with exponential backoff
    (up to 30 s) when the serial port drops.
  - Auto-detection: scans /dev/ttyACM* and /dev/ttyUSB* for known Arduino
    vendor IDs (Arduino, CH340, CP210x, FTDI).

Firmware Protocol (plain text, NO checksums):
  TX: CMD[,arg1,arg2,...]\\n
  RX: READY | OK | DONE | BUSY | ERROR: msg
  Telemetry: ODOM,vx,vy,wz | ENC,FL:t,RL:t,RR:t,FR:t,t_us:t
  Watchdog: 200 ms in VEL mode (must resend VEL before timeout or motors stop)

Related modules:
    hardware/arduino.py   -- blocking model, protocol constants, MockArduino.
    app/arduino_node.py   -- ArduinoDriverNode that owns this bridge.
"""

import glob
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable

import serial
from serial.tools import list_ports

from target_nav.hardware.arduino import DEFAULT_ARDUINO_PORT, DEFAULT_BAUDRATE


class ArduinoState(Enum):
    """Connection state machine for the Arduino serial bridge."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class ArduinoConfig:
    """Configuration for Arduino connection."""
    port: str = DEFAULT_ARDUINO_PORT
    baudrate: int = DEFAULT_BAUDRATE
    timeout: float = 0.1
    reconnect_interval: float = 2.0
    command_timeout: float = 0.3
    max_retries: int = 3


class ArduinoBridge:
    """Non-blocking serial bridge with background threads for the Arduino.

    Used by ArduinoDriverNode (ROS2 process) for continuous communication
    with the motor controller firmware. Two daemon threads run in the
    background:

        _read_loop: Reads and parses incoming serial lines. ODOM and ENC
            telemetry plus command responses (OK, DONE, ERROR, etc.) are
            dispatched to the ``on_response`` callback.

        _reconnect_loop: Monitors connection state and reconnects with
            exponential backoff (2s initial, 30s max) when the USB serial
            port drops (e.g. cable disconnect or Arduino reset).

    The firmware has a 200ms watchdog in VEL mode -- ``send_velocity()``
    must be called at least every 200ms or motors will auto-stop.
    VEL commands use mm/s and mrad/s units; the firmware converts to
    motor PWM internally using encoder feedback.

    Auto-detection probes ``/dev/ttyACM*`` and ``/dev/ttyUSB*`` ports and
    keeps only the one that answers the firmware handshake (see
    ``_probe_port``). Identity is decided by the handshake, NOT by USB
    vendor ID -- so a peripheral that merely shares a USB-serial chip
    (e.g. an RPLidar on a CP2102) is never mistaken for the Arduino.
    The vendor-ID tiers below only set the *order* ports are probed in.
    """

    # VID tiers control probe ORDER only -- never device identity.
    # Tier 1: chips used almost exclusively by Arduino boards.
    VID_TIER1 = (0x2341, 0x2A03, 0x1A86)   # Arduino LLC, Arduino SRL, CH340
    # Tier 2: generic USB-serial chips (could be an Arduino OR a peripheral).
    VID_TIER2 = (0x10C4, 0x0403)           # CP210x, FTDI

    def __init__(
        self,
        config: Optional[ArduinoConfig] = None,
        on_response: Optional[Callable[[str, list], None]] = None,
        on_state_change: Optional[Callable[[ArduinoState], None]] = None
    ):
        """Initialize the bridge (does not open serial port yet).

        Args:
            config: Serial port configuration. Uses defaults if None.
            on_response: Callback(command_str, args_list) for each parsed line.
            on_state_change: Callback(ArduinoState) on connection state changes.
        """
        self.config = config or ArduinoConfig()
        self.on_response = on_response
        self.on_state_change = on_state_change

        self._serial: Optional[serial.Serial] = None
        self._state = ArduinoState.DISCONNECTED
        self._lock = threading.Lock()
        self._running = False

        self._read_thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None

        self._last_command_time = 0.0

        self.stats = {
            'tx_count': 0,
            'rx_count': 0,
            'errors': 0,
            'reconnects': 0,
        }

    @property
    def state(self) -> ArduinoState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._state == ArduinoState.CONNECTED

    @property
    def actual_port(self) -> Optional[str]:
        """The serial device actually opened, or None if not connected.

        Differs from ``config.port`` when ``find_arduino_port()`` auto-detected
        a device other than the configured/default one. Use this (not
        ``config.port``) for logging and status so the reported port reflects
        the real hardware connection.
        """
        s = self._serial
        return s.port if s is not None else None

    def _set_state(self, new_state: ArduinoState):
        """Transition to a new state and notify the callback if changed."""
        if new_state != self._state:
            self._state = new_state
            if self.on_state_change:
                try:
                    self.on_state_change(new_state)
                except Exception:
                    pass

    @staticmethod
    def _port_in_use(device: str) -> bool:
        """True if another process currently holds this serial device open.

        Prevents the probe from opening a port already owned by another
        driver (e.g. the lidar's rplidar_node), which would disrupt that
        device mid-operation. This is what keeps Arduino auto-detection from
        ever touching the lidar -- without hardcoding the lidar's port.

        Best-effort: scans /proc/*/fd for an open handle to the device
        (requires permission to read other processes' fds; the robot runs
        as root). Returns False if it cannot determine usage.

        Args:
            device: Serial device path (e.g. '/dev/ttyUSB0').
        """
        try:
            target = os.path.realpath(device)
        except OSError:
            return False
        for fd in glob.glob('/proc/[0-9]*/fd/*'):
            try:
                if os.path.realpath(fd) == target:
                    return True
            except OSError:
                continue  # fd vanished or permission denied -- ignore
        return False

    def _probe_port(self, device: str) -> bool:
        """Open a port briefly and check whether it speaks the motor firmware.

        Sends the harmless ``READ`` query (no motion) and accepts the port
        only if the reply matches the firmware protocol. This is what tells
        the Arduino apart from any other USB-serial device (e.g. the lidar),
        independent of which USB-serial chip the board uses.

        Args:
            device: Serial device path to probe (e.g. '/dev/ttyUSB1').

        Returns:
            True if the device responded like the motor firmware.
        """
        try:
            probe = serial.Serial(
                device, self.config.baudrate, timeout=0.3, write_timeout=0.3
            )
        except Exception:
            return False

        try:
            time.sleep(2.0)              # boards that reset on open need to boot
            probe.reset_input_buffer()
            probe.write(b"READ\n")
            probe.flush()
            time.sleep(0.3)
            reply = probe.read(256).decode("ascii", errors="ignore")
            if any(tok in reply for tok in ("READY", "OK", "ODOM", "ENC", "DONE")):
                return True
            # Encoder reply is bare integers, one per line.
            return any(
                ln.strip().lstrip("-").isdigit()
                for ln in reply.splitlines() if ln.strip()
            )
        except Exception:
            return False
        finally:
            try:
                probe.close()
            except Exception:
                pass

    def find_arduino_port(self) -> Optional[str]:
        """Find the serial port that actually speaks the motor firmware.

        Probes candidate ports in priority order (configured port first,
        then likely-Arduino chips, then generic serial chips) and returns
        the FIRST one that answers the firmware handshake. Because identity
        is confirmed by the handshake, a peripheral that shares a USB-serial
        chip (e.g. the RPLidar) can never be selected -- and probing stops
        as soon as the Arduino is found, so the lidar is normally never
        touched.

        Returns:
            Device path string (e.g. '/dev/ttyUSB1') or None if nothing
            responded like the firmware.
        """
        ports = list(list_ports.comports())

        def rank(p):
            if p.device == self.config.port:
                return 0
            if p.vid in self.VID_TIER1:
                return 1
            if p.vid in self.VID_TIER2:
                return 2
            return 3

        for port in sorted(ports, key=rank):
            # Never probe a port another process already owns (e.g. the lidar).
            # Opening it would toggle DTR and inject bytes, disrupting that
            # device. This is what keeps detection from ever touching the lidar.
            if self._port_in_use(port.device):
                continue
            if self._probe_port(port.device):
                return port.device

        return None

    def connect(self) -> bool:
        """Open serial port, wait up to 5s for firmware READY, and reset buffers.

        Returns:
            True if connected (even if READY was missed -- firmware may
            already be past the boot message).
        """
        with self._lock:
            if self._serial and self._serial.is_open:
                return True

            self._set_state(ArduinoState.CONNECTING)

            port = self.find_arduino_port()
            if not port:
                self._set_state(ArduinoState.ERROR)
                return False

            try:
                self._serial = serial.Serial(
                    port=port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.timeout,
                    write_timeout=self.config.timeout
                )

                # Wait for Arduino READY after reset
                ready = False
                start_time = time.time()
                while time.time() - start_time < 5.0:
                    if self._serial.in_waiting:
                        try:
                            line = self._serial.readline().decode('ascii', errors='ignore').strip()
                            if 'READY' in line:
                                ready = True
                                break
                        except Exception:
                            pass
                    time.sleep(0.05)

                if not ready:
                    time.sleep(0.5)

                self._serial.reset_input_buffer()
                self._set_state(ArduinoState.CONNECTED)
                return True

            except serial.SerialException:
                self._set_state(ArduinoState.ERROR)
                return False

    def disconnect(self):
        """Send STOP, flush, and close the serial port."""
        with self._lock:
            if self._serial:
                try:
                    self._serial.write(b"STOP\n")
                    self._serial.flush()
                    time.sleep(0.05)
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._set_state(ArduinoState.DISCONNECTED)

    def start(self):
        """Start background read and reconnect threads."""
        if self._running:
            return

        self._running = True

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self._reconnect_thread = threading.Thread(target=self._reconnect_loop, daemon=True)
        self._reconnect_thread.start()

    def stop(self):
        """Stop background threads and disconnect."""
        self._running = False

        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self._reconnect_thread:
            self._reconnect_thread.join(timeout=1.0)

        self.disconnect()

    def send_command(self, command: str, *args) -> bool:
        """Send plain-text command to Arduino.

        Args:
            command: Command name (e.g., "VEL", "STOP")
            *args: Command arguments

        Returns:
            True if sent successfully
        """
        if not self.is_connected:
            return False

        if args:
            packet = f"{command},{','.join(str(a) for a in args)}\n".encode('ascii')
        else:
            packet = f"{command}\n".encode('ascii')

        with self._lock:
            try:
                if self._serial and self._serial.is_open:
                    self._serial.write(packet)
                    self._serial.flush()
                    self.stats['tx_count'] += 1
                    self._last_command_time = time.time()
                    return True
            except serial.SerialException:
                self.stats['errors'] += 1
                self._set_state(ArduinoState.ERROR)

        return False

    def send_velocity(self, vx_mm: int, vy_mm: int, wz_mrad: int) -> bool:
        """Send VEL command for continuous mecanum velocity.

        Must be resent within 200ms (firmware watchdog).
        The firmware converts mm/s to motor PWM internally using
        encoder feedback.

        Args:
            vx_mm: forward/backward velocity in mm/s
            vy_mm: left/right strafe velocity in mm/s
            wz_mrad: rotation velocity in mrad/s
        """
        # Deadzone: stop if all velocities are near-zero.
        # 3 mm/s threshold allows creep commands for arrival approach.
        if abs(vx_mm) < 3 and abs(vy_mm) < 3 and abs(wz_mrad) < 3:
            return self.send_stop()

        return self.send_command("VEL", vx_mm, vy_mm, wz_mrad)

    def send_move(self, direction: str, speed: int, ticks: int) -> bool:
        """Send a timed movement command (encoder-counted).

        Args:
            direction: Movement direction (FWD, BWD, LEFT, RIGHT, DIAG*).
            speed: PWM value 20-255 (clamped).
            ticks: Encoder ticks > 0 (clamped).

        Returns:
            True if sent successfully.
        """
        speed = max(20, min(255, speed))
        ticks = max(1, ticks)
        return self.send_command(direction, speed, ticks)

    def send_turn(self, speed: int, ticks: int, clockwise: bool = False) -> bool:
        """Send a rotation-in-place command.

        Args:
            speed: PWM value 20-255 (clamped).
            ticks: Encoder ticks (absolute value used).
            clockwise: If True, negate ticks for CW rotation.

        Returns:
            True if sent successfully.
        """
        speed = max(20, min(255, speed))
        ticks = max(1, abs(ticks))
        if clockwise:
            ticks = -ticks
        return self.send_command("TURN", speed, ticks)

    def send_stop(self) -> bool:
        """Send emergency stop."""
        return self.send_command("STOP")

    def _read_loop(self):
        """Background thread reading serial responses and telemetry.

        Parses firmware responses:
        - ODOM,vx,vy,wz (VEL mode telemetry at 20Hz)
        - ENC,FL:xxx,RL:xxx,RR:xxx,FR:xxx,t_us:xxx (encoder telemetry)
        - DONE, OK, BUSY, ERROR, READY (command responses)
        - CALIB,... (calibration progress)
        - STALL,... (stall detection)
        """
        while self._running:
            if not self.is_connected or not self._serial:
                time.sleep(0.1)
                continue

            try:
                if self._serial.in_waiting:
                    line = self._serial.readline()
                else:
                    line = None

                if line:
                    text = line.decode('ascii', errors='ignore').strip()
                    if text:
                        self.stats['rx_count'] += 1
                        if self.on_response:
                            parts = text.split(',') if ',' in text else [text]
                            self.on_response(parts[0], parts[1:])
                else:
                    time.sleep(0.005)

            except serial.SerialException:
                self.stats['errors'] += 1
                self._set_state(ArduinoState.ERROR)
            except Exception:
                time.sleep(0.01)

    def _reconnect_loop(self):
        """Background thread for automatic reconnection with exponential backoff.

        Starts at ``reconnect_interval`` (default 2s) and grows by 1.5x
        on each failure, capped at 30 seconds. Resets to the base interval
        on successful reconnection. This avoids hammering a disconnected
        port while still recovering quickly when the Arduino comes back.
        """
        backoff = self.config.reconnect_interval

        while self._running:
            if self._state in (ArduinoState.DISCONNECTED, ArduinoState.ERROR):
                if self.connect():
                    self.stats['reconnects'] += 1
                    backoff = self.config.reconnect_interval  # Reset backoff on success
                else:
                    backoff = min(backoff * 1.5, 30.0)  # Exponential backoff, max 30s

            time.sleep(backoff)

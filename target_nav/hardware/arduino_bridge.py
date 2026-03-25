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

    Auto-detection scans ``/dev/ttyACM*`` and ``/dev/ttyUSB*`` for known
    Arduino vendor IDs (0x2341 Arduino, 0x1A86 CH340, 0x10C4 CP210x,
    0x0403 FTDI).
    """

    ARDUINO_VIDS = [0x2341, 0x1A86, 0x10C4, 0x0403]

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

    def _set_state(self, new_state: ArduinoState):
        """Transition to a new state and notify the callback if changed."""
        if new_state != self._state:
            self._state = new_state
            if self.on_state_change:
                try:
                    self.on_state_change(new_state)
                except Exception:
                    pass

    def find_arduino_port(self) -> Optional[str]:
        """Auto-detect Arduino serial port by trying configured port, then VID match.

        Returns:
            Device path string (e.g. '/dev/ttyACM0') or None.
        """
        ports = list_ports.comports()

        # Try configured port first
        for port in ports:
            if port.device == self.config.port:
                return port.device

        # Try known Arduino vendor IDs
        for port in ports:
            if port.vid in self.ARDUINO_VIDS:
                return port.device

        # Try common patterns
        for port in ports:
            if 'ACM' in port.device or 'USB' in port.device:
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

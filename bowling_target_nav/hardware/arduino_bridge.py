#!/usr/bin/env python3
"""
Arduino Serial Bridge for ROS2 Driver Node
===========================================

Production-quality serial bridge used by the ArduinoDriverNode.
Provides async read loop, auto-reconnection, and telemetry parsing.

Firmware Protocol (plain text, NO checksums):
  TX: CMD[,arg1,arg2,...]\n
  RX: READY | OK | DONE | BUSY | ERROR: msg
  Telemetry: ODOM,vx,vy,wz | ENC,FL:t,FR:t,RL:t,RR:t,t_us:t
  Watchdog: 200ms in VEL mode
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable

import serial
from serial.tools import list_ports


class ArduinoState(Enum):
    """Connection state machine."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class ArduinoConfig:
    """Configuration for Arduino connection."""
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout: float = 0.1
    reconnect_interval: float = 2.0
    command_timeout: float = 0.3
    max_retries: int = 3


class ArduinoBridge:
    """
    Serial bridge with background read thread and auto-reconnection.

    Used by ArduinoDriverNode for non-blocking communication with the
    Arduino motor controller firmware.
    """

    ARDUINO_VIDS = [0x2341, 0x1A86, 0x10C4, 0x0403]

    def __init__(
        self,
        config: Optional[ArduinoConfig] = None,
        on_response: Optional[Callable[[str, list], None]] = None,
        on_state_change: Optional[Callable[[ArduinoState], None]] = None
    ):
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
        if new_state != self._state:
            self._state = new_state
            if self.on_state_change:
                try:
                    self.on_state_change(new_state)
                except Exception:
                    pass

    def find_arduino_port(self) -> Optional[str]:
        """Auto-detect Arduino serial port."""
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
        """Establish serial connection and wait for READY."""
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
        """Close serial connection."""
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

    def send_velocity(self, vx: int, vy: int, wz: int) -> bool:
        """Send VEL command for continuous mecanum velocity.

        Must be resent within 200ms (firmware watchdog).

        Args:
            vx: forward/backward PWM (-255..255)
            vy: left/right strafe PWM (-255..255)
            wz: rotation PWM (-255..255)
        """
        vx = max(-255, min(255, vx))
        vy = max(-255, min(255, vy))
        wz = max(-255, min(255, wz))

        if abs(vx) < 10 and abs(vy) < 10 and abs(wz) < 10:
            return self.send_stop()

        return self.send_command("VEL", vx, vy, wz)

    def send_move(self, direction: str, speed: int, ticks: int) -> bool:
        """Send timed movement. Firmware requires speed>0 AND ticks>0."""
        speed = max(20, min(255, speed))
        ticks = max(1, ticks)
        return self.send_command(direction, speed, ticks)

    def send_turn(self, speed: int, ticks: int, clockwise: bool = False) -> bool:
        """Send turn. Positive ticks=CCW, negative=CW."""
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
        - ENC,FL:xxx,FR:xxx,RL:xxx,RR:xxx,t_us:xxx (encoder telemetry)
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
        """Background thread for automatic reconnection with backoff."""
        backoff = self.config.reconnect_interval

        while self._running:
            if self._state in (ArduinoState.DISCONNECTED, ArduinoState.ERROR):
                if self.connect():
                    self.stats['reconnects'] += 1
                    backoff = self.config.reconnect_interval  # Reset backoff
                else:
                    backoff = min(backoff * 1.5, 30.0)  # Exponential backoff, max 30s

            time.sleep(backoff)

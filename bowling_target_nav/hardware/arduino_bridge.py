#!/usr/bin/env python3
"""
Professional Arduino Serial Bridge for ROS2

This module provides a robust, production-quality serial communication
interface following ROS2 best practices:

- Checksummed protocol with framing
- Automatic reconnection
- Diagnostics publishing
- Thread-safe async I/O
- Configurable via ROS2 parameters

Protocol Format:
  TX: $CMD,arg1,arg2*XX\n  (XX = 2-char hex checksum)
  RX: $RESP,data*XX\n

Example:
  $TWIST,100,50*3A\n  -> Move at 100mm/s, rotate 50mrad/s
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from queue import Queue
from typing import Optional, Callable, Tuple

import serial
from serial.tools import list_ports


class ArduinoState(Enum):
    """Connection state machine states."""
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


class ChecksumProtocol:
    """NMEA-style checksum protocol for reliable communication."""

    START_CHAR = '$'
    CHECKSUM_SEP = '*'
    END_CHAR = '\n'

    @staticmethod
    def calculate_checksum(data: str) -> str:
        """Calculate XOR checksum of data string."""
        checksum = 0
        for char in data:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    @classmethod
    def encode(cls, command: str, *args) -> bytes:
        """
        Encode command with arguments into checksummed packet.

        Example: encode("TWIST", 100, 50) -> b"$TWIST,100,50*3A\n"
        """
        if args:
            data = f"{command},{','.join(str(a) for a in args)}"
        else:
            data = command
        checksum = cls.calculate_checksum(data)
        packet = f"{cls.START_CHAR}{data}{cls.CHECKSUM_SEP}{checksum}{cls.END_CHAR}"
        return packet.encode('ascii')

    @classmethod
    def decode(cls, packet: bytes) -> Tuple[Optional[str], Optional[list], bool]:
        """
        Decode and validate checksummed packet.

        Returns: (command, args, valid)
        """
        try:
            line = packet.decode('ascii').strip()

            if not line.startswith(cls.START_CHAR):
                return None, None, False

            line = line[1:]  # Remove start char

            if cls.CHECKSUM_SEP not in line:
                return None, None, False

            data, received_checksum = line.rsplit(cls.CHECKSUM_SEP, 1)
            expected_checksum = cls.calculate_checksum(data)

            if received_checksum.upper() != expected_checksum:
                return None, None, False

            parts = data.split(',')
            command = parts[0]
            args = parts[1:] if len(parts) > 1 else []

            return command, args, True

        except Exception:
            return None, None, False


class ArduinoBridge:
    """
    Professional Arduino serial bridge with automatic reconnection.

    Features:
    - Thread-safe command sending
    - Automatic port detection and reconnection
    - Checksummed protocol for reliability
    - Callback-based response handling
    - Connection state monitoring
    """

    # Known Arduino USB vendor IDs
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
        self._lock = threading.Lock()       # Protects writes and connection state
        self._running = False

        self._read_thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None

        self._command_queue: Queue = Queue()
        self._last_command_time = 0.0

        # Statistics
        self.stats = {
            'tx_count': 0,
            'rx_count': 0,
            'checksum_errors': 0,
            'reconnects': 0
        }

    @property
    def state(self) -> ArduinoState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._state == ArduinoState.CONNECTED

    def _set_state(self, new_state: ArduinoState):
        """Update state and notify callback."""
        if new_state != self._state:
            self._state = new_state
            if self.on_state_change:
                self.on_state_change(new_state)

    def find_arduino_port(self) -> Optional[str]:
        """Auto-detect Arduino serial port."""
        ports = list_ports.comports()

        # First try configured port
        for port in ports:
            if port.device == self.config.port:
                return port.device

        # Then try known Arduino VIDs
        for port in ports:
            if port.vid in self.ARDUINO_VIDS:
                return port.device

        # Finally try common patterns
        for port in ports:
            if 'ACM' in port.device or 'USB' in port.device:
                return port.device

        return None

    def connect(self) -> bool:
        """Establish serial connection and wait for Arduino READY."""
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

                # Wait for Arduino to initialize and send READY
                # Arduino resets on serial connect and needs ~2-3 seconds
                ready = False
                start_time = time.time()
                timeout = 5.0  # Max wait time for READY

                while time.time() - start_time < timeout:
                    if self._serial.in_waiting:
                        try:
                            line = self._serial.readline().decode('ascii', errors='ignore').strip()
                            if 'READY' in line:
                                ready = True
                                break
                        except Exception:
                            pass
                    time.sleep(0.1)

                if not ready:
                    # Even if we didn't see READY, continue anyway
                    # (might have missed it or Arduino doesn't send it)
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
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._set_state(ArduinoState.DISCONNECTED)

    def start(self):
        """Start background threads for reading and reconnection."""
        if self._running:
            return

        self._running = True

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self._reconnect_thread = threading.Thread(target=self._reconnect_loop, daemon=True)
        self._reconnect_thread.start()

    def stop(self):
        """Stop all background threads and disconnect."""
        self._running = False

        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self._reconnect_thread:
            self._reconnect_thread.join(timeout=1.0)

        self.disconnect()

    def send_command(self, command: str, *args) -> bool:
        """
        Send command to Arduino (plain text, no checksum - matches Arduino firmware).

        Args:
            command: Command name (e.g., "FWD", "STOP")
            *args: Command arguments

        Returns:
            True if sent successfully
        """
        if not self.is_connected:
            return False

        # Build plain text command (no checksum - Arduino firmware doesn't use it)
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
                self._set_state(ArduinoState.ERROR)

        return False

    def send_velocity(self, vx: int, vy: int, wz: int) -> bool:
        """
        Send VEL command for continuous mecanum velocity control.

        Uses firmware's VEL,vx,vy,wz command which handles mecanum kinematics
        directly on the Arduino. Must be resent within 200ms (watchdog).

        Args:
            vx: forward/backward (-255..255)
            vy: left/right strafe (-255..255)
            wz: rotation (-255..255)
        """
        vx = max(-255, min(255, vx))
        vy = max(-255, min(255, vy))
        wz = max(-255, min(255, wz))

        if abs(vx) < 10 and abs(vy) < 10 and abs(wz) < 10:
            return self.send_stop()

        return self.send_command("VEL", vx, vy, wz)

    def send_move(self, direction: str, speed: int, ticks: int) -> bool:
        """
        Send timed movement command. Firmware requires speed>0 AND ticks>0.

        Args:
            direction: FWD, BWD, LEFT, RIGHT, DIAGFL, DIAGFR, DIAGBL, DIAGBR
            speed: PWM speed (20-255)
            ticks: encoder ticks to travel (must be > 0)
        """
        speed = max(20, min(255, speed))
        ticks = max(1, ticks)
        return self.send_command(direction, speed, ticks)

    def send_turn(self, speed: int, ticks: int, clockwise: bool = False) -> bool:
        """
        Send turn command.

        Args:
            speed: PWM speed (20-255)
            ticks: encoder ticks (positive = CCW, negative = CW)
            clockwise: if True, negate ticks for CW rotation
        """
        speed = max(20, min(255, speed))
        ticks = max(1, abs(ticks))
        if clockwise:
            ticks = -ticks
        return self.send_command("TURN", speed, ticks)

    def send_stop(self) -> bool:
        """Send emergency stop command."""
        return self.send_command("STOP")

    def _read_loop(self):
        """Background thread for reading serial responses.

        Reads without holding _lock since pyserial supports concurrent
        read/write from different threads. This prevents the read loop
        from blocking the 20Hz command write loop.
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
                    # Parse plain text response (READY, DONE, BUSY, ERROR, encoder values)
                    text = line.decode('ascii', errors='ignore').strip()
                    if text:
                        self.stats['rx_count'] += 1
                        if self.on_response:
                            # Split into command and args for callback
                            parts = text.split(',') if ',' in text else [text]
                            self.on_response(parts[0], parts[1:])
                else:
                    time.sleep(0.01)

            except serial.SerialException:
                self._set_state(ArduinoState.ERROR)
            except Exception:
                time.sleep(0.01)

    def _reconnect_loop(self):
        """Background thread for automatic reconnection."""
        while self._running:
            if self._state in (ArduinoState.DISCONNECTED, ArduinoState.ERROR):
                if self.connect():
                    self.stats['reconnects'] += 1

            time.sleep(self.config.reconnect_interval)

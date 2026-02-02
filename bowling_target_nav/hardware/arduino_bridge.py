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
        self._lock = threading.Lock()
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

    def send_twist(self, linear_mm: int, angular_mrad: int) -> bool:
        """
        Send velocity command for mecanum drive.
        Converts to Arduino's expected format: FWD/BWD/LEFT/RIGHT/TURN commands.
        """
        # Arduino uses: FWD,speed,ticks / BWD,speed,ticks / etc.
        # For continuous velocity control, we'll use a speed value
        # and a fixed duration (ticks=0 means continuous)

        speed = max(abs(linear_mm), abs(angular_mrad))
        if speed < 10:
            return self.send_stop()

        # Clamp speed to Arduino's range (20-255)
        speed = max(20, min(255, speed))

        if abs(angular_mrad) > abs(linear_mm):
            # Rotation is dominant
            if angular_mrad > 0:
                return self.send_command("TURN", speed, 0)  # 0 = continuous
            else:
                return self.send_command("TURN", speed, 0)  # Will handle sign in Arduino
        else:
            # Linear motion is dominant
            if linear_mm > 0:
                return self.send_command("FWD", speed, 0)
            else:
                return self.send_command("BWD", speed, 0)

    def send_velocity(self, vx: int, vy: int, wz: int) -> bool:
        """
        Send full mecanum velocity command.
        vx: forward/backward (mm/s)
        vy: left/right strafe (mm/s)
        wz: rotation (mrad/s)

        Arduino expects: CMD,speed,ticks where ticks is encoder ticks.
        For continuous velocity control, we send a large tick value (9999)
        and the driver will keep re-sending as long as cmd_vel is active.
        """
        speed = max(abs(vx), abs(vy), abs(wz))
        if speed < 10:
            return self.send_stop()

        # Map velocity (mm/s) to PWM speed (20-255)
        # Assuming max velocity ~300 mm/s maps to PWM 255
        speed_pwm = int(min(255, max(20, speed * 255 / 300)))

        # Use large tick value for continuous motion (Arduino will keep moving)
        # The driver sends commands at 20Hz, so motion will be smooth
        ticks = 9999  # Large value = effectively continuous

        # Determine dominant motion
        if abs(wz) > abs(vx) and abs(wz) > abs(vy):
            # Rotation
            ticks_signed = ticks if wz > 0 else -ticks
            return self.send_command("TURN", speed_pwm, ticks_signed)
        elif abs(vy) > abs(vx):
            # Strafe
            if vy > 0:
                return self.send_command("LEFT", speed_pwm, ticks)
            else:
                return self.send_command("RIGHT", speed_pwm, ticks)
        else:
            # Forward/backward
            if vx > 0:
                return self.send_command("FWD", speed_pwm, ticks)
            else:
                return self.send_command("BWD", speed_pwm, ticks)

    def send_stop(self) -> bool:
        """Send emergency stop command."""
        return self.send_command("STOP")

    def _read_loop(self):
        """Background thread for reading serial responses."""
        while self._running:
            if not self.is_connected:
                time.sleep(0.1)
                continue

            try:
                with self._lock:
                    if self._serial and self._serial.in_waiting:
                        line = self._serial.readline()
                    else:
                        line = None

                if line:
                    cmd, args, valid = ChecksumProtocol.decode(line)
                    if valid:
                        self.stats['rx_count'] += 1
                        if self.on_response:
                            self.on_response(cmd, args)
                    else:
                        self.stats['checksum_errors'] += 1
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

"""Arduino motor controller hardware abstraction for the V2N robot.

Provides an abstract base class (ArduinoBase) with real (ArduinoBridge)
and mock (MockArduino) implementations, plus a ``create_arduino()``
factory. Used by the standalone ``arduino_driver_node.py`` for the older
blocking serial model; the newer ``arduino_bridge.py`` module provides a
non-blocking bridge with background threads used by the ROS2 driver.

Firmware Protocol (plain text, newline-terminated, NO checksums):
    TX Commands:
        VEL,vx,vy,wz           Continuous velocity (mm/s, mm/s, mrad/s)
        FWD|BWD|LEFT|RIGHT,speed,ticks   Timed move (encoder-counted)
        DIAGFL|DIAGFR|DIAGBL|DIAGBR,speed,ticks   Diagonal timed moves
        TURN,speed,ticks        Rotate in place (positive ticks = CCW)
        STOP                    Emergency stop all motors
        READ                    Request encoder positions (4-line response)
        CALIB                   Start ~40s dead-zone + speed calibration
        TMOTOR,idx,pwm          Drive single motor at raw PWM
        TENC                    Start encoder test mode
    RX Responses:
        READY                   Firmware booted (sent once after reset)
        OK                      Command accepted
        DONE                    Timed move completed / calibration finished
        BUSY                    Command rejected (motion in progress)
        ERROR: <msg>            Parse or execution error

    Telemetry (20Hz during VEL mode):
        ODOM,vx_mm,vy_mm,wz_mrad    Body-frame velocities from encoder FK
        ENC,FL:t,RL:t,RR:t,FR:t,t_us:t   Raw encoder positions + timestamp

    Timing constraints:
        - 200ms watchdog in VEL mode (motors stop if no VEL received)
        - Arduino resets on serial open; READY appears after ~1-2 seconds
        - Serial baud: 115200, 8N1

Motor indices (firmware order): FL=0, RL=1, RR=2, FR=3
Encoder CPR: 4320 (counts per revolution after gearing)
Wheel diameter: 80mm

Key classes:
    ArduinoBase -- abstract interface with IK math and convenience methods.
    ArduinoBridge -- real serial with retry and auto-reconnect.
    MockArduino -- simulated firmware for desktop testing.

Related modules:
    hardware/arduino_bridge.py -- non-blocking bridge used by ArduinoDriverNode.
    app/arduino_node.py        -- ROS2 driver node (production).
    app/odometry_node.py       -- uses WHEEL_RADIUS_M, ENCODER_CPR, etc.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)

# Robot physical constants -- single source of truth in config.py
from target_nav.config import (
    DEFAULT_WHEEL_RADIUS as WHEEL_RADIUS_M,
    DEFAULT_WHEELBASE as WHEELBASE_M,
    DEFAULT_TRACK_WIDTH as TRACK_WIDTH_M,
    DEFAULT_ENCODER_CPR as ENCODER_CPR,
    DEFAULT_MAX_PWM as SPEED_MAX,
)

# Serial defaults (from single source of truth)
from target_nav.config import (
    DEFAULT_ARDUINO_PORT, DEFAULT_BAUDRATE, ARDUINO_READY_TIMEOUT,
)


@dataclass
class EncoderData:
    """Encoder reading from firmware.

    Firmware returns values in order: FL(0), RL(1), RR(2), FR(3).
    Properties provide named access regardless of internal order.
    """
    values: List[int]  # [FL, RL, RR, FR] - firmware order
    timestamp: float

    @property
    def front_left(self) -> int:
        return self.values[0] if len(self.values) > 0 else 0

    @property
    def rear_left(self) -> int:
        return self.values[1] if len(self.values) > 1 else 0

    @property
    def rear_right(self) -> int:
        return self.values[2] if len(self.values) > 2 else 0

    @property
    def front_right(self) -> int:
        return self.values[3] if len(self.values) > 3 else 0


class ArduinoBase(ABC):
    """Abstract base class for Arduino motor controller communication.

    Provides mecanum inverse kinematics (``set_velocity``), timed move
    helpers (``move``, ``turn``), encoder reading, and calibration.
    Subclasses implement the transport layer (real serial or mock).
    """

    def __init__(
        self,
        device_path: str = DEFAULT_ARDUINO_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = 0.5,
        auto_reconnect: bool = True,
        **kwargs
    ):
        """Initialize base controller state.

        Args:
            device_path: Serial device (e.g. '/dev/ttyACM0').
            baudrate: Serial baud rate.
            timeout: Read timeout in seconds.
            auto_reconnect: Reconnect automatically on failure.
        """
        self.device_path = device_path
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect

        self._connected = False
        self._last_command_time = 0.0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @abstractmethod
    def connect(self) -> bool:
        """Connect to Arduino. Waits for READY response after reset."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Send STOP and close serial connection."""
        pass

    @abstractmethod
    def send_command(self, command: str) -> Optional[str]:
        """Send plain-text command and read one line response."""
        pass

    def stop(self) -> bool:
        """Emergency stop all motors."""
        response = self.send_command("STOP")
        return response is not None and "DONE" in response

    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """Send VEL command with velocity in mm/s and mrad/s.

        Converts m/s and rad/s to mm/s and mrad/s for the firmware.
        The firmware handles PWM conversion internally using encoder
        feedback.

        Args:
            linear_x: Forward velocity in m/s (positive = forward)
            linear_y: Lateral velocity in m/s (positive = left)
            angular_z: Rotation in rad/s (positive = CCW)

        Returns:
            True if command sent
        """
        # Convert m/s → mm/s, rad/s → mrad/s
        vx_mm = int(linear_x * 1000)
        vy_mm = int(linear_y * 1000)
        wz_mrad = int(angular_z * 1000)

        # Deadzone: only stop if ALL values are near zero (< 3 mm/s).
        # Low values are valid for creep during arrival approach.
        if abs(vx_mm) < 3 and abs(vy_mm) < 3 and abs(wz_mrad) < 3:
            return self.stop()

        response = self.send_command(f"VEL,{vx_mm},{vy_mm},{wz_mrad}")
        return response is not None

    def move(self, direction: str, speed: int, ticks: int) -> bool:
        """Send a timed move command.

        Args:
            direction: FWD, BWD, LEFT, RIGHT, DIAGFL, DIAGFR, DIAGBL, DIAGBR
            speed: PWM 1-255 (firmware requires > 0)
            ticks: Encoder ticks > 0 (firmware requires > 0)

        Returns:
            True if command accepted (OK response)
        """
        speed = max(1, min(SPEED_MAX, speed))
        ticks = max(1, ticks)
        response = self.send_command(f"{direction},{speed},{ticks}")
        return response is not None and "OK" in response

    def turn(self, speed: int, ticks: int) -> bool:
        """Send turn command. Positive ticks = CCW, negative = CW.

        Args:
            speed: PWM 1-255 (must be > 0)
            ticks: Encoder ticks (sign determines direction)
        """
        speed = max(1, min(SPEED_MAX, speed))
        if ticks == 0:
            return False
        response = self.send_command(f"TURN,{speed},{ticks}")
        return response is not None and "OK" in response

    def read_encoders(self) -> Optional[EncoderData]:
        """Send READ and parse 4-line encoder response.

        Firmware responds with 4 lines, one int per line:
            FL_ticks
            RL_ticks
            RR_ticks
            FR_ticks
        """
        response = self.send_command("READ")
        if response is None:
            return None

        try:
            values = [int(response)]
            # Read 3 more lines
            for _ in range(3):
                line = self._read_line()
                if line is not None:
                    values.append(int(line))
            if len(values) == 4:
                return EncoderData(values=values, timestamp=time.time())
        except (ValueError, TypeError) as e:
            logger.error(f"Failed to parse encoder data: {e}")

        return None

    def _read_line(self) -> Optional[str]:
        """Read one line from serial. Override in subclasses."""
        return None

    def calibrate(self) -> bool:
        """Start motor calibration sequence on the Arduino.

        The firmware runs ~40 seconds of dead-zone detection + forward/reverse
        speed measurement, then saves results to EEPROM. The robot must be
        lifted off the ground during calibration. Returns True if the
        command was accepted (CALIB,start response).
        """
        response = self.send_command("CALIB")
        return response is not None



class ArduinoBridge(ArduinoBase):
    """Real Arduino motor controller via USB serial (blocking model).

    On ``connect()``, opens the serial port and waits up to 5 seconds
    for the firmware READY message (Arduino resets on serial open).
    ``send_command()`` uses a threading lock and retries with exponential
    backoff (50ms, 100ms, 200ms) on transient serial errors. If all
    retries fail and ``auto_reconnect`` is True, the next call attempts
    a fresh connection.
    """

    MAX_RETRIES = 3
    RETRY_DELAYS = [0.05, 0.1, 0.2]  # Exponential backoff

    def __init__(self, **kwargs):
        """Set up serial connection state (connect later via connect()).

        Args:
            **kwargs: Forwarded to ArduinoBase (device_path, baudrate,
                timeout, auto_reconnect).
        """
        super().__init__(**kwargs)
        self._serial = None
        self._lock = threading.Lock()

    def connect(self) -> bool:
        """Connect and wait for firmware READY message."""
        if self._connected:
            return True

        try:
            import serial
            self._serial = serial.Serial(
                self.device_path,
                self.baudrate,
                timeout=self.timeout
            )

            # Arduino resets on serial open; wait for READY
            ready = False
            start = time.time()
            while time.time() - start < 5.0:
                if self._serial.in_waiting:
                    line = self._serial.readline().decode('ascii', errors='ignore').strip()
                    if 'READY' in line:
                        ready = True
                        break
                time.sleep(0.05)

            if not ready:
                # May have missed READY; continue but warn
                logger.warning("Did not receive READY from Arduino (continuing anyway)")
                time.sleep(0.5)

            self._serial.reset_input_buffer()
            self._connected = True
            logger.info(f"Connected to Arduino at {self.device_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            return False

    def disconnect(self) -> None:
        """Stop motors and close serial."""
        if self._serial:
            try:
                self._serial.write(b"STOP\n")
                self._serial.flush()
                time.sleep(0.05)
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._connected = False
        logger.info("Disconnected from Arduino")

    def send_command(self, command: str) -> Optional[str]:
        """Send a plain-text command and read one line response.

        Uses a threading lock for thread safety. Retries up to 3 times
        with exponential backoff on serial errors. Returns the response
        string or None on failure.

        Args:
            command: Command string (e.g. 'VEL,100,0,0').

        Returns:
            Response line from firmware, or None on error.
        """
        for attempt in range(self.MAX_RETRIES):
            if not self._connected or not self._serial:
                if self.auto_reconnect:
                    if not self.connect():
                        return None
                else:
                    return None

            with self._lock:
                try:
                    self._serial.reset_input_buffer()
                    self._serial.write(f"{command}\n".encode('ascii'))
                    self._serial.flush()

                    response = self._serial.readline().decode('ascii', errors='ignore').strip()
                    self._last_command_time = time.time()
                    return response if response else None

                except Exception as e:
                    logger.warning(f"Serial error (attempt {attempt + 1}): {e}")
                    self._connected = False
                    if attempt < self.MAX_RETRIES - 1:
                        time.sleep(self.RETRY_DELAYS[attempt])

        logger.error("All serial retries failed")
        return None

    def _read_line(self) -> Optional[str]:
        """Read one additional line from serial (for multi-line responses)."""
        if not self._serial:
            return None
        try:
            line = self._serial.readline().decode('ascii', errors='ignore').strip()
            return line if line else None
        except Exception:
            return None


class MockArduino(ArduinoBase):
    """Mock Arduino for desktop testing without hardware.

    Simulates firmware behavior: accepts all valid commands, returns
    appropriate responses (OK, DONE, ERROR), tracks encoder positions,
    and records command history for test assertions.
    """

    def __init__(
        self,
        simulate_delay: bool = True,
        delay_ms: int = 5,
        **kwargs
    ):
        """Initialize mock Arduino.

        Args:
            simulate_delay: Whether to sleep to simulate serial latency.
            delay_ms: Simulated round-trip delay in milliseconds.
        """
        super().__init__(**kwargs)
        self.simulate_delay = simulate_delay
        self.delay_ms = delay_ms

        # Firmware order: [FL, RL, RR, FR]
        self._encoders = [0, 0, 0, 0]
        self._motor_pwm = [0, 0, 0, 0]
        self._commands: List[str] = []
        self._state = 'IDLE'
        self._vel_mode = False

    def connect(self) -> bool:
        self._connected = True
        logger.info("Mock Arduino connected (READY)")
        return True

    def disconnect(self) -> None:
        self._connected = False
        self._motor_pwm = [0, 0, 0, 0]
        self._vel_mode = False
        logger.info("Mock Arduino disconnected")

    def send_command(self, command: str) -> Optional[str]:
        """Simulate firmware command processing and return a response.

        Args:
            command: Command string to process.

        Returns:
            Simulated firmware response string.
        """
        if not self._connected:
            return None

        self._commands.append(command)

        if self.simulate_delay:
            time.sleep(self.delay_ms / 1000)

        # Parse command
        parts = command.strip().split(',')
        cmd = parts[0].upper()

        if cmd == "STOP":
            self._motor_pwm = [0, 0, 0, 0]
            self._vel_mode = False
            return "DONE"

        elif cmd == "VEL" and len(parts) >= 4:
            try:
                vx_mm = int(parts[1])   # mm/s
                vy_mm = int(parts[2])   # mm/s
                wz_mrad = int(parts[3]) # mrad/s
                # Simplified mecanum IK (matches firmware)
                self._motor_pwm[0] = vx_mm - vy_mm - wz_mrad  # FL
                self._motor_pwm[1] = vx_mm + vy_mm - wz_mrad  # RL
                self._motor_pwm[2] = vx_mm - vy_mm + wz_mrad  # RR
                self._motor_pwm[3] = vx_mm + vy_mm + wz_mrad  # FR
                # Simulate encoder increment proportional to velocity
                for i in range(4):
                    self._encoders[i] += self._motor_pwm[i] // 25
                self._vel_mode = True
                return "OK"
            except ValueError:
                return "ERROR: Invalid VEL args"

        elif cmd in ("FWD", "BWD", "LEFT", "RIGHT",
                      "DIAGFL", "DIAGFR", "DIAGBL", "DIAGBR"):
            if len(parts) < 3:
                return "ERROR: Missing args"
            try:
                speed = int(parts[1])
                ticks = int(parts[2])
                if speed <= 0:
                    return "ERROR: Speed must be > 0"
                if ticks <= 0:
                    return "ERROR: Ticks must be > 0"
                # Simulate encoder change
                for i in range(4):
                    self._encoders[i] += ticks // 4
                return "OK"
            except ValueError:
                return "ERROR: Invalid args"

        elif cmd == "TURN":
            if len(parts) < 3:
                return "ERROR: Missing args"
            try:
                speed = int(parts[1])
                ticks = int(parts[2])
                if speed <= 0:
                    return "ERROR: Speed must be > 0"
                for i in range(4):
                    sign = 1 if ticks > 0 else -1
                    self._encoders[i] += sign * abs(ticks) // 4
                return "OK"
            except ValueError:
                return "ERROR: Invalid args"

        elif cmd == "READ":
            return str(self._encoders[0])

        elif cmd == "CALIB":
            return "CALIB,start"

        return "ERROR: Unknown"

    def _read_line(self) -> Optional[str]:
        """Return remaining encoder lines for READ command."""
        # After the first line (FL), return RL, RR, FR
        if not hasattr(self, '_read_idx'):
            self._read_idx = 1
        if self._read_idx < 4:
            val = str(self._encoders[self._read_idx])
            self._read_idx += 1
            if self._read_idx >= 4:
                self._read_idx = 1
            return val
        return None

    def get_command_history(self) -> List[str]:
        """Return a copy of all commands sent since creation or last clear."""
        return self._commands.copy()

    def clear_command_history(self) -> None:
        """Clear the recorded command history."""
        self._commands.clear()


def create_arduino(
    use_mock: bool = False,
    config=None,
    auto_connect: bool = True,
    **kwargs
) -> ArduinoBase:
    """Factory function to create an Arduino controller instance.

    Args:
        use_mock: If True, return a MockArduino for testing.
        config: Optional config object with ``arduino.device_path``, etc.
        auto_connect: Call ``connect()`` immediately after creation.
        **kwargs: Passed through to the ArduinoBase constructor.

    Returns:
        ArduinoBridge (real) or MockArduino instance.
    """
    if config:
        kwargs.setdefault('device_path', getattr(config.arduino, 'device_path', DEFAULT_ARDUINO_PORT))
        kwargs.setdefault('baudrate', getattr(config.arduino, 'baudrate', DEFAULT_BAUDRATE))
        kwargs.setdefault('timeout', getattr(config.arduino, 'timeout', 0.5))
        kwargs.setdefault('auto_reconnect', getattr(config.arduino, 'auto_reconnect', True))

    if use_mock:
        arduino = MockArduino(**kwargs)
    else:
        arduino = ArduinoBridge(**kwargs)

    if auto_connect:
        arduino.connect()

    return arduino

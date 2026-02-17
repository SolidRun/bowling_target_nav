"""
Arduino Motor Controller
========================

Hardware abstraction for the V2N Arduino motor controller.
Real serial implementation and mock for testing.

Firmware Protocol (plain text, NO checksums):
  Commands: FWD, BWD, LEFT, RIGHT, TURN, VEL, STOP, READ, CALIB, TMOTOR, TENC
  DIAG commands: DIAGFL, DIAGFR, DIAGBL, DIAGBR
  Format: CMD[,arg1,arg2,...]\n
  Responses: READY, OK, DONE, BUSY, ERROR: msg
  Telemetry (20Hz): ODOM,vx,vy,wz  or  ENC,FL:t,FR:t,RL:t,RR:t,t_us:t
  Watchdog: 200ms timeout in VEL mode

Motor indices (firmware order): FL=0, RL=1, RR=2, FR=3
Encoder CPR: 4320
Wheel diameter: 80mm
"""

import logging
import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

# Robot physical constants (from URDF and firmware config.h)
WHEEL_RADIUS_M = 0.04        # 80mm diameter / 2
WHEELBASE_M = 0.190           # front-rear distance
TRACK_WIDTH_M = 0.210         # left-right distance
ENCODER_CPR = 4320
SPEED_MIN = 20                # Minimum PWM that overcomes friction
SPEED_MAX = 255
WATCHDOG_TIMEOUT_MS = 200


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
    """Abstract base class for Arduino motor controller."""

    def __init__(
        self,
        device_path: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.5,
        auto_reconnect: bool = True,
        **kwargs
    ):
        self.device_path = device_path
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect

        self._connected = False
        self._last_command_time = 0.0
        self._command_count = 0

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
        """Send VEL command using proper mecanum inverse kinematics.

        Converts m/s and rad/s to PWM-scale values (-255..255) that the
        firmware's VEL command expects. The firmware handles per-motor
        PID control internally.

        Args:
            linear_x: Forward velocity in m/s (positive = forward)
            linear_y: Lateral velocity in m/s (positive = left)
            angular_z: Rotation in rad/s (positive = CCW)

        Returns:
            True if command sent
        """
        # Mecanum inverse kinematics: compute wheel angular velocities
        # v_wheel = (vx +/- vy +/- (L+W)*wz) / R
        lw = (WHEELBASE_M + TRACK_WIDTH_M) / 2.0  # 0.200m

        w_fl = (linear_x - linear_y - lw * angular_z) / WHEEL_RADIUS_M
        w_fr = (linear_x + linear_y + lw * angular_z) / WHEEL_RADIUS_M
        w_rl = (linear_x + linear_y - lw * angular_z) / WHEEL_RADIUS_M
        w_rr = (linear_x - linear_y + lw * angular_z) / WHEEL_RADIUS_M

        # Convert rad/s to PWM: max wheel speed ~ 11.2 rad/s at PWM 255
        # (from firmware: ~4320 CPR, max ~7680 ticks/sec => 1.78 rev/s => 11.2 rad/s)
        max_wheel_radps = 11.2
        pwm_per_radps = SPEED_MAX / max_wheel_radps

        vx_pwm = int(linear_x * pwm_per_radps * WHEEL_RADIUS_M / WHEEL_RADIUS_M)
        # Simpler: firmware VEL does its own IK, so send body-frame values
        # Scale: map max_linear (m/s) to PWM 255
        # From firmware: VEL,vx,vy,wz where each is -255..255
        # We need a consistent mapping. Use max achievable speed.
        max_linear = max_wheel_radps * WHEEL_RADIUS_M  # ~0.448 m/s
        max_angular = max_linear / lw  # ~2.24 rad/s

        vx_pwm = int(linear_x / max_linear * SPEED_MAX)
        vy_pwm = int(linear_y / max_linear * SPEED_MAX)
        wz_pwm = int(angular_z / max_angular * SPEED_MAX)

        # Clamp
        vx_pwm = max(-SPEED_MAX, min(SPEED_MAX, vx_pwm))
        vy_pwm = max(-SPEED_MAX, min(SPEED_MAX, vy_pwm))
        wz_pwm = max(-SPEED_MAX, min(SPEED_MAX, wz_pwm))

        # Deadzone: firmware ignores very small PWM values
        if abs(vx_pwm) < 10 and abs(vy_pwm) < 10 and abs(wz_pwm) < 10:
            return self.stop()

        response = self.send_command(f"VEL,{vx_pwm},{vy_pwm},{wz_pwm}")
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
        """Start motor calibration (measures max tick rates per motor).

        Takes ~15 seconds. Returns True if DONE received.
        """
        response = self.send_command("CALIB")
        return response is not None

    @property
    def max_linear_speed(self) -> float:
        """Maximum achievable linear speed in m/s."""
        return 11.2 * WHEEL_RADIUS_M  # ~0.448 m/s

    @property
    def max_angular_speed(self) -> float:
        """Maximum achievable angular speed in rad/s."""
        lw = (WHEELBASE_M + TRACK_WIDTH_M) / 2.0
        return self.max_linear_speed / lw  # ~2.24 rad/s


class ArduinoBridge(ArduinoBase):
    """Real Arduino motor controller via serial.

    Waits for READY on connect, sends plain-text commands,
    retries on transient failures, and auto-reconnects.
    """

    MAX_RETRIES = 3
    RETRY_DELAYS = [0.05, 0.1, 0.2]  # Exponential backoff

    def __init__(self, **kwargs):
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
        """Send command with retry logic and auto-reconnect."""
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
                    self._command_count += 1
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
    """Mock Arduino for testing without hardware.

    Simulates firmware behavior including encoder counting and VEL mode.
    """

    def __init__(
        self,
        simulate_delay: bool = True,
        delay_ms: int = 5,
        **kwargs
    ):
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
        if not self._connected:
            return None

        self._commands.append(command)
        self._command_count += 1

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
                vx = int(parts[1])
                vy = int(parts[2])
                wz = int(parts[3])
                # Simplified mecanum IK (matches firmware)
                self._motor_pwm[0] = vx - vy - wz  # FL
                self._motor_pwm[1] = vx + vy - wz  # RL
                self._motor_pwm[2] = vx - vy + wz  # RR
                self._motor_pwm[3] = vx + vy + wz  # FR
                # Simulate encoder increment
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
        return self._commands.copy()

    def clear_command_history(self) -> None:
        self._commands.clear()


def create_arduino(
    use_mock: bool = False,
    config=None,
    auto_connect: bool = True,
    **kwargs
) -> ArduinoBase:
    """Factory function to create Arduino instance."""
    if config:
        kwargs.setdefault('device_path', getattr(config.arduino, 'device_path', '/dev/ttyACM0'))
        kwargs.setdefault('baudrate', getattr(config.arduino, 'baudrate', 115200))
        kwargs.setdefault('timeout', getattr(config.arduino, 'timeout', 0.5))
        kwargs.setdefault('auto_reconnect', getattr(config.arduino, 'auto_reconnect', True))

    if use_mock:
        arduino = MockArduino(**kwargs)
    else:
        arduino = ArduinoBridge(**kwargs)

    if auto_connect:
        arduino.connect()

    return arduino

"""
Arduino Motor Controller
========================

Abstract base class and implementations for Arduino motor controller.
Supports real serial communication and mock for testing.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class EncoderData:
    """Encoder reading data."""
    values: List[int]  # [FL, FR, RL, RR]
    timestamp: float

    @property
    def front_left(self) -> int:
        return self.values[0] if len(self.values) > 0 else 0

    @property
    def front_right(self) -> int:
        return self.values[1] if len(self.values) > 1 else 0

    @property
    def rear_left(self) -> int:
        return self.values[2] if len(self.values) > 2 else 0

    @property
    def rear_right(self) -> int:
        return self.values[3] if len(self.values) > 3 else 0


class ArduinoBase(ABC):
    """
    Abstract base class for Arduino motor controller.

    Provides interface for motor control and encoder reading.
    """

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
        """Connect to Arduino."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from Arduino."""
        pass

    @abstractmethod
    def send_command(self, command: str) -> Optional[str]:
        """Send command and get response."""
        pass

    def sync(self) -> bool:
        """Synchronize with Arduino."""
        response = self.send_command("SYNC")
        return response is not None and "OK" in response.upper()

    def stop(self) -> bool:
        """Emergency stop all motors."""
        response = self.send_command("STOP")
        return response is not None

    def set_motors(self, fl: int, fr: int, rl: int, rr: int) -> bool:
        """
        Set individual motor speeds.

        Args:
            fl, fr, rl, rr: Motor speeds (-255 to 255)

        Returns:
            True if command sent successfully
        """
        command = f"M {fl} {fr} {rl} {rr}"
        response = self.send_command(command)
        return response is not None

    def set_velocity(self, linear: float, angular: float) -> bool:
        """
        Set robot velocity (mecanum kinematics).

        Args:
            linear: Linear velocity in m/s
            angular: Angular velocity in rad/s

        Returns:
            True if command sent successfully
        """
        # Convert to motor values using mecanum kinematics
        # Simplified: assumes equal contribution
        linear_component = int(linear * 255 / 0.3)  # Normalize to max speed
        angular_component = int(angular * 100)

        fl = linear_component - angular_component
        fr = linear_component + angular_component
        rl = linear_component - angular_component
        rr = linear_component + angular_component

        # Clamp values
        fl = max(-255, min(255, fl))
        fr = max(-255, min(255, fr))
        rl = max(-255, min(255, rl))
        rr = max(-255, min(255, rr))

        return self.set_motors(fl, fr, rl, rr)

    def read_encoders(self) -> Optional[EncoderData]:
        """Read encoder values."""
        response = self.send_command("READ")
        if response is None:
            return None

        try:
            # Parse response: "ENC FL FR RL RR"
            parts = response.strip().split()
            if len(parts) >= 5 and parts[0] == "ENC":
                values = [int(parts[i]) for i in range(1, 5)]
                return EncoderData(values=values, timestamp=time.time())
        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse encoder data: {e}")

        return None

    def reset_encoders(self) -> bool:
        """Reset encoder counters."""
        response = self.send_command("RESET")
        return response is not None


class ArduinoBridge(ArduinoBase):
    """
    Real Arduino motor controller using serial communication.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._serial = None
        self._lock = threading.Lock()

    def connect(self) -> bool:
        """Connect to Arduino via serial port."""
        if self._connected:
            return True

        try:
            import serial
            self._serial = serial.Serial(
                self.device_path,
                self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino reset

            # Verify connection
            if self.sync():
                self._connected = True
                logger.info(f"Connected to Arduino at {self.device_path}")
                return True
            else:
                logger.error("Arduino sync failed")
                self._serial.close()
                return False

        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from Arduino."""
        if self._serial:
            try:
                self.stop()
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._connected = False
        logger.info("Disconnected from Arduino")

    def send_command(self, command: str) -> Optional[str]:
        """Send command and get response."""
        if not self._connected or not self._serial:
            if self.auto_reconnect:
                if not self.connect():
                    return None
            else:
                return None

        with self._lock:
            try:
                # Clear input buffer
                self._serial.reset_input_buffer()

                # Send command
                self._serial.write(f"{command}\n".encode())
                self._serial.flush()

                # Read response
                response = self._serial.readline().decode().strip()

                self._last_command_time = time.time()
                self._command_count += 1

                return response if response else None

            except Exception as e:
                logger.error(f"Serial communication error: {e}")
                self._connected = False
                return None


class MockArduino(ArduinoBase):
    """
    Mock Arduino for testing without hardware.

    Simulates Arduino behavior including encoder counting.
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

        self._encoders = [0, 0, 0, 0]
        self._motor_speeds = [0, 0, 0, 0]
        self._commands: List[str] = []

    def connect(self) -> bool:
        """Simulate connection."""
        self._connected = True
        logger.info("Mock Arduino connected")
        return True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self._connected = False
        self._motor_speeds = [0, 0, 0, 0]
        logger.info("Mock Arduino disconnected")

    def send_command(self, command: str) -> Optional[str]:
        """Simulate command processing."""
        if not self._connected:
            return None

        self._commands.append(command)
        self._command_count += 1

        if self.simulate_delay:
            time.sleep(self.delay_ms / 1000)

        # Parse and respond
        parts = command.strip().split()
        cmd = parts[0].upper()

        if cmd == "SYNC":
            return "OK SYNC"

        elif cmd == "STOP":
            self._motor_speeds = [0, 0, 0, 0]
            return "OK STOP"

        elif cmd == "M" and len(parts) >= 5:
            try:
                self._motor_speeds = [int(parts[i]) for i in range(1, 5)]
                # Simulate encoder increment
                for i, speed in enumerate(self._motor_speeds):
                    self._encoders[i] += speed // 10
                return "OK M"
            except ValueError:
                return "ERR"

        elif cmd == "READ":
            return f"ENC {self._encoders[0]} {self._encoders[1]} {self._encoders[2]} {self._encoders[3]}"

        elif cmd == "RESET":
            self._encoders = [0, 0, 0, 0]
            return "OK RESET"

        return "OK"

    def get_command_history(self) -> List[str]:
        """Get list of commands sent (for testing)."""
        return self._commands.copy()

    def clear_command_history(self) -> None:
        """Clear command history."""
        self._commands.clear()


def create_arduino(
    use_mock: bool = False,
    config: Optional[object] = None,
    auto_connect: bool = True,
    **kwargs
) -> ArduinoBase:
    """
    Factory function to create Arduino instance.

    Args:
        use_mock: Use mock Arduino for testing
        config: Configuration object
        auto_connect: Automatically connect
        **kwargs: Additional arguments

    Returns:
        Arduino instance
    """
    if config:
        kwargs.setdefault('device_path', config.arduino.device_path)
        kwargs.setdefault('baudrate', config.arduino.baudrate)
        kwargs.setdefault('timeout', config.arduino.timeout)
        kwargs.setdefault('auto_reconnect', config.arduino.auto_reconnect)

    if use_mock:
        arduino = MockArduino(**kwargs)
    else:
        arduino = ArduinoBridge(**kwargs)

    if auto_connect:
        arduino.connect()

    return arduino

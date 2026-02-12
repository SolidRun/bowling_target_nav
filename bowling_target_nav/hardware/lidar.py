"""
LiDAR Sensor
============

Abstract base class and implementations for LiDAR sensor.
Supports real RPLidar communication and mock for testing.
"""

import logging
import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class LidarPoint:
    """Single LiDAR point."""
    angle: float      # Angle in degrees
    distance: float   # Distance in meters
    quality: int      # Signal quality (0-255)

    @property
    def angle_rad(self) -> float:
        """Angle in radians."""
        return math.radians(self.angle)

    @property
    def x(self) -> float:
        """X coordinate in meters."""
        return self.distance * math.cos(self.angle_rad)

    @property
    def y(self) -> float:
        """Y coordinate in meters."""
        return self.distance * math.sin(self.angle_rad)

    def as_tuple(self) -> Tuple[float, float]:
        """Return as (x, y) tuple."""
        return (self.x, self.y)


@dataclass
class LidarScan:
    """Complete LiDAR scan data."""
    points: List[LidarPoint] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)
    scan_number: int = 0

    @property
    def num_points(self) -> int:
        return len(self.points)

    @property
    def min_distance(self) -> float:
        """Minimum distance in scan."""
        if not self.points:
            return float('inf')
        return min(p.distance for p in self.points if p.distance > 0)

    @property
    def min_distance_angle(self) -> float:
        """Angle of minimum distance point."""
        if not self.points:
            return 0.0
        min_point = min(self.points, key=lambda p: p.distance if p.distance > 0 else float('inf'))
        return min_point.angle

    def get_range(self, start_angle: float, end_angle: float) -> List[LidarPoint]:
        """Get points within angle range."""
        return [p for p in self.points if start_angle <= p.angle <= end_angle]

    def get_front_distance(self, angle_range: float = 30.0) -> float:
        """Get minimum distance in front of robot."""
        front_points = [p for p in self.points
                       if -angle_range/2 <= p.angle <= angle_range/2 or
                          360-angle_range/2 <= p.angle <= 360]
        if not front_points:
            return float('inf')
        return min(p.distance for p in front_points if p.distance > 0)

    def to_numpy(self) -> np.ndarray:
        """Convert to numpy array (N x 3: angle, distance, quality)."""
        if not self.points:
            return np.array([])
        return np.array([[p.angle, p.distance, p.quality] for p in self.points])

    def to_cartesian(self) -> np.ndarray:
        """Convert to cartesian coordinates (N x 2: x, y)."""
        if not self.points:
            return np.array([])
        return np.array([[p.x, p.y] for p in self.points])


class LidarBase(ABC):
    """
    Abstract base class for LiDAR sensor.

    Provides interface for LiDAR scanning.
    """

    def __init__(
        self,
        device_path: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        min_range: float = 0.15,
        max_range: float = 12.0,
        auto_reconnect: bool = True,
        **kwargs
    ):
        self.device_path = device_path
        self.baudrate = baudrate
        self.min_range = min_range
        self.max_range = max_range
        self.auto_reconnect = auto_reconnect

        self._connected = False
        self._scanning = False
        self._scan_count = 0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_scanning(self) -> bool:
        return self._scanning

    @abstractmethod
    def connect(self) -> bool:
        """Connect to LiDAR."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from LiDAR."""
        pass

    @abstractmethod
    def start_scanning(self) -> bool:
        """Start continuous scanning."""
        pass

    @abstractmethod
    def stop_scanning(self) -> None:
        """Stop scanning."""
        pass

    @abstractmethod
    def get_scan(self) -> Optional[LidarScan]:
        """Get latest scan data."""
        pass

    def get_device_info(self) -> dict:
        """Get device information."""
        return {
            'device_path': self.device_path,
            'connected': self._connected,
            'scanning': self._scanning,
            'scan_count': self._scan_count,
        }


class LidarBridge(LidarBase):
    """
    Real LiDAR using RPLidar library.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._lidar = None
        self._iterator = None
        self._lock = threading.Lock()
        self._current_scan: List[LidarPoint] = []

    def connect(self) -> bool:
        """Connect to RPLidar."""
        if self._connected:
            return True

        try:
            from rplidar import RPLidar
            self._lidar = RPLidar(self.device_path, baudrate=self.baudrate)

            # Get device info
            info = self._lidar.get_info()
            logger.info(f"RPLidar connected: {info}")

            # Check health
            health = self._lidar.get_health()
            if health[0] != 'Good':
                logger.warning(f"LiDAR health: {health}")

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"Failed to connect to LiDAR: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from LiDAR."""
        self.stop_scanning()

        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.disconnect()
            except Exception:
                pass
            self._lidar = None

        self._connected = False
        logger.info("LiDAR disconnected")

    def start_scanning(self) -> bool:
        """Start continuous scanning."""
        if not self._connected or not self._lidar:
            if self.auto_reconnect:
                if not self.connect():
                    return False
            else:
                return False

        if self._scanning:
            return True

        try:
            self._iterator = self._lidar.iter_scans()
            self._scanning = True
            logger.info("LiDAR scanning started")
            return True
        except Exception as e:
            logger.error(f"Failed to start scanning: {e}")
            return False

    def stop_scanning(self) -> None:
        """Stop scanning."""
        self._scanning = False
        self._iterator = None

        if self._lidar:
            try:
                self._lidar.stop_motor()
            except Exception:
                pass

        logger.info("LiDAR scanning stopped")

    def get_scan(self) -> Optional[LidarScan]:
        """Get latest scan."""
        if not self._scanning or not self._iterator:
            return None

        with self._lock:
            try:
                scan_data = next(self._iterator)
                self._scan_count += 1

                points = []
                for quality, angle, distance in scan_data:
                    # Convert distance from mm to m
                    distance_m = distance / 1000.0

                    # Filter by range
                    if self.min_range <= distance_m <= self.max_range:
                        points.append(LidarPoint(
                            angle=angle,
                            distance=distance_m,
                            quality=quality
                        ))

                return LidarScan(
                    points=points,
                    timestamp=time.time(),
                    scan_number=self._scan_count
                )

            except StopIteration:
                self._scanning = False
                return None
            except Exception as e:
                logger.error(f"Scan read error: {e}")
                return None


class MockLidar(LidarBase):
    """
    Mock LiDAR for testing without hardware.

    Generates simulated scan data.
    """

    def __init__(
        self,
        num_points: int = 360,
        noise_level: float = 0.05,
        obstacles: Optional[List[dict]] = None,
        scan_delay: float = 0.1,
        **kwargs
    ):
        """
        Initialize mock LiDAR.

        Args:
            num_points: Number of points per scan
            noise_level: Distance noise as fraction
            obstacles: List of obstacle definitions
            scan_delay: Delay between scans (simulates rotation speed)
        """
        super().__init__(**kwargs)
        self.num_points = num_points
        self.noise_level = noise_level
        self.obstacles = obstacles or []
        self.scan_delay = scan_delay

        self._last_scan_time = 0.0

    def connect(self) -> bool:
        """Simulate connection."""
        self._connected = True
        logger.info("Mock LiDAR connected")
        return True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self.stop_scanning()
        self._connected = False
        logger.info("Mock LiDAR disconnected")

    def start_scanning(self) -> bool:
        """Start simulated scanning."""
        if not self._connected:
            return False
        self._scanning = True
        logger.info("Mock LiDAR scanning started")
        return True

    def stop_scanning(self) -> None:
        """Stop simulated scanning."""
        self._scanning = False
        logger.info("Mock LiDAR scanning stopped")

    def get_scan(self) -> Optional[LidarScan]:
        """Generate simulated scan."""
        if not self._scanning:
            return None

        # Simulate scan rate
        elapsed = time.time() - self._last_scan_time
        if elapsed < self.scan_delay:
            time.sleep(self.scan_delay - elapsed)

        self._last_scan_time = time.time()
        self._scan_count += 1

        points = []
        for i in range(self.num_points):
            angle = (i / self.num_points) * 360.0

            # Default distance (max range with some variation)
            distance = self.max_range * 0.8

            # Check obstacles
            for obstacle in self.obstacles:
                obs_angle = obstacle.get('angle', 0)
                obs_distance = obstacle.get('distance', 1.0)
                obs_width = obstacle.get('width', 10)  # Angular width

                # Check if angle is within obstacle
                angle_diff = abs(angle - obs_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff

                if angle_diff <= obs_width / 2:
                    distance = min(distance, obs_distance)

            # Add noise
            noise = (np.random.random() - 0.5) * 2 * self.noise_level * distance
            distance = max(self.min_range, distance + noise)

            # Random quality
            quality = np.random.randint(10, 50)

            points.append(LidarPoint(
                angle=angle,
                distance=distance,
                quality=quality
            ))

        return LidarScan(
            points=points,
            timestamp=time.time(),
            scan_number=self._scan_count
        )

    def add_obstacle(self, angle: float, distance: float, width: float = 10) -> None:
        """Add obstacle to simulation."""
        self.obstacles.append({
            'angle': angle,
            'distance': distance,
            'width': width
        })

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        self.obstacles.clear()


def create_lidar(
    use_mock: bool = False,
    config: Optional[object] = None,
    auto_connect: bool = True,
    **kwargs
) -> LidarBase:
    """
    Factory function to create LiDAR instance.

    Args:
        use_mock: Use mock LiDAR for testing
        config: Configuration object
        auto_connect: Automatically connect
        **kwargs: Additional arguments

    Returns:
        LiDAR instance
    """
    if config:
        kwargs.setdefault('device_path', config.lidar.device_path)
        kwargs.setdefault('baudrate', config.lidar.baudrate)
        kwargs.setdefault('min_range', config.lidar.min_range)
        kwargs.setdefault('max_range', config.lidar.max_range)
        kwargs.setdefault('auto_reconnect', config.lidar.auto_reconnect)

    if use_mock:
        lidar = MockLidar(**kwargs)
    else:
        lidar = LidarBridge(**kwargs)

    if auto_connect:
        lidar.connect()

    return lidar

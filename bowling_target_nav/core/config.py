"""
Configuration Management
========================

Provides a centralized configuration system with:
- YAML file loading
- Environment variable overrides
- Default values
- Type validation
- Singleton pattern for global access
"""

import os
import yaml
import logging
from pathlib import Path
from typing import Any, Dict, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class DetectionConfig:
    """Detection system configuration."""
    detector_type: str = "yolo_onnx"
    confidence_threshold: float = 0.5
    target_class: str = "bowling_pin"
    max_detection_rate: int = 10

    # YOLO ONNX settings
    yolo_model_path: str = "models/bowling_yolov5.onnx"
    yolo_input_size: tuple = (640, 640)
    yolo_class_names: list = field(default_factory=lambda: ["bowling_pin"])

    # DRP Binary settings
    drp_binary_path: str = "/opt/drp/bowling_detector"
    drp_input_method: str = "shared_memory"
    drp_input_path: str = "/dev/shm/camera_frame"
    drp_output_method: str = "shared_memory"
    drp_output_path: str = "/dev/shm/detection_result"
    drp_timeout: float = 1.0


@dataclass
class CameraConfig:
    """Camera configuration."""
    device_id: int = 0
    device_path: str = "/dev/video0"
    width: int = 640
    height: int = 480
    fps: int = 30
    auto_reconnect: bool = True
    reconnect_delay: float = 1.0


@dataclass
class LidarConfig:
    """LiDAR configuration."""
    device_path: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    min_angle: float = -180.0
    max_angle: float = 180.0
    min_range: float = 0.15
    max_range: float = 12.0
    auto_reconnect: bool = True
    reconnect_delay: float = 2.0


@dataclass
class ArduinoConfig:
    """Arduino configuration."""
    device_path: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout: float = 0.5
    sync_interval: float = 5.0
    auto_reconnect: bool = True
    reconnect_delay: float = 1.0


@dataclass
class NavigationConfig:
    """Navigation configuration."""
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.5
    default_linear_speed: float = 0.15
    default_angular_speed: float = 0.3
    obstacle_distance_threshold: float = 0.4
    obstacle_slowdown_distance: float = 0.8
    target_reached_distance: float = 0.3

    # Search behavior
    search_enabled: bool = True
    lost_timeout: float = 10.0
    search_timeout: float = 30.0
    search_angular_speed: float = 0.25

    # Smoothing
    velocity_smoothing: bool = True
    smoothing_factor: float = 0.3


@dataclass
class DistanceEstimationConfig:
    """Distance estimation configuration."""
    method: str = "bbox_height"
    focal_length: float = 500.0
    real_object_height: float = 0.38
    reference_area: int = 10000


@dataclass
class GUIConfig:
    """GUI configuration."""
    window_title: str = "V2N Robot Control"
    window_width: int = 1280
    window_height: int = 720
    fullscreen: bool = False
    show_lidar_overlay: bool = True
    show_detection_boxes: bool = True
    show_distance_labels: bool = True
    show_status_bar: bool = True


@dataclass
class LoggingConfig:
    """Logging configuration."""
    level: str = "INFO"
    file_enabled: bool = True
    file_path: str = "/var/log/v2n_robot.log"
    max_file_size: int = 10485760
    backup_count: int = 3
    console_enabled: bool = True


class Config:
    """
    Central configuration manager.

    Loads configuration from YAML file with environment variable overrides.
    Uses singleton pattern for global access.

    Usage:
        config = Config.load("config/robot_config.yaml")
        # or
        config = get_config()  # Gets existing instance

        speed = config.navigation.default_linear_speed
        detector = config.detection.detector_type
    """

    _instance: Optional['Config'] = None
    _initialized: bool = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, config_path: Optional[str] = None):
        if Config._initialized and config_path is None:
            return

        self._raw: Dict[str, Any] = {}
        self._config_path: Optional[Path] = None

        # Initialize sub-configs with defaults
        self.detection = DetectionConfig()
        self.camera = CameraConfig()
        self.lidar = LidarConfig()
        self.arduino = ArduinoConfig()
        self.navigation = NavigationConfig()
        self.distance_estimation = DistanceEstimationConfig()
        self.gui = GUIConfig()
        self.logging = LoggingConfig()

        if config_path:
            self.load(config_path)

        Config._initialized = True

    @classmethod
    def load(cls, config_path: str) -> 'Config':
        """Load configuration from YAML file."""
        instance = cls(config_path)
        instance._load_file(config_path)
        return instance

    @classmethod
    def reset(cls):
        """Reset singleton instance (for testing)."""
        cls._instance = None
        cls._initialized = False

    def _load_file(self, config_path: str) -> None:
        """Load and parse YAML configuration file."""
        path = Path(config_path)

        # Search for config file in common locations
        search_paths = [
            path,
            Path(__file__).parent.parent.parent / "config" / path.name,
            Path.home() / "ros2_ws/src/bowling_target_nav/config" / path.name,
            Path("/etc/v2n_robot") / path.name,
        ]

        for search_path in search_paths:
            if search_path.exists():
                self._config_path = search_path
                break

        if self._config_path is None or not self._config_path.exists():
            logger.warning(f"Config file not found: {config_path}, using defaults")
            return

        try:
            with open(self._config_path, 'r') as f:
                self._raw = yaml.safe_load(f) or {}
            logger.info(f"Loaded config from: {self._config_path}")
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            return

        self._parse_config()
        self._apply_env_overrides()

    def _parse_config(self) -> None:
        """Parse raw config into typed dataclasses."""
        # Detection
        if 'detection' in self._raw:
            d = self._raw['detection']
            self.detection = DetectionConfig(
                detector_type=d.get('detector_type', 'yolo_onnx'),
                confidence_threshold=d.get('confidence_threshold', 0.5),
                target_class=d.get('target_class', 'bowling_pin'),
                max_detection_rate=d.get('max_detection_rate', 10),
                yolo_model_path=d.get('yolo_onnx', {}).get('model_path', 'models/bowling_yolov5.onnx'),
                yolo_input_size=tuple(d.get('yolo_onnx', {}).get('input_size', [640, 640])),
                yolo_class_names=d.get('yolo_onnx', {}).get('class_names', ['bowling_pin']),
                drp_binary_path=d.get('drp_binary', {}).get('binary_path', '/opt/drp/bowling_detector'),
                drp_input_method=d.get('drp_binary', {}).get('input_method', 'shared_memory'),
                drp_input_path=d.get('drp_binary', {}).get('input_path', '/dev/shm/camera_frame'),
                drp_output_method=d.get('drp_binary', {}).get('output_method', 'shared_memory'),
                drp_output_path=d.get('drp_binary', {}).get('output_path', '/dev/shm/detection_result'),
                drp_timeout=d.get('drp_binary', {}).get('timeout', 1.0),
            )

        # Camera
        if 'camera' in self._raw:
            c = self._raw['camera']
            self.camera = CameraConfig(
                device_id=c.get('device_id', 0),
                device_path=c.get('device_path', '/dev/video0'),
                width=c.get('width', 640),
                height=c.get('height', 480),
                fps=c.get('fps', 30),
                auto_reconnect=c.get('auto_reconnect', True),
                reconnect_delay=c.get('reconnect_delay', 1.0),
            )

        # LiDAR
        if 'lidar' in self._raw:
            l = self._raw['lidar']
            self.lidar = LidarConfig(
                device_path=l.get('device_path', '/dev/ttyUSB0'),
                baudrate=l.get('baudrate', 115200),
                min_angle=l.get('min_angle', -180.0),
                max_angle=l.get('max_angle', 180.0),
                min_range=l.get('min_range', 0.15),
                max_range=l.get('max_range', 12.0),
                auto_reconnect=l.get('auto_reconnect', True),
                reconnect_delay=l.get('reconnect_delay', 2.0),
            )

        # Arduino
        if 'arduino' in self._raw:
            a = self._raw['arduino']
            self.arduino = ArduinoConfig(
                device_path=a.get('device_path', '/dev/ttyACM0'),
                baudrate=a.get('baudrate', 115200),
                timeout=a.get('timeout', 0.5),
                sync_interval=a.get('sync_interval', 5.0),
                auto_reconnect=a.get('auto_reconnect', True),
                reconnect_delay=a.get('reconnect_delay', 1.0),
            )

        # Navigation
        if 'navigation' in self._raw:
            n = self._raw['navigation']
            search = n.get('search', {})
            self.navigation = NavigationConfig(
                max_linear_speed=n.get('max_linear_speed', 0.3),
                max_angular_speed=n.get('max_angular_speed', 0.5),
                default_linear_speed=n.get('default_linear_speed', 0.15),
                default_angular_speed=n.get('default_angular_speed', 0.3),
                obstacle_distance_threshold=n.get('obstacle_distance_threshold', 0.4),
                obstacle_slowdown_distance=n.get('obstacle_slowdown_distance', 0.8),
                target_reached_distance=n.get('target_reached_distance', 0.3),
                search_enabled=search.get('enabled', True),
                lost_timeout=search.get('lost_timeout', 10.0),
                search_timeout=search.get('search_timeout', 30.0),
                search_angular_speed=search.get('angular_speed', 0.25),
                velocity_smoothing=n.get('velocity_smoothing', True),
                smoothing_factor=n.get('smoothing_factor', 0.3),
            )

        # Distance Estimation
        if 'distance_estimation' in self._raw:
            de = self._raw['distance_estimation']
            bbox = de.get('bbox_height', {})
            self.distance_estimation = DistanceEstimationConfig(
                method=de.get('method', 'bbox_height'),
                focal_length=bbox.get('focal_length', 500.0),
                real_object_height=bbox.get('real_object_height', 0.38),
                reference_area=de.get('bbox_area', {}).get('reference_area', 10000),
            )

        # GUI
        if 'gui' in self._raw:
            g = self._raw['gui']
            self.gui = GUIConfig(
                window_title=g.get('window_title', 'V2N Robot Control'),
                window_width=g.get('window_width', 1280),
                window_height=g.get('window_height', 720),
                fullscreen=g.get('fullscreen', False),
                show_lidar_overlay=g.get('show_lidar_overlay', True),
                show_detection_boxes=g.get('show_detection_boxes', True),
                show_distance_labels=g.get('show_distance_labels', True),
                show_status_bar=g.get('show_status_bar', True),
            )

        # Logging
        if 'logging' in self._raw:
            log = self._raw['logging']
            self.logging = LoggingConfig(
                level=log.get('level', 'INFO'),
                file_enabled=log.get('file_enabled', True),
                file_path=log.get('file_path', '/var/log/v2n_robot.log'),
                max_file_size=log.get('max_file_size', 10485760),
                backup_count=log.get('backup_count', 3),
                console_enabled=log.get('console_enabled', True),
            )

    def _apply_env_overrides(self) -> None:
        """Apply environment variable overrides."""
        # Detection
        if os.environ.get('V2N_DETECTOR_TYPE'):
            self.detection.detector_type = os.environ['V2N_DETECTOR_TYPE']
        if os.environ.get('V2N_DETECTION_CONFIDENCE'):
            self.detection.confidence_threshold = float(os.environ['V2N_DETECTION_CONFIDENCE'])
        if os.environ.get('V2N_YOLO_MODEL'):
            self.detection.yolo_model_path = os.environ['V2N_YOLO_MODEL']
        if os.environ.get('V2N_DRP_BINARY'):
            self.detection.drp_binary_path = os.environ['V2N_DRP_BINARY']

        # Hardware paths
        if os.environ.get('V2N_ARDUINO_PORT'):
            self.arduino.device_path = os.environ['V2N_ARDUINO_PORT']
        if os.environ.get('V2N_LIDAR_PORT'):
            self.lidar.device_path = os.environ['V2N_LIDAR_PORT']
        if os.environ.get('V2N_CAMERA_ID'):
            self.camera.device_id = int(os.environ['V2N_CAMERA_ID'])

        # Navigation
        if os.environ.get('V2N_LINEAR_SPEED'):
            self.navigation.default_linear_speed = float(os.environ['V2N_LINEAR_SPEED'])
        if os.environ.get('V2N_ANGULAR_SPEED'):
            self.navigation.default_angular_speed = float(os.environ['V2N_ANGULAR_SPEED'])

        # Logging
        if os.environ.get('V2N_LOG_LEVEL'):
            self.logging.level = os.environ['V2N_LOG_LEVEL']

    def get(self, key: str, default: Any = None) -> Any:
        """Get raw config value by dot-notation key."""
        keys = key.split('.')
        value = self._raw
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        return value

    def __repr__(self) -> str:
        return f"Config(path={self._config_path}, detector={self.detection.detector_type})"


# Global config accessor
_config: Optional[Config] = None


def get_config() -> Config:
    """Get the global configuration instance."""
    global _config
    if _config is None:
        _config = Config()
    return _config


def load_config(config_path: str) -> Config:
    """Load configuration from file and set as global."""
    global _config
    _config = Config.load(config_path)
    return _config

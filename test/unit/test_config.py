"""
Configuration Tests
===================

Unit tests for configuration system.
"""

import os
import pytest
import tempfile
import yaml

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from bowling_target_nav.core.config import Config, get_config, load_config


class TestConfig:
    """Test configuration loading and parsing."""

    def setup_method(self):
        """Reset config singleton before each test."""
        Config.reset()

    def test_default_config(self):
        """Test default configuration values."""
        config = Config()

        assert config.detection.detector_type == "yolo_onnx"
        assert config.detection.confidence_threshold == 0.5
        assert config.navigation.default_linear_speed == 0.15
        assert config.camera.width == 640
        assert config.camera.height == 480

    def test_load_yaml_config(self):
        """Test loading configuration from YAML file."""
        config_data = {
            'detection': {
                'detector_type': 'drp_binary',
                'confidence_threshold': 0.7,
            },
            'navigation': {
                'default_linear_speed': 0.2,
                'obstacle_distance_threshold': 0.5,
            },
            'camera': {
                'device_id': 1,
                'width': 1280,
                'height': 720,
            }
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_data, f)
            config_path = f.name

        try:
            config = Config.load(config_path)

            assert config.detection.detector_type == "drp_binary"
            assert config.detection.confidence_threshold == 0.7
            assert config.navigation.default_linear_speed == 0.2
            assert config.camera.device_id == 1
            assert config.camera.width == 1280
        finally:
            os.unlink(config_path)

    def test_environment_overrides(self):
        """Test environment variable overrides."""
        os.environ['V2N_DETECTOR_TYPE'] = 'mock'
        os.environ['V2N_LINEAR_SPEED'] = '0.25'

        try:
            Config.reset()
            config = Config()
            config._apply_env_overrides()

            assert config.detection.detector_type == 'mock'
            assert config.navigation.default_linear_speed == 0.25
        finally:
            del os.environ['V2N_DETECTOR_TYPE']
            del os.environ['V2N_LINEAR_SPEED']

    def test_singleton_pattern(self):
        """Test that Config is a singleton."""
        config1 = Config()
        config2 = Config()

        assert config1 is config2

    def test_get_config_function(self):
        """Test get_config() global accessor."""
        Config.reset()
        config = get_config()

        assert config is not None
        assert isinstance(config, Config)

    def test_detection_config_dataclass(self):
        """Test DetectionConfig dataclass."""
        config = Config()

        assert hasattr(config.detection, 'detector_type')
        assert hasattr(config.detection, 'confidence_threshold')
        assert hasattr(config.detection, 'yolo_model_path')
        assert hasattr(config.detection, 'drp_binary_path')

    def test_navigation_config_search_settings(self):
        """Test navigation search behavior settings."""
        config = Config()

        assert config.navigation.search_enabled == True
        assert config.navigation.lost_timeout == 10.0
        assert config.navigation.search_timeout == 30.0
        assert config.navigation.search_angular_speed == 0.25

    def test_get_raw_value(self):
        """Test getting raw config values."""
        config_data = {
            'custom': {
                'nested': {
                    'value': 42
                }
            }
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_data, f)
            config_path = f.name

        try:
            config = Config.load(config_path)
            assert config.get('custom.nested.value') == 42
            assert config.get('nonexistent', 'default') == 'default'
        finally:
            os.unlink(config_path)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

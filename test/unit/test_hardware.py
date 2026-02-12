"""
Hardware Tests
==============

Unit tests for hardware abstractions using mock implementations.
"""

import os
import pytest
import numpy as np

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from bowling_target_nav.hardware.arduino import MockArduino, EncoderData, create_arduino
from bowling_target_nav.hardware.camera import MockCamera, FrameData, create_camera
from bowling_target_nav.hardware.lidar import MockLidar, LidarScan, LidarPoint, create_lidar


class TestMockArduino:
    """Test MockArduino implementation."""

    def test_connect(self):
        """Test connection."""
        arduino = MockArduino()
        result = arduino.connect()

        assert result == True
        assert arduino.is_connected == True

    def test_disconnect(self):
        """Test disconnection."""
        arduino = MockArduino()
        arduino.connect()
        arduino.disconnect()

        assert arduino.is_connected == False

    def test_sync_command(self):
        """Test SYNC command."""
        arduino = MockArduino()
        arduino.connect()

        result = arduino.sync()
        assert result == True

    def test_stop_command(self):
        """Test STOP command."""
        arduino = MockArduino()
        arduino.connect()

        result = arduino.stop()
        assert result == True

    def test_set_motors(self):
        """Test motor control."""
        arduino = MockArduino()
        arduino.connect()

        result = arduino.set_motors(100, 100, 100, 100)
        assert result == True

    def test_set_velocity(self):
        """Test velocity control."""
        arduino = MockArduino()
        arduino.connect()

        result = arduino.set_velocity(0.1, 0.0)
        assert result == True

    def test_read_encoders(self):
        """Test encoder reading."""
        arduino = MockArduino()
        arduino.connect()

        # Send some motor commands to generate encoder data
        arduino.set_motors(100, 100, 100, 100)

        data = arduino.read_encoders()

        assert data is not None
        assert isinstance(data, EncoderData)
        assert len(data.values) == 4

    def test_reset_encoders(self):
        """Test encoder reset."""
        arduino = MockArduino()
        arduino.connect()

        result = arduino.reset_encoders()
        assert result == True

    def test_command_history(self):
        """Test command history tracking."""
        arduino = MockArduino()
        arduino.connect()

        arduino.sync()
        arduino.set_motors(100, 100, 100, 100)
        arduino.stop()

        history = arduino.get_command_history()

        assert "SYNC" in history
        assert "STOP" in history
        assert any("M " in cmd for cmd in history)


class TestEncoderData:
    """Test EncoderData dataclass."""

    def test_encoder_properties(self):
        """Test encoder value properties."""
        data = EncoderData(values=[100, 200, 150, 175], timestamp=0.0)

        assert data.front_left == 100
        assert data.front_right == 200
        assert data.rear_left == 150
        assert data.rear_right == 175


class TestMockCamera:
    """Test MockCamera implementation."""

    def test_open(self):
        """Test camera open."""
        camera = MockCamera()
        result = camera.open()

        assert result == True
        assert camera.is_opened == True

    def test_close(self):
        """Test camera close."""
        camera = MockCamera()
        camera.open()
        camera.close()

        assert camera.is_opened == False

    def test_read_frame(self):
        """Test frame capture."""
        camera = MockCamera(width=640, height=480)
        camera.open()

        data = camera.read()

        assert data is not None
        assert isinstance(data, FrameData)
        assert data.width == 640
        assert data.height == 480
        assert data.frame.shape == (480, 640, 3)

    def test_get_frame(self):
        """Test get_frame convenience method."""
        camera = MockCamera()
        camera.open()

        frame = camera.get_frame()

        assert frame is not None
        assert isinstance(frame, np.ndarray)

    def test_gradient_pattern(self):
        """Test gradient pattern generation."""
        camera = MockCamera(pattern="gradient")
        camera.open()

        frame = camera.get_frame()

        # Gradient should vary across width
        assert frame[0, 0, 0] != frame[0, -1, 0]

    def test_noise_pattern(self):
        """Test noise pattern generation."""
        camera = MockCamera(pattern="noise")
        camera.open()

        frame1 = camera.get_frame()
        frame2 = camera.get_frame()

        # Noise should be different each frame
        assert not np.array_equal(frame1, frame2)

    def test_checkerboard_pattern(self):
        """Test checkerboard pattern."""
        camera = MockCamera(pattern="checkerboard")
        camera.open()

        frame = camera.get_frame()

        # Should have both black and white pixels
        assert np.any(frame == 0)
        assert np.any(frame == 255)

    def test_static_image(self):
        """Test static image mode."""
        static = np.full((480, 640, 3), 128, dtype=np.uint8)
        camera = MockCamera(pattern="static", static_image=static)
        camera.open()

        frame = camera.get_frame()

        assert np.array_equal(frame, static)

    def test_frame_counting(self):
        """Test frame number tracking."""
        camera = MockCamera(frame_delay=0.001)
        camera.open()

        data1 = camera.read()
        data2 = camera.read()

        assert data2.frame_number == data1.frame_number + 1


class TestMockLidar:
    """Test MockLidar implementation."""

    def test_connect(self):
        """Test LiDAR connection."""
        lidar = MockLidar()
        result = lidar.connect()

        assert result == True
        assert lidar.is_connected == True

    def test_disconnect(self):
        """Test LiDAR disconnection."""
        lidar = MockLidar()
        lidar.connect()
        lidar.disconnect()

        assert lidar.is_connected == False

    def test_start_scanning(self):
        """Test starting scan."""
        lidar = MockLidar()
        lidar.connect()

        result = lidar.start_scanning()

        assert result == True
        assert lidar.is_scanning == True

    def test_stop_scanning(self):
        """Test stopping scan."""
        lidar = MockLidar()
        lidar.connect()
        lidar.start_scanning()
        lidar.stop_scanning()

        assert lidar.is_scanning == False

    def test_get_scan(self):
        """Test getting scan data."""
        lidar = MockLidar(num_points=360, scan_delay=0.001)
        lidar.connect()
        lidar.start_scanning()

        scan = lidar.get_scan()

        assert scan is not None
        assert isinstance(scan, LidarScan)
        assert scan.num_points > 0

    def test_scan_with_obstacle(self):
        """Test scan with simulated obstacle."""
        lidar = MockLidar(scan_delay=0.001)
        lidar.add_obstacle(angle=0, distance=1.0, width=20)
        lidar.connect()
        lidar.start_scanning()

        scan = lidar.get_scan()

        # Front should have closer readings
        front_min = scan.get_front_distance(angle_range=30)
        assert front_min < 2.0

    def test_clear_obstacles(self):
        """Test clearing obstacles."""
        lidar = MockLidar()
        lidar.add_obstacle(angle=0, distance=0.5, width=10)
        lidar.clear_obstacles()

        assert len(lidar.obstacles) == 0


class TestLidarPoint:
    """Test LidarPoint dataclass."""

    def test_point_creation(self):
        """Test creating a point."""
        point = LidarPoint(angle=45.0, distance=2.0, quality=30)

        assert point.angle == 45.0
        assert point.distance == 2.0
        assert point.quality == 30

    def test_angle_radians(self):
        """Test angle conversion to radians."""
        point = LidarPoint(angle=90.0, distance=1.0, quality=30)

        assert abs(point.angle_rad - 1.5708) < 0.001

    def test_cartesian_coordinates(self):
        """Test cartesian coordinate conversion."""
        point = LidarPoint(angle=0.0, distance=1.0, quality=30)

        assert abs(point.x - 1.0) < 0.001
        assert abs(point.y) < 0.001

    def test_as_tuple(self):
        """Test tuple conversion."""
        point = LidarPoint(angle=0.0, distance=1.0, quality=30)

        x, y = point.as_tuple()
        assert abs(x - 1.0) < 0.001


class TestLidarScan:
    """Test LidarScan dataclass."""

    def test_empty_scan(self):
        """Test empty scan."""
        scan = LidarScan()

        assert scan.num_points == 0
        assert scan.min_distance == float('inf')

    def test_min_distance(self):
        """Test minimum distance calculation."""
        points = [
            LidarPoint(0.0, 2.0, 30),
            LidarPoint(45.0, 1.0, 30),
            LidarPoint(90.0, 3.0, 30),
        ]
        scan = LidarScan(points=points)

        assert scan.min_distance == 1.0

    def test_min_distance_angle(self):
        """Test angle of minimum distance."""
        points = [
            LidarPoint(0.0, 2.0, 30),
            LidarPoint(45.0, 1.0, 30),
            LidarPoint(90.0, 3.0, 30),
        ]
        scan = LidarScan(points=points)

        assert scan.min_distance_angle == 45.0

    def test_get_range(self):
        """Test getting points in angle range."""
        points = [
            LidarPoint(0.0, 1.0, 30),
            LidarPoint(45.0, 1.0, 30),
            LidarPoint(90.0, 1.0, 30),
            LidarPoint(135.0, 1.0, 30),
        ]
        scan = LidarScan(points=points)

        range_points = scan.get_range(30, 100)
        assert len(range_points) == 2

    def test_to_numpy(self):
        """Test numpy conversion."""
        points = [
            LidarPoint(0.0, 1.0, 30),
            LidarPoint(45.0, 2.0, 25),
        ]
        scan = LidarScan(points=points)

        arr = scan.to_numpy()

        assert arr.shape == (2, 3)
        assert arr[0, 0] == 0.0
        assert arr[0, 1] == 1.0

    def test_to_cartesian(self):
        """Test cartesian conversion."""
        points = [
            LidarPoint(0.0, 1.0, 30),
            LidarPoint(90.0, 1.0, 30),
        ]
        scan = LidarScan(points=points)

        cart = scan.to_cartesian()

        assert cart.shape == (2, 2)


class TestFactoryFunctions:
    """Test factory functions."""

    def test_create_arduino_mock(self):
        """Test creating mock Arduino."""
        arduino = create_arduino(use_mock=True, auto_connect=True)

        assert arduino is not None
        assert arduino.is_connected

    def test_create_camera_mock(self):
        """Test creating mock camera."""
        camera = create_camera(use_mock=True, auto_open=True)

        assert camera is not None
        assert camera.is_opened

    def test_create_lidar_mock(self):
        """Test creating mock LiDAR."""
        lidar = create_lidar(use_mock=True, auto_connect=True)

        assert lidar is not None
        assert lidar.is_connected


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

"""
Camera Capture
==============

Abstract base class and implementations for camera capture.
Supports real OpenCV capture and mock for testing.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class FrameData:
    """Camera frame data."""
    frame: np.ndarray
    timestamp: float
    frame_number: int
    width: int
    height: int

    @property
    def size(self) -> Tuple[int, int]:
        return (self.width, self.height)


class CameraBase(ABC):
    """
    Abstract base class for camera capture.

    Provides interface for frame capture.
    """

    def __init__(
        self,
        device_id: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        auto_reconnect: bool = True,
        **kwargs
    ):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.auto_reconnect = auto_reconnect

        self._opened = False
        self._frame_count = 0

    @property
    def is_opened(self) -> bool:
        return self._opened

    @abstractmethod
    def open(self) -> bool:
        """Open camera."""
        pass

    @abstractmethod
    def close(self) -> None:
        """Close camera."""
        pass

    @abstractmethod
    def read(self) -> Optional[FrameData]:
        """Read a frame."""
        pass

    def get_frame(self) -> Optional[np.ndarray]:
        """Get raw frame (convenience method)."""
        data = self.read()
        return data.frame if data else None


class CameraCapture(CameraBase):
    """
    Real camera capture using OpenCV.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._cap = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """Open camera with OpenCV."""
        if self._opened:
            return True

        try:
            import cv2
            self._cap = cv2.VideoCapture(self.device_id)

            if not self._cap.isOpened():
                logger.error(f"Failed to open camera {self.device_id}")
                return False

            # Set properties
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._cap.set(cv2.CAP_PROP_FPS, self.fps)

            # Verify
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            self._opened = True
            logger.info(f"Camera opened: {actual_w}x{actual_h}")
            return True

        except Exception as e:
            logger.error(f"Failed to open camera: {e}")
            return False

    def close(self) -> None:
        """Close camera."""
        if self._cap:
            self._cap.release()
            self._cap = None
        self._opened = False
        logger.info("Camera closed")

    def read(self) -> Optional[FrameData]:
        """Read a frame from camera."""
        if not self._opened or not self._cap:
            if self.auto_reconnect:
                if not self.open():
                    return None
            else:
                return None

        with self._lock:
            try:
                ret, frame = self._cap.read()

                if not ret or frame is None:
                    logger.warning("Failed to read frame")
                    self._opened = False
                    return None

                self._frame_count += 1

                return FrameData(
                    frame=frame,
                    timestamp=time.time(),
                    frame_number=self._frame_count,
                    width=frame.shape[1],
                    height=frame.shape[0]
                )

            except Exception as e:
                logger.error(f"Frame read error: {e}")
                self._opened = False
                return None


class MockCamera(CameraBase):
    """
    Mock camera for testing without hardware.

    Generates test patterns or static images.
    """

    def __init__(
        self,
        pattern: str = "gradient",
        frame_delay: float = 0.033,
        static_image: Optional[np.ndarray] = None,
        **kwargs
    ):
        """
        Initialize mock camera.

        Args:
            pattern: "gradient" | "noise" | "checkerboard" | "static"
            frame_delay: Delay between frames (simulates FPS)
            static_image: Static image to return (for "static" pattern)
        """
        super().__init__(**kwargs)
        self.pattern = pattern
        self.frame_delay = frame_delay
        self.static_image = static_image

        self._last_frame_time = 0.0

    def open(self) -> bool:
        """Simulate camera open."""
        self._opened = True
        logger.info(f"Mock camera opened ({self.pattern} pattern)")
        return True

    def close(self) -> None:
        """Simulate camera close."""
        self._opened = False
        logger.info("Mock camera closed")

    def read(self) -> Optional[FrameData]:
        """Generate a test frame."""
        if not self._opened:
            return None

        # Simulate frame rate
        elapsed = time.time() - self._last_frame_time
        if elapsed < self.frame_delay:
            time.sleep(self.frame_delay - elapsed)

        self._last_frame_time = time.time()
        self._frame_count += 1

        frame = self._generate_frame()

        return FrameData(
            frame=frame,
            timestamp=time.time(),
            frame_number=self._frame_count,
            width=self.width,
            height=self.height
        )

    def _generate_frame(self) -> np.ndarray:
        """Generate test pattern frame."""
        if self.pattern == "static" and self.static_image is not None:
            return self.static_image.copy()

        elif self.pattern == "noise":
            return np.random.randint(0, 256, (self.height, self.width, 3), dtype=np.uint8)

        elif self.pattern == "checkerboard":
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            block_size = 50
            for y in range(0, self.height, block_size):
                for x in range(0, self.width, block_size):
                    if ((x // block_size) + (y // block_size)) % 2 == 0:
                        frame[y:y+block_size, x:x+block_size] = [255, 255, 255]
            return frame

        elif self.pattern == "moving":
            # Moving gradient based on frame count
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            offset = (self._frame_count * 5) % 256
            for x in range(self.width):
                val = (x + offset) % 256
                frame[:, x] = [val, val, 255 - val]
            return frame

        else:  # "gradient" (default)
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            for x in range(self.width):
                val = int((x / self.width) * 255)
                frame[:, x] = [val, 255 - val, 128]
            return frame

    def set_static_image(self, image: np.ndarray) -> None:
        """Set static image to return."""
        self.static_image = image
        self.pattern = "static"


def create_camera(
    use_mock: bool = False,
    config: Optional[object] = None,
    auto_open: bool = True,
    **kwargs
) -> CameraBase:
    """
    Factory function to create camera instance.

    Args:
        use_mock: Use mock camera for testing
        config: Configuration object
        auto_open: Automatically open camera
        **kwargs: Additional arguments

    Returns:
        Camera instance
    """
    if config:
        kwargs.setdefault('device_id', config.camera.device_id)
        kwargs.setdefault('width', config.camera.width)
        kwargs.setdefault('height', config.camera.height)
        kwargs.setdefault('fps', config.camera.fps)
        kwargs.setdefault('auto_reconnect', config.camera.auto_reconnect)

    if use_mock:
        camera = MockCamera(**kwargs)
    else:
        camera = CameraCapture(**kwargs)

    if auto_open:
        camera.open()

    return camera

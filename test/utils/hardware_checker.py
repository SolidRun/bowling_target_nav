#!/usr/bin/env python3
"""
Hardware Checker for V2N Robot Tests
=====================================

Utility to check hardware availability before running tests.
Detects Arduino, LiDAR, and Camera connections.

Usage:
    from test.utils import HardwareChecker

    checker = HardwareChecker()
    if checker.arduino_available:
        # Run Arduino tests

    # Or check all at once
    checker.print_status()
"""

import os
import subprocess
from dataclasses import dataclass
from typing import Optional, List, Tuple
from .debug_logger import DebugLogger, Colors


@dataclass
class HardwareStatus:
    """Status of a hardware component."""
    name: str
    available: bool
    device: str
    details: str = ""
    error: str = ""


class HardwareChecker:
    """
    Check availability of V2N robot hardware components.

    Components checked:
        - Arduino: Motor controller on /dev/ttyACM*
        - LiDAR: RPLidar on /dev/ttyUSB*
        - Camera: USB camera on /dev/video*
        - Display: Wayland/X11 display
    """

    # Default device paths
    ARDUINO_DEVICES = ["/dev/ttyACM0", "/dev/ttyACM1"]
    LIDAR_DEVICES = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    CAMERA_DEVICES = ["/dev/video0", "/dev/video1", "/dev/video2"]

    def __init__(self, logger: Optional[DebugLogger] = None):
        self.logger = logger or DebugLogger("HardwareChecker")
        self._arduino_status: Optional[HardwareStatus] = None
        self._lidar_status: Optional[HardwareStatus] = None
        self._camera_status: Optional[HardwareStatus] = None
        self._display_status: Optional[HardwareStatus] = None

    # -------------------------------------------------------------------------
    # Arduino Check
    # -------------------------------------------------------------------------

    def check_arduino(self) -> HardwareStatus:
        """Check if Arduino is connected."""
        for device in self.ARDUINO_DEVICES:
            if os.path.exists(device):
                # Check if device is accessible
                try:
                    with open(device, 'r') as f:
                        pass
                    self._arduino_status = HardwareStatus(
                        name="Arduino",
                        available=True,
                        device=device,
                        details="Motor controller ready"
                    )
                    return self._arduino_status
                except PermissionError:
                    self._arduino_status = HardwareStatus(
                        name="Arduino",
                        available=False,
                        device=device,
                        error=f"Permission denied - run: sudo chmod 666 {device}"
                    )
                    return self._arduino_status
                except Exception as e:
                    continue

        self._arduino_status = HardwareStatus(
            name="Arduino",
            available=False,
            device="",
            error="Not found on /dev/ttyACM*"
        )
        return self._arduino_status

    @property
    def arduino_available(self) -> bool:
        """Quick check if Arduino is available."""
        if self._arduino_status is None:
            self.check_arduino()
        return self._arduino_status.available

    @property
    def arduino_device(self) -> str:
        """Get Arduino device path."""
        if self._arduino_status is None:
            self.check_arduino()
        return self._arduino_status.device

    # -------------------------------------------------------------------------
    # LiDAR Check
    # -------------------------------------------------------------------------

    def check_lidar(self) -> HardwareStatus:
        """Check if LiDAR is connected."""
        for device in self.LIDAR_DEVICES:
            if os.path.exists(device):
                try:
                    with open(device, 'r') as f:
                        pass
                    self._lidar_status = HardwareStatus(
                        name="LiDAR",
                        available=True,
                        device=device,
                        details="RPLidar ready"
                    )
                    return self._lidar_status
                except PermissionError:
                    self._lidar_status = HardwareStatus(
                        name="LiDAR",
                        available=False,
                        device=device,
                        error=f"Permission denied - run: sudo chmod 666 {device}"
                    )
                    return self._lidar_status
                except Exception:
                    continue

        self._lidar_status = HardwareStatus(
            name="LiDAR",
            available=False,
            device="",
            error="Not found on /dev/ttyUSB*"
        )
        return self._lidar_status

    @property
    def lidar_available(self) -> bool:
        """Quick check if LiDAR is available."""
        if self._lidar_status is None:
            self.check_lidar()
        return self._lidar_status.available

    @property
    def lidar_device(self) -> str:
        """Get LiDAR device path."""
        if self._lidar_status is None:
            self.check_lidar()
        return self._lidar_status.device

    # -------------------------------------------------------------------------
    # Camera Check
    # -------------------------------------------------------------------------

    def check_camera(self) -> HardwareStatus:
        """Check if camera is connected."""
        for device in self.CAMERA_DEVICES:
            if os.path.exists(device):
                # Try to get camera info
                try:
                    import cv2
                    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                    if cap.isOpened():
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        cap.release()
                        self._camera_status = HardwareStatus(
                            name="Camera",
                            available=True,
                            device=device,
                            details=f"{width}x{height} resolution"
                        )
                        return self._camera_status
                    cap.release()
                except ImportError:
                    # OpenCV not available, just check device exists
                    self._camera_status = HardwareStatus(
                        name="Camera",
                        available=True,
                        device=device,
                        details="Device exists (OpenCV not available)"
                    )
                    return self._camera_status
                except Exception:
                    continue

        self._camera_status = HardwareStatus(
            name="Camera",
            available=False,
            device="",
            error="Not found on /dev/video*"
        )
        return self._camera_status

    @property
    def camera_available(self) -> bool:
        """Quick check if camera is available."""
        if self._camera_status is None:
            self.check_camera()
        return self._camera_status.available

    @property
    def camera_device(self) -> str:
        """Get camera device path."""
        if self._camera_status is None:
            self.check_camera()
        return self._camera_status.device

    # -------------------------------------------------------------------------
    # Display Check
    # -------------------------------------------------------------------------

    def check_display(self) -> HardwareStatus:
        """Check if display is available (Wayland or X11)."""
        display = os.environ.get('DISPLAY', '')
        wayland = os.environ.get('WAYLAND_DISPLAY', '')

        if wayland:
            self._display_status = HardwareStatus(
                name="Display",
                available=True,
                device=wayland,
                details="Wayland display"
            )
        elif display:
            self._display_status = HardwareStatus(
                name="Display",
                available=True,
                device=display,
                details="X11 display"
            )
        else:
            self._display_status = HardwareStatus(
                name="Display",
                available=False,
                device="",
                error="No DISPLAY or WAYLAND_DISPLAY set"
            )
        return self._display_status

    @property
    def display_available(self) -> bool:
        """Quick check if display is available."""
        if self._display_status is None:
            self.check_display()
        return self._display_status.available

    # -------------------------------------------------------------------------
    # Combined Checks
    # -------------------------------------------------------------------------

    def check_all(self) -> List[HardwareStatus]:
        """Check all hardware components."""
        return [
            self.check_arduino(),
            self.check_lidar(),
            self.check_camera(),
            self.check_display(),
        ]

    def print_status(self):
        """Print status of all hardware components."""
        self.logger.header("Hardware Status Check")

        statuses = self.check_all()

        # Build table data
        rows = []
        for status in statuses:
            if status.available:
                state = "OK"
                info = f"{status.device} - {status.details}"
            else:
                state = "MISSING"
                info = status.error

            rows.append([status.name, state, info])

        self.logger.table(["Component", "Status", "Info"], rows)

        # Summary
        available = sum(1 for s in statuses if s.available)
        total = len(statuses)
        print()
        if available == total:
            self.logger.success(f"All {total} components available")
        else:
            self.logger.warning(f"{available}/{total} components available")

        return all(s.available for s in statuses)

    def require(self, arduino: bool = False, lidar: bool = False,
                camera: bool = False, display: bool = False) -> Tuple[bool, str]:
        """
        Check required hardware and return (success, message).

        Args:
            arduino: Require Arduino
            lidar: Require LiDAR
            camera: Require camera
            display: Require display

        Returns:
            Tuple of (all_available, error_message)
        """
        missing = []

        if arduino and not self.arduino_available:
            missing.append("Arduino")
        if lidar and not self.lidar_available:
            missing.append("LiDAR")
        if camera and not self.camera_available:
            missing.append("Camera")
        if display and not self.display_available:
            missing.append("Display")

        if missing:
            return False, f"Missing required hardware: {', '.join(missing)}"
        return True, "All required hardware available"


# =============================================================================
# Standalone Execution
# =============================================================================

if __name__ == "__main__":
    checker = HardwareChecker()
    checker.print_status()

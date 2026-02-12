"""
Pytest Configuration for V2N Robot Tests
=========================================

Provides fixtures and configuration for the test suite.
"""

import pytest
import sys
import os

# Add package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def pytest_configure(config):
    """Configure pytest."""
    config.addinivalue_line(
        "markers", "hardware: mark test as requiring hardware"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection."""
    # Add skip markers based on available hardware
    try:
        from test.utils.hardware_checker import HardwareChecker
        checker = HardwareChecker()

        for item in items:
            # Skip Arduino tests if not available
            if "arduino" in item.nodeid.lower():
                if not checker.arduino_available:
                    item.add_marker(pytest.mark.skip(
                        reason="Arduino not available"
                    ))

            # Skip LiDAR tests if not available
            if "lidar" in item.nodeid.lower():
                if not checker.lidar_available:
                    item.add_marker(pytest.mark.skip(
                        reason="LiDAR not available"
                    ))

            # Skip Camera tests if not available
            if "camera" in item.nodeid.lower():
                if not checker.camera_available:
                    item.add_marker(pytest.mark.skip(
                        reason="Camera not available"
                    ))
    except Exception:
        pass  # Continue without hardware checks


@pytest.fixture(scope="session")
def hardware_checker():
    """Provide hardware checker fixture."""
    from test.utils.hardware_checker import HardwareChecker
    return HardwareChecker()


@pytest.fixture(scope="session")
def logger():
    """Provide debug logger fixture."""
    from test.utils.debug_logger import DebugLogger
    return DebugLogger("pytest")

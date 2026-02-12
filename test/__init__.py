"""
Bowling Target Navigation - Test Suite
=======================================

Comprehensive tests for RZ/V2N robot hardware and software components.

Test Categories:
    - test_arduino.py      : Arduino motor controller tests
    - test_lidar.py        : LiDAR sensor tests
    - test_camera.py       : Camera and YOLO detection tests
    - test_lidar_camera.py : LiDAR + Camera integration tests
    - test_full_system.py  : Full system tests with GUI

Usage:
    # Run all tests
    pytest test/

    # Run specific test with verbose output
    pytest test/test_arduino.py -v

    # Run with debug logging
    pytest test/test_arduino.py -v -s

    # Run hardware tests on V2N
    python3 test/test_arduino.py --standalone
"""

__version__ = "1.0.0"
__author__ = "V2N Robot Team"

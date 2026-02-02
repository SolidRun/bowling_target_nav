#!/bin/bash
# Test RPLidar with different baud rates
# Usage: ~/test_lidar.sh [baudrate]
# Default tries 256000, then 115200

export XDG_RUNTIME_DIR=/run/user/996
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

LIDAR_PORT=${1:-/dev/ttyUSB0}
BAUDRATE=${2:-256000}

echo "=========================================="
echo "  RPLidar Test - Bowling Target Nav"
echo "=========================================="
echo ""

# Cleanup
echo "Cleaning up old processes..."
pkill -9 -f rplidar 2>/dev/null
sleep 1
fuser -k $LIDAR_PORT 2>/dev/null
sleep 2

# Check if port exists
if [ ! -e "$LIDAR_PORT" ]; then
    echo "ERROR: Port $LIDAR_PORT not found!"
    echo "Available ports:"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial ports found"
    exit 1
fi

echo "Testing LiDAR on $LIDAR_PORT at $BAUDRATE baud..."
echo "Press Ctrl+C to stop"
echo ""

# Test with specified baud rate
timeout 10 ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=$LIDAR_PORT \
    -p serial_baudrate:=$BAUDRATE \
    -p frame_id:=laser

EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo ""
    echo "SUCCESS: LiDAR works at $BAUDRATE baud"
elif [ $EXIT_CODE -eq 124 ]; then
    echo ""
    echo "SUCCESS: LiDAR running (timeout reached - this is normal)"
else
    echo ""
    echo "FAILED at $BAUDRATE baud (exit code: $EXIT_CODE)"

    # Try alternative baud rate
    if [ "$BAUDRATE" = "256000" ]; then
        ALT_BAUD=115200
    else
        ALT_BAUD=256000
    fi

    echo ""
    echo "Trying alternative baud rate: $ALT_BAUD..."
    sleep 2

    timeout 10 ros2 run rplidar_ros rplidar_node --ros-args \
        -p serial_port:=$LIDAR_PORT \
        -p serial_baudrate:=$ALT_BAUD \
        -p frame_id:=laser

    EXIT_CODE2=$?
    if [ $EXIT_CODE2 -eq 0 ] || [ $EXIT_CODE2 -eq 124 ]; then
        echo ""
        echo "SUCCESS: LiDAR works at $ALT_BAUD baud!"
        echo "Update your launch with: lidar_baudrate:=$ALT_BAUD"
    else
        echo ""
        echo "FAILED at both baud rates"
        echo ""
        echo "Troubleshooting:"
        echo "  1. Check if LiDAR motor is spinning"
        echo "  2. Check USB cable connection"
        echo "  3. Try: sudo chmod 666 $LIDAR_PORT"
        echo "  4. Check dmesg for USB errors: dmesg | tail -20"
    fi
fi

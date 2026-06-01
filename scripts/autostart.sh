#!/bin/bash
# ============================================================================
# V2N Robot Auto-Start Script
# ============================================================================
# Started by robot.service on boot. Launches bringup (LiDAR, Arduino,
# Odometry, RSP). The GUI is started separately by the launcher button
# (target-nav-launcher.service).
#
# This script guarantees a clean start:
#   1. Kills ALL old ROS2/robot processes
#   2. Releases serial ports
#   3. Cleans shared memory
#   4. Waits for hardware
#   5. Launches bringup
#   6. Keeps running via `wait` (systemd monitors this PID)
# ============================================================================

LOG_FILE="/var/log/robot_autostart.log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo ""
echo "============================================"
echo "  V2N Robot Auto-Start"
echo "  $(date)"
echo "============================================"

# ------------------------------------------------------------------
# Step 1: Kill ALL old processes (guarantee clean slate)
# ------------------------------------------------------------------
echo "Cleaning up old processes..."

# Kill by process name (covers all cases)
for proc in rplidar_node arduino_driver_node odometry_node \
            robot_state_publisher main_gui app_yolo_cam nav_node camera_node; do
    pkill -9 -f "$proc" 2>/dev/null
done
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 1

# ------------------------------------------------------------------
# Step 2: Release serial ports (port-agnostic: any ttyUSB*/ttyACM*)
# ------------------------------------------------------------------
fuser -k /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sleep 2
# Double-check — kill any process still holding the ports
fuser -k -9 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sleep 1

# ------------------------------------------------------------------
# Step 3: Clean shared memory
# ------------------------------------------------------------------
rm -f /dev/shm/v2n_camera /dev/shm/v2n_detections /dev/shm/v2n_calibration

# ------------------------------------------------------------------
# Step 4: Source ROS2 and set environment
# ------------------------------------------------------------------
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "ERROR: ROS2 not found"; exit 1; }
source ~/ros2_ws/install/setup.bash 2>/dev/null || { echo "ERROR: Workspace not built"; exit 1; }

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

# ------------------------------------------------------------------
# Step 5: Wait for hardware (max 15 seconds)
# ------------------------------------------------------------------
# Port-agnostic: we don't care which numbers the lidar/Arduino enumerated
# as (ttyUSB0/1, ttyACM0, ...). We just wait until at least 2 USB-serial
# devices are present (lidar + Arduino); the drivers auto-detect which is
# which. Camera is optional.
# List present USB-serial devices via globs (no `ls`; safe on no-match).
serial_devs() {
    local d out=""
    for d in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$d" ] && out="$out $d"
    done
    echo "$out"
}

echo "Waiting for serial hardware..."
for _ in $(seq 1 15); do
    # shellcheck disable=SC2046  # word-splitting the device list is intended
    if [ $(serial_devs | wc -w) -ge 2 ]; then
        break
    fi
    sleep 1
done

SERIAL_DEVS=$(serial_devs)
[ -n "$SERIAL_DEVS" ] && echo "[OK] Serial devices:$SERIAL_DEVS" \
                      || echo "[WARN] No USB-serial devices found"
[ -e /dev/video0 ] && echo "[OK] Camera (/dev/video0)" || echo "[WARN] Camera not found"

# ------------------------------------------------------------------
# Step 6: Cleanup on exit (called by systemd on service stop)
# ------------------------------------------------------------------
cleanup() {
    echo "$(date) Shutting down..."
    for proc in rplidar_node arduino_driver_node odometry_node \
                robot_state_publisher; do
        pkill -9 -f "$proc" 2>/dev/null
    done
    pkill -9 -f "ros2 launch" 2>/dev/null
    fuser -k /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
    echo "Cleanup done"
}
trap cleanup EXIT

# ------------------------------------------------------------------
# Step 7: Launch bringup (LiDAR, Arduino, Odometry, RSP)
# ------------------------------------------------------------------
echo ""
echo "Launching bringup..."
ros2 launch target_nav bringup.launch.py &
BRINGUP_PID=$!
sleep 4

if ! kill -0 $BRINGUP_PID 2>/dev/null; then
    echo "ERROR: Bringup failed to start!"
    exit 1
fi
echo "[OK] Bringup running (pid=$BRINGUP_PID)"

# ------------------------------------------------------------------
# Done — wait for child processes
# ------------------------------------------------------------------
echo ""
echo "============================================"
echo "  Robot Ready! (GUI via launcher button)"
echo "============================================"
echo ""

wait

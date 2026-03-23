#!/bin/bash
# ============================================================================
# Target Nav GUI Launcher
# ============================================================================
# Starts the main_gui. Reuses bringup if already running (robot.service).
# If not running, starts bringup here.
#
# Guarantees clean GUI start:
#   1. Kills any old GUI + DRP-AI processes
#   2. Cleans shared memory
#   3. Checks if bringup is running
#   4. Starts bringup if needed
#   5. Launches main_gui (blocking)
#   6. Cleanup on exit
# ============================================================================

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

export HOME=/root
# Must match robot.service / autostart.sh so DDS discovers bringup nodes
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
# Wayland display — required for GTK to find Weston
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/996}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-1}

# ------------------------------------------------------------------
# Step 1: Kill old GUI processes (never kill bringup)
# ------------------------------------------------------------------
echo "[gui] Cleaning up old GUI..."
pkill -9 -f "main_gui" 2>/dev/null
pkill -9 -f "app_yolo_cam" 2>/dev/null
pkill -9 -f "nav_node" 2>/dev/null
pkill -9 -f "camera_node" 2>/dev/null
sleep 1

# ------------------------------------------------------------------
# Step 2: Clean shared memory
# ------------------------------------------------------------------
rm -f /dev/shm/v2n_camera /dev/shm/v2n_detections /dev/shm/v2n_calibration /dev/shm/v2n_settings
rm -f /dev/shm/v2n_nav /dev/shm/v2n_laser /dev/shm/v2n_det /dev/shm/v2n_cmd
rm -f /dev/shm/v2n_bridge_laser /dev/shm/v2n_bridge_nav /dev/shm/v2n_bridge_det /dev/shm/v2n_bridge_cmd

# ------------------------------------------------------------------
# Step 3: Check if bringup is already running
# ------------------------------------------------------------------
STARTED_BRINGUP=false

if pgrep -f "rplidar_node" > /dev/null 2>&1 && pgrep -f "arduino_driver" > /dev/null 2>&1; then
    echo "[gui] Bringup already running (robot.service)"
else
    echo "[gui] Starting bringup..."
    # Release serial ports from any dead processes
    fuser -k /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null
    sleep 0.5

    ros2 launch target_nav bringup.launch.py > /tmp/bringup.log 2>&1 &
    BRINGUP_PID=$!
    STARTED_BRINGUP=true
    sleep 2

    if ! kill -0 $BRINGUP_PID 2>/dev/null; then
        echo "[gui] ERROR: Bringup failed! Check /tmp/bringup.log"
        exit 1
    fi
    echo "[gui] Bringup running (pid=$BRINGUP_PID)"
fi

# ------------------------------------------------------------------
# Step 4: Cleanup on exit
# ------------------------------------------------------------------
cleanup() {
    echo "[gui] Cleaning up..."
    pkill -9 -f "main_gui" 2>/dev/null
    pkill -9 -f "app_yolo_cam" 2>/dev/null
    pkill -9 -f "nav_node" 2>/dev/null
    pkill -9 -f "camera_node" 2>/dev/null
    
    # Only kill bringup if WE started it
    if $STARTED_BRINGUP && [ -n "$BRINGUP_PID" ]; then
        kill $BRINGUP_PID 2>/dev/null
        wait $BRINGUP_PID 2>/dev/null
    fi
    echo "[gui] Done"
}
trap cleanup EXIT

# ------------------------------------------------------------------
# Step 5: Start main_gui (blocking)
# ------------------------------------------------------------------
echo "[gui] Starting main_gui..."
ros2 run target_nav main_gui 2>&1 | tee /tmp/gui_output.log

echo "[gui] GUI exited"

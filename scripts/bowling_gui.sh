#!/bin/bash
# ============================================================================
# Bowling Target Nav GUI Launcher
# ============================================================================
# Starts the full robot stack: hardware bringup + GUI.
# Called by bowling_launcher.py's Start button.
# ============================================================================

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

kill_all() {
    pkill -f "main_gui" 2>/dev/null
    pkill -f "app_yolo_cam" 2>/dev/null
    pkill -f "bringup.launch" 2>/dev/null
    pkill -f "arduino_driver" 2>/dev/null
    pkill -f "odometry_node" 2>/dev/null
    pkill -f "rplidar" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "cartographer_node" 2>/dev/null
    pkill -f "cartographer_occupancy_grid_node" 2>/dev/null
}

# Kill ALL related processes from previous run
kill_all
sleep 0.5

# Clean shared memory and free camera
rm -f /dev/shm/v2n_camera
fuser -k /dev/video0 2>/dev/null
sleep 0.3

export HOME=/root

# Cleanup on exit: kill everything when this script exits
trap kill_all EXIT

# Start bringup (LiDAR + Arduino + Odometry + robot_state_publisher)
echo "[bowling_gui] Starting bringup (LiDAR, Arduino, Odometry)..."
ros2 launch bowling_target_nav bringup.launch.py > /tmp/bringup.log 2>&1 &
BRINGUP_PID=$!
sleep 3

# Verify bringup nodes started
if ! kill -0 $BRINGUP_PID 2>/dev/null; then
    echo "[bowling_gui] ERROR: Bringup failed to start! Check /tmp/bringup.log"
    exit 1
fi
echo "[bowling_gui] Bringup running (pid=$BRINGUP_PID)"

# Start main_gui (blocking — script waits here until GUI exits)
echo "[bowling_gui] Starting main_gui..."
ros2 run bowling_target_nav main_gui 2>&1 | tee /tmp/gui_output.log

# GUI exited — trap will clean up bringup
echo "[bowling_gui] GUI exited"

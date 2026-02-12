#!/bin/bash
# =============================================================================
# Save Map Script
# =============================================================================
#
# Saves the current SLAM map to a file for later use with Nav2 localization.
#
# Usage:
#   ./save_map.sh                    # Saves to maps/room_YYYYMMDD_HHMMSS
#   ./save_map.sh my_room            # Saves to maps/my_room
#   ./save_map.sh /path/to/my_map    # Saves to specified path
#
# Requirements:
#   - Cartographer SLAM running (mapping.launch.py)
#   - nav2_map_server package installed
#
# =============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Get script directory and package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
MAPS_DIR="$PKG_DIR/maps"

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Determine output path
if [ -z "$1" ]; then
    # Generate timestamp-based name
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    MAP_NAME="room_$TIMESTAMP"
    OUTPUT_PATH="$MAPS_DIR/$MAP_NAME"
else
    # Check if absolute path or just name
    if [[ "$1" == /* ]]; then
        OUTPUT_PATH="$1"
    else
        OUTPUT_PATH="$MAPS_DIR/$1"
    fi
fi

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Save SLAM Map${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "Output: ${GREEN}$OUTPUT_PATH${NC}"
echo ""

# Check if map topic is available
echo "Checking for map topic..."
if ! ros2 topic list | grep -q "^/map$"; then
    echo -e "${YELLOW}[WARN]${NC} /map topic not found. Is SLAM running?"
    echo "  Start SLAM with: ros2 launch bowling_target_nav mapping.launch.py"
    exit 1
fi

# Save map using nav2_map_server
echo "Saving map..."
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PATH" --ros-args -p save_map_timeout:=10.0

# Check if files were created
if [ -f "${OUTPUT_PATH}.yaml" ] && [ -f "${OUTPUT_PATH}.pgm" ]; then
    echo ""
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}  Map Saved Successfully!${NC}"
    echo -e "${GREEN}============================================${NC}"
    echo ""
    echo "Files created:"
    echo "  - ${OUTPUT_PATH}.yaml"
    echo "  - ${OUTPUT_PATH}.pgm"
    echo ""
    echo "To navigate using this map:"
    echo "  ros2 launch bowling_target_nav navigation.launch.py use_slam:=false map:=${OUTPUT_PATH}.yaml"
    echo ""
else
    echo -e "${YELLOW}[ERROR]${NC} Map files not created. Check if SLAM is running properly."
    exit 1
fi

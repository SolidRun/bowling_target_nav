#!/bin/bash
# =============================================================================
# V2N Robot Test Runner
# =============================================================================
#
# Run all tests or specific test suites for the V2N robot.
#
# Usage:
#   ./run_tests.sh              # Run all tests
#   ./run_tests.sh arduino      # Run Arduino tests only
#   ./run_tests.sh lidar        # Run LiDAR tests only
#   ./run_tests.sh camera       # Run Camera tests only
#   ./run_tests.sh integration  # Run LiDAR+Camera integration tests
#   ./run_tests.sh system       # Run full system tests
#   ./run_tests.sh --gui        # Run full system GUI
#   ./run_tests.sh --check      # Just check hardware
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
TEST_DIR="$PKG_DIR/test"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

print_header() {
    echo ""
    echo -e "${CYAN}============================================${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""
}

# Source ROS2 if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Add package to Python path
export PYTHONPATH="$PKG_DIR:$PYTHONPATH"

case "${1:-all}" in
    arduino)
        print_header "Arduino Tests"
        python3 "$TEST_DIR/test_arduino.py" --standalone
        ;;

    lidar)
        print_header "LiDAR Tests"
        python3 "$TEST_DIR/test_lidar.py" --standalone
        ;;

    camera)
        print_header "Camera Tests"
        python3 "$TEST_DIR/test_camera.py" --standalone
        ;;

    integration)
        print_header "LiDAR + Camera Integration Tests"
        python3 "$TEST_DIR/test_lidar_camera.py" --standalone
        ;;

    system)
        print_header "Full System Tests"
        python3 "$TEST_DIR/test_full_system.py" --standalone
        ;;

    --gui)
        print_header "Full System GUI"
        python3 "$TEST_DIR/test_full_system.py" --gui
        ;;

    --check)
        print_header "Hardware Check"
        python3 "$TEST_DIR/utils/hardware_checker.py"
        ;;

    --visualize-lidar)
        print_header "LiDAR Visualization"
        python3 "$TEST_DIR/test_lidar.py" --visualize
        ;;

    --visualize-camera)
        print_header "Camera Detection Demo"
        python3 "$TEST_DIR/test_camera.py" --detect
        ;;

    --visualize-fusion)
        print_header "Sensor Fusion Visualization"
        python3 "$TEST_DIR/test_lidar_camera.py" --visualize
        ;;

    all)
        print_header "Running All Tests"

        echo -e "${YELLOW}[1/5] Arduino Tests${NC}"
        python3 "$TEST_DIR/test_arduino.py" --standalone || echo -e "${RED}Arduino tests failed${NC}"

        echo ""
        echo -e "${YELLOW}[2/5] LiDAR Tests${NC}"
        python3 "$TEST_DIR/test_lidar.py" --standalone || echo -e "${RED}LiDAR tests failed${NC}"

        echo ""
        echo -e "${YELLOW}[3/5] Camera Tests${NC}"
        python3 "$TEST_DIR/test_camera.py" --standalone || echo -e "${RED}Camera tests failed${NC}"

        echo ""
        echo -e "${YELLOW}[4/5] Integration Tests${NC}"
        python3 "$TEST_DIR/test_lidar_camera.py" --standalone || echo -e "${RED}Integration tests failed${NC}"

        echo ""
        echo -e "${YELLOW}[5/5] System Tests${NC}"
        python3 "$TEST_DIR/test_full_system.py" --standalone || echo -e "${RED}System tests failed${NC}"

        print_header "All Tests Complete"
        ;;

    --help|-h)
        echo "V2N Robot Test Runner"
        echo ""
        echo "Usage: $0 [OPTION]"
        echo ""
        echo "Test Suites:"
        echo "  arduino       Run Arduino motor controller tests"
        echo "  lidar         Run LiDAR sensor tests"
        echo "  camera        Run camera and YOLO detection tests"
        echo "  integration   Run LiDAR + Camera integration tests"
        echo "  system        Run full system tests"
        echo "  all           Run all tests (default)"
        echo ""
        echo "Interactive:"
        echo "  --gui               Launch full system test GUI"
        echo "  --visualize-lidar   Launch LiDAR visualization"
        echo "  --visualize-camera  Launch camera detection demo"
        echo "  --visualize-fusion  Launch sensor fusion visualization"
        echo ""
        echo "Utilities:"
        echo "  --check       Check hardware availability"
        echo "  --help        Show this help"
        ;;

    *)
        echo -e "${RED}Unknown option: $1${NC}"
        echo "Use --help for usage information"
        exit 1
        ;;
esac

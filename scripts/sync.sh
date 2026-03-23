#!/bin/bash
# ============================================================================
# Sync project to V2N and rebuild
# ============================================================================
# Copies source files to V2N, cleans build, and rebuilds the package.
# Must rebuild because colcon --symlink-install doesn't create real
# symlinks on the V2N (setuptools copies instead). Without rebuild,
# code changes in src/ are NOT picked up by the running system.
#
# Usage:
#   ./sync.sh                    # Sync entire project + rebuild
#   ./sync.sh --no-build         # Sync only (skip rebuild)
#   ./sync.sh file1 file2 ...    # Sync specific files + rebuild
#
# Always use this script instead of raw scp.
# ============================================================================

set -e

V2N="root@192.168.50.1"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
DST="/root/ros2_ws/src/target_nav"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}[sync] Source: $SRC${NC}"

# Test connection
if ! ssh -o ConnectTimeout=3 "$V2N" "echo ok" > /dev/null 2>&1; then
    echo -e "${RED}[sync] Cannot connect to $V2N${NC}"
    exit 1
fi
echo -e "${GREEN}[sync] Connected to $V2N${NC}"

NO_BUILD=false
if [ "$1" = "--no-build" ]; then
    NO_BUILD=true
    shift
fi

if [ $# -gt 0 ]; then
    # ---- Sync specific files ----
    echo -e "${YELLOW}[sync] Syncing $# file(s)...${NC}"
    for f in "$@"; do
        echo "  $f"
        scp "$SRC/$f" "$V2N:$DST/$f"
    done
else
    # ---- Sync entire project ----
    echo -e "${YELLOW}[sync] Syncing entire project...${NC}"

    # Remove old Python package to avoid stale .pyc or deleted files
    ssh "$V2N" "rm -rf $DST/target_nav"

    # Copy all project directories
    for dir in target_nav launch config urdf resource scripts test; do
        [ -d "$SRC/$dir" ] && scp -r "$SRC/$dir" "$V2N:$DST/"
    done

    # Copy root files
    for f in setup.py setup.cfg package.xml README.md; do
        [ -f "$SRC/$f" ] && scp "$SRC/$f" "$V2N:$DST/"
    done

    # Make scripts executable
    ssh "$V2N" "chmod +x $DST/scripts/*.sh $DST/scripts/*.py $DST/scripts/setup/*.sh 2>/dev/null || true"

    # Update symlinks
    ssh "$V2N" "ln -sf $DST/scripts/gui.sh /root/gui.sh; ln -sf $DST/scripts/start.sh /root/start.sh"

    echo -e "${GREEN}[sync] Files synced${NC}"
fi

# ---- Rebuild ----
if $NO_BUILD; then
    echo -e "${YELLOW}[sync] Skipping rebuild (--no-build)${NC}"
    echo "  WARNING: Changes won't take effect until rebuild"
    exit 0
fi

echo -e "${YELLOW}[sync] Rebuilding on V2N...${NC}"
ssh "$V2N" "cd /root/ros2_ws && \
    rm -rf build/target_nav install/target_nav && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select target_nav 2>&1 | tail -5"

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}[sync] Done. Restart GUI to apply:${NC}"
    echo "  ./gui.sh"
else
    echo -e "${RED}[sync] Build FAILED${NC}"
    exit 1
fi

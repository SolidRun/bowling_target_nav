#!/bin/bash
# YOLO DRP-AI Detection — Run Script

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP="$SCRIPT_DIR/app_yolo_cam"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ── Check root ──
if [ "$(id -u)" -ne 0 ]; then
    echo -e "${YELLOW}DRP-AI requires root. Re-running with su...${NC}"
    exec su -c "\"$0\" $*"
fi

# ── Check binary ──
if [ ! -f "$APP" ]; then
    echo -e "${RED}ERROR: app_yolo_cam not found at: $APP${NC}"
    exit 1
fi

# ── Install runtime libraries ──
if [ -d "$SCRIPT_DIR/lib" ]; then
    LIBS_INSTALLED=0
    for lib in "$SCRIPT_DIR/lib"/*.so*; do
        [ -f "$lib" ] || continue
        LIBNAME="$(basename "$lib")"
        if [ ! -f "/usr/lib64/$LIBNAME" ]; then
            cp "$lib" /usr/lib64/
            LIBS_INSTALLED=$((LIBS_INSTALLED + 1))
        fi
    done
    if [ $LIBS_INSTALLED -gt 0 ]; then
        ldconfig 2>/dev/null || true
        echo -e "${GREEN}Installed $LIBS_INSTALLED runtime libraries${NC}"
    fi
fi
export LD_LIBRARY_PATH=/usr/lib64:$LD_LIBRARY_PATH

# ── Wayland display setup ──
FOUND_WAYLAND=0
for udir in /run/user/* /run /tmp; do
    [ -d "$udir" ] || continue
    for sock in "$udir"/wayland-*; do
        case "$sock" in *.lock) continue;; esac
        if [ -S "$sock" ]; then
            export XDG_RUNTIME_DIR="$udir"
            export WAYLAND_DISPLAY="$(basename "$sock")"
            FOUND_WAYLAND=1
            break 2
        fi
    done
done
if [ $FOUND_WAYLAND -eq 0 ]; then
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/0}"
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
fi
export HOME="${HOME:-/root}"

# ── Help ──
for arg in "$@"; do
    case "$arg" in
        --help|-h)
            exec "$APP" --help
            ;;
    esac
done

# ── Parse mode flags ──
MODE_FLAG=""
for arg in "$@"; do
    case "$arg" in
        --console) MODE_FLAG="--console" ;;
        --display) MODE_FLAG="--display" ;;
    esac
done

# ── Print summary ──
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  YOLO DRP-AI Detection${NC}"
echo -e "${CYAN}============================================${NC}"
echo -e "  Classes: 1 ("bowling-pins")"
echo -e "  Input:   640x640"
if [ "$MODE_FLAG" = "--console" ]; then
    echo -e "  Display: console (no GUI)"
else
    echo -e "  Display: $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY"
fi
if [ -f "$SCRIPT_DIR/config.ini" ]; then
    echo -e "  Config:  config.ini (found — edit to override defaults)"
else
    echo -e "  Config:  config.ini (not found — using compile-time defaults)"
fi
echo -e "${CYAN}============================================${NC}"
echo ""

# ── Run ──
cd "$SCRIPT_DIR"
mkdir -p logs
exec "$APP" "$@"

#!/bin/bash
# ============================================================================
# V2N Robot Complete Provisioning Script
# ============================================================================
#
# Sets up EVERYTHING on a fresh RZ/V2N device:
#   1. Verifies ROS2 environment
#   2. Builds the ROS2 package
#   3. Installs helper scripts (bowling_launcher, bowling_gui, remote_desktop)
#   4. Sets up WiFi Access Point (hostapd + udhcpd)
#   5. Creates udev rules (touchscreen, rfkill)
#   6. Creates all systemd services (robot, bowling-launcher, remote-desktop, wifi-ap)
#   7. Enables all services for auto-start on boot
#   8. Checks hardware
#
# Prerequisites (manual steps before running this script):
#   1. Copy ROS2 project:
#      scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/
#   2. Copy DRP-AI app (optional, for hardware-accelerated detection):
#      scp -r drpai_app root@192.168.50.1:/home/root/deploy/
#
# Usage:
#   ssh root@192.168.50.1
#   cd ~/ros2_ws/src/bowling_target_nav/scripts
#   chmod +x v2n_setup.sh && ./v2n_setup.sh
#
# Options:
#   ./v2n_setup.sh              # Full setup (build + services + WiFi AP)
#   ./v2n_setup.sh --no-build   # Skip colcon build (if already built)
#   ./v2n_setup.sh --wifi-only  # Only set up WiFi AP
#   ./v2n_setup.sh --status     # Show status of all services
#
# ============================================================================

set -e

# ─── Colors ───
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ─── Configuration ───
WIFI_SSID="RZV2N_Robot"
WIFI_PASS="robot1234"
AP_IP="192.168.50.1"
DHCP_START="192.168.50.100"
DHCP_END="192.168.50.200"
REMOTE_DESKTOP_PORT=8080

# ─── Paths ───
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$HOME/ros2_ws"

STEP=0

# ─── Helpers ───
step() {
    STEP=$((STEP + 1))
    echo ""
    echo -e "${CYAN}────────────────────────────────────────────${NC}"
    echo -e "${CYAN}  Step $STEP: $1${NC}"
    echo -e "${CYAN}────────────────────────────────────────────${NC}"
}

ok()   { echo -e "  ${GREEN}[OK]${NC} $1"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; }
info() { echo -e "  $1"; }

# ─── Parse args ───
SKIP_BUILD=false
WIFI_ONLY=false
SHOW_STATUS=false

for arg in "$@"; do
    case "$arg" in
        --no-build)   SKIP_BUILD=true ;;
        --wifi-only)  WIFI_ONLY=true ;;
        --status)     SHOW_STATUS=true ;;
        --help|-h)
            echo "V2N Robot Provisioning Script"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  (none)       Full setup"
            echo "  --no-build   Skip colcon build"
            echo "  --wifi-only  Only set up WiFi AP"
            echo "  --status     Show service status"
            echo "  --help       Show this help"
            exit 0
            ;;
    esac
done

# ═══════════════════════════════════════════════════════════════════════
# Status mode
# ═══════════════════════════════════════════════════════════════════════
if [ "$SHOW_STATUS" = true ]; then
    echo -e "${CYAN}══════════════════════════════════════════${NC}"
    echo -e "${CYAN}  V2N Service Status${NC}"
    echo -e "${CYAN}══════════════════════════════════════════${NC}"
    echo ""
    for svc in seatd robot bowling-launcher remote-desktop wifi-ap; do
        if systemctl is-active --quiet "$svc" 2>/dev/null; then
            echo -e "  ${GREEN}●${NC} $svc: ${GREEN}active${NC}"
        elif systemctl is-enabled --quiet "$svc" 2>/dev/null; then
            echo -e "  ${YELLOW}●${NC} $svc: ${YELLOW}enabled but not running${NC}"
        else
            echo -e "  ${RED}●${NC} $svc: ${RED}not installed${NC}"
        fi
    done
    echo ""
    echo "Hardware:"
    [ -e /dev/ttyACM0 ] && echo -e "  ${GREEN}●${NC} Arduino (/dev/ttyACM0)" || echo -e "  ${RED}●${NC} Arduino not found"
    [ -e /dev/ttyUSB0 ] && echo -e "  ${GREEN}●${NC} LiDAR (/dev/ttyUSB0)" || echo -e "  ${RED}●${NC} LiDAR not found"
    [ -e /dev/video0 ]  && echo -e "  ${GREEN}●${NC} Camera (/dev/video0)" || echo -e "  ${RED}●${NC} Camera not found"
    echo ""
    echo "DRP-AI:"
    [ -f /home/root/deploy/app_yolo_cam ] && echo -e "  ${GREEN}●${NC} app_yolo_cam found" || echo -e "  ${YELLOW}●${NC} app_yolo_cam not found (ONNX fallback)"
    echo ""
    exit 0
fi

# ═══════════════════════════════════════════════════════════════════════
# Main setup
# ═══════════════════════════════════════════════════════════════════════

echo -e "${CYAN}══════════════════════════════════════════${NC}"
echo -e "${CYAN}  V2N Robot Complete Provisioning${NC}"
echo -e "${CYAN}══════════════════════════════════════════${NC}"
echo ""
echo "  WiFi SSID:     $WIFI_SSID"
echo "  WiFi Pass:     $WIFI_PASS"
echo "  AP IP:         $AP_IP"
echo "  Remote Desktop: http://$AP_IP:$REMOTE_DESKTOP_PORT"
echo ""

# ═══════════════════════════════════════════════════════════════════════
# Step 1: Verify environment
# ═══════════════════════════════════════════════════════════════════════
step "Verifying environment"

if [ ! -d /opt/ros/humble ]; then
    fail "ROS2 Humble not found at /opt/ros/humble"
    fail "Is this an RZ/V2N device with ROS2?"
    exit 1
fi
ok "ROS2 Humble found"

if [ ! -f "$PKG_DIR/package.xml" ]; then
    fail "package.xml not found in $PKG_DIR"
    fail "Run this script from: ~/ros2_ws/src/bowling_target_nav/scripts/"
    exit 1
fi
ok "Package found at $PKG_DIR"

# Create workspace if needed
mkdir -p "$WS_DIR/src"

# Ensure package is in workspace
if [[ "$PKG_DIR" != *"ros2_ws/src"* ]]; then
    info "Copying package to workspace..."
    cp -r "$PKG_DIR" "$WS_DIR/src/"
    PKG_DIR="$WS_DIR/src/bowling_target_nav"
    SCRIPT_DIR="$PKG_DIR/scripts"
    ok "Package copied to $WS_DIR/src/"
else
    ok "Package already in workspace"
fi

source /opt/ros/humble/setup.bash

# ═══════════════════════════════════════════════════════════════════════
# Step 2: Install Python dependencies
# ═══════════════════════════════════════════════════════════════════════
step "Checking Python dependencies"

for mod in serial onnxruntime; do
    if python3 -c "import $mod" 2>/dev/null; then
        ok "$mod already installed"
    else
        info "Installing $mod via pip3..."
        pip3 install --break-system-packages "$( [ "$mod" = "serial" ] && echo "pyserial" || echo "$mod" )" 2>&1 | tail -1
        if python3 -c "import $mod" 2>/dev/null; then
            ok "$mod installed"
        else
            fail "$mod failed to install"
        fi
    fi
done

# ═══════════════════════════════════════════════════════════════════════
# Step 3: Build ROS2 package
# ═══════════════════════════════════════════════════════════════════════
if [ "$WIFI_ONLY" = false ]; then

if [ "$SKIP_BUILD" = true ]; then
    step "Build (skipped with --no-build)"
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
        ok "Using existing build"
    else
        fail "No existing build found. Remove --no-build flag."
        exit 1
    fi
else
    step "Building ROS2 package"

    # Make scripts executable
    chmod +x "$SCRIPT_DIR/"*.sh 2>/dev/null || true
    chmod +x "$SCRIPT_DIR/"*.py 2>/dev/null || true

    cd "$WS_DIR"
    rm -rf build/bowling_target_nav install/bowling_target_nav 2>/dev/null || true

    if colcon build --packages-select bowling_target_nav --symlink-install; then
        ok "Package built successfully"
    else
        fail "Build failed"
        exit 1
    fi

    source "$WS_DIR/install/setup.bash"

    if ros2 pkg list 2>/dev/null | grep -q bowling_target_nav; then
        ok "Package registered with ROS2"
    else
        fail "Package not found in ROS2 after build"
        exit 1
    fi
fi

# ═══════════════════════════════════════════════════════════════════════
# Step 4: Install helper scripts to /root/
# ═══════════════════════════════════════════════════════════════════════
step "Installing helper scripts"

# bowling_launcher.py - GTK desktop launcher with Start/Stop GUI button
cp "$SCRIPT_DIR/bowling_launcher.py" /root/bowling_launcher.py
chmod +x /root/bowling_launcher.py
ok "bowling_launcher.py -> /root/"

# bowling_gui.sh - shell script that kills old processes and starts main_gui
cp "$SCRIPT_DIR/bowling_gui.sh" /root/bowling_gui.sh
chmod +x /root/bowling_gui.sh
ok "bowling_gui.sh -> /root/"

# remote_desktop.py - DRM-based web remote desktop server
cp "$SCRIPT_DIR/remote_desktop.py" /root/remote_desktop.py
chmod +x /root/remote_desktop.py
ok "remote_desktop.py -> /root/"

fi  # end WIFI_ONLY check

# ═══════════════════════════════════════════════════════════════════════
# Step 5: Set up WiFi Access Point
# ═══════════════════════════════════════════════════════════════════════
step "Setting up WiFi Access Point"

# hostapd configuration
cat > /etc/hostapd.conf << EOF
interface=wlan0
driver=nl80211
ssid=$WIFI_SSID
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$WIFI_PASS
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF
ok "hostapd.conf (SSID: $WIFI_SSID)"

# DHCP configuration
cat > /etc/udhcpd.conf << EOF
interface       wlan0
start           $DHCP_START
end             $DHCP_END
opt     subnet  255.255.255.0
opt     router  $AP_IP
opt     dns     8.8.8.8
opt     lease   3600
EOF
ok "udhcpd.conf ($DHCP_START - $DHCP_END)"

# Ensure /usr/local/bin exists
mkdir -p /usr/local/bin

# WiFi AP start script
cat > /usr/local/bin/wifi-ap-start << 'WIFISTART'
#!/bin/bash
# Start V2N as WiFi Access Point with DHCP

AP_IP="192.168.50.1"

# Log to syslog
logger -t wifi-ap "Starting WiFi Access Point..."

# Stop existing services
killall wpa_supplicant hostapd udhcpd 2>/dev/null
connmanctl disable wifi 2>/dev/null
sleep 1

# Unblock WiFi
rfkill unblock wifi
sleep 1

# Configure interface
ip link set wlan0 down
iw dev wlan0 set type __ap 2>/dev/null || true
ip link set wlan0 up
ip addr flush dev wlan0
ip addr add ${AP_IP}/24 dev wlan0

# Start hostapd
hostapd -B /etc/hostapd.conf
if [ $? -ne 0 ]; then
    logger -t wifi-ap "Failed to start hostapd"
    exit 1
fi

# Start DHCP server
mkdir -p /var/lib/misc
touch /var/lib/misc/udhcpd.leases
udhcpd /etc/udhcpd.conf 2>/dev/null

logger -t wifi-ap "WiFi AP started: SSID=RZV2N_Robot IP=${AP_IP}"
echo "WiFi AP Started - SSID: RZV2N_Robot, IP: ${AP_IP}"
WIFISTART
chmod +x /usr/local/bin/wifi-ap-start
ok "wifi-ap-start script"

# WiFi AP stop script
cat > /usr/local/bin/wifi-ap-stop << 'WIFISTOP'
#!/bin/bash
killall hostapd udhcpd 2>/dev/null
ip addr flush dev wlan0
ip link set wlan0 down
logger -t wifi-ap "WiFi AP stopped"
echo "WiFi AP stopped"
WIFISTOP
chmod +x /usr/local/bin/wifi-ap-stop
ok "wifi-ap-stop script"

# ═══════════════════════════════════════════════════════════════════════
# Step 6: Create udev rules
# ═══════════════════════════════════════════════════════════════════════
step "Creating udev rules"

# Touchscreen symlink rule
cat > /etc/udev/rules.d/touchscreen.rules << 'TSRULE'
# Create a symlink to any touchscreen input device
SUBSYSTEM=="input", KERNEL=="event[0-9]*", ATTRS{modalias}=="input:*-e0*,3,*a0,1,*18,*", SYMLINK+="input/touchscreen0"
SUBSYSTEM=="input", KERNEL=="event[0-9]*", ATTRS{modalias}=="ads7846", SYMLINK+="input/touchscreen0"
TSRULE
ok "touchscreen.rules"

# WiFi/Bluetooth rfkill unblock rule
cat > /etc/udev/rules.d/99-rfkill-unblock.rules << 'RFKILL'
# Unblock WiFi and Bluetooth when devices appear
# BCM43455 WiFi (brcmfmac)
ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlan*", RUN+="/usr/sbin/rfkill unblock wifi"

# Bluetooth
ACTION=="add", SUBSYSTEM=="bluetooth", RUN+="/usr/sbin/rfkill unblock bluetooth"
RFKILL
ok "99-rfkill-unblock.rules"

# Reload udev rules
udevadm control --reload-rules 2>/dev/null || true
ok "udev rules reloaded"

# ═══════════════════════════════════════════════════════════════════════
# Step 7: Configure display (screen rotation)
# ═══════════════════════════════════════════════════════════════════════
step "Configuring display"

WESTON_INI="/etc/xdg/weston/weston.ini"
mkdir -p "$(dirname "$WESTON_INI")"

cat > "$WESTON_INI" << 'WESTONEOF'
[core]
idle-time=0
require-input=false

[output]
name=DSI-1
transform=rotate-180
WESTONEOF
ok "weston.ini (DSI-1 rotate-180)"

# ═══════════════════════════════════════════════════════════════════════
# Step 8: Configure seat management and weston boot fixes
# ═══════════════════════════════════════════════════════════════════════
step "Configuring seat management (seatd + weston runtime dir)"

# seatd service - manages DRM master for weston (MUST start before weston)
cat > /etc/systemd/system/seatd.service << 'SVCEOF'
[Unit]
Description=Seat management daemon
Documentation=man:seatd(1)
Before=weston.service

[Service]
Type=simple
ExecStart=/usr/bin/seatd -g wayland
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
SVCEOF
ok "seatd.service (DRM master management)"

# weston service drop-in: create XDG_RUNTIME_DIR before weston starts
WESTON_UID=$(id -u weston 2>/dev/null || echo 996)
mkdir -p /etc/systemd/system/weston.service.d
cat > /etc/systemd/system/weston.service.d/runtime-dir.conf << DROPEOF
[Service]
ExecStartPre=/bin/bash -c "uid=\$(id -u weston) && mkdir -p /run/user/\$uid && chown weston:weston /run/user/\$uid && chmod 700 /run/user/\$uid"
DROPEOF
ok "weston.service.d/runtime-dir.conf (XDG_RUNTIME_DIR)"

# tmpfiles.d for runtime dir persistence
cat > /etc/tmpfiles.d/weston.conf << TMPEOF
d /run/user/$WESTON_UID 0700 weston weston -
TMPEOF
ok "tmpfiles.d/weston.conf"

# ═══════════════════════════════════════════════════════════════════════
# Step 9: Create systemd services
# ═══════════════════════════════════════════════════════════════════════
step "Creating systemd services"

# --- wifi-ap.service ---
cat > /etc/systemd/system/wifi-ap.service << 'SVCEOF'
[Unit]
Description=WiFi Access Point with DHCP
After=network.target
Wants=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/wifi-ap-start
ExecStop=/usr/local/bin/wifi-ap-stop

[Install]
WantedBy=multi-user.target
SVCEOF
ok "wifi-ap.service (WiFi AP + DHCP)"

# --- robot.service ---
cat > /etc/systemd/system/robot.service << SVCEOF
[Unit]
Description=V2N Robot Auto-Start (ROS2 Bowling Target Nav)
After=network.target multi-user.target
Wants=network.target

[Service]
Type=simple
User=root
WorkingDirectory=$WS_DIR/src/bowling_target_nav/scripts
ExecStartPre=/bin/sleep 5
ExecStart=$WS_DIR/src/bowling_target_nav/scripts/robot_autostart.sh
ExecStop=/bin/bash -c 'pkill -f "ros2 launch bowling" || true; pkill -f arduino_driver || true; pkill -f rplidar || true'
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

Environment="HOME=/root"
Environment="ROS_DOMAIN_ID=0"
Environment="ROS_LOCALHOST_ONLY=0"
Environment="XDG_RUNTIME_DIR=/run/user/996"
Environment="WAYLAND_DISPLAY=wayland-1"

[Install]
WantedBy=multi-user.target
SVCEOF
ok "robot.service (bringup + SLAM)"

# --- bowling-launcher.service ---
# MUST start after weston - needs Wayland display
cat > /etc/systemd/system/bowling-launcher.service << 'SVCEOF'
[Unit]
Description=Bowling GUI Desktop Launcher
After=weston.service
Wants=weston.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 3
ExecStart=/usr/bin/python3 /root/bowling_launcher.py
Restart=always
RestartSec=5
Environment=HOME=/root
Environment=XDG_RUNTIME_DIR=/run/user/996
Environment=WAYLAND_DISPLAY=wayland-1

[Install]
WantedBy=graphical.target
SVCEOF
ok "bowling-launcher.service (GUI Start/Stop button)"

# --- remote-desktop.service ---
# MUST start after weston to avoid stealing DRM master
cat > /etc/systemd/system/remote-desktop.service << 'SVCEOF'
[Unit]
Description=V2N Remote Desktop Server
After=weston.service
Wants=weston.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 3
ExecStart=/usr/bin/python3 -u /root/remote_desktop.py
Restart=always
RestartSec=5
Environment=HOME=/root
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=graphical.target
SVCEOF
ok "remote-desktop.service (http://$AP_IP:$REMOTE_DESKTOP_PORT)"

# ═══════════════════════════════════════════════════════════════════════
# Step 10: Enable all services
# ═══════════════════════════════════════════════════════════════════════
step "Enabling services"

systemctl daemon-reload
ok "systemd reloaded"

for svc in seatd wifi-ap robot; do
    systemctl enable "$svc" 2>/dev/null
    ok "Enabled: $svc"
done

# These two belong to graphical.target - reenable to move symlinks if needed
for svc in bowling-launcher remote-desktop; do
    systemctl reenable "$svc" 2>/dev/null
    ok "Enabled: $svc (graphical.target)"
done

# ═══════════════════════════════════════════════════════════════════════
# Step 11: Start/restart services
# ═══════════════════════════════════════════════════════════════════════
step "Starting services"

# WiFi AP
if ! systemctl is-active --quiet wifi-ap; then
    systemctl start wifi-ap
    sleep 2
fi
if ip addr show wlan0 2>/dev/null | grep -q "$AP_IP"; then
    ok "WiFi AP running (SSID: $WIFI_SSID, IP: $AP_IP)"
else
    warn "WiFi AP may not have started. Check: systemctl status wifi-ap"
fi

# seatd must start before weston, remote-desktop must start after weston
systemctl stop remote-desktop 2>/dev/null
systemctl restart seatd
sleep 1
ok "seatd started"

systemctl restart weston
sleep 3
if systemctl is-active --quiet weston; then
    ok "weston running (DSI-1 rotate-180)"
else
    warn "weston failed to start. Check: journalctl -u weston"
fi

systemctl restart remote-desktop
sleep 2
if systemctl is-active --quiet remote-desktop; then
    ok "remote-desktop running (http://$AP_IP:$REMOTE_DESKTOP_PORT)"
else
    warn "remote-desktop failed. Check: journalctl -u remote-desktop"
fi

# bowling-launcher needs weston's Wayland display
systemctl restart bowling-launcher
sleep 2
if systemctl is-active --quiet bowling-launcher; then
    ok "bowling-launcher running (Start GUI button on screen)"
else
    warn "bowling-launcher failed. Check: journalctl -u bowling-launcher"
fi

# ═══════════════════════════════════════════════════════════════════════
# Step 12: Check hardware
# ═══════════════════════════════════════════════════════════════════════
step "Checking hardware"

[ -e /dev/ttyACM0 ] && ok "Arduino found (/dev/ttyACM0)" || warn "Arduino not found (/dev/ttyACM0)"
[ -e /dev/ttyUSB0 ] && ok "LiDAR found (/dev/ttyUSB0)" || warn "LiDAR not found (/dev/ttyUSB0)"
[ -e /dev/video0 ]  && ok "Camera found (/dev/video0)" || warn "Camera not found (/dev/video0)"
[ -e /dev/dri/card0 ] && ok "DRM framebuffer (/dev/dri/card0)" || warn "DRM framebuffer not found"

if [ -f /home/root/deploy/app_yolo_cam ]; then
    ok "DRP-AI app found (/home/root/deploy/app_yolo_cam)"
    # Install DRP-AI runtime libraries to system path
    if [ -d /home/root/deploy/lib ]; then
        LIBS_INSTALLED=0
        for lib in /home/root/deploy/lib/*.so*; do
            [ -f "$lib" ] || continue
            LIBNAME="$(basename "$lib")"
            if [ ! -f "/usr/lib64/$LIBNAME" ]; then
                cp "$lib" /usr/lib64/
                LIBS_INSTALLED=$((LIBS_INSTALLED + 1))
            fi
        done
        if [ $LIBS_INSTALLED -gt 0 ]; then
            ldconfig 2>/dev/null || true
            ok "Installed $LIBS_INSTALLED DRP-AI runtime libraries to /usr/lib64/"
        else
            ok "DRP-AI runtime libraries already installed"
        fi
    fi
else
    warn "DRP-AI app not found - will use ONNX CPU fallback"
    info "  To add DRP-AI: scp -r drpai_app root@$AP_IP:/home/root/deploy/"
fi

# ═══════════════════════════════════════════════════════════════════════
# Done!
# ═══════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}══════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  Setup Complete!${NC}"
echo -e "${CYAN}══════════════════════════════════════════${NC}"
echo ""
echo "  What happens on reboot:"
echo "    1. seatd.service     -> Seat/DRM master management"
echo "    2. weston.service    -> Wayland compositor (DSI-1 rotate-180)"
echo "    3. wifi-ap.service   -> WiFi AP starts (SSID: $WIFI_SSID)"
echo "    4. robot.service     -> Bringup + SLAM mapping start"
echo "    5. bowling-launcher  -> Start/Stop GUI button appears on screen"
echo "    6. remote-desktop    -> Web remote at http://$AP_IP:$REMOTE_DESKTOP_PORT"
echo ""
echo "  Connect to robot:"
echo "    1. Connect to WiFi: $WIFI_SSID (password: $WIFI_PASS)"
echo "    2. Open browser: http://$AP_IP:$REMOTE_DESKTOP_PORT"
echo "    3. Click 'Start GUI' on the launcher"
echo ""
echo "  Service commands:"
echo "    systemctl status robot bowling-launcher remote-desktop wifi-ap"
echo "    journalctl -u robot -f           # Robot logs"
echo "    journalctl -u bowling-launcher -f # Launcher logs"
echo ""
echo "  Check status anytime:"
echo "    $SCRIPT_DIR/v2n_setup.sh --status"
echo ""
echo -e "  ${YELLOW}Reboot to verify auto-start:${NC}  reboot"
echo ""

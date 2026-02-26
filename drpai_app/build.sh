#!/bin/bash
# Build script for YOLO DRP-AI application
# Run inside the DRP-AI TVM Docker container.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== YOLO DRP-AI Application Builder ==="
echo ""

# --- Source SDK environment ---
unset LD_LIBRARY_PATH 2>/dev/null
if [ -z "$SDKTARGETSYSROOT" ]; then
    SDK_ENV=$(find /opt/ -maxdepth 3 -name "environment-setup-*poky-linux" 2>/dev/null | head -1)
    if [ -n "$SDK_ENV" ]; then
        echo "Sourcing SDK: $SDK_ENV"
        source "$SDK_ENV"
    else
        echo "ERROR: SDK env script not found under /opt/"
        exit 1
    fi
fi
echo "Sysroot: $SDKTARGETSYSROOT"

# --- Resolve TVM_ROOT ---
export TVM_ROOT="${TVM_ROOT:-/drp-ai_tvm}"
if [ ! -d "$TVM_ROOT" ]; then
    echo "ERROR: TVM_ROOT not found: $TVM_ROOT"
    exit 1
fi
echo "TVM_ROOT: $TVM_ROOT"

export PRODUCT="${PRODUCT:-V2N}"
echo "PRODUCT: $PRODUCT"
echo ""

# --- Build (clean build to avoid stale CMake cache) ---
echo "Building app_yolo_cam..."
cd "$SCRIPT_DIR"
rm -rf build
mkdir -p build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain/runtime.cmake \
      -DAPP_NAME=app_yolo_cam ..
make -j$(nproc)

echo ""
echo "============================================"
echo "  BUILD COMPLETE: build/app_yolo_cam"
echo "============================================"
echo ""
echo "Next: ./package.sh"
echo ""

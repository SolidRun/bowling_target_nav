#!/bin/bash
# Package for V2H/V2N Deployment

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEPLOY="$SCRIPT_DIR/deploy"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== Packaging for Deployment ===${NC}"

BINARY="$SCRIPT_DIR/build/app_yolo_cam"
if [ ! -f "$BINARY" ]; then
    echo "ERROR: Binary not found: $BINARY"
    echo "  Run build.sh first."
    exit 1
fi

rm -rf "$DEPLOY"
mkdir -p "$DEPLOY"

cp "$BINARY" "$DEPLOY/"
echo -e "  ${GREEN}OK${NC} app_yolo_cam"

[ -f "$SCRIPT_DIR/labels.txt" ] && cp "$SCRIPT_DIR/labels.txt" "$DEPLOY/"
[ -f "$SCRIPT_DIR/run.sh" ] && cp "$SCRIPT_DIR/run.sh" "$DEPLOY/" && chmod +x "$DEPLOY/run.sh"
[ -f "$SCRIPT_DIR/config.ini" ] && cp "$SCRIPT_DIR/config.ini" "$DEPLOY/" && echo -e "  ${GREEN}OK${NC} config.ini"

if [ -d "$SCRIPT_DIR/lib" ]; then
    cp -r "$SCRIPT_DIR/lib" "$DEPLOY/"
    echo -e "  ${GREEN}OK${NC} lib/"
fi

# Copy model directory — only runtime files, skip compilation artifacts
# (_calibration/, compile.log, interpreter_out/, input_0.bin, etc.)
MODEL_FOUND=0
for model_src in "$SCRIPT_DIR/../drpai_model" "$SCRIPT_DIR/../../drpai_model" "$SCRIPT_DIR/../model"; do
    if [ -d "$model_src" ] && ls "$model_src"/sub_*__CPU_DRP_TVM 1>/dev/null 2>&1; then
        mkdir -p "$DEPLOY/model"
        # Copy TVM runtime dirs (deploy.so, deploy.json, deploy.params)
        for tvm_dir in "$model_src"/sub_*__CPU_DRP_TVM; do
            cp -r "$tvm_dir" "$DEPLOY/model/"
        done
        # Copy preprocess dir
        [ -d "$model_src/preprocess" ] && cp -r "$model_src/preprocess" "$DEPLOY/model/"
        # Copy mera.plan (runtime metadata)
        [ -f "$model_src/mera.plan" ] && cp "$model_src/mera.plan" "$DEPLOY/model/"
        echo -e "  ${GREEN}OK${NC} model/ (runtime files only)"
        MODEL_FOUND=1
        break
    fi
done
[ $MODEL_FOUND -eq 0 ] && echo -e "  ${YELLOW}WARN${NC} drpai_model/ not found — copy manually as deploy/model/"

mkdir -p "$DEPLOY/logs"

TOTAL_SIZE=$(du -sh "$DEPLOY" | cut -f1)
echo ""
echo -e "${GREEN}Deploy folder ready: $DEPLOY ($TOTAL_SIZE)${NC}"
echo "  scp -r $DEPLOY root@<board-ip>:/home/root/"
echo "  ssh root@<board-ip> 'cd /home/root/deploy && ./run.sh'"
echo ""

#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║  run_step2.sh — Semantic Projection (YOLO + Depth → 3D)         ║
# ║                                                                  ║
# ║  Usage:                                                          ║
# ║    ./run_step2.sh                   → default (rate 0.5, cpu)   ║
# ║    ./run_step2.sh 0.3               → rate lebih lambat         ║
# ║    ./run_step2.sh 0.5 cuda          → pakai GPU                 ║
# ║    ./run_step2.sh 0.5 cpu 6.0      → max depth 6 meter         ║
# ║    ./run_step2.sh diagnose          → cek dependensi            ║
# ║                                                                  ║
# ║  CATATAN: Script ini HANYA untuk Step 2.                        ║
# ║           Untuk Step 1 saja → pakai run_step1.sh               ║
# ║           Untuk full RTABMap  → pakai run.sh                   ║
# ╚══════════════════════════════════════════════════════════════════╝

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Default arguments ─────────────────────────────────────────────
BAG_PATH="/root/ws/rosbag2_2026_04_10-03_17_38"
MODEL_PATH="$SCRIPT_DIR/models/yolov8n_coco.pt"
RATE="${1:-0.5}"
DEVICE="${2:-cpu}"
MAX_DEPTH="${3:-8.0}"

# ── Banner ────────────────────────────────────────────────────────
print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   STEP 2 — Semantic Projection           ║"
    echo "  ║   YOLO + Depth → Semantic Point Cloud    ║"
    echo "  ║   Tanpa RTABMap                           ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ── Diagnose ──────────────────────────────────────────────────────
diagnose() {
    echo -e "\n${CYAN}${BOLD}Cek dependensi Step 2...${NC}\n"

    echo -e "${CYAN}ROS packages:${NC}"
    for pkg in topic_tools vision_msgs rviz2; do
        ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg — install: sudo apt install ros-humble-$(echo $pkg | tr '_' '-')${NC}"
    done

    echo -e "\n${CYAN}Python packages:${NC}"
    for pkg in ultralytics cv2 numpy; do
        python3 -c "import $pkg" 2>/dev/null && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg${NC}"
    done

    # message_filters (built-in ROS, cek via python)
    python3 -c "import message_filters" 2>/dev/null && \
        echo -e "  ${GREEN}✓ message_filters${NC}" || \
        echo -e "  ${RED}✗ message_filters (harusnya ada bersama ROS2)${NC}"

    echo -e "\n${CYAN}Files:${NC}"
    for f in \
        "$MODEL_PATH" \
        "$SCRIPT_DIR/semantic/yolo_detector.py" \
        "$SCRIPT_DIR/semantic/semantic_projector.py" \
        "$SCRIPT_DIR/decompress_depth_fix.py"; do
        [ -f "$f" ] && \
            echo -e "  ${GREEN}✓ $f${NC}" || \
            echo -e "  ${RED}✗ $f${NC}"
    done

    echo -e "\n${CYAN}Rosbag:${NC}"
    [ -d "$BAG_PATH" ] && \
        echo -e "  ${GREEN}✓ $BAG_PATH${NC}" || \
        echo -e "  ${RED}✗ $BAG_PATH${NC}"
    echo ""
}

# ── Main launch ───────────────────────────────────────────────────
launch() {
    print_banner

    echo -e "${CYAN}Konfigurasi:${NC}"
    echo -e "  Bag       : $BAG_PATH"
    echo -e "  Model     : $MODEL_PATH"
    echo -e "  Rate      : ${RATE}x"
    echo -e "  Device    : $DEVICE"
    echo -e "  Max depth : ${MAX_DEPTH} m"
    echo ""

    [ ! -d "$BAG_PATH" ] && \
        echo -e "${RED}✗ Rosbag tidak ditemukan: $BAG_PATH${NC}" && exit 1
    [ ! -f "$MODEL_PATH" ] && \
        echo -e "${RED}✗ Model tidak ditemukan: $MODEL_PATH${NC}" && exit 1
    [ ! -f "$SCRIPT_DIR/semantic/semantic_projector.py" ] && \
        echo -e "${RED}✗ semantic_projector.py tidak ditemukan${NC}" && exit 1

    [ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash
    [ -f "/root/ws/install/setup.bash" ] && source /root/ws/install/setup.bash

    echo -e "${GREEN}✓ Semua siap — launching Step 2...${NC}\n"
    echo -e "${YELLOW}  Di RViz:${NC}"
    echo -e "${YELLOW}    Add PointCloud2 → topic: /semantic/pointcloud${NC}"
    echo -e "${YELLOW}    Add Image       → topic: /semantic/image_annotated${NC}\n"

    ros2 launch "$SCRIPT_DIR/launch/step2_semantic_projection.launch.py" \
        bag_path:="$BAG_PATH" \
        rate:="$RATE" \
        model_path:="$MODEL_PATH" \
        device:="$DEVICE" \
        max_depth:="$MAX_DEPTH" \
        rviz:=true
}

# ── Entry point ───────────────────────────────────────────────────
case "$1" in
    diagnose) source /opt/ros/humble/setup.bash 2>/dev/null; diagnose ;;
    help|-h)
        echo ""
        echo -e "${CYAN}Usage: ./run_step2.sh [rate] [device] [max_depth]${NC}"
        echo ""
        echo "  ./run_step2.sh                 default: rate 0.5, cpu, depth 8m"
        echo "  ./run_step2.sh 0.3             rate lebih lambat"
        echo "  ./run_step2.sh 0.5 cuda        pakai GPU"
        echo "  ./run_step2.sh 0.5 cpu 6.0    max depth 6 meter"
        echo "  ./run_step2.sh diagnose        cek semua dependensi"
        echo ""
        ;;
    *) launch ;;
esac

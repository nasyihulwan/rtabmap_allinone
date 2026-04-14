#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║  run_step1.sh — YOLO Standalone Detection                        ║
# ║                                                                  ║
# ║  Usage:                                                          ║
# ║    ./run_step1.sh                  → default (rate 0.5, cpu)    ║
# ║    ./run_step1.sh 0.3              → rate 0.3x                  ║
# ║    ./run_step1.sh 0.5 cuda         → pakai GPU                  ║
# ║    ./run_step1.sh 0.5 cpu 0.4     → confidence threshold 0.4    ║
# ║    ./run_step1.sh diagnose         → cek dependensi             ║
# ║                                                                  ║
# ║  CATATAN: Script ini HANYA untuk Step 1.                        ║
# ║           Untuk full RTABMap pipeline → pakai run.sh            ║
# ╚══════════════════════════════════════════════════════════════════╝

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Default arguments ─────────────────────────────────────────────
BAG_PATH="/root/ws/rosbag2_2026_04_10-03_17_38"
MODEL_PATH="$SCRIPT_DIR/models/yolov8n_coco.pt"
RATE="${1:-0.5}"
DEVICE="${2:-cpu}"
CONFIDENCE="${3:-0.5}"

# ── Banner ────────────────────────────────────────────────────────
print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   STEP 1 — YOLO Standalone Detection     ║"
    echo "  ║   YOLOv8n COCO | D455 Feed               ║"
    echo "  ║   Tanpa RTABMap                           ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ── Diagnose: cek semua dependensi ───────────────────────────────
diagnose() {
    echo -e "\n${CYAN}${BOLD}Cek dependensi Step 1...${NC}\n"

    # ROS packages
    echo -e "${CYAN}ROS packages:${NC}"
    for pkg in topic_tools vision_msgs rviz2; do
        ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg — install: sudo apt install ros-humble-$(echo $pkg | tr '_' '-')${NC}"
    done

    # Python packages
    echo -e "\n${CYAN}Python packages:${NC}"
    for pkg in ultralytics cv2 numpy; do
        python3 -c "import $pkg" 2>/dev/null && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg — install: pip install $pkg${NC}"
    done

    # Model file
    echo -e "\n${CYAN}Model file:${NC}"
    [ -f "$MODEL_PATH" ] && \
        echo -e "  ${GREEN}✓ $MODEL_PATH${NC}" || \
        echo -e "  ${RED}✗ Model tidak ditemukan: $MODEL_PATH${NC}"
    echo -e "  ${YELLOW}→ Taruh file yolov8n_coco.pt di: $SCRIPT_DIR/models/${NC}"

    # Rosbag
    echo -e "\n${CYAN}Rosbag:${NC}"
    [ -d "$BAG_PATH" ] && \
        echo -e "  ${GREEN}✓ $BAG_PATH${NC}" || \
        echo -e "  ${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}"

    echo ""
}

# ── Main launch ───────────────────────────────────────────────────
launch() {
    print_banner

    echo -e "${CYAN}Konfigurasi:${NC}"
    echo -e "  Bag        : $BAG_PATH"
    echo -e "  Model      : $MODEL_PATH"
    echo -e "  Rate       : ${RATE}x"
    echo -e "  Device     : $DEVICE"
    echo -e "  Confidence : $CONFIDENCE"
    echo ""

    # Validasi bag
    [ ! -d "$BAG_PATH" ] && \
        echo -e "${RED}✗ Rosbag tidak ditemukan: $BAG_PATH${NC}" && exit 1

    # Validasi model
    [ ! -f "$MODEL_PATH" ] && \
        echo -e "${RED}✗ Model tidak ditemukan: $MODEL_PATH${NC}" && \
        echo -e "${YELLOW}  Taruh yolov8n_coco.pt di: $SCRIPT_DIR/models/${NC}" && exit 1

    # Source ROS
    [ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash
    [ -f "/root/ws/install/setup.bash" ] && source /root/ws/install/setup.bash

    echo -e "${GREEN}✓ Semua siap — launching Step 1...${NC}\n"
    echo -e "${YELLOW}  Di RViz: Add → Image → topic: /semantic/image_annotated${NC}\n"

    # Launch
    ros2 launch "$SCRIPT_DIR/launch/step1_yolo_detection.launch.py" \
        bag_path:="$BAG_PATH" \
        rate:="$RATE" \
        model_path:="$MODEL_PATH" \
        device:="$DEVICE" \
        confidence:="$CONFIDENCE" \
        rviz:=true
}

# ── Entry point ───────────────────────────────────────────────────
case "$1" in
    diagnose) source /opt/ros/humble/setup.bash 2>/dev/null; diagnose ;;
    help|-h)
        echo ""
        echo -e "${CYAN}Usage: ./run_step1.sh [rate] [device] [confidence]${NC}"
        echo ""
        echo "  ./run_step1.sh                 default: rate 0.5, cpu, conf 0.5"
        echo "  ./run_step1.sh 0.3             rate 0.3x"
        echo "  ./run_step1.sh 0.5 cuda        pakai GPU"
        echo "  ./run_step1.sh 0.5 cpu 0.4    confidence threshold 0.4"
        echo "  ./run_step1.sh diagnose        cek semua dependensi"
        echo ""
        ;;
    *) launch ;;
esac

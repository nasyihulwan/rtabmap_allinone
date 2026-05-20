#!/bin/bash

# ╔══════════════════════════════════════════════════════════════════╗
# ║  run_semantic.sh — Semantic Nodes (Terminal 2)                   ║
# ║                                                                  ║
# ║  PENTING: Jalankan ini SETELAH run.sh sudah jalan               ║
# ║  di Terminal 1 dan RTABMap sudah stabil                         ║
# ║                                                                  ║
# ║  Usage:                                                          ║
# ║    ./run_semantic.sh           → default (cpu, conf 0.5)         ║
# ║    ./run_semantic.sh cuda      → pakai GPU                       ║
# ║    ./run_semantic.sh cpu 0.4   → confidence threshold 0.4        ║
# ╚══════════════════════════════════════════════════════════════════╝

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODEL_PATH="$SCRIPT_DIR/models/yolov8n-seg_coco.pt"
DEVICE="${1:-cpu}"
CONFIDENCE="${2:-0.5}"

print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   Semantic Nodes — Terminal 2             ║"
    echo "  ║   YOLO + Projector + Grid                 ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_banner

# ── Validasi ──────────────────────────────────────────────────────
[ ! -f "$MODEL_PATH" ] && \
    echo -e "${RED}✗ Model tidak ditemukan: $MODEL_PATH${NC}" && exit 1

[ ! -f "$SCRIPT_DIR/semantic/yolo_detector.py" ] && \
    echo -e "${RED}✗ yolo_detector.py tidak ditemukan${NC}" && exit 1

[ ! -f "$SCRIPT_DIR/semantic/semantic_projector.py" ] && \
    echo -e "${RED}✗ semantic_projector.py tidak ditemukan${NC}" && exit 1

[ ! -f "$SCRIPT_DIR/semantic/semantic_grid_node.py" ] && \
    echo -e "${RED}✗ semantic_grid_node.py tidak ditemukan${NC}" && exit 1

# ── Setup FastDDS — harus sama dengan Terminal 1 ──────────────────
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/config/fastdds_rtabmap.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_STATISTICS=0

# ── Source ROS ────────────────────────────────────────────────────
[ -z "$ROS_DISTRO" ]                    && source /opt/ros/humble/setup.bash
[ -f "/root/ws/install/setup.bash" ]     && source /root/ws/install/setup.bash

echo -e "${CYAN}Konfigurasi:${NC}"
echo -e "  Model      : $MODEL_PATH"
echo -e "  Device     : $DEVICE"
echo -e "  Confidence : $CONFIDENCE"
echo ""
echo -e "${YELLOW}  Pastikan Terminal 1 (run.sh) sudah jalan dan RTABMap stabil${NC}"
echo -e "${YELLOW}  Di RViz Terminal 1, tambahkan:${NC}"
echo -e "${YELLOW}    Map        → /semantic/grid_map${NC}"
echo -e "${YELLOW}    PointCloud2 → /semantic/grid_colored${NC}"
echo ""
echo -e "${GREEN}✓ Launching semantic nodes...${NC}\n"

ros2 launch "$SCRIPT_DIR/launch/semantic_only.launch.py" \
    model_path:="$MODEL_PATH" \
    device:="$DEVICE" \
    confidence:="$CONFIDENCE"

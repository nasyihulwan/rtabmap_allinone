#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║  run_step3.sh — Full Semantic Mapping                            ║
# ║                                                                  ║
# ║  RTABMap + YOLO + Semantic Projector + Semantic Grid            ║
# ║                                                                  ║
# ║  Usage:                                                          ║
# ║    ./run_step3.sh                        → default              ║
# ║    ./run_step3.sh 0.3                    → rate 0.3x            ║
# ║    ./run_step3.sh 0.5 mapping false /odom → tanpa EKF          ║
# ║    ./run_step3.sh diagnose               → cek dependensi       ║
# ║                                                                  ║
# ║  CATATAN:                                                        ║
# ║    - RTABMap database disimpan terpisah di rtabmap_semantic.db  ║
# ║      agar tidak menimpa hasil mapping sebelumnya                ║
# ║    - Untuk full RTABMap saja → pakai run.sh                    ║
# ╚══════════════════════════════════════════════════════════════════╝

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Default arguments ─────────────────────────────────────────────
BAG_PATH="/root/ws/rosbag2_2026_04_10-03_17_38"
MODEL_PATH="$SCRIPT_DIR/models/yolov8n_coco.pt"
DB_PATH="/root/.ros/rtabmap_semantic.db"
RATE="${1:-0.5}"
MODE="${2:-mapping}"
USE_EKF="${3:-true}"
ODOM_TOPIC="${4:-/odometry/filtered}"

# ── Banner ────────────────────────────────────────────────────────
print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   STEP 3 — Full Semantic Mapping          ║"
    echo "  ║   RTABMap + YOLO + Semantic Grid          ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ── Diagnose ──────────────────────────────────────────────────────
diagnose() {
    echo -e "\n${CYAN}${BOLD}Cek dependensi Step 3...${NC}\n"

    echo -e "${CYAN}ROS packages:${NC}"
    for pkg in rtabmap_slam rtabmap_viz topic_tools vision_msgs \
               robot_localization rviz2; do
        ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg${NC}"
    done

    echo -e "\n${CYAN}Python packages:${NC}"
    for pkg in ultralytics cv2 numpy message_filters; do
        python3 -c "import $pkg" 2>/dev/null && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg${NC}"
    done

    echo -e "\n${CYAN}Semantic files:${NC}"
    for f in \
        "$SCRIPT_DIR/semantic/yolo_detector.py" \
        "$SCRIPT_DIR/semantic/semantic_projector.py" \
        "$SCRIPT_DIR/semantic/semantic_grid_node.py" \
        "$SCRIPT_DIR/decompress_depth_fix.py" \
        "$MODEL_PATH"; do
        [ -f "$f" ] && \
            echo -e "  ${GREEN}✓ $(basename $f)${NC}" || \
            echo -e "  ${RED}✗ $f${NC}"
    done

    echo -e "\n${CYAN}Rosbag:${NC}"
    [ -d "$BAG_PATH" ] && \
        echo -e "  ${GREEN}✓ $BAG_PATH${NC}" || \
        echo -e "  ${RED}✗ $BAG_PATH${NC}"

    echo -e "\n${CYAN}Database:${NC}"
    echo -e "  ${YELLOW}→ Akan disimpan di: $DB_PATH${NC}"
    [ -f "$DB_PATH" ] && \
        echo -e "  ${YELLOW}  (file sudah ada, akan dilanjutkan dari sesi sebelumnya)${NC}" || \
        echo -e "  ${CYAN}  (file baru akan dibuat)${NC}"
    echo ""
}

# ── Setup FastDDS + OS buffer (sama persis dengan run.sh) ─────────
setup() {
    echo -e "${YELLOW}→ Setup OS network buffer + FastDDS...${NC}"
    sudo sysctl -w net.core.rmem_max=33554432 2>/dev/null || true
    sudo sysctl -w net.core.rmem_default=33554432 2>/dev/null || true
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/config/fastdds_rtabmap.xml"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_STATISTICS=0
    echo -e "${GREEN}✓ FastDDS setup selesai${NC}"
}

# ── Main launch ───────────────────────────────────────────────────
launch() {
    print_banner

    echo -e "${CYAN}Konfigurasi:${NC}"
    echo -e "  Bag        : $BAG_PATH"
    echo -e "  Model      : $MODEL_PATH"
    echo -e "  Database   : $DB_PATH"
    echo -e "  Rate       : ${RATE}x"
    echo -e "  Mode       : $MODE"
    echo -e "  EKF        : $USE_EKF"
    echo -e "  Odom topic : $ODOM_TOPIC"
    echo ""

    [ ! -d "$BAG_PATH" ] && \
        echo -e "${RED}✗ Rosbag tidak ditemukan${NC}" && exit 1
    [ ! -f "$MODEL_PATH" ] && \
        echo -e "${RED}✗ Model tidak ditemukan: $MODEL_PATH${NC}" && exit 1
    [ ! -f "$SCRIPT_DIR/semantic/semantic_grid_node.py" ] && \
        echo -e "${RED}✗ semantic_grid_node.py tidak ditemukan${NC}" && exit 1

    if [ "$USE_EKF" = "true" ]; then
        ros2 pkg list 2>/dev/null | grep -q "^robot_localization$" || {
            echo -e "${RED}✗ robot_localization tidak ada${NC}"
            echo -e "${YELLOW}  Jalankan: ./run_step3.sh $RATE $MODE false /odom${NC}"
            exit 1
        }
    fi

    [ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash
    [ -f "/root/ws/install/setup.bash" ] && source /root/ws/install/setup.bash

    # Setup FastDDS — penting untuk stabilitas RTABMap
    setup

    echo -e "${GREEN}✓ Semua siap — launching Step 3...${NC}\n"
    echo -e "${YELLOW}  Di RViz:${NC}"
    echo -e "${YELLOW}    Map        → /rtabmap/grid_map    (RTABMap biasa)${NC}"
    echo -e "${YELLOW}    Map        → /semantic/grid_map   (Semantic labels)${NC}"
    echo -e "${YELLOW}    PointCloud2 → /semantic/grid_colored (Warna per class)${NC}\n"

    ros2 launch "$SCRIPT_DIR/launch/step3_semantic_mapping.launch.py" \
        bag_path:="$BAG_PATH" \
        rate:="$RATE" \
        db_path:="$DB_PATH" \
        use_ekf:="$USE_EKF" \
        odom_topic:="$ODOM_TOPIC" \
        model_path:="$MODEL_PATH" \
        localization:="$([ "$MODE" = "localize" ] && echo true || echo false)" \
        rviz:=true
}

# ── Entry point ───────────────────────────────────────────────────
case "$1" in
    diagnose) source /opt/ros/humble/setup.bash 2>/dev/null; diagnose ;;
    help|-h)
        echo ""
        echo -e "${CYAN}Usage: ./run_step3.sh [rate] [mode] [use_ekf] [odom_topic]${NC}"
        echo ""
        echo "  ./run_step3.sh                          default"
        echo "  ./run_step3.sh 0.3                      rate 0.3x"
        echo "  ./run_step3.sh 0.5 mapping false /odom  tanpa EKF"
        echo "  ./run_step3.sh diagnose                 cek dependensi"
        echo ""
        ;;
    *) launch ;;
esac

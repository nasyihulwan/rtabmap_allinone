#!/bin/bash

# ═══════════════════════════════════════════════════════════════
# RTAB-Map All-in-One Run Script — Modular
#
# Usage:
#   ./run.sh [rate] [mode] [use_ekf] [odom_topic]
#
# Contoh:
#   ./run.sh                    → rate 0.5, bag baru, odom filtered dari bag
#   ./run.sh 0.3                → rate 0.3
#   ./run.sh 0.5 localize       → localization mode
#   ./run.sh setup              → setup OS buffer
#   ./run.sh diagnose           → cek sistem
#
# CATATAN BAG BARU (2026-04-15):
#   /a200_1060/platform/odom/filtered sudah ada di bag → use_ekf=false
#   Image sudah compressed → decompress node otomatis dijalankan
# ═══════════════════════════════════════════════════════════════

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── [UBAH DI SINI] Path bag baru ─────────────────────────────────
BAG_PATH="/root/ws/project/rosbag_launch/bags/rosbag2_2026_04_15-08_39_23"

# ── Default: bag baru sudah ada odom/filtered, EKF tidak perlu live
RATE="${1:-0.5}"
MODE="${2:-mapping}"
USE_EKF="${3:-false}"
ODOM_TOPIC="${4:-/a200_1060/platform/odom/filtered}"

print_banner() {
    echo -e "${BLUE}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   RTAB-Map All-in-One — Modular          ║"
    echo "  ║   VLP-32C + D455 | Rosbag Replay         ║"
    echo "  ║   Bag: 2026-04-15 (dengan EKF filtered)  ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

setup() {
    echo -e "${YELLOW}→ Setup OS network buffer...${NC}"
    sudo sysctl -w net.core.rmem_max=33554432    2>/dev/null || true
    sudo sysctl -w net.core.rmem_default=33554432 2>/dev/null || true
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/config/fastdds_rtabmap.xml"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_STATISTICS=0
    echo -e "${GREEN}✓ Setup selesai${NC}"
}

diagnose() {
    echo -e "\n${CYAN}Cek package...${NC}"
    for pkg in rtabmap_slam rtabmap_odom rtabmap_viz topic_tools \
               image_transport image_transport_plugins robot_localization; do
        ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
            echo -e "  ${GREEN}✓ $pkg${NC}" || \
            echo -e "  ${RED}✗ $pkg — install: sudo apt install ros-humble-$(echo $pkg | tr '_' '-')${NC}"
    done

    echo -e "\n${CYAN}Cek rosbag...${NC}"
    [ -d "$BAG_PATH" ] && \
        echo -e "  ${GREEN}✓ Bag: $BAG_PATH${NC}" || \
        echo -e "  ${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}"

    echo -e "\n${CYAN}Cek topic di bag...${NC}"
    if [ -d "$BAG_PATH" ]; then
        ros2 bag info "$BAG_PATH" 2>/dev/null | grep "Topic:" | while read line; do
            echo -e "  ${CYAN}$line${NC}"
        done
    fi
}

launch_all() {
    print_banner

    echo -e "${CYAN}Konfigurasi:${NC}"
    echo -e "  Bag       : $BAG_PATH"
    echo -e "  Rate      : ${RATE}x"
    echo -e "  Mode      : $MODE"
    echo -e "  EKF       : $USE_EKF"
    echo -e "  Odom      : $ODOM_TOPIC"
    echo ""

    [ ! -d "$BAG_PATH" ] && \
        echo -e "${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}" && exit 1

    setup

    [ -z "$ROS_DISTRO" ]                       && source /opt/ros/humble/setup.bash
    [ -f "/root/ws/install/setup.bash" ]        && source /root/ws/install/setup.bash

    if [ "$USE_EKF" = "true" ]; then
        if ! ros2 pkg list 2>/dev/null | grep -q "^robot_localization$"; then
            echo -e "${RED}✗ robot_localization tidak ada!${NC}"
            echo -e "${YELLOW}  Install: sudo apt install ros-humble-robot-localization${NC}"
            exit 1
        fi
        echo -e "${GREEN}✓ EKF aktif — fuse wheel odom + IMU${NC}\n"
    else
        echo -e "${CYAN}→ EKF nonaktif — pakai $ODOM_TOPIC langsung dari bag${NC}\n"
    fi

    cd "$SCRIPT_DIR/launch"

    ros2 launch rtabmap_rosbag.launch.py \
        bag_path:="$BAG_PATH" \
        rate:="$RATE" \
        localization:="$([ "$MODE" = "localize" ] && echo true || echo false)" \
        use_ekf:="$USE_EKF" \
        odom_topic:="$ODOM_TOPIC" \
        rviz:=true
}

case "$1" in
    setup)    setup ;;
    diagnose) source /opt/ros/humble/setup.bash 2>/dev/null; diagnose ;;
    help|-h)
        echo ""
        echo -e "${CYAN}Usage: ./run.sh [rate] [mode] [use_ekf] [odom_topic]${NC}"
        echo ""
        echo "  # Bag baru 2026-04-15 (default — odom/filtered sudah di bag)"
        echo "  ./run.sh"
        echo "  ./run.sh 0.3"
        echo ""
        echo "  # Localization mode (setelah map sudah ada)"
        echo "  ./run.sh 0.5 localize"
        echo ""
        echo "  # Bag lama (EKF dijalankan live)"
        echo "  ./run.sh 0.5 mapping true /odometry/filtered"
        echo ""
        echo "  ./run.sh setup     Setup OS buffer"
        echo "  ./run.sh diagnose  Cek package dan bag"
        echo ""
        ;;
    *)        launch_all ;;
esac

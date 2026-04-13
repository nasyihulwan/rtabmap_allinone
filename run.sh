#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  RTAB-Map All-in-One Run Script — Modular
#
#  Usage:
#    ./run.sh [rate] [mode] [use_ekf] [odom_topic]
#
#  Contoh:
#    ./run.sh                          → rate 0.5, mapping, EKF aktif
#    ./run.sh 0.3                      → rate 0.3, EKF aktif
#    ./run.sh 0.5 mapping false /odom  → tanpa EKF, pakai /odom langsung
#    ./run.sh 0.5 mapping false /odometry/filtered  → data baru ada filtered odom
#    ./run.sh setup                    → setup OS buffer
#    ./run.sh diagnose                 → cek sistem
# ═══════════════════════════════════════════════════════════════

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH="/root/ws/rosbag2_2026_04_10-03_17_38"
RATE="${1:-0.5}"
MODE="${2:-mapping}"
USE_EKF="${3:-true}"
ODOM_TOPIC="${4:-/odometry/filtered}"

print_banner() {
  echo -e "${BLUE}"
  echo "  ╔══════════════════════════════════════════╗"
  echo "  ║   RTAB-Map All-in-One — Modular          ║"
  echo "  ║   VLP-32C + D455 | Rosbag Replay         ║"
  echo "  ╚══════════════════════════════════════════╝"
  echo -e "${NC}"
}

setup() {
  echo -e "${YELLOW}→ Setup OS network buffer...${NC}"
  sudo sysctl -w net.core.rmem_max=33554432 2>/dev/null || true
  sudo sysctl -w net.core.rmem_default=33554432 2>/dev/null || true
  export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/config/fastdds_rtabmap.xml"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_STATISTICS=0
  echo -e "${GREEN}✓ Setup selesai${NC}"
}

diagnose() {
  echo -e "\n${CYAN}Cek package...${NC}"
  for pkg in rtabmap_slam rtabmap_odom rtabmap_viz topic_tools \
             image_transport robot_localization; do
    ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
      echo -e "  ${GREEN}✓ $pkg${NC}" || \
      echo -e "  ${RED}✗ $pkg — install: sudo apt install ros-humble-$(echo $pkg | tr '_' '-')${NC}"
  done

  echo -e "\n${CYAN}Cek rosbag...${NC}"
  [ -d "$BAG_PATH" ] && \
    echo -e "  ${GREEN}✓ Bag: $BAG_PATH${NC}" || \
    echo -e "  ${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}"
}

launch_all() {
  print_banner

  echo -e "${CYAN}Konfigurasi:${NC}"
  echo -e "  Bag      : $BAG_PATH"
  echo -e "  Rate     : ${RATE}x"
  echo -e "  Mode     : $MODE"
  echo -e "  EKF      : $USE_EKF"
  echo -e "  Odom     : $ODOM_TOPIC"
  echo ""

  [ ! -d "$BAG_PATH" ] && \
    echo -e "${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}" && exit 1

  setup

  [ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash
  [ -f "/root/ws/install/setup.bash" ] && source /root/ws/install/setup.bash

  # Cek robot_localization kalau use_ekf=true
  if [ "$USE_EKF" = "true" ]; then
    if ! ros2 pkg list 2>/dev/null | grep -q "^robot_localization$"; then
      echo -e "${RED}✗ robot_localization tidak ada!${NC}"
      echo -e "${YELLOW}  Install: sudo apt install ros-humble-robot-localization${NC}"
      echo -e "${YELLOW}  Atau jalankan tanpa EKF: ./run.sh $RATE $MODE false /odom${NC}"
      exit 1
    fi
    echo -e "${GREEN}✓ EKF aktif — fuse wheel odom + IMU${NC}\n"
  else
    echo -e "${CYAN}→ EKF nonaktif — pakai $ODOM_TOPIC langsung${NC}\n"
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
    echo "  # Data lama (wheel odom + IMU raw) — EKF aktif"
    echo "  ./run.sh 0.5 mapping true /odometry/filtered"
    echo ""
    echo "  # Data baru dengan /odometry/filtered di bag"
    echo "  ./run.sh 0.5 mapping false /odometry/filtered"
    echo ""
    echo "  # Data baru wheel odom saja tanpa EKF"
    echo "  ./run.sh 0.5 mapping false /odom"
    echo ""
    echo "  ./run.sh setup     Setup OS buffer"
    echo "  ./run.sh diagnose  Cek package dan bag"
    echo ""
    ;;
  *) launch_all ;;
esac
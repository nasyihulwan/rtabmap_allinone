#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  ALL-IN-ONE Run Script
#  Velodyne VLP-32C + RealSense D455 + RTAB-Map
#  Rosbag: rosbag2_2026_04_10-03_17_38
#
#  Cara pakai:
#    ./run.sh              → mapping mode, rate 0.5x (default)
#    ./run.sh 0.3          → mapping mode, rate 0.3x (lebih pelan)
#    ./run.sh 1.0          → mapping mode, rate 1.0x (real-time)
#    ./run.sh 0.5 localize → localization mode
#    ./run.sh diagnose     → cek sistem sebelum mulai
#    ./run.sh setup        → setup OS buffer (sekali saja)
# ═══════════════════════════════════════════════════════════════

set -e
CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH="/root/ws/rosbag2_2026_04_10-03_17_38"
RATE="${1:-0.5}"
MODE="${2:-mapping}"

print_banner() {
  echo -e "${BLUE}"
  echo "  ╔══════════════════════════════════════════╗"
  echo "  ║   RTAB-Map All-in-One                    ║"
  echo "  ║   VLP-32C + D455 | Rosbag Replay         ║"
  echo "  ╚══════════════════════════════════════════╝"
  echo -e "${NC}"
}

# ─────────────────────────────────────────
# SETUP: OS buffer (jalankan sekali)
# ─────────────────────────────────────────
setup() {
  echo -e "${YELLOW}→ Setup OS network buffer...${NC}"
  sudo sysctl -w net.core.rmem_max=33554432 2>/dev/null || true
  sudo sysctl -w net.core.rmem_default=33554432 2>/dev/null || true
  sudo sysctl -w net.core.wmem_max=33554432 2>/dev/null || true

  export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/config/fastdds_rtabmap.xml"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_STATISTICS=0

  echo -e "${GREEN}✓ OS buffer: 32MB${NC}"
  echo -e "${GREEN}✓ FastDDS profile: $FASTRTPS_DEFAULT_PROFILES_FILE${NC}"
}

# ─────────────────────────────────────────
# DIAGNOSE: Cek sistem sebelum mulai
# ─────────────────────────────────────────
diagnose() {
  echo -e "\n${CYAN}[1/4] Cek ROS2${NC}"
  echo "  ROS_DISTRO : ${ROS_DISTRO:-TIDAK DISET}"
  echo "  RMW        : ${RMW_IMPLEMENTATION:-default}"

  echo -e "\n${CYAN}[2/4] Cek package wajib${NC}"
  for pkg in rtabmap_slam rtabmap_odom rtabmap_viz rtabmap_util topic_tools image_transport; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
      echo -e "  ${GREEN}✓ $pkg${NC}"
    else
      echo -e "  ${RED}✗ $pkg — BELUM TERINSTALL${NC}"
      echo -e "    ${YELLOW}Install: sudo apt install ros-${ROS_DISTRO}-$(echo $pkg | tr '_' '-')${NC}"
    fi
  done

  echo -e "\n${CYAN}[3/4] Cek rosbag${NC}"
  if [ -d "$BAG_PATH" ]; then
    echo -e "  ${GREEN}✓ Bag ditemukan: $BAG_PATH${NC}"
    ros2 bag info "$BAG_PATH" 2>/dev/null | grep -E "Duration|Messages|Bag size" | \
      sed 's/^/  /'
  else
    echo -e "  ${RED}✗ Bag tidak ditemukan: $BAG_PATH${NC}"
  fi

  echo -e "\n${CYAN}[4/4] Cek disk write speed${NC}"
  speed=$(dd if=/dev/zero of=/tmp/disktest bs=1M count=200 conv=fdatasync 2>&1 | \
    grep -oP '[0-9.]+ [MG]B/s' | tail -1)
  rm -f /tmp/disktest
  echo -e "  Disk write: ${GREEN}${speed:-unknown}${NC}"

  echo -e "\n${GREEN}✓ Diagnosis selesai${NC}\n"
}

# ─────────────────────────────────────────
# MAIN: Launch semua
# ─────────────────────────────────────────
launch_all() {
  print_banner

  echo -e "${CYAN}Konfigurasi:${NC}"
  echo -e "  Bag  : $BAG_PATH"
  echo -e "  Rate : ${RATE}x"
  echo -e "  Mode : $MODE"
  echo ""

  # Cek bag ada
  if [ ! -d "$BAG_PATH" ]; then
    echo -e "${RED}✗ Rosbag tidak ditemukan: $BAG_PATH${NC}"
    echo -e "${YELLOW}  Edit variabel BAG_PATH di run.sh${NC}"
    exit 1
  fi

  # Setup environment
  setup

  # Source ROS2 jika belum
  if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}→ Source ROS2 Humble...${NC}"
    source /opt/ros/humble/setup.bash
  fi

  # Source workspace jika ada
  if [ -f "/root/ws/install/setup.bash" ]; then
    source /root/ws/install/setup.bash
    echo -e "${GREEN}✓ Workspace sourced${NC}"
  fi

  echo -e "\n${YELLOW}→ Meluncurkan semua node...${NC}"
  echo -e "${CYAN}  Urutan: rosbag → relay/decompress (2s) → ICP odom (4s) → RTAB-Map (6s) → RViz (7s)${NC}\n"

  # Launch all-in-one
  # cd ke direktori launch agar ROS2 tidak bingung dengan path
  cd "$SCRIPT_DIR/launch"
  ros2 launch rtabmap_rosbag.launch.py \
    bag_path:="$BAG_PATH" \
    rate:="$RATE" \
    localization:="$([ "$MODE" = "localize" ] && echo true || echo false)" \
    rviz:=true
}

# ─────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────
case "$1" in
  setup)
    setup
    echo -e "${GREEN}✓ Setup selesai${NC}"
    ;;
  diagnose)
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    diagnose
    ;;
  help|-h|--help)
    echo ""
    echo -e "${CYAN}Usage: ./run.sh [rate] [mode]${NC}"
    echo ""
    echo "  ./run.sh              Mapping, rate 0.5x (recommended untuk debug)"
    echo "  ./run.sh 0.3          Mapping, rate 0.3x (paling pelan)"
    echo "  ./run.sh 1.0          Mapping, rate 1.0x (real-time)"
    echo "  ./run.sh 0.5 localize Localization mode"
    echo "  ./run.sh setup        Setup OS buffer (sekali saja, butuh sudo)"
    echo "  ./run.sh diagnose     Cek package dan rosbag"
    echo ""
    ;;
  *)
    launch_all
    ;;
esac

#!/usr/bin/env python3
"""
sanity_check.py — Verifikasi semua prasyarat sebelum jalankan semantic nodes.

Jalankan di Terminal BARU setelah run.sh sudah stabil:
    python3 sanity_check.py

Exit code:
    0 → semua OK, siap jalankan run_semantic.sh
    1 → ada yang bermasalah
"""

import subprocess
import sys
import time

GREEN  = '\033[0;32m'
RED    = '\033[0;31m'
YELLOW = '\033[1;33m'
CYAN   = '\033[0;36m'
NC     = '\033[0m'

def get_active_topics():
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=5
        )
        return set(result.stdout.strip().split('\n'))
    except Exception:
        return set()

def get_topic_hz(topic, duration=2.0):
    """Ukur Hz sebuah topic selama `duration` detik."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'hz', topic, '--window', '10'],
            capture_output=True, text=True, timeout=duration + 1
        )
        for line in result.stdout.split('\n'):
            if 'average rate' in line:
                hz = float(line.split(':')[1].strip().split()[0])
                return hz
    except Exception:
        pass
    return 0.0

def check(label, condition, ok_msg='', fail_msg=''):
    if condition:
        print(f"  {GREEN}✓{NC} {label}" + (f" — {ok_msg}" if ok_msg else ""))
        return True
    else:
        print(f"  {RED}✗{NC} {label}" + (f" — {fail_msg}" if fail_msg else ""))
        return False

def main():
    print(f"\n{CYAN}══════════════════════════════════════════{NC}")
    print(f"{CYAN}  Sanity Check — Bag Baru 2026-04-15      {NC}")
    print(f"{CYAN}══════════════════════════════════════════{NC}\n")

    all_ok = True
    topics = get_active_topics()

    # ── 1. Cek ROS2 berjalan ─────────────────────────────────────────────
    print(f"{YELLOW}[1] ROS2 Environment{NC}")
    ok = check("ROS2 aktif", len(topics) > 0,
               fail_msg="Pastikan run.sh sudah dijalankan di Terminal 1")
    all_ok = all_ok and ok

    # ── 2. Cek topic hasil decompress ────────────────────────────────────
    print(f"\n{YELLOW}[2] Topic Decompress (dari run.sh){NC}")

    required_topics = {
        '/camera/color/image_raw':         'Color RGB (decompress_color)',
        '/camera/depth/image_rect_raw':    'Depth aligned (decompress_depth)',
        '/camera/color/camera_info':       'Camera info color (relay)',
        '/camera/depth/camera_info':       'Camera info depth (relay)',
    }

    for topic, desc in required_topics.items():
        ok = check(f"{topic}", topic in topics,
                   ok_msg=desc,
                   fail_msg=f"Topic {desc} belum tersedia")
        all_ok = all_ok and ok

    # ── 3. Cek odometry ──────────────────────────────────────────────────
    print(f"\n{YELLOW}[3] Odometry{NC}")
    odom_topics = [
        '/a200_1060/platform/odom/filtered',
        '/odom',
    ]
    for t in odom_topics:
        check(t, t in topics,
              ok_msg="tersedia",
              fail_msg="tidak tersedia (warning, bukan error)")

    # ── 4. Cek RTABMap output ────────────────────────────────────────────
    print(f"\n{YELLOW}[4] RTABMap Output{NC}")
    rtabmap_topics = {
        '/map':             'OccupancyGrid base map',
        '/rtabmap/odom':    'RTABMap odometry',
    }
    for topic, desc in rtabmap_topics.items():
        ok = check(topic, topic in topics,
                   ok_msg=desc,
                   fail_msg=f"RTABMap belum stabil atau belum tersedia")
        all_ok = all_ok and ok

    # ── 5. Cek topic bag masih mengalir ──────────────────────────────────
    print(f"\n{YELLOW}[5] Topic dari Bag (cek Hz sebentar...){NC}")
    hz_checks = [
        ('/camera/color/image_raw',      10.0, 'Color RGB'),
        ('/camera/depth/image_rect_raw', 10.0, 'Depth'),
        ('/velodyne_points',              5.0, 'LiDAR'),
    ]
    for topic, min_hz, desc in hz_checks:
        if topic not in topics:
            check(f"{topic} ({desc})", False, fail_msg="tidak ada di topic list")
            all_ok = False
            continue
        print(f"  {CYAN}→{NC} Mengukur Hz {topic}...", end='', flush=True)
        hz = get_topic_hz(topic, duration=2.5)
        ok = hz >= min_hz
        status = f"{hz:.1f} Hz" if hz > 0 else "0 Hz / timeout"
        check(f"{topic} ({desc})", ok,
              ok_msg=status,
              fail_msg=f"{status} — bag mungkin sudah selesai atau lambat")
        all_ok = all_ok and ok

    # ── Kesimpulan ────────────────────────────────────────────────────────
    print(f"\n{CYAN}══════════════════════════════════════════{NC}")
    if all_ok:
        print(f"{GREEN}  ✓ SEMUA OK — Siap jalankan run_semantic.sh{NC}")
        print(f"{GREEN}  Buka Terminal 2: ./run_semantic.sh{NC}")
    else:
        print(f"{RED}  ✗ ADA MASALAH — Periksa Terminal 1 dulu{NC}")
        print(f"{YELLOW}  Tips:{NC}")
        print(f"{YELLOW}    - Pastikan run.sh sudah jalan{NC}")
        print(f"{YELLOW}    - Tunggu RTABMap stabil (map terlihat di RViz){NC}")
        print(f"{YELLOW}    - Cek log Terminal 1 apakah ada error decompress{NC}")
    print(f"{CYAN}══════════════════════════════════════════{NC}\n")

    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())

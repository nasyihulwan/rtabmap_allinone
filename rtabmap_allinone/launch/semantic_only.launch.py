#!/usr/bin/env python3

# ═══════════════════════════════════════════════════════════════════════
# semantic_only.launch.py
#
# Semantic pipeline nodes (Terminal 2).
# Script Python dijalankan langsung via ExecuteProcess.
#
# PENTING: Script yolo_detector.py, semantic_projector.py, dan
# semantic_grid_node.py di-run as-is. Pastikan topik yang mereka
# subscribe sudah tersedia dari Terminal 1 (decompress nodes).
#
# Topik yang HARUS sudah tersedia dari Terminal 1:
#   /camera/color/image_raw       ← decompress_color
#   /camera/depth/image_rect_raw  ← decompress_depth
#   /camera/color/camera_info     ← relay_color_info
#   /map                          ← RTABMap
# ═══════════════════════════════════════════════════════════════════════

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    semantic_dir = os.path.join(script_dir, 'semantic')
    model_default = os.path.join(script_dir, 'models', 'yolov8n-seg_coco.pt')

    # ── Argumen (diteruskan ke script via env var / ROS param override) ──
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_default)
    device_arg     = DeclareLaunchArgument('device',     default_value='cpu')
    confidence_arg = DeclareLaunchArgument('confidence', default_value='0.5')

    model_path = LaunchConfiguration('model_path')
    device     = LaunchConfiguration('device')
    confidence = LaunchConfiguration('confidence')

    # ── Node 1: YOLO Detector ─────────────────────────────────────────────
    # Konfigurasi diteruskan via ROS2 parameter syntax (--ros-args -p)
    # agar kompatibel dengan script yang pakai rclpy + node.declare_parameter
    yolo_detector = ExecuteProcess(
        cmd=[
            'python3', os.path.join(semantic_dir, 'yolo_detector.py'),
            '--ros-args',
            '-p', ['model_path:=', model_path],
            '-p', ['device:=',     device],
            '-p', ['confidence:=', confidence],
        ],
        output='screen',
    )

    # ── Node 2: Semantic Projector (delay 2s) ────────────────────────────
    semantic_projector = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_projector.py'),
                    '--ros-args',
                ],
                output='screen',
            )
        ]
    )

    # ── Node 3: Semantic Grid (delay 3s) ─────────────────────────────────
    semantic_grid = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_grid_node.py'),
                    '--ros-args',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        confidence_arg,
        yolo_detector,
        semantic_projector,
        semantic_grid,
    ])

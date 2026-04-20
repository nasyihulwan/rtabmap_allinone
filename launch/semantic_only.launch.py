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
#
# Parameter Map-level IoU (semantic_grid_node):
#   map_iou_threshold  — minimum IoU untuk dianggap objek sama (default: 0.1)
#   min_cells_to_check — minimum cell sebelum instance dievaluasi (default: 20)
# ═══════════════════════════════════════════════════════════════════════

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    script_dir   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    semantic_dir = os.path.join(script_dir, 'semantic')
    model_default = os.path.join(script_dir, 'models', 'yolov8n-seg_coco.pt')

    # ── Argumen YOLO ──────────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_default)
    device_arg     = DeclareLaunchArgument('device',     default_value='cpu')
    confidence_arg = DeclareLaunchArgument('confidence', default_value='0.5')

    # ── Argumen Map-level IoU ─────────────────────────────────────────────
    iou_threshold_arg = DeclareLaunchArgument(
        'map_iou_threshold',
        default_value='0.1',
        description='Minimum IoU antar footprint cell untuk dianggap objek sama')
    min_cells_arg = DeclareLaunchArgument(
        'min_cells_to_check',
        default_value='20',
        description='Minimum cell yang harus terakumulasi sebelum instance dievaluasi')

    model_path    = LaunchConfiguration('model_path')
    device        = LaunchConfiguration('device')
    confidence    = LaunchConfiguration('confidence')
    iou_threshold = LaunchConfiguration('map_iou_threshold')
    min_cells     = LaunchConfiguration('min_cells_to_check')

    # ── Node 1: YOLO Detector ─────────────────────────────────────────────
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
                    '-p', ['map_iou_threshold:=', iou_threshold],
                    '-p', ['min_cells_to_check:=', min_cells],
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        confidence_arg,
        iou_threshold_arg,
        min_cells_arg,
        yolo_detector,
        semantic_projector,
        semantic_grid,
    ])
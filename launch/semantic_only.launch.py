"""
╔══════════════════════════════════════════════════════════════════╗
║  Semantic Only Launch — Segmentation + Instance Tracking         ║
║                                                                  ║
║  Jalankan SETELAH run.sh sudah stabil di Terminal 1.            ║
║                                                                  ║
║  Yang jalan:                                                     ║
║    - YOLO-seg detector   → /semantic/label_image                ║
║                             /semantic/instance_image            ║
║                             /semantic/image_annotated           ║
║    - Semantic Projector  → /semantic/pointcloud                 ║
║    - Semantic Grid       → /semantic/grid_map                   ║
║                             /semantic/grid_colored              ║
╚══════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    device     = LaunchConfiguration('device')

    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/root/ws/project/rtabmap_allinone/models/yolov8n-seg_coco.pt'
    )
    declare_confidence = DeclareLaunchArgument('confidence', default_value='0.5')
    declare_device     = DeclareLaunchArgument('device',     default_value='cpu')

    # ── YOLO-seg + Instance Tracker ───────────────────────────────
    yolo_detector = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/yolo_detector.py',
            '--ros-args',
            '-p', ['model_path:=', model_path],
            '-p', ['confidence:=', confidence],
            '-p', ['device:=',     device],
            '-p', 'process_every_n:=3',
            '-p', 'iou_threshold:=0.15',  # toleran untuk skip-frame
            '-p', 'max_track_age:=30',   # track bertahan lebih lama
            '-p', 'use_sim_time:=true',
        ],
        output='screen', name='yolo_detector'
    )

    # ── Semantic Projector ────────────────────────────────────────
    semantic_projector = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/semantic_projector.py',
            '--ros-args',
            '-p', 'max_depth:=8.0',
            '-p', 'min_depth:=0.2',
            '-p', 'sample_step:=4',
            '-p', 'sync_slop:=0.5',
            '-p', 'use_sim_time:=true',
        ],
        output='screen', name='semantic_projector'
    )

    # ── Semantic Grid ─────────────────────────────────────────────
    semantic_grid = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/semantic_grid_node.py',
            '--ros-args',
            '-p', 'map_frame:=map',
            '-p', 'resolution:=0.05',
            '-p', 'min_height:=0.1',
            '-p', 'max_height:=2.0',
            '-p', 'publish_rate:=1.0',
            '-p', 'decay_time:=0.0',
            '-p', 'use_sim_time:=true',
        ],
        output='screen', name='semantic_grid'
    )

    return LaunchDescription([
        declare_model_path, declare_confidence, declare_device,

        LogInfo(msg='════════════════════════════════════════════'),
        LogInfo(msg='  Semantic — Segmentation + Instance Tracking'),
        LogInfo(msg='  Pastikan run.sh sudah jalan di Terminal 1'),
        LogInfo(msg='════════════════════════════════════════════'),

        LogInfo(msg='>>> [T+0s] YOLO-seg + Tracker...'),
        yolo_detector,

        TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+2s] Semantic Projector...'),
            semantic_projector,
        ]),

        TimerAction(period=3.0, actions=[
            LogInfo(msg='>>> [T+3s] Semantic Grid...'),
            semantic_grid,
        ]),
    ])

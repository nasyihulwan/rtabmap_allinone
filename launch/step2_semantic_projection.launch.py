"""
╔══════════════════════════════════════════════════════════════════╗
║  STEP 2 — Semantic Projection (YOLO + Depth → 3D)               ║
║                                                                  ║
║  Tujuan  : Validasi bahwa label dari YOLO menempel di posisi    ║
║            3D yang benar menggunakan depth image D455.          ║
║            Belum ada integrasi ke RTABMap.                      ║
║                                                                  ║
║  Yang jalan di sini:                                            ║
║    - Rosbag play                                                 ║
║    - Relay TF                                                    ║
║    - Decompress color image  → /camera/color/image_raw          ║
║    - Decompress depth image  → /camera/depth/image_rect_raw     ║
║    - YOLO detector           → /semantic/detections             ║
║    - Semantic projector      → /semantic/pointcloud             ║
║    - RViz2                                                       ║
║                                                                  ║
║  Yang TIDAK jalan di sini:                                      ║
║    - RTABMap (step 3)                                           ║
║    - EKF (tidak diperlukan untuk validasi ini)                  ║
║                                                                  ║
║  Usage:                                                          ║
║    ./run_step2.sh                                               ║
║    ./run_step2.sh 0.3          → rate lebih lambat              ║
║                                                                  ║
║  Topic untuk RViz:                                              ║
║    /semantic/image_annotated  → Image (annotated RGB)           ║
║    /semantic/pointcloud       → PointCloud2 (labeled 3D)        ║
╚══════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Frame constants — sama dengan file launch lainnya ─────────────
ROBOT_BASE_FRAME = 'a200_1060/base_link'
ODOM_FRAME       = 'odom'


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────
    bag_path   = LaunchConfiguration('bag_path')
    rate       = LaunchConfiguration('rate')
    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    device     = LaunchConfiguration('device')
    max_depth  = LaunchConfiguration('max_depth')
    rviz       = LaunchConfiguration('rviz')

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ws/rosbag2_2026_04_10-03_17_38'
    )
    declare_rate = DeclareLaunchArgument(
        'rate',
        default_value='0.5'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/root/ws/project/rtabmap_allinone/models/yolov8n_coco.pt'
    )
    declare_confidence = DeclareLaunchArgument(
        'confidence',
        default_value='0.5'
    )
    declare_device = DeclareLaunchArgument(
        'device',
        default_value='cpu'
    )
    declare_max_depth = DeclareLaunchArgument(
        'max_depth',
        default_value='8.0',
        description='Batas depth maksimum dalam meter'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+0s — ROSBAG
    # ══════════════════════════════════════════════════════════════════
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', rate,
            '--read-ahead-queue-size', '1000',
        ],
        output='screen',
        name='rosbag_play'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+2s — RELAY TF
    # ══════════════════════════════════════════════════════════════════
    relay_tf = Node(
        package='topic_tools', executable='relay', name='relay_tf',
        arguments=['/a200_1060/tf', '/tf'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    relay_tf_static = Node(
        package='topic_tools', executable='relay', name='relay_tf_static',
        arguments=['/a200_1060/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    relay_camera_info = Node(
        package='topic_tools', executable='relay', name='relay_camera_info',
        arguments=[
            '/camera/camera/color/camera_info',
            '/camera/color/camera_info',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+2s — STATIC TF (sama persis dengan rtabmap_rosbag.launch.py)
    # Diperlukan agar RViz bisa render point cloud di frame yang benar
    # ══════════════════════════════════════════════════════════════════
    tf_base_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.2', '0', '0.25', '0', '0', '0',
                   'a200_1060/base_link', 'camera_link'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    tf_camera_color_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_color_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    tf_camera_depth_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_depth_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_depth_optical_frame'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    tf_bridge_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_bridge_base',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'a200_1060/base_link'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+2s — DECOMPRESS COLOR
    # Output: /camera/color/image_raw (rgb8)
    # ══════════════════════════════════════════════════════════════════
    decompress_color = ExecuteProcess(
        cmd=['python3', '-c', """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2, numpy as np

class ColorDecompress(Node):
    def __init__(self):
        super().__init__('decompress_color')
        self.count = 0
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.cb, 10)
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.get_logger().info('Color decompressor ready')

    def cb(self, msg):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                return
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            out = Image()
            out.header = msg.header
            out.height, out.width = img_rgb.shape[:2]
            out.encoding = 'rgb8'
            out.is_bigendian = 0
            out.step = int(out.width * 3)
            out.data = bytes(img_rgb)
            self.pub.publish(out)
            self.count += 1
            if self.count % 100 == 1:
                self.get_logger().info(f'Color: {self.count} frames')
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

rclpy.init()
try:
    rclpy.spin(ColorDecompress())
except KeyboardInterrupt:
    pass
"""],
        output='screen',
        name='decompress_color'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+2s — DECOMPRESS DEPTH
    # Output: /camera/depth/image_rect_raw (16UC1, mm)
    # ══════════════════════════════════════════════════════════════════
    decompress_depth = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/decompress_depth_fix.py'
        ],
        output='screen',
        name='decompress_depth'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+4s — YOLO DETECTOR
    # Subscribe: /camera/color/image_raw
    # Publish  : /semantic/detections, /semantic/image_annotated
    # ══════════════════════════════════════════════════════════════════
    yolo_detector = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/yolo_detector.py',
            '--ros-args',
            '-p', ['model_path:=', model_path],
            '-p', ['confidence:=', confidence],
            '-p', ['device:=',     device],
            '-p', 'verbose:=false',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        name='yolo_detector'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+5s — SEMANTIC PROJECTOR
    # Subscribe: /semantic/detections + /camera/depth/image_rect_raw
    # Publish  : /semantic/pointcloud
    # Dimulai setelah YOLO sudah warmup 1 detik
    # ══════════════════════════════════════════════════════════════════
    semantic_projector = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/semantic_projector.py',
            '--ros-args',
            '-p', ['max_depth:=',   max_depth],
            '-p', 'min_depth:=0.2',
            '-p', 'sample_step:=4',
            '-p', 'sync_slop:=0.5',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        name='semantic_projector'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+6s — RViz2
    # ══════════════════════════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen'
    )

    # ── Susun launch sequence ─────────────────────────────────────────
    return LaunchDescription([
        # Declarations
        declare_bag_path, declare_rate, declare_model_path,
        declare_confidence, declare_device, declare_max_depth, declare_rviz,

        LogInfo(msg='════════════════════════════════════════════'),
        LogInfo(msg='  STEP 2 — Semantic Projection'),
        LogInfo(msg='  YOLO + Depth → Semantic Point Cloud'),
        LogInfo(msg='  Belum ada RTABMap.'),
        LogInfo(msg='════════════════════════════════════════════'),

        # T+0s
        LogInfo(msg='>>> [T+0s] Rosbag play...'),
        rosbag_play,

        # T+2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+2s] Relay TF + static TF + decompress color + depth...'),
            relay_tf,
            relay_tf_static,
            relay_camera_info,
            tf_base_camera,
            tf_camera_color_optical,
            tf_camera_depth_optical,
            tf_bridge_base,
            decompress_color,
            decompress_depth,
        ]),

        # T+4s
        TimerAction(period=4.0, actions=[
            LogInfo(msg='>>> [T+4s] YOLO Detector...'),
            yolo_detector,
        ]),

        # T+5s
        TimerAction(period=5.0, actions=[
            LogInfo(msg='>>> [T+5s] Semantic Projector...'),
            semantic_projector,
        ]),

        # T+6s
        TimerAction(period=6.0, actions=[
            LogInfo(msg='>>> [T+6s] RViz2...'),
            LogInfo(msg='  Add PointCloud2 → topic: /semantic/pointcloud'),
            LogInfo(msg='  Add Image       → topic: /semantic/image_annotated'),
            rviz_node,
        ]),
    ])

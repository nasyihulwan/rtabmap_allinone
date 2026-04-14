"""
╔══════════════════════════════════════════════════════════════════╗
║  STEP 1 — YOLO Standalone Detection                              ║
║                                                                  ║
║  Tujuan  : Validasi YOLOv8n bisa detect objek dari feed D455.  ║
║            Tidak ada RTABMap, tidak ada EKF, tidak ada depth.  ║
║                                                                  ║
║  Yang jalan di sini:                                            ║
║    - Rosbag play (color only, TF, odom)                        ║
║    - Relay TF                                                   ║
║    - Decompress color image                                     ║
║    - YOLO detector node                                         ║
║    - RViz2                                                      ║
║                                                                  ║
║  Yang TIDAK jalan di sini:                                      ║
║    - RTABMap (step 3)                                           ║
║    - Depth decompressor (step 2)                                ║
║    - EKF (tidak diperlukan untuk validasi ini)                  ║
║                                                                  ║
║  Usage:                                                          ║
║    ./run_step1.sh                                               ║
║    ./run_step1.sh 0.3                    → rate 0.3x            ║
║    ./run_step1.sh 0.5 /path/to/model.pt → custom model path    ║
║                                                                  ║
║  Topic yang dipublish:                                          ║
║    /semantic/image_annotated  → lihat di RViz (Image)          ║
║    /semantic/detections       → data mentah (Detection2DArray) ║
╚══════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Frame constants — sama dengan rtabmap_rosbag.launch.py ────────
ROBOT_BASE_FRAME = 'a200_1060/base_link'
ODOM_FRAME       = 'odom'


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────
    bag_path   = LaunchConfiguration('bag_path')
    rate       = LaunchConfiguration('rate')
    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    device     = LaunchConfiguration('device')
    verbose    = LaunchConfiguration('verbose')
    rviz       = LaunchConfiguration('rviz')

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ws/rosbag2_2026_04_10-03_17_38',
        description='Path ke folder rosbag'
    )
    declare_rate = DeclareLaunchArgument(
        'rate',
        default_value='0.5',
        description='Kecepatan replay rosbag (0.1 - 1.0)'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/root/ws/project/rtabmap_allinone/models/yolov8n_coco.pt',
        description='Path ke file YOLOv8 .pt'
    )
    declare_confidence = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Confidence threshold YOLO (0.0 - 1.0)'
    )
    declare_device = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device inferensi: cpu atau cuda'
    )
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Print detail deteksi tiap frame ke terminal'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Buka RViz2'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+0s — ROSBAG
    # Hanya play topic yang diperlukan untuk Step 1:
    #   - Color image (compressed)
    #   - TF
    #   - Odom (tidak dipakai tapi tidak di-filter agar rosbag normal)
    # ══════════════════════════════════════════════════════════════════
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', rate,
            '--read-ahead-queue-size', '500',
        ],
        output='screen',
        name='rosbag_play'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+2s — RELAY TF
    # Sama persis dengan rtabmap_rosbag.launch.py
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
    # T+2s — DECOMPRESS COLOR
    # Sama persis dengan rtabmap_rosbag.launch.py
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
    # T+4s — YOLO DETECTOR NODE
    # Dijalankan langsung sebagai script Python (seperti decompress_color)
    # karena rtabmap_allinone bukan ROS2 package yang ter-install
    # ══════════════════════════════════════════════════════════════════
    yolo_detector = ExecuteProcess(
        cmd=[
            'python3',
            '/root/ws/project/rtabmap_allinone/semantic/yolo_detector.py',
            '--ros-args',
            '-p', ['model_path:=', model_path],
            '-p', ['confidence:=', confidence],
            '-p', ['device:=',     device],
            '-p', ['verbose:=',    verbose],
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        name='yolo_detector'
    )

    # ══════════════════════════════════════════════════════════════════
    # T+5s — RViz2
    # Buka setelah YOLO sudah mulai publish
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
        declare_confidence, declare_device, declare_verbose, declare_rviz,

        LogInfo(msg='════════════════════════════════════════════'),
        LogInfo(msg='  STEP 1 — YOLO Standalone Detection'),
        LogInfo(msg='  Tidak ada RTABMap di sini.'),
        LogInfo(msg='════════════════════════════════════════════'),

        # T+0s
        LogInfo(msg='>>> [T+0s] Rosbag play...'),
        rosbag_play,

        # T+2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+2s] Relay TF + decompress color...'),
            relay_tf,
            relay_tf_static,
            relay_camera_info,
            decompress_color,
        ]),

        # T+4s
        TimerAction(period=4.0, actions=[
            LogInfo(msg='>>> [T+4s] YOLO Detector...'),
            yolo_detector,
        ]),

        # T+5s
        TimerAction(period=5.0, actions=[
            LogInfo(msg='>>> [T+5s] RViz2...'),
            LogInfo(msg='  Tambahkan Image display → topic: /semantic/image_annotated'),
            rviz_node,
        ]),
    ])
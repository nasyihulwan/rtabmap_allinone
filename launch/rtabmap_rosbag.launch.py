"""
ALL-IN-ONE Launch File — Fixed v10 + bag baru 2026-04-15

Perubahan dari v10 original (bag lama):
  1. bag_path default → bag baru 2026-04-15
  2. use_ekf default → false  (odom/filtered sudah ada di bag baru)
  3. odom_topic default → /a200_1060/platform/odom/filtered
  4. decompress_depth → pakai decompress_depth_node.py (fix Humble compressedDepth bug)
     sekaligus relay camera_info depth + color di dalamnya
  5. relay_camera_info → hapus (sudah dihandle decompress_depth_node.py)

SEMUA YANG LAIN IDENTIK dengan v10 yang sudah terbukti jalan.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_BASE_FRAME = 'a200_1060/base_link'
ODOM_FRAME       = 'odom'
MAP_FRAME        = 'map'


def generate_launch_description():

    bag_path    = LaunchConfiguration('bag_path')
    rate        = LaunchConfiguration('rate')
    rviz        = LaunchConfiguration('rviz')
    db_path     = LaunchConfiguration('db_path')
    use_ekf     = LaunchConfiguration('use_ekf')
    odom_topic  = LaunchConfiguration('odom_topic')

    # ── Argumen — hanya 3 default yang berubah dari v10 ──────────────────
    declare_bag_path   = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ws/project/rosbag_launch/bags/rosbag2_2026_04_15-08_39_23')
    declare_rate       = DeclareLaunchArgument('rate',    default_value='0.5')
    declare_rviz       = DeclareLaunchArgument('rviz',    default_value='true')
    declare_db_path    = DeclareLaunchArgument('db_path', default_value='/root/.ros/rtabmap.db')
    declare_use_ekf    = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='false=pakai odom/filtered dari bag | true=EKF live')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/a200_1060/platform/odom/filtered',
        description='Topic odom ke RTAB-Map')

    # ═══════════════════════════════════════════
    # T+0s — ROSBAG
    # ═══════════════════════════════════════════
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '--rate', rate,
            '--read-ahead-queue-size', '1000',
        ],
        output='screen', name='rosbag_play')

    # ═══════════════════════════════════════════
    # T+2s — RELAY TF (penting! bag pakai namespace a200_1060)
    # ═══════════════════════════════════════════
    relay_tf = Node(
        package='topic_tools', executable='relay', name='relay_tf',
        arguments=['/a200_1060/tf', '/tf'],
        parameters=[{'use_sim_time': True}], output='screen')

    relay_tf_static = Node(
        package='topic_tools', executable='relay', name='relay_tf_static',
        arguments=['/a200_1060/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}], output='screen')

    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/a200_1060/platform/odom', '/odom'],
        parameters=[{'use_sim_time': True}], output='screen')

    relay_imu = Node(
        package='topic_tools', executable='relay', name='relay_imu',
        arguments=['/a200_1060/sensors/imu_0/data', '/imu/data'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_ekf),
        output='screen')

    # ═══════════════════════════════════════════
    # T+2s — STATIC TF (kamera ke base_link)
    # ═══════════════════════════════════════════
    tf_base_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.2', '0', '0.25', '0', '0', '0',
                   'a200_1060/base_link', 'camera_link'],
        parameters=[{'use_sim_time': True}], output='screen')

    tf_camera_color_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_color_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': True}], output='screen')

    tf_camera_depth_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_depth_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_depth_optical_frame'],
        parameters=[{'use_sim_time': True}], output='screen')

    tf_bridge_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_bridge_base',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'a200_1060/base_link'],
        parameters=[{'use_sim_time': True}], output='screen')

    # ═══════════════════════════════════════════
    # T+2s — IMAGE DECOMPRESS
    # ═══════════════════════════════════════════

    # Color: inline script (identik v10, terbukti jalan)
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
        self.get_logger().info('Color decompressor ready (fast bytes mode)')

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
        output='screen', name='decompress_color')

    # Depth: pakai node Python kustom (fix Humble compressedDepth bug)
    # decompress_depth_node.py juga sekaligus relay camera_info color + depth
    import os
    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    decompress_depth = ExecuteProcess(
        cmd=['python3',
             os.path.join(script_dir, 'decompress_depth_node.py')],
        output='screen', name='decompress_depth')

    # ═══════════════════════════════════════════
    # T+3s — EKF NODE (kondisional, false untuk bag baru)
    # ═══════════════════════════════════════════
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'use_sim_time':     True,
            'frequency':        50.0,
            'sensor_timeout':   0.1,
            'two_d_mode':       True,
            'publish_tf':       False,
            'map_frame':        MAP_FRAME,
            'odom_frame':       ODOM_FRAME,
            'base_link_frame':  ROBOT_BASE_FRAME,
            'world_frame':      ODOM_FRAME,
            'odom0': '/odom',
            'odom0_config': [
                True,  True,  False,
                False, False, True,
                True,  True,  False,
                False, False, True,
                False, False, False,
            ],
            'odom0_differential': False,
            'odom0_relative':     False,
            'odom0_queue_size':   10,
            'imu0': '/imu/data',
            'imu0_config': [
                False, False, False,
                False, False, True,
                False, False, False,
                True,  True,  True,
                True,  True,  False,
            ],
            'imu0_differential':  False,
            'imu0_relative':      False,
            'imu0_queue_size':    50,
            'imu0_remove_gravitational_acceleration': True,
        }])

    # ═══════════════════════════════════════════
    # T+6s — RTAB-MAP SLAM (identik v10)
    # ═══════════════════════════════════════════
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':         True,
            'database_path':        db_path,
            'frame_id':             ROBOT_BASE_FRAME,
            'map_frame_id':         MAP_FRAME,
            'odom_frame_id':        ODOM_FRAME,
            'subscribe_depth':      True,
            'subscribe_rgb':        True,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info':  False,
            'approx_sync':              True,
            'approx_sync_max_interval': 1.0,
            'sync_queue_size':          50,
            'topic_queue_size':         50,

            'Mem/STMSize':             '30',
            'Mem/RehearsalSimilarity': '0.45',
            'Mem/NotLinkedNodesKept':  'false',
            'Mem/IncrementalMemory':   'true',
            'Mem/InitWMWithAllNodes':  'false',

            'Kp/DetectorStrategy': '6',
            'Vis/FeatureType':     '6',
            'Kp/MaxFeatures':      '600',
            'Vis/MinInliers':      '10',
            'Vis/InlierDistance':  '0.1',
            'Vis/EstimationType':  '1',
            'Vis/MaxDepth':        '10.0',

            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/OptimizeMaxError':     '3.0',
            'Optimizer/Strategy':        '1',
            'Optimizer/Iterations':      '20',
            'Optimizer/Robust':          'true',

            'RGBD/ProximityBySpace':          'true',
            'RGBD/ProximityMaxGraphDepth':    '0',
            'RGBD/ProximityPathMaxNeighbors': '5',
            'RGBD/NeighborLinkRefining':      'true',
            'RGBD/AngularUpdate':             '0.01',
            'RGBD/LinearUpdate':              '0.01',

            'RGBD/CreateOccupancyGrid': 'true',
            'Grid/Sensor':              '2',
            'Grid/3D':                  'false',
            'Grid/MaxObstacleHeight':   '2.0',
            'Grid/MinGroundHeight':     '-0.1',
            'Grid/MaxGroundHeight':     '0.1',
            'Grid/RayTracing':          'true',
            'Grid/FootprintLength':     '0.8',
            'Grid/FootprintWidth':      '0.5',
            'Grid/FootprintHeight':     '0.5',
            'Grid/ClusterRadius':       '0.1',
            'Grid/GroundIsObstacle':    'false',
            'Grid/NormalsSegmentation': 'true',

            'Reg/Strategy':                  '2',
            'Reg/Force3DoF':                 'true',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '30',
            'Icp/VoxelSize':                 '0.1',
            'Icp/Epsilon':                   '0.001',
            'Icp/MaxTranslation':            '1.5',
            'Icp/MaxRotation':               '1',
            'Icp/OutlierRatio':              '0.1',

            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr':       '0',
            'Rtabmap/MemoryThr':     '0',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_rect_raw'),
            ('scan_cloud',      '/velodyne_points'),
            ('odom',            odom_topic),
            ('map',             '/map'),
            ('cloud_map',       '/rtabmap/cloud_map'),
            ('grid_map',        '/rtabmap/grid_map'),
        ])

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time':             True,
            'subscribe_depth':          True,
            'subscribe_rgb':            True,
            'subscribe_scan_cloud':     False,
            'subscribe_odom_info':      False,
            'approx_sync':              True,
            'approx_sync_max_interval': 2.0,
            'sync_queue_size':          50,
            'topic_queue_size':         50,
            'frame_id':                 ROBOT_BASE_FRAME,
            'odom_frame_id':            ODOM_FRAME,
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_rect_raw'),
            ('odom',            odom_topic),
        ])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription([
        declare_bag_path, declare_rate, declare_rviz,
        declare_db_path, declare_use_ekf, declare_odom_topic,

        LogInfo(msg='>>> [T+0s] Rosbag play...'),
        rosbag_play,

        TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+2s] Relay TF + Static TF + Decompress...'),
            relay_tf, relay_tf_static, relay_odom, relay_imu,
            tf_base_camera, tf_camera_color_optical,
            tf_camera_depth_optical, tf_bridge_base,
            decompress_color, decompress_depth,
        ]),

        TimerAction(period=3.0, actions=[
            LogInfo(msg='>>> [T+3s] EKF filter (jika use_ekf=true)...'),
            ekf_node,
        ]),

        TimerAction(period=6.0, actions=[
            LogInfo(msg='>>> [T+6s] RTAB-Map SLAM...'),
            rtabmap, rtabmap_viz,
        ]),

        TimerAction(period=8.0, actions=[
            LogInfo(msg='>>> [T+8s] RViz2...'),
            rviz_node,
        ]),
    ])

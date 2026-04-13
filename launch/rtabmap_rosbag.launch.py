"""
═══════════════════════════════════════════════════════════════════
  ALL-IN-ONE Launch File — FIXED v3
  Hardware : Velodyne VLP-32C + RealSense D455
  Rosbag   : rosbag2_2026_04_10-03_17_38

  FIX:
  1. Static TF pakai use_sim_time=FALSE agar langsung publish
     tanpa tunggu /clock dari rosbag
  2. FastDDS XML dinonaktifkan (tag invalid di versi ini)
  3. rtabmap parameter dibersihkan dari yang conflict
  4. Urutan launch diperbaiki: static TF dulu, baru rosbag

  TF Tree dari bag:
    /a200_1060/tf : odom → base_link
    /tf           : base_link → wheels
    (tambah kita) : base_link → velodyne
    (tambah kita) : base_link → camera_link → optical frames
═══════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    bag_path     = LaunchConfiguration('bag_path')
    rate         = LaunchConfiguration('rate')
    rviz         = LaunchConfiguration('rviz')
    localization = LaunchConfiguration('localization')
    db_path      = LaunchConfiguration('db_path')

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ws/rosbag2_2026_04_10-03_17_38')
    declare_rate = DeclareLaunchArgument('rate', default_value='0.5')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')
    declare_localization = DeclareLaunchArgument('localization', default_value='false')
    declare_db_path = DeclareLaunchArgument('db_path', default_value='/root/.ros/rtabmap.db')

    # ─────────────────────────────────────────
    # STATIC TF — use_sim_time=FALSE (wall clock)
    # Ini KRITIS: harus publish sebelum rosbag play
    # agar TF sudah ada saat data velodyne datang
    # ─────────────────────────────────────────
    tf_base_velodyne = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_velodyne',
        arguments=['0.05', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
        parameters=[{'use_sim_time': False}],
        output='screen')

    tf_base_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.2', '0', '0.25', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': False}],
        output='screen')

    tf_camera_color_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_color_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': False}],
        output='screen')

    tf_camera_depth_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_depth_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_depth_optical_frame'],
        parameters=[{'use_sim_time': False}],
        output='screen')

    # ─────────────────────────────────────────
    # ROSBAG PLAY
    # ─────────────────────────────────────────
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path,
             '--clock', '--rate', rate,
             '--read-ahead-queue-size', '1000'],
        output='screen', name='rosbag_play')

    # ─────────────────────────────────────────
    # RELAY — namespace /a200_1060 ke global
    # ─────────────────────────────────────────
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

    relay_camera_info = Node(
        package='topic_tools', executable='relay', name='relay_camera_info',
        arguments=['/camera/camera/color/camera_info', '/camera/color/camera_info'],
        parameters=[{'use_sim_time': True}], output='screen')

    # ─────────────────────────────────────────
    # IMAGE DECOMPRESS
    # ─────────────────────────────────────────
    decompress_color = Node(
        package='image_transport', executable='republish',
        name='decompress_color',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/camera/color/image_raw/compressed'),
            ('out', '/camera/color/image_raw'),
        ],
        parameters=[{'use_sim_time': True}], output='screen')

    decompress_depth = Node(
        package='image_transport', executable='republish',
        name='decompress_depth',
        arguments=['compressedDepth', 'raw'],
        remappings=[
            ('in/compressedDepth',
             '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'),
            ('out', '/camera/depth/image_rect_raw'),
        ],
        parameters=[{'use_sim_time': True}], output='screen')

    # ─────────────────────────────────────────
    # RTAB-MAP SLAM
    # ─────────────────────────────────────────
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':         True,
            'database_path':        db_path,
            'frame_id':             'base_link',
            'map_frame_id':         'map',
            'odom_frame_id':        'odom',
            'subscribe_depth':      True,
            'subscribe_rgb':        True,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info':  False,
            'approx_sync':               True,
            'approx_sync_max_interval':  0.5,
            'queue_size':                10,
            # Memory
            'Mem/STMSize':             '30',
            'Mem/RehearsalSimilarity': '0.45',
            'Mem/NotLinkedNodesKept':  'false',
            'Mem/IncrementalMemory':   PythonExpression(
                ["'false' if '", localization, "' == 'true' else 'true'"]),
            'Mem/InitWMWithAllNodes':  PythonExpression(
                ["'true' if '", localization, "' == 'true' else 'false'"]),
            # Loop closure
            'Kp/MaxFeatures':      '400',
            'Kp/DetectorStrategy': '6',
            'Vis/MinInliers':      '15',
            'Vis/InlierDistance':  '0.1',
            'Vis/EstimationType':  '1',
            'Vis/MaxDepth':        '10.0',
            # Optimizer
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/OptimizeMaxError':     '3.0',
            'Optimizer/Strategy':        '1',
            'Optimizer/Iterations':      '20',
            'Optimizer/Robust':          'true',
            # Proximity
            'RGBD/ProximityBySpace':       'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/AngularUpdate':          '0.05',
            'RGBD/LinearUpdate':           '0.1',
            # 2D grid
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
            # ICP
            'Reg/Strategy':                  '2',
            'Reg/Force3DoF':                 'false',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '30',
            'Icp/VoxelSize':                 '0.1',
            'Icp/Epsilon':                   '0.001',
            'Icp/MaxTranslation':            '1.5',
            'Icp/MaxRotation':               '0.5',
            'Icp/OutlierRatio':              '0.1',
            # Rate
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr':       '0',
            'Rtabmap/MemoryThr':     '0',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_rect_raw'),
            ('scan_cloud',      '/velodyne_points'),
            ('odom',            '/odom'),
            ('map',             '/map'),
            ('cloud_map',       '/rtabmap/cloud_map'),
            ('grid_map',        '/rtabmap/grid_map'),
        ])

    # ─────────────────────────────────────────
    # RTAB-Map Viz
    # ─────────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time':         True,
            'subscribe_depth':      True,
            'subscribe_rgb':        True,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info':  False,
            'approx_sync':          True,
            'frame_id':             'base_link',
            'odom_frame_id':        'odom',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_rect_raw'),
            ('scan_cloud',      '/velodyne_points'),
            ('odom',            '/odom'),
        ])

    # ─────────────────────────────────────────
    # RViz2
    # ─────────────────────────────────────────
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    # ═════════════════════════════════════════
    # URUTAN LAUNCH (dengan timer delay)
    # T+0s  : Static TF (wall clock) — HARUS DULUAN
    # T+1s  : Rosbag play
    # T+3s  : Relay TF/odom + decompress image
    # T+6s  : RTAB-Map + Viz
    # T+8s  : RViz
    # ═════════════════════════════════════════
    return LaunchDescription([
        declare_bag_path, declare_rate, declare_rviz,
        declare_localization, declare_db_path,

        # T+0s — Static TF dulu, pakai wall clock
        LogInfo(msg='>>> [T+0s] Static TF publishers starting (wall clock)...'),
        tf_base_velodyne,
        tf_base_camera,
        tf_camera_color_optical,
        tf_camera_depth_optical,

        # T+1s — Rosbag
        TimerAction(period=1.0, actions=[
            LogInfo(msg='>>> [T+1s] Rosbag play starting...'),
            rosbag_play,
        ]),

        # T+3s — Relay + decompress
        TimerAction(period=3.0, actions=[
            LogInfo(msg='>>> [T+3s] Relay & image decompress starting...'),
            relay_tf,
            relay_tf_static,
            relay_odom,
            relay_camera_info,
            decompress_color,
            decompress_depth,
        ]),

        # T+6s — RTAB-Map
        TimerAction(period=6.0, actions=[
            LogInfo(msg='>>> [T+6s] RTAB-Map SLAM starting...'),
            rtabmap,
            rtabmap_viz,
        ]),

        # T+8s — RViz
        TimerAction(period=8.0, actions=[
            LogInfo(msg='>>> [T+8s] RViz2 starting...'),
            rviz_node,
        ]),
    ])

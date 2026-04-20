#!/usr/bin/env python3

# ═══════════════════════════════════════════════════════════════════════
# rtabmap_rosbag.launch.py
#
# RTABMap pipeline untuk rosbag replay — VLP-32C + D455
# Bag baru 2026-04-15:
#   - Image compressed → node decompress otomatis ditambahkan
#   - /a200_1060/platform/odom/filtered sudah di bag → use_ekf default false
#   - aligned_depth_to_color dipakai (sudah aligned, lebih akurat)
# ═══════════════════════════════════════════════════════════════════════

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, OpaqueFunction, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():

    # ── Argumen ──────────────────────────────────────────────────────────
    bag_path_arg     = DeclareLaunchArgument('bag_path',
        default_value='/root/ws/project/rosbag_launch/bags/rosbag2_2026_04_15-08_39_23',
        description='Path ke folder rosbag2')
    rate_arg         = DeclareLaunchArgument('rate',         default_value='0.5')
    localization_arg = DeclareLaunchArgument('localization', default_value='false')
    use_ekf_arg      = DeclareLaunchArgument('use_ekf',      default_value='false')
    odom_topic_arg   = DeclareLaunchArgument('odom_topic',
        default_value='/a200_1060/platform/odom/filtered')
    rviz_arg         = DeclareLaunchArgument('rviz',         default_value='true')

    bag_path     = LaunchConfiguration('bag_path')
    rate         = LaunchConfiguration('rate')
    localization = LaunchConfiguration('localization')
    use_ekf      = LaunchConfiguration('use_ekf')
    odom_topic   = LaunchConfiguration('odom_topic')
    rviz         = LaunchConfiguration('rviz')

    # ── 1. Decompress color image ─────────────────────────────────────────
    # Bag baru: /camera/camera/color/image_raw/compressed → /camera/color/image_raw
    decompress_color = Node(
        package='image_transport',
        executable='republish',
        name='decompress_color',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/camera/color/image_raw/compressed'),
            ('out',           '/camera/color/image_raw'),
        ],
        output='screen',
    )

    # ── 2. Decompress depth (aligned to color) ────────────────────────────
    # Bag baru: /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
    #         → /camera/depth/image_rect_raw
    decompress_depth = Node(
        package='image_transport',
        executable='republish',
        name='decompress_depth',
        arguments=['compressedDepth', 'raw'],
        remappings=[
            ('in/compressedDepth', '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'),
            ('out',                '/camera/depth/image_rect_raw'),
        ],
        output='screen',
    )

    # ── 3. Relay camera_info (namespace /camera/camera/ → /camera/) ───────
    relay_color_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_color_info',
        arguments=[
            '/camera/camera/color/camera_info',
            '/camera/color/camera_info',
        ],
        output='screen',
    )

    relay_depth_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth_info',
        arguments=[
            '/camera/camera/aligned_depth_to_color/camera_info',
            '/camera/depth/camera_info',
        ],
        output='screen',
    )

    # ── 4. Relay odom → nama standar (opsional, untuk kompatibilitas) ──────
    # RTABMap akan pakai odom_topic langsung via remapping di bawah
    # Tapi relay ini membantu kalau ada node lain yang butuh /odom
    relay_odom = Node(
        package='topic_tools',
        executable='relay',
        name='relay_odom_filtered',
        arguments=[
            '/a200_1060/platform/odom/filtered',
            '/odom',
        ],
        output='screen',
    )

    # ── 5. EKF (robot_localization) — hanya kalau use_ekf=true ───────────
    # Untuk bag baru, use_ekf=false karena odom/filtered sudah ada di bag
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 50.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/a200_1060/platform/odom',
            'odom0_config': [
                True, True, False,
                False, False, True,
                True, True, False,
                False, False, True,
                False, False, False,
            ],
            'imu0': '/a200_1060/sensors/imu_0/data',
            'imu0_config': [
                False, False, False,
                True, True, True,
                False, False, False,
                True, True, True,
                True, False, False,
            ],
            'imu0_remove_gravitational_acceleration': True,
        }],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        condition=IfCondition(use_ekf),
    )

    # ── 6. ICP Odometry (dari LiDAR) ─────────────────────────────────────
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            # ICP
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/PointToPlane':              'true',
            'Icp/MaxRotation':               '1.0',
            'Icp/MaxTranslation':            '2.0',
            'Icp/VoxelSize':                 '0.1',
            'Icp/Epsilon':                   '0.001',
            # Odom
            'Odom/Strategy':          '0',   # Frame-to-map
            'Odom/GuessMotion':       'true',
            'Odom/ResetCountdown':    '0',
            'Reg/Force3DoF':          'true',
            # Frame
            'frame_id':               'base_link',
            'odom_frame_id':          'odom',
            'wait_for_transform':     '0.2',
            'approx_sync':            'true',
            'approx_sync_max_interval': '0.1',
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
        ],
    )

    # ── 7. RTABMap SLAM ───────────────────────────────────────────────────
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # ── Mode ──────────────────────────────────────────────────
            'Mem/IncrementalMemory':      PythonExpression(['"false" if "', localization, '" == "true" else "true"']),
            'Mem/InitWMWithAllNodes':     localization,

            # ── SLAM 2D ───────────────────────────────────────────────
            'Reg/Force3DoF':              'true',

            # ── Feature detection ─────────────────────────────────────
            'Kp/DetectorStrategy':        '6',   # ORB
            'Vis/FeatureType':            '6',   # ORB
            'Kp/MaxFeatures':             '600',
            'Vis/MinInliers':             '10',

            # ── ICP / Loop closure ────────────────────────────────────
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/PointToPlane':           'true',
            'Icp/MaxRotation':            '1.0',
            'Reg/Strategy':               '1',   # ICP

            # ── Proximity detection ───────────────────────────────────
            'RGBD/ProximityPathMaxNeighbors': '5',
            'RGBD/AngularUpdate':         '0.05',
            'RGBD/LinearUpdate':          '0.05',

            # ── Sync ──────────────────────────────────────────────────
            'approx_sync':                'true',
            'approx_sync_max_interval':   '2.0',

            # ── Frame ─────────────────────────────────────────────────
            'frame_id':                   'base_link',
            'map_frame_id':               'map',
            'odom_frame_id':              'odom',
            'wait_for_transform':         '0.3',

            # ── Database ──────────────────────────────────────────────
            'database_path':              '/root/.ros/rtabmap.db',
        }],
        remappings=[
            ('rgb/image',        '/camera/color/image_raw'),
            ('rgb/camera_info',  '/camera/color/camera_info'),
            ('depth/image',      '/camera/depth/image_rect_raw'),
            ('scan_cloud',       '/velodyne_points'),
            ('odom',             odom_topic),
            ('map',              '/map'),
        ],
    )

    # ── 8. RTABMap Viz ────────────────────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id':        'base_link',
            'odom_frame_id':   'odom',
            'subscribe_odom_info': True,
            'approx_sync':     True,
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/depth/image_rect_raw'),
            ('odom',            odom_topic),
        ],
        condition=IfCondition(rviz),
    )

    # ── 9. Rosbag play — delay 3 detik biar node siap ────────────────────
    rosbag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag_path,
                    '--rate', rate,
                    '--clock',
                    '--topics',
                        '/camera/camera/color/image_raw/compressed',
                        '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
                        '/camera/camera/aligned_depth_to_color/camera_info',
                        '/camera/camera/color/camera_info',
                        '/camera/camera/depth/image_rect_raw/compressedDepth',
                        '/camera/camera/depth/camera_info',
                        '/velodyne_points',
                        '/a200_1060/platform/odom/filtered',
                        '/a200_1060/sensors/imu_0/data',
                        '/tf',
                        '/tf_static',
                        '/a200_1060/tf',
                        '/a200_1060/tf_static',
                        '/a200_1060/platform/joint_states',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        # Argumen
        bag_path_arg,
        rate_arg,
        localization_arg,
        use_ekf_arg,
        odom_topic_arg,
        rviz_arg,

        # Decompress & relay (jalankan duluan)
        decompress_color,
        decompress_depth,
        relay_color_info,
        relay_depth_info,
        relay_odom,

        # EKF (hanya kalau use_ekf=true)
        ekf_node,

        # ICP Odometry dari LiDAR
        # CATATAN: kalau odom sudah dari bag (use_ekf=false),
        # icp_odometry di bawah bisa dinonaktifkan dengan
        # menghapus/comment blok ini — RTABMap akan pakai odom dari bag
        # icp_odometry,  # ← uncomment kalau mau ICP odom dari LiDAR

        # RTABMap
        rtabmap_node,
        rtabmap_viz,

        # Rosbag play (delay 3 detik)
        rosbag_play,
    ])

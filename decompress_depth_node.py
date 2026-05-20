#!/usr/bin/env python3
"""
decompress_depth_node.py

Workaround untuk bug ROS2 Humble: image_transport republish tidak support
compressedDepth via CLI args (SubscriberPlugin::subscribeImpl error).

Node ini subscribe ke /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
dan publish ke /camera/depth/image_rect_raw sebagai sensor_msgs/Image (16UC1).

Usage (standalone):
    python3 decompress_depth_node.py

Usage (via launch file):
    ExecuteProcess(cmd=['python3', 'decompress_depth_node.py'])
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
import cv2


class DecompressDepthNode(Node):

    def __init__(self):
        super().__init__('decompress_depth')

        # ── Subscribe compressedDepth dari bag ────────────────────────────
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
            self.callback_depth,
            10
        )

        # ── Subscribe camera_info dan relay ke namespace lama ─────────────
        self.sub_info_depth = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.callback_info_depth,
            10
        )
        self.sub_info_color = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.callback_info_color,
            10
        )

        # ── Publish raw depth & camera_infos ke namespace lama ───────────
        self.pub_depth = self.create_publisher(
            Image,
            '/camera/depth/image_rect_raw',
            10
        )
        self.pub_info_depth = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            10
        )
        self.pub_info_color = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            10
        )

        self.get_logger().info(
            'decompress_depth_node started\n'
            '  IN : /camera/camera/aligned_depth_to_color/image_raw/compressedDepth\n'
            '  OUT: /camera/depth/image_rect_raw'
        )

    def callback_depth(self, msg: CompressedImage):
        """
        Decode compressedDepth → 16UC1 raw Image.

        compressedDepth format: 12-byte header + PNG/RVL encoded depth data
        Header: [width(2) height(2) depth_quantization(4) compression_method(4)]
        """
        try:
            data = bytes(msg.data)

            # compressedDepth punya 12-byte header sebelum PNG data
            # Coba decode dengan skip header dulu
            if len(data) > 12 and data[12:16] == b'\x89PNG':
                png_data = np.frombuffer(data[12:], dtype=np.uint8)
            elif data[:8] == b'\x89PNG\r\n\x1a\n':
                # Langsung PNG tanpa header (format lama)
                png_data = np.frombuffer(data, dtype=np.uint8)
            else:
                # Coba skip header standar 12 byte
                png_data = np.frombuffer(data[12:], dtype=np.uint8)

            # Decode PNG → depth array
            depth_cv = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)

            if depth_cv is None:
                # Fallback: coba decode tanpa skip header
                raw = np.frombuffer(data, dtype=np.uint8)
                depth_cv = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)

            if depth_cv is None:
                self.get_logger().warn('Gagal decode depth frame', throttle_duration_sec=5.0)
                return

            # Pastikan 16UC1
            if depth_cv.dtype != np.uint16:
                depth_cv = depth_cv.astype(np.uint16)

            # Buat Image message
            out = Image()
            out.header         = msg.header
            out.height         = depth_cv.shape[0]
            out.width          = depth_cv.shape[1]
            out.encoding       = '16UC1'
            out.is_bigendian   = False
            out.step           = depth_cv.shape[1] * 2
            out.data           = depth_cv.tobytes()

            self.pub_depth.publish(out)

        except Exception as e:
            self.get_logger().error(
                f'Error decompress depth: {e}',
                throttle_duration_sec=5.0
            )

    def callback_info_depth(self, msg: CameraInfo):
        self.pub_info_depth.publish(msg)

    def callback_info_color(self, msg: CameraInfo):
        self.pub_info_color.publish(msg)


def main():
    rclpy.init()
    node = DecompressDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

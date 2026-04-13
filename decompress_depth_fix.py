#!/usr/bin/env python3
"""
Depth decompressor untuk format ROS compressedDepth
Format: '16UC1; compressedDepth'
Encoding: PNG 16-bit dengan 12-byte header ROS
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import struct
import zlib

class DepthDecompress(Node):
    def __init__(self):
        super().__init__('decompress_depth')
        self.count = 0
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
            self.cb, 10)
        self.pub = self.create_publisher(
            Image, '/camera/depth/image_rect_raw', 10)
        self.get_logger().info('Depth decompressor v2 ready (16UC1 compressedDepth)')

    def cb(self, msg):
        try:
            data = bytes(msg.data)

            # compressedDepth format dari ROS:
            # Byte 0-3   : header (config value, biasanya 0x00000000)
            # Byte 4-end : PNG data (16-bit grayscale)
            #
            # Cari PNG signature untuk skip header dengan tepat
            PNG_SIG = b'\x89PNG\r\n\x1a\n'
            png_start = data.find(PNG_SIG)

            if png_start == -1:
                # Coba RVL compressed format (alternative)
                self.get_logger().warn(
                    'PNG not found, trying raw decode',
                    throttle_duration_sec=10.0)
                return

            png_data = data[png_start:]

            # Decode PNG 16-bit
            import cv2
            buf = np.frombuffer(png_data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_ANYDEPTH)

            if img is None:
                self.get_logger().warn(
                    'cv2.imdecode failed',
                    throttle_duration_sec=5.0)
                return

            # Pastikan 16-bit
            if img.dtype != np.uint16:
                img = img.astype(np.uint16)

            out = Image()
            out.header = msg.header
            out.height, out.width = img.shape[:2]
            out.encoding = '16UC1'
            out.is_bigendian = 0
            out.step = out.width * 2
            out.data = img.tobytes()
            self.pub.publish(out)

            self.count += 1
            if self.count % 30 == 1:
                self.get_logger().info(
                    f'Depth published: {self.count} frames '
                    f'({img.shape[1]}x{img.shape[0]}, '
                    f'max={img.max()}, min={img[img>0].min() if img.any() else 0})')

        except Exception as e:
            self.get_logger().warn(
                f'Decompress error: {e}',
                throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = DepthDecompress()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

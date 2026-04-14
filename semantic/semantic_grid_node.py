#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  Semantic Grid Node — Instance-aware Version                    ║
║                                                                  ║
║  Setiap instance objek punya warna unik di map.                 ║
║  Contoh: chair_1 = biru, chair_2 = hijau, person_1 = merah     ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/pointcloud  (PointCloud2)                          ║
║      → decode class_id dari R, instance_id dari G              ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/grid_map     (OccupancyGrid) — instance_id/cell   ║
║    /semantic/grid_colored (PointCloud2)   — warna per instance ║
╚══════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.duration
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import time
from collections import defaultdict


# ── Warna per instance (RGB) — 40 warna unik ─────────────────────
INSTANCE_COLORS_RGB = [
    (255,56,56),(56,56,255),(56,255,56),(255,178,29),(207,10,249),
    (72,249,10),(146,23,204),(219,61,134),(147,26,52),(0,212,187),
    (168,153,44),(0,194,255),(52,147,69),(100,0,255),(255,24,0),
    (132,255,56),(82,133,0),(203,56,255),(255,55,149),(55,199,255),
    (200,100,50),(50,200,100),(100,50,200),(255,200,0),(0,255,200),
    (200,0,255),(255,100,200),(100,255,0),(0,100,255),(255,0,100),
    (150,75,0),(0,150,75),(75,0,150),(255,150,75),(75,255,150),
    (150,255,75),(75,150,255),(255,75,150),(150,0,75),(0,75,150),
]

COCO_NAMES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck',
    'boat','traffic light','fire hydrant','stop sign','parking meter','bench',
    'bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe',
    'backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard',
    'sports ball','kite','baseball bat','baseball glove','skateboard','surfboard',
    'tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl',
    'banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza',
    'donut','cake','chair','couch','potted plant','bed','dining table','toilet',
    'tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven',
    'toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush'
]


def pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (r<<16)|(g<<8)|b))[0]


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])


class SemanticGridNode(Node):
    def __init__(self):
        super().__init__('semantic_grid_node')

        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('resolution',   0.05)
        self.declare_parameter('min_height',   0.1)
        self.declare_parameter('max_height',   2.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('decay_time',   0.0)

        self.map_frame  = self.get_parameter('map_frame').value
        self.res        = self.get_parameter('resolution').value
        self.min_h      = self.get_parameter('min_height').value
        self.max_h      = self.get_parameter('max_height').value
        pub_rate        = self.get_parameter('publish_rate').value
        self.decay_time = self.get_parameter('decay_time').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Grid: {(gx,gy): {instance_id: count}}
        self.grid_votes = defaultdict(lambda: defaultdict(int))
        self.grid_ts    = {}

        # Instance → class mapping untuk label
        self.inst_to_class = {}  # {instance_id: class_id}

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.sub = self.create_subscription(
            PointCloud2, '/semantic/pointcloud', self._cloud_cb, qos_be)

        self.pub_grid    = self.create_publisher(
            OccupancyGrid, '/semantic/grid_map',    10)
        self.pub_colored = self.create_publisher(
            PointCloud2,   '/semantic/grid_colored', 10)

        self.create_timer(1.0 / pub_rate, self._publish)

        self._cc, self._tf_fail = 0, 0
        self.get_logger().info(
            f'SemanticGridNode ready | res: {self.res}m | '
            f'height: {self.min_h}-{self.max_h}m | frame: {self.map_frame}')

    def _cloud_cb(self, msg):
        self._cc += 1

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self._tf_fail += 1
            if self._tf_fail % 10 == 1:
                self.get_logger().warn(
                    f'TF fail ({self._tf_fail}x): {e}',
                    throttle_duration_sec=5.0)
            return

        # Parse PointCloud2 → numpy
        n = msg.width
        if n == 0:
            return
        raw  = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(n, 4)
        xyz  = raw[:, :3]
        rgb_raw = raw[:, 3].view(np.uint32)

        # Decode class dan instance dari RGB
        class_ids = ((rgb_raw >> 16) & 0xFF).astype(np.int32)   # R = class_id
        inst_ids  = ((rgb_raw >> 8)  & 0xFF).astype(np.int32)   # G = instance_id

        # Transform ke map frame
        R = quat_to_rot(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        xyz_map = (R @ xyz.T).T + t

        # Filter height
        z = xyz_map[:, 2]
        mask = (z >= self.min_h) & (z <= self.max_h)
        xyz_map   = xyz_map[mask]
        class_ids = class_ids[mask]
        inst_ids  = inst_ids[mask]

        if len(xyz_map) == 0:
            return

        now = time.time()
        for i in range(len(xyz_map)):
            gx = int(np.floor(xyz_map[i, 0] / self.res))
            gy = int(np.floor(xyz_map[i, 1] / self.res))
            iid = int(inst_ids[i])
            cid = int(class_ids[i])

            self.grid_votes[(gx, gy)][iid] += 1
            self.grid_ts[(gx, gy)]  = now

            # Simpan mapping instance → class
            if iid not in self.inst_to_class:
                self.inst_to_class[iid] = cid

        if self._cc % 20 == 1:
            self.get_logger().info(
                f'[Cloud {self._cc}] Points: {mask.sum()} | '
                f'Cells: {len(self.grid_votes)} | '
                f'Instances: {len(self.inst_to_class)}')

    def _publish(self):
        if not self.grid_votes:
            return

        # Decay
        if self.decay_time > 0:
            now = time.time()
            for k in [k for k, t in self.grid_ts.items()
                      if now - t > self.decay_time]:
                del self.grid_votes[k]
                del self.grid_ts[k]

        if not self.grid_votes:
            return

        cells   = list(self.grid_votes.keys())
        gx_vals = [c[0] for c in cells]
        gy_vals = [c[1] for c in cells]
        gx_min, gx_max = min(gx_vals), max(gx_vals)
        gy_min, gy_max = min(gy_vals), max(gy_vals)
        width  = gx_max - gx_min + 1
        height = gy_max - gy_min + 1

        data     = np.full(width * height, -1, dtype=np.int8)
        colored  = []
        now_s    = self.get_clock().now().to_msg()

        for (gx, gy), votes in self.grid_votes.items():
            best_inst = max(votes, key=votes.get)
            idx = (gy - gy_min) * width + (gx - gx_min)
            data[idx] = int(best_inst % 127)  # encode instance ke int8

            r, g, b = INSTANCE_COLORS_RGB[best_inst % len(INSTANCE_COLORS_RGB)]
            wx = (gx + 0.5) * self.res
            wy = (gy + 0.5) * self.res
            colored.append((wx, wy, 0.0, pack_rgb(r, g, b)))

        # Publish OccupancyGrid
        grid           = OccupancyGrid()
        grid.header.stamp    = now_s
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.res
        grid.info.width      = width
        grid.info.height     = height
        grid.info.origin.position.x = gx_min * self.res
        grid.info.origin.position.y = gy_min * self.res
        grid.info.origin.orientation.w = 1.0
        grid.data = data.tolist()
        self.pub_grid.publish(grid)

        # Publish colored point cloud
        if colored:
            self.pub_colored.publish(self._make_cloud(colored, now_s))

        # Log instance summary setiap 10 publish
        if hasattr(self, '_pub_count'):
            self._pub_count += 1
        else:
            self._pub_count = 1

        if self._pub_count % 10 == 1:
            summary = []
            active_insts = set()
            for votes in self.grid_votes.values():
                active_insts.add(max(votes, key=votes.get))
            for iid in sorted(active_insts):
                cid  = self.inst_to_class.get(iid, -1)
                name = COCO_NAMES[cid] if 0 <= cid < len(COCO_NAMES) else '?'
                summary.append(f'{name}_{iid}')
            self.get_logger().info(
                f'Grid {width}x{height} | '
                f'Instances di map: {", ".join(summary) if summary else "none"}')

    def _make_cloud(self, points, stamp):
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for (x, y, z, rgb) in points:
            data += struct.pack('ffff', x, y, z, rgb)

        msg = PointCloud2()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.map_frame
        msg.height = 1
        msg.width  = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = 16 * len(points)
        msg.data       = bytes(data)
        msg.is_dense   = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SemanticGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

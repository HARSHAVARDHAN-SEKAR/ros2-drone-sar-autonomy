#!/usr/bin/env python3
"""Convert bridged Gazebo depth images to a ROS PointCloud2 in optical frame."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from drone_perception.depth_utils import depth_to_meters


class DepthToCloud(Node):
    def __init__(self):
        super().__init__('depth_to_cloud')
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('downsample', 4)
        self.declare_parameter('output_frame', 'camera_optical_frame')

        self._min = float(self.get_parameter('min_depth').value)
        self._max = float(self.get_parameter('max_depth').value)
        self._ds = max(1, int(self.get_parameter('downsample').value))
        self._frame = str(self.get_parameter('output_frame').value)
        self._bridge = CvBridge()
        self._k = None

        self._pub = self.create_publisher(PointCloud2, '/camera/depth/points', 5)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)

    def _on_info(self, msg: CameraInfo):
        self._k = tuple(msg.k)

    def _on_depth(self, msg: Image):
        if self._k is None:
            return
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = depth_to_meters(raw, msg.encoding)
        except Exception as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')
            return

        h, w = depth.shape[:2]
        rows = np.arange(0, h, self._ds)
        cols = np.arange(0, w, self._ds)
        vv, uu = np.meshgrid(rows, cols, indexing='ij')
        z = depth[vv, uu]
        valid = np.isfinite(z) & (z >= self._min) & (z <= self._max)
        if not np.any(valid):
            return

        z = z[valid]
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        fx, fy, cx, cy = self._k[0], self._k[4], self._k[2], self._k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self._frame
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = False
        cloud.data = points.tobytes()
        self._pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

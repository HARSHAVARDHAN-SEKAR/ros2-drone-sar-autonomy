#!/usr/bin/env python3
"""YOLO person detection + synchronized depth + TF map localization.

A detected person is treated as a *victim candidate*. Spatial deduplication
prevents one person from being assigned a new victim ID on every frame.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs
import tf2_ros

from drone_msgs.msg import VictimDetection
from drone_perception.depth_utils import (
    backproject_depth_pixel,
    depth_to_meters,
    project_rgb_pixel_to_depth,
)

try:
    from ultralytics import YOLO
except ImportError:  # runtime error is reported cleanly in __init__
    YOLO = None


@dataclass
class KnownVictim:
    victim_id: int
    x: float
    y: float
    z: float
    last_seen: float
    last_published: float = 0.0


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.65)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('depth_roi_px', 8)
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('dedup_radius_m', 1.25)
        self.declare_parameter('detection_republish_s', 2.0)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('device', '')

        self._conf = float(self.get_parameter('confidence').value)
        self._iou = float(self.get_parameter('iou').value)
        self._roi = max(1, int(self.get_parameter('depth_roi_px').value))
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._dedup = float(self.get_parameter('dedup_radius_m').value)
        self._republish = max(
            0.1,
            float(self.get_parameter('detection_republish_s').value),
        )
        self._world = str(self.get_parameter('world_frame').value)
        self._camera = str(self.get_parameter('camera_frame').value)
        self._device = str(self.get_parameter('device').value)

        if YOLO is None:
            raise RuntimeError(
                'Ultralytics is not installed. Run: pip3 install --user "numpy<2" ultralytics')
        self._model = YOLO(str(self.get_parameter('model_path').value))
        self._bridge = CvBridge()
        self._rgb_k = None
        self._depth_k = None
        self._known: list[KnownVictim] = []
        self._next_id = 1

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_detection = self.create_publisher(
            VictimDetection, '/victims/detected', 10)
        self._pub_poses = self.create_publisher(PoseArray, '/victims/poses', 10)
        self._pub_image = self.create_publisher(Image, '/victims/image_annotated', 5)

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_rgb_info, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._on_depth_info, qos_profile_sensor_data)

        rgb_sub = Subscriber(self, Image, '/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=8, slop=0.12)
        self._sync.registerCallback(self._on_frame_pair)
        self.get_logger().info('Victim detector ready.')

    def _on_rgb_info(self, msg: CameraInfo):
        self._rgb_k = tuple(msg.k)

    def _on_depth_info(self, msg: CameraInfo):
        self._depth_k = tuple(msg.k)

    def _on_frame_pair(self, rgb_msg: Image, depth_msg: Image):
        if self._rgb_k is None or self._depth_k is None:
            return
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = depth_to_meters(depth_raw, depth_msg.encoding)
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return

        kwargs = dict(conf=self._conf, iou=self._iou, classes=[0], verbose=False)
        if self._device:
            kwargs['device'] = self._device
        results = self._model.predict(rgb, **kwargs)
        annotated = rgb.copy()

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                u_rgb = 0.5 * (x1 + x2)
                v_rgb = 0.5 * (y1 + y2)
                u_d, v_d = project_rgb_pixel_to_depth(
                    u_rgb, v_rgb, self._rgb_k, self._depth_k)
                distance = self._sample_depth(depth, u_d, v_d)

                label = f'person {conf:.2f}'
                if distance is not None:
                    world = self._localize(u_d, v_d, distance, rgb_msg.header.stamp)
                    if world is not None:
                        victim, is_new = self._associate(world)
                        label = f'candidate#{victim.victim_id} {conf:.2f} {distance:.1f}m'
                        now = time.monotonic()
                        publish_due = (
                            is_new
                            or now - victim.last_published >= self._republish
                        )

                        if publish_due:
                            self._publish_detection(
                                victim.victim_id, conf, distance, world,
                                rgb, x1, y1, x2, y2,
                                rgb_msg.header.stamp,
                            )
                            victim.last_published = now

                        if is_new:
                            self.get_logger().warn(
                                f'New victim candidate #{victim.victim_id} at '
                                f'({world.pose.position.x:.1f}, '
                                f'{world.pose.position.y:.1f}, '
                                f'{world.pose.position.z:.1f}) m, '
                                f'confidence={conf:.2f}.')

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated, label, (max(0, x1), max(15, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        out = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = rgb_msg.header
        self._pub_image.publish(out)
        self._publish_pose_array(rgb_msg.header.stamp)

    def _sample_depth(self, depth: np.ndarray, u: float, v: float):
        ui, vi = int(round(u)), int(round(v))
        h, w = depth.shape[:2]
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        r = self._roi
        x1, x2 = max(0, ui - r), min(w, ui + r + 1)
        y1, y2 = max(0, vi - r), min(h, vi + r + 1)
        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi >= self._min_depth) & (roi <= self._max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _localize(self, u: float, v: float, depth_m: float, stamp):
        x, y, z = backproject_depth_pixel(u, v, depth_m, self._depth_k)
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = self._camera
        p.point.x, p.point.y, p.point.z = x, y, z
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world, self._camera, Time.from_msg(stamp),
                timeout=Duration(seconds=0.15))
            pw = tf2_geometry_msgs.do_transform_point(p, tf)
        except Exception as exc:
            self.get_logger().debug(f'Victim TF unavailable: {exc}')
            return None

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._world
        pose.pose.position = pw.point
        pose.pose.orientation.w = 1.0
        return pose

    def _associate(self, pose: PoseStamped):
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        now = time.monotonic()
        for item in self._known:
            d = math.sqrt((x-item.x)**2 + (y-item.y)**2 + (z-item.z)**2)
            if d <= self._dedup:
                item.x, item.y, item.z, item.last_seen = x, y, z, now
                return item, False
        item = KnownVictim(self._next_id, x, y, z, now)
        self._next_id += 1
        self._known.append(item)
        return item, True

    def _publish_detection(self, victim_id, conf, distance, pose, rgb, x1, y1, x2, y2, stamp):
        det = VictimDetection()
        det.header.stamp = stamp
        det.header.frame_id = self._world
        det.victim_id = int(victim_id)
        det.label = 'person'
        det.confidence = float(conf)
        det.distance_m = float(distance)
        det.pose = pose
        crop = rgb[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if crop.size:
            det.thumbnail = self._bridge.cv2_to_imgmsg(crop, encoding='bgr8')
            det.thumbnail.header.stamp = stamp
        self._pub_detection.publish(det)

    def _publish_pose_array(self, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world
        for item in self._known:
            p = Pose()
            p.position.x, p.position.y, p.position.z = item.x, item.y, item.z
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._pub_poses.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

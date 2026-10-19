#!/usr/bin/env python3
"""Broadcast map->base_link from MAVROS local pose.

The fixed base_link->camera frames come from drone_description's
robot_state_publisher. Using ROS time for the transform stamp keeps the TF tree
aligned with Gazebo sensor timestamps when use_sim_time is enabled.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class TfBridge(Node):
    def __init__(self):
        super().__init__('drone_tf_bridge')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self._map = str(self.get_parameter('map_frame').value)
        self._base = str(self.get_parameter('base_frame').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._pose = None
        self._br = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        self.create_timer(1.0 / max(rate, 1.0), self._publish)

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _publish(self):
        if self._pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map
        t.child_frame_id = self._base
        t.transform.translation.x = self._pose.pose.position.x
        t.transform.translation.y = self._pose.pose.position.y
        t.transform.translation.z = self._pose.pose.position.z
        t.transform.rotation = self._pose.pose.orientation
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
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

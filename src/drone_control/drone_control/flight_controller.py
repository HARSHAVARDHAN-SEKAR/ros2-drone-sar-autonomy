#!/usr/bin/env python3
"""Single-owner PX4/MAVROS offboard controller.

Only this node is allowed to command arming, PX4 flight modes, and local
position setpoints. Higher-level nodes request behavior via /flight/command
and /autonomy/path.
"""

from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, SetMode


class FlightPhase(Enum):
    WAIT_FCU = auto()
    IDLE = auto()
    PRESTREAM = auto()
    TAKEOFF = auto()
    HOLD = auto()
    FOLLOW_PATH = auto()
    RTL = auto()
    LAND = auto()


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('prestream_seconds', 1.5)
        self.declare_parameter('takeoff_altitude', 2.5)
        self.declare_parameter('takeoff_tolerance', 0.20)
        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('service_retry_seconds', 1.0)
        self.declare_parameter('path_lookahead_index', 1)

        self._rate = float(self.get_parameter('setpoint_rate_hz').value)
        self._prestream_seconds = float(self.get_parameter('prestream_seconds').value)
        self._takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self._takeoff_tol = float(self.get_parameter('takeoff_tolerance').value)
        self._wp_tol = float(self.get_parameter('waypoint_tolerance').value)
        self._retry = float(self.get_parameter('service_retry_seconds').value)
        self._lookahead = int(self.get_parameter('path_lookahead_index').value)

        self._mav = MavState()
        self._pose = PoseStamped()
        self._pose_received = False
        self._phase = FlightPhase.WAIT_FCU
        self._target: Optional[PoseStamped] = None
        self._home: Optional[PoseStamped] = None
        self._path: list[PoseStamped] = []
        self._path_index = 0
        self._prestream_started = 0.0
        self._last_mode_request = 0.0
        self._last_arm_request = 0.0
        self._mode_future = None
        self._arm_future = None

        self._pub_setpoint = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self._pub_status = self.create_publisher(String, '/flight/status', 10)

        self.create_subscription(MavState, '/mavros/state', self._on_mav_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        self.create_subscription(String, '/flight/command', self._on_command, 10)
        self.create_subscription(Path, '/autonomy/path', self._on_path, 10)

        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(1.0 / max(self._rate, 3.0), self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info('Flight controller ready; waiting for MAVROS/PX4.')

    def _on_mav_state(self, msg: MavState):
        self._mav = msg
        if msg.connected and self._phase == FlightPhase.WAIT_FCU:
            self._phase = FlightPhase.IDLE
            self.get_logger().info('PX4 connected. Flight controller is IDLE.')
        elif not msg.connected and self._phase != FlightPhase.WAIT_FCU:
            self.get_logger().error('PX4 connection lost.')
            self._phase = FlightPhase.WAIT_FCU
            self._path.clear()

        if not msg.armed and self._phase in (
            FlightPhase.TAKEOFF, FlightPhase.HOLD, FlightPhase.FOLLOW_PATH
        ):
            self.get_logger().warn('Vehicle became disarmed; returning to IDLE.')
            self._phase = FlightPhase.IDLE
            self._path.clear()

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg
        self._pose_received = True
        if self._home is None and self._mav.connected:
            self._home = self._copy_pose(msg)

    def _on_command(self, msg: String):
        command = msg.data.strip().upper()
        if command == 'TAKEOFF':
            self._begin_takeoff()
        elif command == 'HOLD':
            self._hold_here('hold requested')
        elif command in ('RTH', 'RTL'):
            self._path.clear()
            self._phase = FlightPhase.RTL
            self._request_mode('AUTO.RTL', force=True)
            self.get_logger().warn('Return-to-launch requested.')
        elif command == 'LAND':
            self._path.clear()
            self._phase = FlightPhase.LAND
            self._request_mode('AUTO.LAND', force=True)
            self.get_logger().warn('Landing requested.')
        elif command == 'ABORT':
            # A controlled AUTO.LAND is safer than motor kill for this project.
            self._path.clear()
            self._phase = FlightPhase.LAND
            self._request_mode('AUTO.LAND', force=True)
            self.get_logger().error('ABORT requested: commanding AUTO.LAND.')
        else:
            self.get_logger().warn(f'Unknown flight command: {command}')

    def _on_path(self, msg: Path):
        if not msg.poses:
            return
        if self._phase not in (FlightPhase.HOLD, FlightPhase.FOLLOW_PATH):
            return
        if not self._mav.armed:
            return

        self._path = [self._copy_pose(p) for p in msg.poses]
        closest = min(
            range(len(self._path)),
            key=lambda i: self._distance_xyz(self._pose, self._path[i]),
        )
        self._path_index = min(closest + max(self._lookahead, 0), len(self._path) - 1)
        self._phase = FlightPhase.FOLLOW_PATH
        self.get_logger().info(
            f'Accepted path with {len(self._path)} poses; tracking from index {self._path_index}.')

    def _begin_takeoff(self):
        if self._phase in (FlightPhase.PRESTREAM, FlightPhase.TAKEOFF):
            return
        if self._phase not in (FlightPhase.IDLE,):
            self.get_logger().warn(f'TAKEOFF ignored in phase {self._phase.name}.')
            return
        if not self._mav.connected or not self._pose_received:
            self.get_logger().error('Cannot take off: PX4/local pose not ready.')
            return
        if self._mav.armed:
            self.get_logger().warn('TAKEOFF ignored: vehicle is already armed.')
            return

        self._home = self._copy_pose(self._pose)
        self._target = self._copy_pose(self._pose)
        self._target.pose.orientation.w = 1.0
        self._prestream_started = time.monotonic()
        self._phase = FlightPhase.PRESTREAM
        self._path.clear()
        self.get_logger().info(
            f'Pre-streaming offboard setpoints for {self._prestream_seconds:.1f} s.')

    def _tick(self):
        self._poll_service_futures()

        if self._phase == FlightPhase.WAIT_FCU:
            return

        if self._phase == FlightPhase.PRESTREAM:
            self._stream_target()
            elapsed = time.monotonic() - self._prestream_started
            if elapsed >= self._prestream_seconds:
                if self._mav.mode != 'OFFBOARD':
                    self._request_mode('OFFBOARD')
                    return
                if not self._mav.armed:
                    self._request_arm(True)
                    return
                self._target = self._copy_pose(self._pose)
                self._target.pose.position.z = (
                    self._home.pose.position.z + self._takeoff_altitude
                    if self._home is not None else self._takeoff_altitude
                )
                self._phase = FlightPhase.TAKEOFF
                self.get_logger().info(
                    f'PX4 armed in OFFBOARD; climbing to {self._target.pose.position.z:.2f} m local Z.')
            return

        if self._phase == FlightPhase.TAKEOFF:
            if self._mav.mode != 'OFFBOARD':
                self._request_mode('OFFBOARD')
            self._stream_target()
            if self._target is not None:
                dz = abs(self._pose.pose.position.z - self._target.pose.position.z)
                if dz <= self._takeoff_tol:
                    self._hold_here('takeoff altitude reached')
            return

        if self._phase == FlightPhase.HOLD:
            if self._mav.mode != 'OFFBOARD':
                self._request_mode('OFFBOARD')
            self._stream_target()
            return

        if self._phase == FlightPhase.FOLLOW_PATH:
            if self._mav.mode != 'OFFBOARD':
                self._request_mode('OFFBOARD')
            self._track_path()
            self._stream_target()
            return

        if self._phase == FlightPhase.RTL:
            if self._mav.armed and self._mav.mode != 'AUTO.RTL':
                self._request_mode('AUTO.RTL')
            if not self._mav.armed:
                self._phase = FlightPhase.IDLE
            return

        if self._phase == FlightPhase.LAND:
            if self._mav.armed and self._mav.mode != 'AUTO.LAND':
                self._request_mode('AUTO.LAND')
            if not self._mav.armed:
                self._phase = FlightPhase.IDLE
            return

    def _track_path(self):
        if not self._path:
            self._hold_here('path exhausted')
            return

        self._path_index = max(0, min(self._path_index, len(self._path) - 1))
        waypoint = self._path[self._path_index]
        distance = self._distance_xyz(self._pose, waypoint)

        if distance <= self._wp_tol:
            if self._path_index >= len(self._path) - 1:
                self._hold_here('path goal reached')
                return
            self._path_index += 1
            waypoint = self._path[self._path_index]

        self._target = self._copy_pose(waypoint)
        dx = waypoint.pose.position.x - self._pose.pose.position.x
        dy = waypoint.pose.position.y - self._pose.pose.position.y
        if abs(dx) + abs(dy) > 1e-6:
            qx, qy, qz, qw = _yaw_to_quat(math.atan2(dy, dx))
            self._target.pose.orientation.x = qx
            self._target.pose.orientation.y = qy
            self._target.pose.orientation.z = qz
            self._target.pose.orientation.w = qw

    def _hold_here(self, reason: str):
        if not self._pose_received:
            return
        self._path.clear()
        self._target = self._copy_pose(self._pose)
        self._phase = FlightPhase.HOLD
        self.get_logger().info(f'HOLD: {reason}.')

    def _stream_target(self):
        if self._target is None:
            return
        msg = self._copy_pose(self._target)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        if msg.pose.orientation.w == 0.0 and msg.pose.orientation.z == 0.0:
            msg.pose.orientation.w = 1.0
        self._pub_setpoint.publish(msg)

    def _request_mode(self, mode: str, force: bool = False):
        now = time.monotonic()
        if self._mode_future is not None and not self._mode_future.done():
            return
        if not force and now - self._last_mode_request < self._retry:
            return
        if not self._mode_client.service_is_ready():
            self._mode_client.wait_for_service(timeout_sec=0.0)
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self._mode_future = self._mode_client.call_async(req)
        self._last_mode_request = now

    def _request_arm(self, arm: bool):
        now = time.monotonic()
        if self._arm_future is not None and not self._arm_future.done():
            return
        if now - self._last_arm_request < self._retry:
            return
        if not self._arm_client.service_is_ready():
            self._arm_client.wait_for_service(timeout_sec=0.0)
            return
        req = CommandBool.Request()
        req.value = arm
        self._arm_future = self._arm_client.call_async(req)
        self._last_arm_request = now

    def _poll_service_futures(self):
        if self._mode_future is not None and self._mode_future.done():
            try:
                result = self._mode_future.result()
                if result is not None and not result.mode_sent:
                    self.get_logger().warn('PX4 mode request was not accepted.')
            except Exception as exc:
                self.get_logger().warn(f'Mode service error: {exc}')
            self._mode_future = None

        if self._arm_future is not None and self._arm_future.done():
            try:
                result = self._arm_future.result()
                if result is not None and not result.success:
                    self.get_logger().warn('PX4 arming request was not accepted.')
            except Exception as exc:
                self.get_logger().warn(f'Arming service error: {exc}')
            self._arm_future = None

    def _publish_status(self):
        target = self._target.pose.position if self._target is not None else None
        target_text = (
            f' target=({target.x:.1f},{target.y:.1f},{target.z:.1f})'
            if target is not None else ''
        )
        msg = String()
        msg.data = (
            f'phase={self._phase.name} connected={self._mav.connected} '
            f'armed={self._mav.armed} mode={self._mav.mode}' + target_text
        )
        self._pub_status.publish(msg)

    @staticmethod
    def _copy_pose(src: PoseStamped) -> PoseStamped:
        dst = PoseStamped()
        dst.header.frame_id = src.header.frame_id
        dst.header.stamp = src.header.stamp
        dst.pose.position.x = src.pose.position.x
        dst.pose.position.y = src.pose.position.y
        dst.pose.position.z = src.pose.position.z
        dst.pose.orientation.x = src.pose.orientation.x
        dst.pose.orientation.y = src.pose.orientation.y
        dst.pose.orientation.z = src.pose.orientation.z
        dst.pose.orientation.w = src.pose.orientation.w
        return dst

    @staticmethod
    def _distance_xyz(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

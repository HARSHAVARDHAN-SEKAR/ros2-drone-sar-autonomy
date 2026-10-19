#!/usr/bin/env python3
"""High-level search-and-rescue mission state machine.

This node never arms PX4 or writes MAVROS setpoints. It only asks the single
flight_controller for TAKEOFF/HOLD/RTH/LAND and enables/disables exploration.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State as MavState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

from drone_msgs.msg import MissionState, SafetyEvent, VictimDetection


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('takeoff_altitude_m', 2.5)
        self.declare_parameter('takeoff_tolerance_m', 0.30)
        self.declare_parameter('inspection_hold_s', 4.0)
        self.declare_parameter('exploration_timeout_s', 300.0)
        self.declare_parameter('mission_timeout_s', 420.0)

        self._takeoff_alt = float(self.get_parameter('takeoff_altitude_m').value)
        self._takeoff_tol = float(self.get_parameter('takeoff_tolerance_m').value)
        self._inspect_s = float(self.get_parameter('inspection_hold_s').value)
        self._explore_timeout = float(self.get_parameter('exploration_timeout_s').value)
        self._mission_timeout = float(self.get_parameter('mission_timeout_s').value)

        self._mav = MavState()
        self._pose: PoseStamped | None = None
        self._home_z: float | None = None
        self._state = MissionState.IDLE
        self._detail = 'waiting for START'
        self._mission_started: float | None = None
        self._explore_started: float | None = None
        self._inspect_until: float | None = None
        self._unique_victims: set[int] = set()
        self._started_once = False
        self._last_flight_command = ''
        self._last_flight_command_at = 0.0

        self._flight_pub = self.create_publisher(String, '/flight/command', 10)
        self._explore_pub = self.create_publisher(Bool, '/exploration/enable', 10)
        self._state_pub = self.create_publisher(MissionState, '/mission/state', 10)

        self.create_subscription(MavState, '/mavros/state', self._on_mav, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        self.create_subscription(VictimDetection, '/victims/detected', self._on_victim, 10)
        self.create_subscription(SafetyEvent, '/safety/event', self._on_safety, 10)
        self.create_subscription(Bool, '/exploration/complete', self._on_exploration_complete, 10)
        self.create_subscription(String, '/mission/command', self._on_command, 10)

        self.create_timer(0.2, self._tick)
        self.create_timer(0.5, self._publish_state)

        if bool(self.get_parameter('auto_start').value):
            self.create_timer(2.0, self._auto_start_once)
        self.get_logger().info('Mission manager ready. Publish START on /mission/command.')

    def _auto_start_once(self):
        if not self._started_once and self._state == MissionState.IDLE:
            self._start_mission('auto_start parameter')

    def _on_mav(self, msg: MavState):
        self._mav = msg

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _on_command(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self._start_mission('operator START')
        elif cmd in ('RTH', 'RTL'):
            self._begin_return('operator requested return')
        elif cmd == 'LAND':
            self._begin_land('operator requested landing')
        elif cmd == 'ABORT':
            self._abort('operator ABORT')
        elif cmd == 'RESET' and not self._mav.armed:
            self._state = MissionState.IDLE
            self._detail = 'reset; waiting for START'
            self._mission_started = None
            self._explore_started = None
            self._inspect_until = None
            self._unique_victims.clear()
            self._enable_exploration(False)
        else:
            self.get_logger().warn(f'Unknown/inapplicable mission command: {cmd}')

    def _start_mission(self, reason: str):
        if self._state not in (MissionState.IDLE, MissionState.COMPLETE):
            self.get_logger().warn('START ignored: a mission is already active.')
            return
        if not self._mav.connected or self._pose is None:
            self.get_logger().error('START rejected: MAVROS/PX4 local pose is not ready.')
            return
        if self._mav.armed:
            self.get_logger().error('START rejected: vehicle is already armed.')
            return
        self._started_once = True
        self._mission_started = time.monotonic()
        self._home_z = self._pose.pose.position.z
        self._unique_victims.clear()
        self._state = MissionState.TAKEOFF
        self._detail = reason
        self._enable_exploration(False)
        self._command_flight('TAKEOFF', force=True)
        self.get_logger().info('Mission started: TAKEOFF.')

    def _on_victim(self, msg: VictimDetection):
        victim_id = int(msg.victim_id)
        if victim_id <= 0 or victim_id in self._unique_victims:
            return
        self._unique_victims.add(victim_id)
        if self._state == MissionState.EXPLORE:
            self._state = MissionState.INSPECT
            self._detail = f'inspecting victim candidate #{victim_id}'
            self._inspect_until = time.monotonic() + max(0.0, self._inspect_s)
            self._enable_exploration(False)
            self._command_flight('HOLD', force=True)
            self.get_logger().warn(
                f'Victim candidate #{victim_id}: holding for {self._inspect_s:.1f} s inspection.')

    def _on_safety(self, msg: SafetyEvent):
        action = msg.recommended_action.strip().upper()
        if msg.severity < SafetyEvent.WARNING or action not in ('RTH', 'RTL', 'LAND', 'ABORT'):
            return
        detail = f'safety {msg.code}: {msg.message}'
        if action in ('RTH', 'RTL'):
            self._begin_return(detail)
        elif action in ('LAND', 'ABORT'):
            self._abort(detail)

    def _on_exploration_complete(self, msg: Bool):
        if msg.data and self._state == MissionState.EXPLORE:
            self._begin_return('exploration frontiers exhausted')

    def _tick(self):
        now = time.monotonic()
        if self._mission_started is not None and self._state in (
            MissionState.TAKEOFF, MissionState.EXPLORE, MissionState.INSPECT
        ):
            if now - self._mission_started > self._mission_timeout:
                self._begin_return('mission timeout')

        if self._state == MissionState.TAKEOFF:
            self._command_flight('TAKEOFF')
            if self._pose is not None and self._home_z is not None:
                altitude_gain = self._pose.pose.position.z - self._home_z
                if (self._mav.armed and self._mav.mode == 'OFFBOARD' and
                        altitude_gain >= self._takeoff_alt - self._takeoff_tol):
                    self._state = MissionState.EXPLORE
                    self._detail = 'takeoff complete; frontier exploration active'
                    self._explore_started = now
                    self._enable_exploration(True)
                    self.get_logger().info('Takeoff complete: EXPLORE.')

        elif self._state == MissionState.EXPLORE:
            if self._explore_started is not None and now - self._explore_started > self._explore_timeout:
                self._begin_return('exploration timeout')

        elif self._state == MissionState.INSPECT:
            if self._inspect_until is not None and now >= self._inspect_until:
                self._inspect_until = None
                self._state = MissionState.EXPLORE
                self._detail = 'inspection hold complete; exploration resumed'
                self._enable_exploration(True)
                self.get_logger().info('Inspection complete: EXPLORE resumed.')

        elif self._state in (MissionState.RETURN, MissionState.LAND, MissionState.ABORT):
            if not self._mav.armed and self._mission_started is not None:
                self._state = MissionState.COMPLETE
                self._detail = f'mission complete with {len(self._unique_victims)} unique victim candidate(s)'
                self._enable_exploration(False)
                self.get_logger().info(self._detail)

    def _begin_return(self, detail: str):
        if self._state in (MissionState.IDLE, MissionState.COMPLETE, MissionState.RETURN):
            return
        self._state = MissionState.RETURN
        self._detail = detail
        self._enable_exploration(False)
        self._command_flight('RTH', force=True)
        self.get_logger().warn(f'RETURN: {detail}')

    def _begin_land(self, detail: str):
        if self._state in (MissionState.IDLE, MissionState.COMPLETE, MissionState.LAND):
            return
        self._state = MissionState.LAND
        self._detail = detail
        self._enable_exploration(False)
        self._command_flight('LAND', force=True)

    def _abort(self, detail: str):
        if self._state in (MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT):
            return
        self._state = MissionState.ABORT
        self._detail = detail
        self._enable_exploration(False)
        self._command_flight('ABORT', force=True)
        self.get_logger().error(f'ABORT: {detail}')

    def _command_flight(self, command: str, force: bool = False):
        now = time.monotonic()
        # Avoid flooding the command topic while still retrying TAKEOFF if needed.
        if not force and command == self._last_flight_command and now - self._last_flight_command_at < 1.0:
            return
        self._flight_pub.publish(String(data=command))
        self._last_flight_command = command
        self._last_flight_command_at = now

    def _enable_exploration(self, enable: bool):
        self._explore_pub.publish(Bool(data=enable))

    def _publish_state(self):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state = int(self._state)
        names = {
            MissionState.IDLE: 'IDLE', MissionState.TAKEOFF: 'TAKEOFF',
            MissionState.EXPLORE: 'EXPLORE', MissionState.INSPECT: 'INSPECT',
            MissionState.RETURN: 'RETURN', MissionState.LAND: 'LAND',
            MissionState.COMPLETE: 'COMPLETE', MissionState.ABORT: 'ABORT',
        }
        msg.state_name = names.get(self._state, 'UNKNOWN')
        msg.detail = self._detail
        msg.unique_victims = len(self._unique_victims)
        msg.elapsed_sec = float(time.monotonic() - self._mission_started) if self._mission_started else 0.0
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

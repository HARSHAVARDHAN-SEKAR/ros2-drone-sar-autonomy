#!/usr/bin/env python3
"""Safety watchdog that requests RTH/LAND without directly controlling PX4."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State as MavState
from sensor_msgs.msg import BatteryState
from drone_msgs.msg import SafetyEvent


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('battery_rth_percent', 25.0)
        self.declare_parameter('battery_land_percent', 15.0)
        self.declare_parameter('geofence_radius_m', 40.0)
        self.declare_parameter('max_altitude_m', 20.0)
        self.declare_parameter('event_cooldown_s', 5.0)

        self._bat_rth = float(self.get_parameter('battery_rth_percent').value)
        self._bat_land = float(self.get_parameter('battery_land_percent').value)
        self._fence = float(self.get_parameter('geofence_radius_m').value)
        self._max_alt = float(self.get_parameter('max_altitude_m').value)
        self._cooldown = float(self.get_parameter('event_cooldown_s').value)

        self._mav = MavState()
        self._pose = None
        self._home = None
        self._battery = None
        self._last_emit = {}

        self._pub_event = self.create_publisher(SafetyEvent, '/safety/event', 10)
        self.create_subscription(MavState, '/mavros/state', self._on_mav, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/mavros/battery', self._on_battery, qos_profile_sensor_data)
        self.create_timer(0.5, self._check)

    def _on_mav(self, msg: MavState):
        was_armed = self._mav.armed
        self._mav = msg
        if msg.armed and not was_armed and self._pose is not None:
            self._home = (self._pose.pose.position.x,
                          self._pose.pose.position.y,
                          self._pose.pose.position.z)

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _on_battery(self, msg: BatteryState):
        p = float(msg.percentage)
        if math.isfinite(p) and p >= 0.0:
            self._battery = 100.0 * p if p <= 1.0 else p

    def _check(self):
        if not self._mav.armed:
            return

        if not self._mav.connected:
            self._emit('FCU_DISCONNECTED', SafetyEvent.CRITICAL,
                       'MAVROS lost the PX4 connection while armed.', 'LAND')
            return

        if self._battery is not None:
            if self._battery <= self._bat_land:
                self._emit('BATTERY_CRITICAL', SafetyEvent.CRITICAL,
                           f'Battery {self._battery:.1f}% <= {self._bat_land:.1f}%.', 'LAND')
            elif self._battery <= self._bat_rth:
                self._emit('BATTERY_LOW', SafetyEvent.WARNING,
                           f'Battery {self._battery:.1f}% <= {self._bat_rth:.1f}%.', 'RTH')

        if self._pose is None or self._home is None:
            return
        x, y, z = (self._pose.pose.position.x,
                   self._pose.pose.position.y,
                   self._pose.pose.position.z)
        hx, hy, hz = self._home
        radius = math.hypot(x - hx, y - hy)
        altitude = z - hz
        if radius > self._fence:
            self._emit('GEOFENCE', SafetyEvent.CRITICAL,
                       f'Geofence exceeded: {radius:.1f} m > {self._fence:.1f} m.', 'RTH')
        if altitude > self._max_alt:
            self._emit('MAX_ALTITUDE', SafetyEvent.CRITICAL,
                       f'Altitude exceeded: {altitude:.1f} m > {self._max_alt:.1f} m.', 'RTH')

    def _emit(self, code, severity, message, action):
        now = time.monotonic()
        if now - self._last_emit.get(code, -1e9) < self._cooldown:
            return
        self._last_emit[code] = now
        event = SafetyEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.severity = int(severity)
        event.code = code
        event.message = message
        event.recommended_action = action
        self._pub_event.publish(event)
        self.get_logger().error(f'{code}: {message} -> {action}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
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

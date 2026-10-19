#!/usr/bin/env python3
"""2.5-D frontier exploration over OctoMap's projected occupancy grid.

The vehicle flies at a fixed local altitude while planning collision-aware XY
paths through already observed free space toward frontiers. This is deliberate:
it does not pretend that a 2-D grid planner is a general 3-D aerial planner.
"""

from __future__ import annotations

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool

from drone_exploration.grid_utils import (
    astar,
    choose_frontier_goal,
    cluster_mask,
    frontier_mask,
    grid_to_world,
    inflate_obstacles,
    nearest_free_cell,
    world_to_grid,
)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('flight_altitude_m', 2.5)
        self.declare_parameter('inflation_radius_m', 0.65)
        self.declare_parameter('min_frontier_cells', 6)
        self.declare_parameter('min_frontier_distance_m', 1.0)
        self.declare_parameter('replan_period_s', 2.0)
        self.declare_parameter('path_stride_cells', 2)
        self.declare_parameter('max_astar_expansions', 80000)
        self.declare_parameter('empty_frontier_cycles_to_complete', 5)

        self._altitude = float(self.get_parameter('flight_altitude_m').value)
        self._inflation_m = float(self.get_parameter('inflation_radius_m').value)
        self._min_frontier = int(self.get_parameter('min_frontier_cells').value)
        self._min_distance_m = float(self.get_parameter('min_frontier_distance_m').value)
        self._stride = max(1, int(self.get_parameter('path_stride_cells').value))
        self._max_astar = int(self.get_parameter('max_astar_expansions').value)
        self._empty_limit = max(1, int(self.get_parameter('empty_frontier_cycles_to_complete').value))

        self._enabled = False
        self._map: OccupancyGrid | None = None
        self._pose: PoseStamped | None = None
        self._empty_cycles = 0
        self._completion_sent = False

        self._path_pub = self.create_publisher(Path, '/autonomy/path', 10)
        self._complete_pub = self.create_publisher(Bool, '/exploration/complete', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/projected_map', self._on_map, map_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, '/exploration/enable', self._on_enable, 10)
        period = max(0.5, float(self.get_parameter('replan_period_s').value))
        self.create_timer(period, self._plan)
        self.get_logger().info('Frontier explorer ready (fixed-altitude 2.5-D planning).')

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _on_enable(self, msg: Bool):
        self._enabled = bool(msg.data)
        if self._enabled:
            self._empty_cycles = 0
            self._completion_sent = False
        self.get_logger().info(f'Exploration enabled={self._enabled}')

    def _plan(self):
        if not self._enabled or self._map is None or self._pose is None:
            return
        info = self._map.info
        if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
            return
        data = np.asarray(self._map.data, dtype=np.int16)
        if data.size != info.width * info.height:
            self.get_logger().warn('Projected map size does not match metadata.')
            return
        grid = data.reshape((info.height, info.width))
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = float(info.resolution)
        start_raw = world_to_grid(
            self._pose.pose.position.x, self._pose.pose.position.y, ox, oy, res)
        radius_cells = int(math.ceil(self._inflation_m / res))
        blocked = inflate_obstacles(grid, radius_cells)
        start = nearest_free_cell(grid, start_raw, blocked)
        if start is None:
            self.get_logger().debug('No known-free cell near current vehicle position yet.')
            return

        fmask = frontier_mask(grid) & ~blocked
        clusters = cluster_mask(fmask, self._min_frontier)
        min_distance_cells = self._min_distance_m / res
        # Try candidate clusters in utility order until one is reachable.
        remaining = list(clusters)
        chosen_path = []
        chosen_goal = None
        while remaining:
            goal = choose_frontier_goal(remaining, start, min_distance_cells)
            if goal is None:
                break
            path = astar(grid, start, goal, blocked, self._max_astar)
            if path:
                chosen_goal, chosen_path = goal, path
                break
            remaining = [c for c in remaining if goal not in c]

        if not chosen_path:
            self._empty_cycles += 1
            self.get_logger().debug(
                f'No reachable frontier ({self._empty_cycles}/{self._empty_limit}).')
            if self._empty_cycles >= self._empty_limit and not self._completion_sent:
                done = Bool(data=True)
                self._complete_pub.publish(done)
                self._completion_sent = True
                self.get_logger().info('Exploration complete: no reachable frontier remains.')
            return

        self._empty_cycles = 0
        self._completion_sent = False
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map.header.frame_id or 'map'
        sampled = chosen_path[::self._stride]
        if sampled[-1] != chosen_path[-1]:
            sampled.append(chosen_path[-1])
        for i, (gx, gy) in enumerate(sampled):
            x, y = grid_to_world(gx, gy, ox, oy, res)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = self._altitude
            # Face next path point when possible.
            if i + 1 < len(sampled):
                nx, ny = grid_to_world(sampled[i + 1][0], sampled[i + 1][1], ox, oy, res)
                yaw = math.atan2(ny - y, nx - x)
                p.pose.orientation.z = math.sin(0.5 * yaw)
                p.pose.orientation.w = math.cos(0.5 * yaw)
            else:
                p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self._path_pub.publish(msg)
        self.get_logger().info(
            f'Frontier path: {len(chosen_path)} grid cells, {len(msg.poses)} setpoints, goal={chosen_goal}.')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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

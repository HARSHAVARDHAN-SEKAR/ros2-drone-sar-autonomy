#!/usr/bin/env bash
set -euo pipefail

need_topic() {
  local topic="$1"
  if ! ros2 topic list | grep -Fxq "$topic"; then
    echo "[FAIL] missing topic: $topic" >&2
    return 1
  fi
  echo "[ OK ] $topic"
}

need_node() {
  local node="$1"
  if ! ros2 node list | grep -Fxq "$node"; then
    echo "[FAIL] missing node: $node" >&2
    return 1
  fi
  echo "[ OK ] $node"
}

need_node /flight_controller
need_node /mission_manager
need_node /frontier_explorer
need_node /depth_to_cloud
need_node /octomap_server
need_topic /mavros/state
need_topic /mavros/local_position/pose
need_topic /camera/color/image_raw
need_topic /camera/depth/image_raw
need_topic /camera/depth/points
need_topic /projected_map
need_topic /mission/state
need_topic /autonomy/path

echo "Smoke check passed. This checks graph availability, not flight safety or mission correctness."

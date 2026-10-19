#!/usr/bin/env bash
set -euo pipefail

compose=(docker compose)
if ! docker compose version >/dev/null 2>&1; then
  echo "docker compose is required" >&2
  exit 1
fi

echo "=== Docker services ==="
"${compose[@]}" ps

echo
echo "=== ROS/PX4 evidence ==="
"${compose[@]}" exec -T ros bash -lc '
  source /opt/ros/humble/setup.bash
  [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
  echo "-- MAVROS state --"
  timeout 8 ros2 topic echo /mavros/state --once || true
  echo "-- Mission state --"
  timeout 8 ros2 topic echo /mission/state --once || true
  echo "-- RGB topic --"
  timeout 8 ros2 topic hz /camera/color/image_raw --window 5 || true
  echo "-- Depth topic --"
  timeout 8 ros2 topic hz /camera/depth/image_raw --window 5 || true
  echo "-- Point cloud topic --"
  timeout 8 ros2 topic hz /camera/depth/points --window 5 || true
  echo "-- Projected map --"
  timeout 8 ros2 topic echo /projected_map --once || true
'

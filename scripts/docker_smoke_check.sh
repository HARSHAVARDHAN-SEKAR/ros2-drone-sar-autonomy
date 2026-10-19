#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

for service in px4 ros; do
  if ! docker compose ps --status running --services | grep -qx "$service"; then
    echo "[FAIL] $service container is not running." >&2
    exit 1
  fi
done

echo "[OK] PX4 and ROS containers are running."

docker compose exec ros bash -lc '
  set -euo pipefail
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 pkg prefix drone_bringup >/dev/null
  ros2 pkg prefix mavros >/dev/null
  ros2 pkg prefix octomap_server >/dev/null
  ros2 pkg prefix ros_gz_bridge >/dev/null
  echo "[OK] Required ROS packages are visible in the container."
'

echo "For live runtime checks after docker_launch.sh starts, verify /mavros/state, RGB, depth, point-cloud and /projected_map as documented in docs/DOCKER.md."

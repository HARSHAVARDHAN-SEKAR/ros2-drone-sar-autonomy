#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

command -v docker >/dev/null 2>&1 || { echo "[ERROR] Docker is not installed." >&2; exit 2; }
docker compose version >/dev/null 2>&1 || { echo "[ERROR] Docker Compose v2 is required." >&2; exit 2; }

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

echo "[1/2] Building ROS 2 autonomy image..."
docker compose build ros

echo "[2/2] Building the ROS 2 workspace inside the container..."
docker compose run --rm ros bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y --skip-keys=ros_gz_bridge
  colcon build --symlink-install
  source install/setup.bash
  python3 scripts/validate_workspace.py
  pytest -q src/drone_exploration/test src/drone_perception/test
'

echo "Docker image and ROS workspace build completed successfully."

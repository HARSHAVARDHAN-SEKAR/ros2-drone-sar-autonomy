#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

if [[ "${PX4_GUI:-0}" == "1" ]]; then
  export HEADLESS=""
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null
  else
    echo "[WARN] xhost not found; Gazebo GUI may not be able to open." >&2
  fi
fi

docker compose up -d ros px4

./scripts/configure_px4_sitl.sh

echo "Containers started."
echo "PX4 logs:  docker compose logs -f px4"
echo "ROS shell: ./scripts/docker_shell.sh"
echo "ROS launch: ./scripts/docker_launch.sh"

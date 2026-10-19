#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -f install/setup.bash ]]; then
  echo "[ERROR] Workspace is not built. Run ./scripts/docker_build.sh first." >&2
  exit 2
fi

docker compose exec ros bash -lc '
  set -euo pipefail
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 launch drone_bringup full.launch.py
'

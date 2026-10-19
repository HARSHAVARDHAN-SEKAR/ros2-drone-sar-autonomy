#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
  fi
  exec bash -i
'

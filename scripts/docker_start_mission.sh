#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

docker compose exec ros bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 topic pub --once /mission/command std_msgs/msg/String "{data: START}"
'

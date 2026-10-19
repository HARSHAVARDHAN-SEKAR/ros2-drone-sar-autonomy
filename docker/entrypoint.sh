#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# Source the local overlay only when the workspace has already been built.
if [[ -f /workspace/install/setup.bash ]]; then
  source /workspace/install/setup.bash
fi

exec "$@"

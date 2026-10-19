#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

python3 scripts/validate_workspace.py
PYTHONPATH="$PWD/src/drone_exploration:$PWD/src/drone_perception" \
  pytest -q src/drone_exploration/test src/drone_perception/test

for f in install_deps.sh docker/entrypoint.sh scripts/*.sh; do
  bash -n "$f"
done

python3 - <<'PY'
import json
from pathlib import Path
import yaml

compose = yaml.safe_load(Path('compose.yaml').read_text())
assert set(compose['services']) == {'px4', 'ros'}
assert compose['services']['px4']['image'] == 'px4io/px4-sitl-gazebo:v1.17.0-alpha1-1551-g381149fb01'
assert compose['services']['px4']['network_mode'] == 'host'
assert compose['services']['ros']['network_mode'] == 'host'
json.loads(Path('.devcontainer/devcontainer.json').read_text())
print('Docker metadata checks passed.')
PY

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  docker compose config --quiet
  echo "Docker Compose semantic validation passed."
else
  echo "Docker CLI not available; skipped 'docker compose config'."
fi

echo "All non-ROS static/unit checks passed."

#!/usr/bin/env python3
"""Fast static checks that do not require a ROS installation."""
from __future__ import annotations

import ast
import pathlib
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    yaml = None

ROOT = pathlib.Path(__file__).resolve().parents[1]
errors = []

for path in ROOT.rglob('*'):
    if any(part in {'build', 'install', 'log', '.git', '__pycache__'} for part in path.parts):
        continue
    if '{' in path.name or '}' in path.name:
        errors.append(f'malformed brace-expanded path: {path.relative_to(ROOT)}')
    if path.suffix == '.py':
        try:
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        except Exception as exc:
            errors.append(f'python: {path.relative_to(ROOT)}: {exc}')
    elif path.name == 'package.xml' or path.suffix in {'.xacro', '.urdf'}:
        try:
            ET.parse(path)
        except Exception as exc:
            errors.append(f'xml: {path.relative_to(ROOT)}: {exc}')
    elif path.suffix in {'.yaml', '.yml'} and yaml is not None:
        try:
            yaml.safe_load(path.read_text(encoding='utf-8'))
        except Exception as exc:
            errors.append(f'yaml: {path.relative_to(ROOT)}: {exc}')

required = [
    'README.md', 'LICENSE', 'src/drone_bringup/rviz/drone_autonomy.rviz',
    'src/drone_bringup/config/autonomy.yaml', 'src/drone_bringup/config/octomap.yaml',
    'src/drone_description/urdf/drone.urdf.xacro',
    'docs/ARCHITECTURE.md', 'docs/TEST_PLAN.md', 'docs/DOCKER.md',
    'compose.yaml', 'docker/Dockerfile', 'docker/entrypoint.sh', '.devcontainer/devcontainer.json',
    'scripts/docker_build.sh', 'scripts/docker_up.sh', 'scripts/docker_launch.sh',
    'scripts/docker_start_mission.sh', 'scripts/docker_shell.sh', 'scripts/docker_stop.sh',
    'scripts/docker_reset.sh', 'scripts/docker_smoke_check.sh',
]
for rel in required:
    if not (ROOT / rel).exists():
        errors.append(f'missing required file: {rel}')

if errors:
    print('VALIDATION FAILED')
    for err in errors:
        print(' -', err)
    sys.exit(1)
print(f'VALIDATION PASSED: {ROOT}')

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

echo "Stopping containers and removing project Docker volumes..."
docker compose down -v --remove-orphans
rm -rf build install log

echo "Reset complete. Rebuild with ./scripts/docker_build.sh"

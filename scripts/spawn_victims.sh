#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

WORLD="${PX4_GZ_WORLD:-walls}"
SERVICE="/world/${WORLD}/create"

echo "Waiting for Gazebo create service: ${SERVICE}"

ready=0
for _ in $(seq 1 30); do
    if docker compose exec -T px4 gz service -l \
        | grep -qx "${SERVICE}"; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "${ready}" != "1" ]]; then
    echo "[ERROR] Gazebo create service not found." >&2
    docker compose exec -T px4 gz service -l | grep "/world/" || true
    exit 1
fi

for victim in victim_1 victim_2 victim_3; do
    file="/workspace/simulation/victims/${victim}.sdf"

    echo "Spawning ${victim}..."

    docker compose exec -T px4 \
        gz service \
        -s "${SERVICE}" \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 30000 \
        --req "sdf_filename: \"${file}\", allow_renaming: false"
done

echo
echo "[OK] Gazebo victim spawn requests completed."
echo "Check the Gazebo Entity Tree for victim_1, victim_2 and victim_3."

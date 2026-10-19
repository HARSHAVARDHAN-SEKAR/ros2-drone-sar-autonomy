#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

echo "Waiting for PX4 SITL parameter interface..."

ready=0

for _ in $(seq 1 30); do
    if docker compose exec -T px4 bash -lc \
        '/opt/px4-gazebo/bin/px4-param show NAV_DLL_ACT >/dev/null 2>&1'
    then
        ready=1
        break
    fi

    sleep 1
done

if [[ "$ready" != "1" ]]; then
    echo "[ERROR] PX4 parameter interface did not become ready." >&2
    exit 1
fi

docker compose exec -T px4 bash -lc '
    /opt/px4-gazebo/bin/px4-param set NAV_DLL_ACT 0
    /opt/px4-gazebo/bin/px4-param set EKF2_MAG_TYPE 0
    /opt/px4-gazebo/bin/px4-param save
'

echo "[OK] PX4 autonomous SITL parameters configured."

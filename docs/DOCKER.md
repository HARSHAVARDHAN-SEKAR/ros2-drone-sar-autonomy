# Docker workflow

This repository is Docker-first on Linux. The host only needs Docker Engine, Docker Compose v2, and an X11 session if Gazebo/RViz GUI windows are desired.

## Container split

- `px4`: official `px4io/px4-sitl-gazebo:v1.17.0-alpha1-1551-g381149fb01`, running PX4 SITL + Gazebo Harmonic with `gz_x500_depth`.
- `ros`: project image based on ROS 2 Humble / Ubuntu 22.04 with MAVROS, OctoMap, ROS↔Gazebo Harmonic bridge, RViz, OpenCV, and Ultralytics.

Both services use host networking. This is deliberate: PX4 MAVLink UDP and Gazebo Transport discovery work without fragile container-to-container port/address rewriting. The supported target for this workflow is Linux.

## First build

```bash
./scripts/docker_build.sh
```

This builds the ROS image, resolves workspace dependencies, runs `colcon build --symlink-install`, then runs the static/unit checks.

## Start PX4 and the development container

Headless Gazebo is the default:

```bash
./scripts/docker_up.sh
```

To show the Gazebo GUI:

```bash
PX4_GUI=1 ./scripts/docker_up.sh
```

The PX4 service defaults are:

```text
PX4_SIM_MODEL=gz_x500_depth
PX4_GZ_WORLD=walls
```

Copy `.env.example` to `.env` to persist overrides.

## Check PX4

```bash
docker compose logs -f px4
```

Basic container/package smoke check:

```bash
./scripts/docker_smoke_check.sh
```

On the host, QGroundControl can connect through PX4's normal host-network MAVLink endpoints.

## Launch the ROS stack

In a new host terminal:

```bash
./scripts/docker_launch.sh
```

This launches MAVROS, Gazebo sensor bridges, TF, depth point-cloud conversion, OctoMap, mission management, frontier exploration, flight control, victim detection, and safety monitoring.

## Verify before arming

Open a ROS shell:

```bash
./scripts/docker_shell.sh
```

Then run:

```bash
ros2 topic echo /mavros/state --once
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/points
ros2 topic echo /projected_map --once
```

Do not start the mission until MAVROS is connected and the RGB-D / mapping topics are healthy.

## Start the autonomous mission

```bash
./scripts/docker_start_mission.sh
```

The mission still follows the same explicit safety policy: launch does not automatically arm the vehicle.

## RViz

From the ROS container shell:

```bash
rviz2 -d /workspace/src/drone_bringup/rviz/drone_autonomy.rviz
```

The host must permit X11 access. `PX4_GUI=1 ./scripts/docker_up.sh` performs the root X11 allowance needed by the PX4/Gazebo container. The ROS container uses the host UID/GID.

## Stop

```bash
./scripts/docker_stop.sh
```

## Full reset

```bash
./scripts/docker_reset.sh
```

This removes the Compose volumes plus local `build/`, `install/`, and `log/` directories. It intentionally does not delete source code.

## VS Code Dev Containers

The repository includes `.devcontainer/devcontainer.json`. Open the repository in VS Code and choose **Dev Containers: Reopen in Container** to work directly in the `ros` service.

## Notes

- Docker Desktop/macOS/Windows networking and X11 behavior differ. This project intentionally targets a native Linux Docker host for PX4/Gazebo simulation.
- The first ROS image build downloads the full perception stack and can be large because Ultralytics/PyTorch are included.
- `network_mode: host` avoids mapping MAVLink/Gazebo discovery ports manually. Do not run another PX4 instance using the same ports at the same time.
- PX4 simulation state is stored in the `px4_state` named volume. Use `docker_reset.sh` if stale PX4 parameters cause unexpected behavior.

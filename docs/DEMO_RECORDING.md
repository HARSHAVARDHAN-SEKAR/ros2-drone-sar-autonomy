# Demo Recording Guide

This repository is designed to be shown with evidence, not only code screenshots.
The recommended public demo is a 60–90 second screen recording showing the autonomy pipeline running in simulation.

## What the video should prove

Show these items in one continuous demo where possible:

1. Docker services are running.
2. PX4 SITL and Gazebo start with the `gz_x500_depth` vehicle.
3. MAVROS reports a connected flight controller.
4. RGB and depth topics are alive.
5. The mission receives `START`.
6. The drone arms and takes off in OFFBOARD mode.
7. The mapping/exploration stack is active.
8. A victim/person candidate is detected and localized when present.
9. The mission state is visible.
10. The drone returns/lands or the video clearly shows the autonomous mission progressing.

## Recommended screen layout

Use OBS Studio at 1920x1080, 30 fps.

- Left ~70%: Gazebo or RViz.
- Right ~30%: terminal showing mission state and ROS topic output.
- Keep the mouse pointer movement minimal.
- Hide unrelated browser tabs, usernames, tokens, SSH keys, and personal notifications.

## Before recording

From the project root:

```bash
./scripts/docker_build.sh
./scripts/docker_up.sh
```

For a visible Gazebo GUI:

```bash
PX4_GUI=1 ./scripts/docker_up.sh
```

In another terminal:

```bash
./scripts/docker_smoke_check.sh
```

Then launch the ROS autonomy stack:

```bash
./scripts/docker_launch.sh
```

## Evidence terminal

Open a separate shell:

```bash
./scripts/docker_shell.sh
```

Useful commands to keep visible during the recording:

```bash
ros2 topic echo /mavros/state
```

or:

```bash
ros2 topic echo /mission/state
```

Before starting the mission, verify sensor topics once:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/points
```

Verify the generated projected map:

```bash
ros2 topic echo /projected_map --once
```

## Start the mission

Only after PX4/MAVROS and sensor checks are healthy:

```bash
./scripts/docker_start_mission.sh
```

## Suggested 75-second edit

- 0–5 s: title card — “Autonomous Search-and-Rescue Drone | ROS 2 + PX4 + Docker”.
- 5–15 s: Docker Compose/PX4 boot and Gazebo X500 depth vehicle.
- 15–25 s: MAVROS connected + RGB/depth topic evidence.
- 25–40 s: `START` command, OFFBOARD/arming, autonomous takeoff.
- 40–58 s: exploration/mapping in Gazebo/RViz.
- 58–68 s: YOLO victim-candidate detection / 3-D localization when available.
- 68–75 s: landing/mission status + GitHub repository name.

Do not add a detection overlay if the run did not actually produce one. It is better to publish a shorter honest demo than a misleading “complete autonomy” claim.

## Recommended export

- MP4 (H.264)
- 1920x1080
- 30 fps
- 8–12 Mbps video bitrate
- Keep the LinkedIn upload under roughly 90 seconds for recruiter-friendly viewing.

## Evidence to add to the repository after recording

Save lightweight media here:

- `assets/demo/architecture.png`
- `assets/demo/gazebo_takeoff.png`
- `assets/demo/octomap.png`
- `assets/demo/victim_detection.png` (only if genuinely produced)

For a large MP4, use a GitHub Release asset, YouTube/LinkedIn video, or Git LFS rather than committing a very large binary directly to normal Git history.

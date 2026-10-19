# Staged Test and Acceptance Plan

Do not jump directly from a successful build to autonomous flight. Validate in layers.

## Stage 0 — static repository validation

```bash
python3 scripts/validate_workspace.py
pytest -q src/drone_exploration/test src/drone_perception/test
```

Acceptance: Python compiles, XML/YAML parse, referenced local files exist, and pure planning/depth tests pass.

## Stage 1 — PX4 + MAVROS connectivity

Start `make px4_sitl gz_x500_depth`, then `simulation.launch.py`.

Check:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 service list | grep -E '/mavros/(cmd/arming|set_mode)'
```

Acceptance: FCU connected, local pose updates, arming/mode services exist.

## Stage 2 — RGB-D bridge

```bash
gz topic -l | grep -E 'IMX214|depth_camera'
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
```

If the Gazebo scoped RGB topic differs, pass the actual path using the `gz_rgb_image_topic` and `gz_rgb_info_topic` launch arguments.

Acceptance: RGB and depth streams are non-zero rate and CameraInfo is available.

## Stage 3 — TF and mapping

```bash
ros2 run tf2_ros tf2_echo map camera_optical_frame
ros2 topic hz /camera/depth/points
ros2 topic echo /projected_map --once
```

Acceptance: continuous map-to-camera transform, point cloud published, OctoMap projected grid populated.

## Stage 4 — control-only takeoff

Launch the autonomy stack but keep exploration disabled. Publish START and observe takeoff.

Acceptance: OFFBOARD is entered only after setpoint pre-streaming; drone arms, climbs to configured altitude, and holds without control oscillation.

Abort test:

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: ABORT}"
```

Acceptance: AUTO.LAND is requested and the vehicle disarms after landing.

## Stage 5 — exploration without perception

Run in an obstacle-containing world. Inspect:

```bash
ros2 topic echo /autonomy/path
ros2 topic echo /exploration/complete
```

Acceptance: paths stay in known-free cells, respect obstacle inflation, and exploration eventually completes or reaches its configured timeout.

## Stage 6 — victim perception

Place a realistic human model in camera view. Inspect annotated detections and map locations:

```bash
ros2 topic echo /victims/detected
ros2 topic echo /victims/poses
```

Acceptance: one visible person does not create a new ID every frame; reported depth is finite and plausible; marker position is consistent in `map`.

## Stage 7 — full mission

Record a rosbag:

```bash
mkdir -p results/raw
ros2 bag record -o results/raw/full_mission \
  /mission/state /mavros/state /mavros/local_position/pose \
  /autonomy/path /projected_map /victims/detected /safety/event
```

Acceptance: TAKEOFF -> EXPLORE -> optional INSPECT -> RETURN -> COMPLETE, with the drone landed/disarmed.

## Metrics to report on GitHub

Only publish measured values. Recommended metrics: mission completion rate, map area/volume explored, path length, minimum obstacle clearance, number of unique candidate detections, duplicate-detection rate, inference latency/FPS, and total mission time.

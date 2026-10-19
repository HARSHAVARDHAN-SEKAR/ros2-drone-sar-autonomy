# GitHub Publication Checklist

Recommended repository name: `ros2-drone-sar-autonomy`

## Before the first push

```bash
python3 scripts/validate_workspace.py
pytest -q src/drone_exploration/test src/drone_perception/test
```

Do not commit ROS build products, PX4 source/build directories, model weight files, rosbag databases, or large videos. `.gitignore` already covers the common cases.


## One-command publication with GitHub CLI

If `gh auth status` works on your laptop, the repository includes a guarded publication script that runs the tests, initializes Git, creates the public repository if needed, pushes `main`, sets the description, and adds topics:

```bash
./scripts/publish_github.sh
```

The script targets `HARSHAVARDHAN-SEKAR/ros2-drone-sar-autonomy`.

## Manual publication

### Create the local history

```bash
cd ~/ros2-drone-sar-autonomy
git init -b main
git config user.name "Harshavardhan Coimbatore Sekar"
git add .
git commit -m "feat: add ROS 2 PX4 search-and-rescue autonomy stack"
```

### Publish after creating an empty GitHub repository

```bash
git remote add origin git@github.com:HARSHAVARDHAN-SEKAR/ros2-drone-sar-autonomy.git
git push -u origin main
```

HTTPS can be used instead of SSH if that is how GitHub authentication is configured on the laptop.

## Repository description

Simulation-first autonomous SAR drone stack with PX4, ROS 2, Gazebo Harmonic, MAVROS, OctoMap, frontier exploration, A* planning, depth perception and YOLO victim-candidate detection.

## Topics

`ros2` `px4` `gazebo` `mavros` `uav` `drone` `autonomous-drone` `octomap` `path-planning` `computer-vision` `yolo` `robotics` `search-and-rescue`

## Evidence to add after testing

Add only genuine results: one short mission GIF/video, RViz screenshot with 3-D OctoMap/projected map, annotated victim detection, a rosbag-derived metrics table, and the tested software/hardware versions. Never label untested behavior as validated.

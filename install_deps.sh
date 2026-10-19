#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "[ERROR] Source ROS 2 Humble first: source /opt/ros/humble/setup.bash" >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-pytest \
  python3-yaml \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-octomap-server \
  ros-humble-ros-gzharmonic \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs

# MAVROS requires GeographicLib geoid datasets.
sudo ros2 run mavros install_geographiclib_datasets.sh

python3 -m pip install --user --upgrade "numpy>=1.23,<2" "ultralytics>=8.2,<9"

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys=ros_gz_bridge

echo "Dependencies installed. Build with: colcon build --symlink-install"

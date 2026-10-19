# LinkedIn Release Notes

## Project name

ROS 2 Autonomous Search-and-Rescue Drone with PX4, 3-D Mapping and Vision

## One-line description

Dockerized autonomous drone stack using ROS 2 Humble, PX4 SITL, Gazebo Harmonic, MAVROS, OctoMap, frontier exploration, A* planning, RGB-D perception and YOLO-based victim-candidate localization.

## What to link

Publish the GitHub repository and attach a short real simulation recording. Prefer one strong video over several screenshots.

## Claims you can safely make now

- Docker-first ROS 2/PX4 development environment.
- PX4 OFFBOARD flight-control integration through MAVROS.
- Mission state machine covering takeoff, exploration, inspection, return and landing logic.
- RGB-D processing and YOLO-based person/victim-candidate detection code.
- OctoMap-based 3-D mapping and projected occupancy-grid exploration interface.
- Frontier selection and A* path planning implementation.
- Safety monitoring for battery, altitude, geofence and FCU connection.
- Unit/static tests included.

## Claims to make only after the recorded run proves them

- “Fully autonomous end-to-end mission completed.”
- “Victim detected and localized successfully.”
- “Returned and landed autonomously.”
- FPS, latency, map accuracy, detection accuracy, mission time or other numeric benchmark claims.

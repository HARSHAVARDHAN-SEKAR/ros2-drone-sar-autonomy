# Changelog

## 0.3.0 — 2026-08-15 — Docker-first runtime

- Added a two-service Docker Compose architecture: official PX4 v1.17.0 Gazebo SITL + a project ROS 2 Humble image.
- Added Gazebo Harmonic / Humble bridge installation through the OSRF repository.
- Added MAVROS GeographicLib datasets at image-build time.
- Added host UID/GID mapping, X11 GUI support, named caches/state volumes, and host networking for MAVLink/Gazebo discovery.
- Added Docker build/start/launch/shell/mission/stop/reset scripts.
- Added VS Code Dev Container configuration and Docker-specific documentation.
- Promoted Docker to the recommended workflow while retaining the native Ubuntu path.

## 0.2.0 — 2026-08-15

- rebuilt the prototype around PX4 v1.17 + Gazebo Harmonic `gz_x500_depth`;
- removed the duplicate simulated sensor vehicle;
- added single-owner MAVROS flight control with OFFBOARD pre-streaming;
- added OctoMap depth-cloud mapping and projected-grid frontier exploration;
- added obstacle inflation and A* path planning;
- added RGB/depth-intrinsics victim-candidate localization with spatial deduplication;
- added mission and safety state machines;
- added static validation, pure-Python tests, GitHub CI, RViz, test plan and publication documentation;
- corrected ROS sensor QoS and repeat-takeoff pre-stream reset hazards.

## v0.4.0 - Portfolio release preparation

- Added LinkedIn/demo recording guide.
- Added evidence collection helper for Docker/MAVROS/ROS topics.
- Added recruiter-facing release notes and honest-claims checklist.
- Added demo-asset directory guidance.

# Architecture and Engineering Decisions

## Mission objective

The project demonstrates a simulation-first search-and-rescue mission:

`IDLE -> TAKEOFF -> EXPLORE -> INSPECT -> RETURN -> COMPLETE`

`LAND` and `ABORT` are pre-emptive states. A detected person is deliberately called a **victim candidate**; visual person detection alone cannot establish medical status.

## Control ownership

`flight_controller` is the only project node allowed to:

- request PX4 arming/disarming through MAVROS;
- request PX4 flight modes;
- publish local-position OFFBOARD setpoints.

All other nodes publish intent. This prevents multiple ROS nodes from fighting over vehicle control.

## PX4 / simulator model

The simulation uses PX4's `gz_x500_depth` target rather than spawning a second ROS URDF into Gazebo. PX4's model composes the X500 vehicle with an OAK-D Lite camera through a fixed joint. The ROS xacro in `drone_description` exists only for RViz and the fixed TF chain.


## Docker deployment boundary

The repository uses two Linux containers with host networking:

```text
Host Ubuntu + Docker
├── px4 service
│   └── official px4io/px4-sitl-gazebo:v1.17.0
│       ├── PX4 SITL
│       ├── Gazebo Harmonic
│       └── x500_depth + OAK-D simulation
└── ros service
    └── ROS 2 Humble project image
        ├── MAVROS
        ├── ros_gz_bridge
        ├── OctoMap
        ├── mission / control / exploration / perception
        └── RViz / YOLO tooling
```

Host networking is intentional. MAVLink UDP and Gazebo Transport discovery remain on the Linux host network, avoiding brittle port/address translation between PX4, Gazebo and MAVROS. The workspace is bind-mounted at `/workspace`, so code edits on the host are immediately visible inside the ROS container.

## Coordinate frames

```text
map
└── base_link              dynamic, from MAVROS local pose
    └── camera_link        fixed OAK-D mounting pose
        └── camera_sensor_link
            └── camera_optical_frame
```

`depth_to_cloud` back-projects depth pixels directly into the optical convention (X right, Y down, Z forward), so the resulting cloud is stamped in `camera_optical_frame`. OctoMap transforms it into `map` through TF.

## 3-D map and 2.5-D planning

OctoMap is the 3-D mapping component. Its `/projected_map` OccupancyGrid is used for exploration planning. This is an intentional 2.5-D architecture:

- 3-D perception/map: depth camera -> PointCloud2 -> OctoMap;
- XY planning: projected known-free/occupied/unknown grid;
- Z control: fixed exploration altitude;
- collision margin: occupied-cell inflation;
- path search: 8-connected A* through known free cells only.

This is suitable for a portfolio SAR simulation with approximately level flight. It is not presented as a general 3-D kinodynamic UAV planner.

## Frontier exploration

A frontier is a known-free grid cell with an unknown 4-neighbour. Frontier cells are clustered, small clusters are rejected, and a utility score favors large nearby clusters. If the best cluster is unreachable, the explorer tries other clusters. Several consecutive cycles with no reachable frontier produce `/exploration/complete`.

## Victim-candidate localization

1. YOLO detects the `person` class in the RGB image.
2. RGB and depth frames are approximately synchronized.
3. The RGB bounding-box center ray is reprojected using RGB and depth intrinsics. The implementation does not assume both images have the same resolution.
4. Robust median depth is sampled around the projected depth pixel.
5. The pixel is back-projected into a 3-D optical-frame point.
6. TF transforms the point into `map`.
7. Detections within the spatial deduplication radius reuse the same candidate ID.

## Safety layer

`safety_monitor` publishes semantic `SafetyEvent` messages for:

- low/critical battery;
- geofence radius;
- maximum altitude;
- FCU connectivity state.

`mission_manager` interprets the recommended action and requests RTH or landing from `flight_controller`. The safety node itself does not issue PX4 actuator or mode commands.

## Production extensions

Useful next steps are: 3-D motion planning (OMPL/Voxblox/ESDF), VIO (VINS-Fusion/ORB-SLAM3 or PX4-compatible estimator integration), multi-view victim confirmation, semantic mapping, behavior trees, coverage metrics, and hardware-in-the-loop validation.

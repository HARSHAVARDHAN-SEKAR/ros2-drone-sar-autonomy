# Original Prototype Audit

This repository is a reconstruction of the initial prototype rather than a cosmetic cleanup. The audit found several architecture-level issues that could produce a demo that looks plausible while behaving incorrectly.

## Corrected blockers

| Original issue | Why it mattered | Resolution in this repository |
|---|---|---|
| Literal brace-expansion directories such as `src/{drone_bringup/...` and `{models,docker}` | Invalid workspace structure and misleading source tree | Removed; every package now has a normal ROS 2 layout |
| Gazebo Classic launch text mixed with Gazebo Harmonic claims | Simulator stack did not match the documented architecture | Standardized on PX4 v1.17 + Gazebo Harmonic |
| PX4 aircraft plus a second spawned URDF sensor carrier | Sensors could exist on a different simulated body from the vehicle PX4 controls | Uses PX4 `gz_x500_depth`; ROS xacro is RViz/TF only |
| Mission and controller both issuing PX4 control requests | Competing control authority and race conditions | `flight_controller` is the only PX4/MAVROS control owner |
| Ground-robot Nav2/explore_lite assumptions | `/cmd_vel` competition and misleading “3-D drone exploration” claim | Dedicated fixed-altitude frontier explorer + A* over OctoMap projection |
| Blocking sleeps in mission logic | ROS callbacks could stall at state transitions | Timer/timestamp-driven state transitions |
| Depth always divided by 1000 | PX4 OAK-D depth is floating-point depth; distances could become wrong by 1000x | Encoding-aware depth conversion |
| RGB bounding-box pixel used directly in lower-resolution depth image | RGB and depth pixel grids differ | Intrinsics-based RGB-ray to depth-camera projection |
| New victim ID every frame | Inflated victim counts | Spatial map-frame deduplication |
| VIO/SLAM placeholder presented too strongly | Suggested localization capability beyond what was validated | Default simulation uses PX4/MAVROS local pose; VIO is documented as a future extension |
| Missing referenced RViz file | Launch/documentation inconsistency | A valid RViz configuration is included |

## What is deliberately *not* claimed

- no real-drone flight validation;
- no medical victim classification;
- no general-purpose 3-D kinodynamic motion planning;
- no measured mission success rate until simulation runs are recorded;
- no guarantee that a synthetic human model will be detected by a pretrained person detector without perception-domain testing.

The README and test plan are written to distinguish implemented architecture from measured validation evidence.

# uav_navigation_ros

ROS 2 adapters for the pure C++ navigation core.

## Nodes

`global_planner_node` loads static box/cylinder collision geometry from the
Gazebo SDF, builds an inflated 2.5D grid, consumes odometry and RViz goals, and
publishes the global costmap and A* path.

`local_navigation_node` composes the C++ MPPI optimizer, trajectory safety
checker and command conditioner in one process. It consumes:

```text
/localization/odometry
/planning/global_path
/perception/obstacles
```

and publishes only the verified output command plus observability topics:

```text
/control/safe_velocity_command
/planning/mppi/predicted_path
/planning/mppi/cost_samples
/diagnostics
```

The node validates frames and freshness before planning. Its explicit modes
are `WAITING_FOR_STATE`, `WAITING_FOR_PATH`, `WAITING_FOR_OBSTACLES`, `ACTIVE`,
`HOLD_STALE`, `NO_SAFE_TRAJECTORY`, `PLANNER_TIMEOUT`, `GOAL_REACHED` and
`INVALID_INPUT`. A result that arrives after the configured deadline is not
published. The full timed trajectory remains in-process; `nav_msgs/Path` is
only a visualization.

For the monocular profile, `min_obstacle_points` requires enough finite points
after cloud filtering and `max_obstacle_source_age_s` checks the cloud's source
timestamp against the ROS clock. Both guards are disabled by default for the
baseline profile and enabled in `config/monocular.yaml`. An insufficient or
replayed cloud causes a hold before MPPI sampling and publishes no new command.

`autopilot_adapter_node` is the C++ boundary between navigation and MAVROS. It
validates the safe ENU command, monitors `/mavros/state`, and republishes only
while the FCU is connected, armed, in `GUIDED`, and the command is fresh.
MAVROS owns ENU-to-NED conversion and MAVLink transport. The adapter never arms
or changes mode. Its states are `DISCONNECTED`, `CONNECTED_NOT_READY`, `READY`,
`STREAMING`, `STALE_COMMAND` and `FAULT`.

`CommandTranslator`, `ConnectionMonitor` and `IAutopilotBackend` keep command
validation, readiness policy and transport boundary independently testable.
The previous Python MAVLink bridge is no longer installed or used by launch.

`monocular_depth_node` receives 640×360 RGB images and runs a fixed-shape
Depth Anything V2 Metric Outdoor Small ONNX model through OpenCV DNN. It
publishes an occupied-point cloud with the camera timestamp and frame only
when image age, inference time, and total processing time pass their gates.
Diagnostics report both latency measurements and process CPU usage as a
percentage of one logical core. The model is exported offline with
`scripts/export_depth_anything_onnx.py`; Python is not used for live inference.
The monocular launch and its current limitations are documented in
`../docs/JETSON_CPP_RUNTIME_VI.md`.

Runtime parameters are in `uav_navigation_bringup/config/navigation.yaml` for
the baseline and `uav_navigation_bringup/config/monocular.yaml` for the RGB
experiment.

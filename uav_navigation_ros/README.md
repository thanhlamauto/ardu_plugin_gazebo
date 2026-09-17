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

`autopilot_adapter_node` is the C++ boundary between navigation and MAVROS. It
validates the safe ENU command, monitors `/mavros/state`, and republishes only
while the FCU is connected, armed, in `GUIDED`, and the command is fresh.
MAVROS owns ENU-to-NED conversion and MAVLink transport. The adapter never arms
or changes mode. Its states are `DISCONNECTED`, `CONNECTED_NOT_READY`, `READY`,
`STREAMING`, `STALE_COMMAND` and `FAULT`.

`CommandTranslator`, `ConnectionMonitor` and `IAutopilotBackend` keep command
validation, readiness policy and transport boundary independently testable.
The previous Python MAVLink bridge is no longer installed or used by launch.

All runtime parameters are owned by
`uav_navigation_bringup/config/navigation.yaml`.

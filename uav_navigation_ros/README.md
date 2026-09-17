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

`safe_twist_to_mavlink.py` is the temporary M5 SITL bridge. It converts the
verified ENU velocity/yaw-rate command to MAVLink local NED. It does not arm,
take off, change mode or invent a command when the C++ node stops publishing.
The C++ autopilot adapter is the scope of M6.

All runtime parameters are owned by
`uav_navigation_bringup/config/navigation.yaml`.

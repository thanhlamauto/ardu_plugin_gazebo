# uav_navigation_bringup

Launch, ROS parameters and RViz configuration for the C++ navigation stack.

After building and sourcing the workspace, start Gazebo, ROS–Gazebo bridges,
the C++ global/local planners and RViz with one command:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml
```

Headless:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml \
  enable_gazebo_gui:=false enable_rviz:=false
```

The temporary M5 Python SITL bridge is disabled by default. Enable it only
when ArduPilot SITL is already listening on `tcp:127.0.0.1:5762`:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml \
  enable_autopilot_adapter:=true
```

Arm and take off in GUIDED before sending the RViz **2D Goal Pose**. If a path
already exists, do not start the command bridge until takeoff has completed;
otherwise ground-level velocity setpoints can interrupt the takeoff command.
The bridge never auto-arms and the planner never auto-lands.

Useful feature flags:

```text
enable_gazebo:=true
enable_gazebo_gui:=true
enable_bridge:=true
enable_global_planner:=true
enable_local_navigation:=true
enable_autopilot_adapter:=false
enable_rviz:=true
```

`config/navigation.yaml` is the source of truth for planner, dynamics, safety,
conditioner, deadline, visualization and adapter parameters. The
`local_navigation_node` publishes MPPI paths and cost markers itself on a
separate 5 Hz timer, outside the 10 Hz control callback.

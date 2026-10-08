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

The M6 C++ adapter and MAVROS are disabled by default. Enable both through the
adapter flag when ArduPilot SITL is listening on `tcp:127.0.0.1:5762`:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml \
  enable_autopilot_adapter:=true
```

Arm and take off in GUIDED before sending the RViz **2D Goal Pose**. If a path
already exists, do not enable the adapter until takeoff has completed;
otherwise ground-level velocity setpoints can interrupt the takeoff command.
The adapter never auto-arms or changes mode, and the planner never auto-lands.

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

For the experimental RGB-only C++ runtime, first export the pinned Depth
Anything ONNX model on a development machine as described in
[`docs/JETSON_CPP_RUNTIME_VI.md`](../docs/JETSON_CPP_RUNTIME_VI.md), then run:

```bash
ros2 launch uav_navigation_bringup monocular_sim.launch.xml \
  model_file:=/path/to/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx \
  depth_backend:=cpu enable_gazebo_gui:=false enable_rviz:=false
```

The RGB bridge, depth inference, MPPI, safety checker, and autopilot adapter
in this launch are C++; `monocular_shadow.launch.xml` is C++ as well. The
model exporter and evaluation scripts remain offboard. The default 100 ms
depth deadline rejects late CPU results, and autonomous flight is not
validated by this launch.
The monocular planner profile also requires at least 12 finite cloud points
and checks the source timestamp; insufficient or replayed clouds hold the
planner before sampling.

M7 performance validation uses the frozen `config/m7_baseline.yaml`, which
disables trajectory visualization and sets the 10 Hz deadline to 100 ms. The
scenario runner, metrics and fault variants are documented in
`../docs/M7_VALIDATION.md` in the source repository.

MAVROS requires the GeographicLib `egm96-5` geoid dataset. On Ubuntu, install
the MAVROS package and run its `install_geographiclib_datasets.sh` helper once
before launch.

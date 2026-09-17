# uav_navigation_bringup

Launch, ROS parameters and RViz configuration for the C++ navigation stack.

After building and sourcing the workspace, Milestone 1 starts Gazebo, the
Gazebo-to-ROS odometry bridge, C++ global planner and RViz with one command:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml
```

Gazebo server and GUI are separate launch processes, which works on both
Ubuntu and macOS. For headless operation, add `enable_gazebo_gui:=false
enable_rviz:=false`.

Use RViz **2D Goal Pose** to publish `/goal_pose`. The costmap and A* path are
latched, so RViz can start after the planner without losing them.

Feature flags keep unfinished milestones disabled:

```text
enable_global_planner:=true
enable_local_navigation:=false
enable_autopilot_adapter:=false
enable_visualizer:=false
```

For planner-only testing against an already-running simulator:

```bash
ros2 launch uav_navigation_bringup sim.launch.xml \
  enable_gazebo:=false enable_bridge:=false
```

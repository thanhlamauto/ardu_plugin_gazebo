# uav_navigation_ros

ROS 2 adapters around `uav_navigation_core`.

Milestone 1 provides `global_planner_node`. It reads static box/cylinder
collisions from the configured Gazebo SDF world, conservatively rasterizes a
2.5D inflated cost grid, consumes odometry and RViz goals, and publishes:

- `/planning/global_costmap` (`nav_msgs/OccupancyGrid`)
- `/planning/global_path` (`nav_msgs/Path`)
- `/planning/global_planner/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

The SDF parser is an adapter in this package. No SDF type crosses into the
core library. Goals outside the map enlarge the grid; blocked and unreachable
goals return `NO_PATH` and publish an empty path without terminating the node.

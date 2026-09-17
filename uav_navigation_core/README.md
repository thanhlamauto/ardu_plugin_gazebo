# uav_navigation_core

Pure C++17 planning library. It has no ROS, Gazebo or SDF dependency.

Milestone 1 implements deterministic 2D A* over `CostGrid2D`, including
optional diagonal motion, corner-cut prevention, inflated traversal costs and
bounded node expansion. Simulation and hardware adapters construct the grid.

Standalone build and tests:

```bash
cmake -S uav_navigation_core -B build/uav_navigation_core \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/uav_navigation_core
ctest --test-dir build/uav_navigation_core --output-on-failure
```

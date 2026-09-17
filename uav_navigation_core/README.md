# uav_navigation_core

Architecture-review skeleton for a ROS/Gazebo-independent C++ navigation
library. Only data and abstract interfaces exist. A*, MPPI, safety and command
conditioning remain in the Python reference until the design is approved.

The public API has no ROS, Gazebo or MAVLink header/runtime dependency. This
skeleton still uses `ament_cmake` for ROS workspace packaging and defines an
`INTERFACE` target, so it does not yet produce `libuav_navigation_core.so`.
Standalone CMake packaging is a deployment requirement after design approval.

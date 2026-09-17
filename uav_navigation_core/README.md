# uav_navigation_core

Pure C++17 planning library. It has no ROS, Gazebo or SDF dependency.

Implemented core components:

- deterministic 2D A* over `CostGrid2D`, including optional diagonal motion,
  corner-cut prevention, inflated traversal costs and bounded node expansion;
- `TrajectorySafetyChecker`, with continuous swept collision and straight-stop
  feasibility against dynamic point obstacles and an abstract static geometry;
- `VelocityCommandConditioner`, with Python-baseline-compatible filtering,
  vector XY slew limiting, scalar Z/yaw limiting, bounds and reset semantics.
- deterministic MPPI multirotor dynamics and single/batch rollout APIs, including
  command conditioning, velocity response, optional XY acceleration/jerk memory
  and timestamp propagation.
- deterministic MPPI cost evaluation for the Python `project` and `paper`
  profiles, with a component-level `CostBreakdown`;
- injected-noise MPPI updates, bounded effective-noise accounting, native C++
  Gaussian sampling, M2 feasible-sample weighting and recovery proposals.

`ICollisionEnvironment` is the only static-geometry contract. Simulation can
adapt an SDF and hardware can adapt an ESDF or voxel map without either
dependency entering this library. Safety, conditioner, MPPI dynamics, objective
and optimizer-update golden fixtures are generated from the validated Python
baseline and consumed by CTest without starting Python.

Standalone build and tests:

```bash
cmake -S uav_navigation_core -B build/uav_navigation_core \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/uav_navigation_core
ctest --test-dir build/uav_navigation_core --output-on-failure
```

Regenerate the committed golden fixtures only when intentionally changing the
validated Python behavior:

```bash
PYTHONPATH=. python3 scripts/generate_m2_golden_fixtures.py
PYTHONPATH=. python3 scripts/generate_m3_mppi_golden_fixtures.py
PYTHONPATH=. python3 scripts/generate_m4_mppi_golden_fixtures.py
```

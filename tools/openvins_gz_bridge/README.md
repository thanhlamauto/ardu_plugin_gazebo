# OpenVINS → Gazebo odometry bridge (macOS experiment)

This adapter reads only `/sensor_suite/rgb` (640×360 RGB, 10 Hz) and
`/sensor_suite/imu` (raw gyroscope/accelerometer, 100 Hz). It feeds upstream
OpenVINS' monocular visual-inertial filter and publishes
`/perception/visual_odometry`. It does not subscribe to `/iris/odometry`.
The first pose is anchored to the declared hover origin `(0, 0, 3 m)` and
aligned with the orientation reported by the simulated IMU. The upstream
`initialize_with_gt` API is seeded with IMU averages and the declared
stationary-hover condition; despite its name, this adapter supplies no
ground-truth position or velocity. Gazebo's IMU message includes an
orientation field; using it here has not validated an attitude filter for
raw IMU hardware. The estimator's trajectory is relative and may drift.
The ArduPilot flight controller still uses its normal simulated navigation
sensors in these trials.
The published orientation is computed from OpenVINS' `q_GtoI`, transformed
into ENU with the same initial anchor as position. The simulated IMU
orientation is used only for that initial anchor and stationary seed. Image
masks are zero (unmasked) because OpenVINS interprets 255 as excluded pixels.
The current SDF declares an ENU orientation reference and Gaussian sample
noise at 100 Hz: gyro stddev 0.0016968 rad/s and accelerometer stddev 0.02
m/s². The bridge uses the matching continuous white-noise densities
0.00016968 rad/s/√Hz and 0.002 m/s²/√Hz, with small bias-walk floors. This
simulation profile is not a hardware IMU noise calibration.

The bridge was built against [OpenVINS](https://github.com/rpng/open_vins)
revision `69488123ed9362dd44b6f28e7f4680abbff1442b` using Homebrew
OpenCV 5, Eigen, Boost, Ceres and Gazebo Transport 13 / Msgs 10. To reproduce
the macOS build, clone that revision outside this repository, change
`find_package(OpenCV 4 REQUIRED)` to `find_package(OpenCV 5 REQUIRED)` in
`ov_msckf/CMakeLists.txt`, then build the standalone library with:

```sh
cmake -S /path/to/open_vins/ov_msckf -B /path/to/open_vins/build \
  -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  '-DCMAKE_CXX_FLAGS=-I/opt/homebrew/include/opencv5 -include cassert -include opencv2/geometry/3d.hpp'
cmake --build /path/to/open_vins/build --target ov_msckf_lib -j4
cmake -S tools/openvins_gz_bridge -B /path/to/bridge_build \
  -DOPENVINS_ROOT=/path/to/open_vins
cmake --build /path/to/bridge_build -j4
```

The extra compiler includes work around upstream OpenVINS headers that do
not compile unchanged with OpenCV 5 on this host. They do not alter the
filter algorithm. The C++ bridge is source code in this repository; the
upstream GPL-3.0 OpenVINS library must be obtained and built separately.

Example trial with the existing ArduPilot/Gazebo harness:

```sh
python scripts/run_monocular_sim.py \
  --output results/monocular_research/openvins_example \
  --world worlds/iris_monocular_visual_ground_pilot.sdf \
  --config config/monocular_speed_visual_0.5.yaml \
  --perception-config config/monocular_openvins_perception.json \
  --eval-scene config/monocular_scene_visual_ground_pilot.json \
  --goal 12 0 3 --seed 7 --duration 45 \
  --openvins-bridge /path/to/bridge_build/openvins_gz_bridge
```

The bridge writes `openvins.csv` in the run folder. Gazebo ground truth is
recorded by the harness only for setup, safety checks and evaluation.
The CSV includes monotonic timestamps for RGB callback, image processing,
OpenVINS completion, and pose publication. On macOS these use
`CLOCK_UPTIME_RAW`, matching Python `time.monotonic()` in the perception log.
`imu_alignment_ms` records the distance from each camera timestamp to the
nearest IMU sample used for attitude alignment. The bridge rejects images
without an IMU orientation within 20 ms. Pass `--openvins-zupt-after-motion`
to the harness to enable OpenVINS' post-motion zero-velocity updates as an
ablation; this is not the default. Run
`python scripts/audit_monocular_calibration.py <run-dir>` to compare the
nominal sensor geometry and timestamps in a trial.
For an isolated estimator test, pass `--vio-only-motion-stop` to
`scripts/run_monocular_sim.py`, optionally together with
`--openvins-zupt-after-motion`. This skips perception and planner, commands
hover → roughly 1 m lateral motion → hover, and enables debug logs of ZUPT.
Then run `python scripts/analyze_vio_motion_stop.py <run-dir>`.

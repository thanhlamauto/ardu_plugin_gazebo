# Report and implementation audit changelog

Date: 2026-09-11

## Smooth fixed-route demo profile (2026-09-13)

- Added `config/experiments/mppi_demo_smooth.yaml` instead of silently
  changing the paper-audit profile.  It keeps the same paper-cost categories
  but uses project-selected `reference_speed_m_s=0.85`, `command_alpha=0.35`,
  `max_accel_xy=0.9 m/s^2`, `wp_radius=1.25 m` and
  `goal_radius=0.35 m` for a slower, smoother demonstration.
- Updated all three fixed-route and interactive-RViz commands in
  `docs/RUN_3_MPPI_MAPS_QUICKSTART_VI.md` to use the demo profile and unique
  `*_demo_smooth_seed7.jsonl` names.
- The narrow-gate demo route still passes the large wall opening and goes
  north around the two posts.  The direct small-slot route remains a
  stress-test and is not represented as demo-ready or safe.
- Offline candidate sweep showed roughly 18% lower command-acceleration RMS
  than the prior paper-reference profile on the sampled slalom cases.  This
  is tuning evidence only; live Gazebo validation of the new profile remains
  pending and must not inherit the older profile's result table.

## Interactive RViz goal interface (2026-09-13)

- Fixed the first-cycle `KeyError: 'mean_ms'` while waiting for the initial
  RViz goal.  `TimingWindow.summary()` now returns a stable zero-sample schema
  before MPPI has executed; a regression test covers this exact condition.
  Current deterministic suite: **41/41 passed**.  A live read-only replay on
  the active Gazebo transport then remained in `hold-await-goal` for seven
  cycles with `samples=0` and no exception; no MAVLink command was sent.
- Added optional `/goal_pose` subscription for RViz `2D Goal Pose`. Before the
  first click the node continuously sends zero velocity; every valid click can
  replace the route while flying or after the previous terminal latch.
- Accepted goals must use the configured frame (`odom` by default). Non-finite
  goals and untransformed frames are rejected explicitly.
- RViz's planar `z=0` is intentionally ignored. The target either keeps current
  altitude or uses `--rviz-goal-altitude`; this is a project safety decision.
- A goal replacement resets waypoint/terminal/recovery state, the command
  conditioner and the MPPI nominal warm start. Paper profile builds a new
  current-position-to-goal time-indexed reference.
- Added `rviz_default_plugins/SetGoal` to `config/sensor_suite.rviz` and a full
  copy/paste command to `docs/RUN_3_MPPI_MAPS_QUICKSTART_VI.md`.
- Verification: **40/40 unit tests passed**; an in-process ROS 2 integration
  check published `PoseStamped(frame_id=odom, xyz=[7.5,-2,0])` and the planner
  interface received revision 1 with identical message coordinates.
- A complete Gazebo flight driven only by RViz clicks has not yet been logged;
  status remains **IMPLEMENTED, NOT LIVE-FLIGHT-VERIFIED**.

## Time-indexed global-reference implementation and live validation (2026-09-13)

This section supersedes the older statement below that the paper profile lacks
a time-indexed reference and that all Gazebo behavior is pending.

- Replaced nearest-polyline-only tracking in `cost_profile: paper` with a
  monotonic path-progress reference: each horizon step receives its own
  `p_ref[j]` and finite-difference `v_ref[j]`. The cost now contains the
  position and velocity categories corresponding to Minařík et al., Eq. 17--18.
- Augmented the reduced planner state with the last feasible velocity/yaw-rate
  setpoint. The same low-pass, slew limits and saturation used at the MAVLink
  boundary now execute inside every rollout. This removes the old mismatch in
  which MPPI optimized instantaneous command changes that the interface could
  not send.
- Applied the input-change cost to the feasible rollout controls and kept the
  paper-profile terminal heuristic disabled. This remains a velocity-level
  project adaptation; it is not the paper's full rigid-body reproduction.
- Added predictive braking, a stop-then-retreat recovery state and release
  hysteresis. Safety logic clears both the external conditioner and the MPPI
  applied-control anchor.
- Added deterministic coverage for time-indexed references, monotonic progress,
  feasible rollout conditioning, command saturation and brake/recovery.
- Updated `config/experiments/mppi_paper_cost_only.yaml` and both Vietnamese run
  guides. The paper's reported `lambda=1e-4` was not copied: the current
  velocity-command units/model produced weight collapse, so `lambda=100` is an
  explicitly offline-tuned project parameter.

Offline evidence:

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python -m pytest -q`
  — **37/37 passed** in 2.90 s.
- Permanent analytical benchmark profile `paper_reference`, three maps × seeds
  7/11/19 — **9/9 reached**. Artifact:
  `output/benchmark/paper_reference_v2_inflated.json`.

Live Gazebo/SITL evidence (2026-09-13, one seed per map, velocity-level only):

| Map | Result | Cycles | Path | Min point clearance | Mean/p95 compute | Deadline misses |
|---|---|---:|---:|---:|---:|---:|
| Right angle | reached, 0.235 m | 128 | 23.15 m | 2.19 m | 14.5/14.9 ms | 0 |
| Narrow gate | reached, 0.230 m | 192 | 35.50 m | 2.07 m | 13.3/14.7 ms | 0 |
| Slalom | reached, 0.220 m | 193 | 37.67 m | 1.30 m | 13.6/15.5 ms | 0 |

Evidence files are
`output/log/{right_angle,narrow_gate,slalom}_paper_v2_live_20260913.jsonl` and
`output/plots/paper_global_path_v2_live_20260913/`. All three runs used seed 7,
ArduPilot `f808f78c`, reached the terminal gate and had no recorded
`hold-brake`, `recover-brake` or deadline miss. Clearance is center-to-nearest
LiDAR point, not footprint-adjusted free space. The slalom minimum of 1.30 m is
below the configured 1.5 m rollout collision radius, so collision inflation,
sensor latency and UAV footprint still require multi-seed Gazebo evaluation;
this is not a hardware-safety claim. Rigid PA-MPPI, unknown-space PA-MPPI,
edge-computer and hardware validation remain pending.

## Verification after paper-cost update (2026-09-11)

- Added `paper_r_delta_u` to YAML loading and the paper-only profile.
- Updated `docs/RUN_3_MPPI_MAPS_QUICKSTART_VI.md`: all three Gazebo map commands
  now use `mppi_paper_cost_only.yaml` and pass an explicit ENU `--global-path`.
- Re-ran the deterministic suite with:
  `KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python -m unittest tests.test_mppi_core -v`
  — **33/33 tests passed** in the `ardupilot-rviz` environment.
- Re-ran the paper-cost-only point-mass smoke test: **PASSED offline**, final
  goal distance 0.24 m and minimum point clearance 2.00 m. This uses a
  point-radius collision adapter and is not Gazebo validation.
- Recompiled the technical report to
  `output/pdf/pa_mppi_ardupilot_technical_report_vi.pdf` (13 pages; only
  underfull-box warnings remain).

## Paper-cost-only comparison profile (2026-09-11)

- Added `cost_profile: paper` and
  `config/experiments/mppi_paper_cost_only.yaml` for a controlled comparison.
- The profile keeps only paper-mapped categories: diagonal input effort,
  position reference/path, endpoint position proxy, and collision indicator.
- Disabled in that profile: project softplus proximity, running waypoint
  distance, yaw heuristic, and current `w_du*(u-v)` term.
- Paper Eq. 16's `R_delta` is now computed from successive controls in the
  full action sequence passed to the terminal callback; diagnostics expose it
  as `cost.input_change`. The profile still does not implement the paper's
  full-state time-indexed reference metric.
- The point-cloud collision radius and static polyline are project adapters;
  no full-state time-indexed reference or geometry collision module is present.
- Unit tests `test_paper_cost_profile_uses_effort_and_collision_indicator` and
  `test_paper_cost_profile_applies_input_change_penalty_to_sequence` added.

Paper-cost smoke command (offline point-mass harness):

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner mppi \
  --config config/experiments/mppi_paper_cost_only.yaml \
  --goal '16,10,20;30,0,20' \
  --global-path '0,0,20;16,10,20;30,0,20' \
  --collision-radius-m 2.0 --margin 0.5 --seed 7
```

Result: **PASSED offline**, final goal distance 0.24 m and minimum point
clearance 2.00 m. `collision-radius-m` is a point-cloud adapter and
`--margin` only satisfies the legacy harness gate; this is not Gazebo validation.

## Global-path reference cost (2026-09-11)

- Added optional `w_path` and `path_scale_m` to the velocity-level MPPI cost.
- Added `--global-path`/YAML runtime input and segment-projection distance to an
  ENU polyline; `--goal` remains the mission waypoint/arrival route.
- Added `cost.path` to nominal diagnostics and equivalent support to the rigid
  experimental branch.
- Design basis: Minařík et al., *Model Predictive Path Integral Control for
  Agile Unmanned Aerial Vehicles*, arXiv:2407.09812, Eq. 16--18. The project
  uses only the position-reference idea; it does not claim the paper's full
  time-indexed state metric or dynamics.
- Default `w_path: 0.0` keeps prior behavior. No Gazebo result was generated;
  live path-following remains pending.
- Offline verification: 30/30 tests passed, including polyline projection,
  cost contribution, parser and non-finite path rejection.

## Live clear-air terminal debug

- Reproduced terminal circling in the running Gazebo/SITL session with
  obstacle cost disabled: a 2 m pure-ENU-X goal produced 5.561 m path length,
  path/straight ratio 2.570 and maximum Y cross-track error 1.322 m. Evidence:
  `output/log/mppi_arrival_envelope_live_debug.jsonl`.
- Corrected the MPPI warm-start contract: the conditioned command actually
  sent to ArduPilot is now written back to the first nominal action.
- Replaced the near-terminal stochastic translation direction with a
  deterministic goal-error arrival vector and retained distance-based speed
  tapering. The repeated 2 m live gate produced 1.789 m path versus 1.787 m
  straight displacement (ratio 1.001), maximum Y error 0.059 m and terminal
  error 0.236 m. Evidence:
  `output/log/mppi_arrival_vector_live_debug.jsonl`.
- Odometry finite differencing now uses actual message receipt intervals and
  separate XY/Z sanity limits rather than multiplying every sample by a fixed
  controller frequency.
- Added active waypoint and obstacle-cloud count/bounds to future JSONL logs.
- Live obstacle avoidance was not rerun in this debug session and remains
  pending; both live gates above intentionally used `w_obstacle=0`.
- Verification: 27/27 unit tests passed; the deterministic offline warehouse
  simulation passed with final goal error 0.23 m and minimum clearance 6.42 m.

## Factual corrections

- Removed the claim that `OBSTACLE_DISTANCE_3D` requires a companion-generated persistent obstacle ID. MAVLink permits `UINT16_MAX` for unknown ID, and target ArduPilot commit `f808f78…` does not read `packet.obstacle_id` in `AP_Proximity_MAV::handle_obstacle_distance_3d_msg`.
- Reframed companion-side mapping/planning as a project architecture decision motivated by direct access to the three-state map and perception objective, not as proof that ArduPilot avoidance is unusable.
- Exposed the `SET_ATTITUDE_TARGET` discrepancy: the current Guided documentation says body rates are unsupported, while the target source accepts all three rates together and rejects a partial vector.
- Derived masks symbolically: velocity+yaw-rate `1479`, velocity-only `3527`, body-rates+thrust with attitude ignored `128`.
- Clarified `GUID_OPTIONS` bit 3 (`8`) and direct-thrust vs climb-rate behavior from exact target source.
- Reclassified the rigid prediction model as a reduced closed-loop model with first-order body-rate tracking, not complete motor/torque/inertia rigid-body dynamics.
- Recorded the SDF mass sum (2.10 kg) and marked normalized hover thrust 0.38 as model-dependent historical-log calibration.

## New sources

- Zhai, Reiter, Scaramuzza, PA-MPPI, IEEE RA-L 2026, DOI `10.1109/LRA.2026.3662653`.
- Williams et al., Information-Theoretic MPC, IEEE T-RO 2018, DOI `10.1109/TRO.2018.2865891`.
- Official MAVLink common and ArduPilotMega message definitions; vendored MAVLink commit `71d925850d7ddb01d0e56defc1046f9d9d3bf8ae`.
- Official ArduPilot Guided, Simple Avoidance, BendyRuler, SITL/Gazebo and ROS 2/Gazebo documentation.
- Exact ArduPilot source at commit `f808f78ce5a518ca96f2fb36420608b2b6254367`.
- Bibliography moved to `reports/pa_mppi_sources.bib` with stable keys and DOI/source metadata.

## Design decisions clarified

- Three staged control levels and why each exists.
- AP avoidance disabled only for controlled experiments to avoid two competing avoidance layers.
- Project mapper is not ROG-Map; near-field free painting is a prototype assumption, not a paper-derived inverse sensor model.
- Same velocity dynamics are used for vanilla and PA-v0 where comparison requires it.
- Fair ablation configs hold base fields constant and only switch perception coefficients.
- Random seed is now explicit in configuration and diagnostics.

## Claims downgraded to pending

- All new Gazebo live behavior, including rigid hover/body-rate tracking, frame signs, clear-air route, static obstacle and C-wall/U-wall ablation.
- 50 Hz live deadline compliance under Gazebo/MAVLink/sensor load.
- Edge-computer latency, thermal and dropout behavior.
- Real LiDAR/depth inverse sensor behavior and miss/max-range rays.
- Hardware hover calibration, HIL/tether and real flight.
- Old optimizer timing numbers were removed from the report because this audit did not regenerate a durable benchmark artifact.

## Implementation and tests added/strengthened

- Deterministic seed in all planner configs.
- Finite-state rejection for velocity and rigid planners.
- Explicit attitude-mask validation and rejection of partial body-rate masks.
- `GUID_OPTIONS` readback gate test.
- Planner-timeout hold behavior.
- State source age/timestamp and expanded JSONL logging.
- Logging includes state, control, thrust, map, clearance, LOS, costs, rollout stats, selected trajectory, deadline, saturation/failsafe and seed.
- Post-processing script `scripts/plot_mppi_experiment.py`.
- Fair ablation configs under `config/`.
- Exact future run gates in `docs/GAZEBO_RUN_CHECKLIST.md`.

## Exact commands run

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python -m unittest tests.test_mppi_core -v
```

Result: **23/23 passed** in repeated audit runs (3.304 s, 2.665 s and 2.528 s;
wall time varies by run).

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner mppi \
  --config mppi_ardupilot/config.yaml
```

Result: **PASSED offline**, seed 7, final goal distance 0.93 m, minimum point clearance 6.30 m.

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner pa-mppi \
  --config mppi_ardupilot/config.yaml
```

Result: **PASSED offline**, seed 7, final goal distance 0.92 m, minimum point clearance 6.11 m.

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner rigid-pa-mppi \
  --config mppi_ardupilot/rigid_pa_mppi_config.yaml
```

Result: **PASSED offline internal dynamics**, position/velocity numerical error 0 after 100 hover steps, quaternion norm 1, hover normalized thrust mapping 0.38.

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner mppi \
  --config mppi_ardupilot/config.yaml --w-path 0.5 \
  --global-path '0,0,20;0,10,20;30,10,20;30,0,20' \
  --goal '16,10,20;30,0,20' --seed 7
```

Result: **PASSED offline path-cost smoke test**, final goal distance 0.23 m,
minimum point clearance 5.85 m. This is the deterministic point-mass test, not
Gazebo/SITL validation.

## Tests deliberately not run

- Gazebo 3D server/client and ArduPilot SITL integration: unavailable on the current machine/session per task constraint.
- MAVLink live heartbeat/EKF/frame sanity/takeoff/hover/body-rate/static-obstacle runs: require Gazebo/SITL.
- C-wall/U-wall MPPI-vs-PA-MPPI ablation: world/logged live map not yet executed.
- Edge computer and real hardware: no target device/vehicle was connected.

## Unresolved questions

- Runtime frame of `Gazebo Odometry.twist.angular` for the exact plugin chain.
- PointCloudPacked miss/max-range semantics after the current decoder/downsampler.
- Current-model hover value and rate time constant under live closed loop.
- Edge compute distribution and deadline misses.

These remain `SOURCE_TODO`/pending in `docs/SOURCE_AUDIT.md`; no result was inferred.

# M7 — Systematic validation, profiling and stress test

M7 freezes the C++ navigation baseline and measures it. No controller tuning is
allowed inside a validation set. A parameter change requires a separate tuning
config, a reviewed replacement of the frozen baseline, and a new result set.

## Frozen baseline

The validation source of truth is
[`m7_baseline.yaml`](../uav_navigation_bringup/config/m7_baseline.yaml). It uses
80 MPPI samples, a 30-step horizon at 0.1 s, path-progress objective, proactive
proposals and a 10 m/s speed cap. The controller runs at 10 Hz with a 100 ms
hard deadline. MPPI trajectory markers are disabled for performance runs.

`navigation.yaml` remains the debug profile. It publishes predicted trajectories
and candidate markers and must not be used for timing claims.

Every run records the Git commit and SHA-256 hashes of the scenario, parameter
file and world. This prevents results from silently mixing configurations.

## Scenario matrix

Scenario definitions live in [`tests/scenarios/`](../tests/scenarios/). Test
conditions are data, not shell-script constants.

| ID | Scenario | Primary observation | Repetitions |
|---|---|---|---:|
| S01 | Straight 30 m | tracking baseline | 10 |
| S02 | 45-degree turn | path tracking | 10 |
| S03 | 90-degree turn | cornering | 10 |
| S04 | Static obstacle detour | A-star/MPPI handoff | 10 |
| S05 | Narrow passage | clearance, `N_safe`, ESS | 10 |
| S06 | High-speed straight | cruise and stopping/runtime safety | 10 |
| S07 | Blocked goal | global planner reaches `NO_PATH` | 5 |
| S08 | Replan while moving | Goal A to Goal B handoff | 5 |
| S09 | Stale input and local-planner faults | local safety states | 5/variant |
| S10 | FCU/adapter faults | actuator boundary safety | 5/variant |

S09 variants are `stale_odometry`, `stale_obstacle`, `planner_deadline` and
`no_safe_trajectory`. S10 variants are `fcu_disconnect`,
`local_planner_death`, `wrong_mode` and `disarm`. The ROS-Gazebo bridges are
separate processes so odometry and obstacle freshness can be tested
independently.

## Data contract

[`run_m7_scenario.py`](../scripts/run_m7_scenario.py) writes an immutable
`manifest.json`, `launch.log` and `events.jsonl` for one run. Every controller
cycle contains:

- measured position and velocity, current goal;
- raw selected MPPI command and conditioned command;
- total, rollout, cost, safety and conditioner time;
- sample count, safe-sample count, ESS and best feasible cost;
- minimum clearance, path progress and cross-track error;
- local mode, adapter state and command ages;
- safe-command and MAVROS-setpoint sequence numbers used to detect publication
  after an adapter safety state.

The local node rejects any result later than the configured deadline before
publishing. Raw command publication is diagnostic only; the adapter subscribes
only to `/control/safe_velocity_command`.

[`analyze_m7_results.py`](../scripts/analyze_m7_results.py) derives time to
goal, path length/efficiency, RMS and p95 cross-track error, minimum clearance,
peak XY speed and acceleration, controller p50/p95/p99/max, deadline misses,
minimum/median `N_safe`, minimum/median ESS, best feasible cost,
`NO_SAFE_TRAJECTORY`/`DISCONNECTED`/`STALE_COMMAND` counts,
accepted-trajectory collision indication, late accepted commands, and MAVROS
setpoints published while stale, disarmed or outside `GUIDED`.
It generates `summary.csv` and
`REPORT.md` from the per-run source logs.

## Run one scenario

Build and source the ROS 2 workspace, then start ArduPilot SITL as documented in
the root README. Start the runner in a second terminal; it launches the
Gazebo/ROS stack and waits up to 60 seconds. During that wait, enter `mode
guided`, `arm throttle`, `takeoff 5` in MAVProxy. The goal is published only
after odometry reports the armed, GUIDED vehicle above 4 m:

```bash
source install/setup.bash
python3 scripts/run_m7_scenario.py tests/scenarios/s01_straight.yaml \
  --run 1 --seed 7
```

Use a unique run number. The runner refuses to overwrite an existing run.
Inspect the exact command and destination without starting ROS:

```bash
python3 scripts/run_m7_scenario.py tests/scenarios/s05_narrow_passage.yaml \
  --run 1 --seed 17 --dry-run
```

For a debug video with RViz and MPPI markers:

```bash
python3 scripts/run_m7_scenario.py tests/scenarios/s08_replan_while_moving.yaml \
  --run 1 --seed 7 --debug-visualization
```

That run is marked `performance_mode: false` and must not enter timing
statistics used for the performance claim.

Fault variants send signals to an exact ROS process or invoke a MAVROS service,
so they require an explicit flag:

```bash
python3 scripts/run_m7_scenario.py tests/scenarios/s09_stale_input.yaml \
  --variant stale_obstacle --run 1 --allow-process-faults

python3 scripts/run_m7_scenario.py tests/scenarios/s10_fcu_disconnect.yaml \
  --variant local_planner_death --run 1 --allow-process-faults
```

The signal-based cases always resume a suspended process during cleanup. The
`wrong_mode` and `disarm` variants intentionally change simulated FCU state;
reset SITL before the next repetition.

S07 can be used as a bringup smoke test without arming because it only verifies
the global planner:

```bash
python3 scripts/run_m7_scenario.py tests/scenarios/s07_goal_blocked.yaml \
  --run 1 --start-immediately
```

## Aggregate results

After collecting runs:

```bash
python3 scripts/analyze_m7_results.py results/m7
```

The generated `results/m7/summary.csv` contains one row per run. The generated
`results/m7/REPORT.md` groups the safety and runtime result by scenario. Keep
the raw run directories; the generated tables are derived artifacts.

For S05, generate the required four-panel clearance/`N_safe`/speed/cost plot:

```bash
python3 scripts/plot_m7_run.py \
  results/m7/s05/run_01_seed_7/events.jsonl
```

## Acceptance criteria

Safety requirements:

- zero accepted-trajectory collision indications;
- zero conditioned commands published after an unsafe local mode;
- zero accepted controller outputs above 100 ms.

Functional requirements:

- reachable cases reach `GOAL_REACHED`;
- S07 reports `NO_PATH`;
- replanning replaces the old goal/path and reaches Goal B;
- each injected fault reaches its declared local or adapter state.

Runtime requirement:

- zero controller deadline misses in baseline validation;
- report p50, p95, p99 and maximum rather than selecting one favorable run.

Tracking and clearance have no invented pass threshold in M7. Their complete
distributions are reported for mentor review before hardware limits are set.
The current simulator model has no contact topic in this harness, so the
`collision` column is the shared safety predicate on accepted trajectories; it
is not a claim from a Gazebo physics contact sensor.

## Current checkpoint

The M7 instrumentation, scenario definitions, fault variants and analysis path
are implemented. The repeated 80-run campaign has not yet been executed, so
this document makes no M7 reliability or hardware-readiness claim. Existing M6
closed-loop results remain the latest flight evidence until the M7 result set
is complete.

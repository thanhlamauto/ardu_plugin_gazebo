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
If the recorder/analyzer is fixed without rebuilding the controller, pass
`--runtime-commit <commit>`; the manifest then records the controller/runtime
commit separately from `harness_commit`.

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
| S08 | Replan while moving | Goal A to Goal B handoff | 10 |
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

On slow startup or automated SITL resets, `--readiness-timeout 120` extends
only the preflight wait. Scenario timing still begins at goal publication.

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

## M7.1 campaign result — 2026-09-18

The campaign used the frozen controller/runtime commit
`7e47663a80c3ef990a2223ed665819035b01cccb` and the frozen
`m7_baseline.yaml` SHA-256
`5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`.
No controller parameter changed during the campaign. The machine was an Apple
M2 MacBook Air with 8 CPU cores and 16 GB RAM, macOS 26.1, ROS 2 Jazzy,
Fast DDS and Gazebo Sim 8.15.0. RViz, MPPI markers and live plotting were off.
Harness-only fixes during collection hardened process cleanup and corrected
the S10 MAVROS/disarm fault injectors; manifests identify harness commits
`4e339a9`, `a0045ec` and `cfa1ef3`. All 115 runs still execute the same
`7e47663` controller binary and the same config hash above.

The 115-run aggregate contains 75 reachable/replanning runs, 5 blocked-goal
runs and 35 fault-injection runs. The committed artifacts are
[`summary.csv`](../results/m7_campaign_7e47663_20260918/summary.csv),
[`manifests.jsonl`](../results/m7_campaign_7e47663_20260918/manifests.jsonl),
and the generated
[`aggregate_report.md`](../results/m7_campaign_7e47663_20260918/aggregate_report.md).
Raw per-cycle JSONL and launch logs total about 90 MB and are retained outside
Git at `/tmp/m7_campaign_7e47663_20260918_combined2` on the campaign machine.

| Scenario/variant | Pass | Median goal time (s) | Median CTE RMS (m) | Minimum clearance (m) | Peak speed (m/s) | Minimum `N_safe` | Worst p99 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| S01/base | 10/10 | 10.91 | 0.443 | 4.531 | 7.61 | 80 | 12.76 |
| S02/base | 10/10 | 13.70 | 1.198 | 0.091 | 6.11 | 0 | 14.21 |
| S03/base | 5/10 | 10.00 | 1.454 | 0.004 | 8.53 | 0 | 19.05 |
| S04/base | 10/10 | 13.00 | 0.807 | 0.833 | 7.63 | 0 | 9.85 |
| S05/base | 9/10 | 14.23 | 1.730 | 0.006 | 9.37 | 0 | 15.75 |
| S06/base | 9/10 | 25.66 | 1.101 | 4.018 | 10.01 | 80 | 12.30 |
| S07/base | 5/5 | — | — | — | — | — | — |
| S08/base | 10/10 | 13.96 | 2.050 | 4.607 | 8.76 | 80 | 6.53 |

All four S09 variants and all four S10 variants passed 5/5. The five deadline
misses in the aggregate belong only to the deliberate S09 `planner_deadline`
injection; each late result was rejected. Reachable baseline runs had zero
deadline misses. Their worst per-run p99 was 19.05 ms and the largest recorded
controller cycle was 51.84 ms, both below the 100 ms control period.

The campaign observed zero accepted-trajectory collision indications, zero
late accepted outputs, zero stale-command violations, zero MAVROS setpoints
while disarmed and zero setpoints in the wrong mode. As stated in the acceptance
criteria, the collision result comes from the shared trajectory predicate and
not a Gazebo contact sensor.

The reported peak-acceleration column is not suitable as a vehicle-dynamics
claim. It finite-differences velocity samples using recorder callback time, so
DDS delivery bursts create artificial spikes. Position, velocity, controller
timing and safety-event metrics remain usable; acceleration requires odometry
header timestamps or a synchronized estimator before publication.

## M7.2 failure triage and exact rerun

The seven failed M7.1 reachable runs were replayed once with the same runtime,
config, scenario, run number and seed. No controller parameter was changed.
The rerun aggregate is in
[`m72_exact_rerun_summary.csv`](../results/m7_campaign_7e47663_20260918/m72_exact_rerun_summary.csv).

| Scenario | Failed in M7.1 | Exact rerun result | Interpretation |
|---|---:|---:|---|
| S03 90-degree turn | 5/10 | 2/5 pass | recurrent controller/closed-loop failure; outcome is not fixed by seed alone |
| S05 narrow passage | 1/10 | 1/1 pass | intermittent; retain as an unresolved reliability failure |
| S06 high-speed straight | 1/10 | 1/1 pass | original run never received the planned path; interface/startup failure |

The original S03 seed 27 run failed preflight, but its exact rerun passed
preflight and then timed out in the turn. Across the four original S03 flight
failures, median peak speed was 7.96 m/s, median RMS cross-track error was
5.24 m, median feasible-sample count was zero and median
`NO_SAFE_TRAJECTORY` count was 543.5. The five successful S03 runs had median
peak speed 4.09 m/s, median RMS cross-track error 0.50 m and median feasible
sample count 78. This separates the observed failure signature clearly:
the fast approach reaches a state where the 80-sample planner repeatedly finds
no safe trajectory, after which the vehicle cannot complete the corner.

An exact seed does not reproduce the outcome deterministically. Seed 37 and 87
failed in M7.1 but passed the rerun; seed 47 and 57 failed both times. Simulator
state, asynchronous sensor/path timing and warm-start history therefore remain
part of the experiment state and must be captured before the next root-cause
experiment. Representative pass/fail plots are under
[`results/m7_campaign_7e47663_20260918/plots/`](../results/m7_campaign_7e47663_20260918/plots/).

## Decision

M7.1 is complete and its failed results remain preserved. The baseline is
**not ready for M8/hardware** because reachable deterministic scenarios did not
meet the required 10/10 result, chiefly S03 at 5/10. Runtime deadline handling
and all injected safety boundaries passed. The next work stays in M7.2: capture
the complete pre-turn state/warm-start/sensor timing for S03, determine why the
same nominal seed enters either the low-speed feasible branch or the high-speed
`N_safe=0` branch, fix that root cause in a new commit/config, then rerun S03
and the full regression campaign without overwriting this baseline.

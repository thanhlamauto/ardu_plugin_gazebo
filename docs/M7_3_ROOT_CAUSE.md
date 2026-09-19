# M7.3 — Root-cause analysis and stabilization

## 1. Scope

M7.3 explains, with evidence, why the S03 90-degree turn, the S05 narrow
passage, the S08 replan and the baseline planner-deadline tails fail. It adds
diagnostics first and does not change controller or safety behavior. No M8 or
PA-MPPI work is started.

The focused campaign contains 50 instrumented runs collected on 2026-09-19:

| Scenario | Runs | Pass | Controller failure | Interface/startup failure |
|---|---:|---:|---:|---:|
| S03 turn 90 (10 seeds x 3) | 30 | 15 | 14 | 1 |
| S05 narrow passage (10 seeds) | 10 | 8 | 1 | 1 |
| S08 replan (10 seeds) | 10 | 9 | 0 | 1 |

Raw per-cycle logs (about 30 MB) remain on the campaign machine at
`/tmp/m73_root_cause_10467cb_20260919`. The committed derived artifacts are in
[`results/m73_root_cause_10467cb_20260919/`](../results/m73_root_cause_10467cb_20260919/):
`run_metrics.csv`, `planner_timeouts.csv`, `s08_handoff.csv`, `m73_summary.md`,
`summary.csv`, `manifests.jsonl`, `campaign.log` and representative `plots/`.

## 2. Frozen controller/config

No MPPI, safety, conditioner or path-selection parameter changed. The frozen
`m7_baseline.yaml` SHA-256 remains
`5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`. The
focused campaign ran runtime commit
`10467cba8307530b88f1fb3417ecd8597d9753f6` (instrumentation commits
`0356f11` and `10467cb` on top of `91fab63`). Historical M7.1/M7.2 result
directories are untouched.

Instrumentation is behavior-neutral by construction and by test:

- the labelled recovery-proposal builder is byte-identical to the previous
  builder and still matches the Python golden fixture;
- the optimizer records rejection reasons and source statistics but does not
  alter weights, selection or the safety predicate;
- the added per-cycle diagnostics cost `t_path_reference_ms` p99 of
  0.03–0.05 ms and `t_input_snapshot_ms` p99 of 0.01 ms;
- the one observed deadline event was the safety checker (see section 7), not
  the added bookkeeping.

## 3. New instrumentation

- MPPI rejection accounting with a deterministic first-failure policy:
  `reject_non_finite`, `reject_swept_collision`, `reject_static_collision`,
  `reject_dynamic_collision` (reserved, always zero today), and
  `reject_stopping_distance`.
- Proposal-source accounting (`gaussian`, `reference`, `braking`, `recovery`,
  `specific_action`) with generated count, safe count and best cost.
- Turn-entry state: total/XY/vertical speed, yaw, yaw rate, path tangent
  heading, heading error, diagnostic-only path curvature and distance to the
  next major turn, stopping distance, and separate minimum collision and
  minimum stopping clearance.
- Cycle timing split: scheduling delay, wall cycle, input snapshot, path
  reference, sampling, optimizer update, rollout, cost, safety, conditioner,
  publish and diagnostics.
- Path versioning: global `goal_id` and `global_path_id`, local
  `local_active_path_id`, the path header stamp join key, first solve and first
  safe command flags.

Deadline holds now publish the measured partial timing instead of zeros, so a
deadline event can be attributed to a component.

## 4. S03 findings

Result: 15/30 pass. Failure is not deterministic by seed: seed 17 and seed 67
passed 3/3, seed 27 failed 3/3, the other seeds were mixed.

At the **first** `N_safe = 0` cycle, passing and failing runs are almost
indistinguishable:

| Group | Speed m/s | Heading error rad | Collision clearance m | Stopping clearance m | Safe samples per source |
|---|---:|---:|---:|---:|---|
| PASS | ~2.4–4.0 | −2.4 … +0.3 | 1.43–2.27 | ~1.5 | 0 / 0 / 0 / 0 |
| FAIL | ~1.8–4.0 | −2.4 … +0.3 | 1.39–2.27 | ~1.5 | 0 / 0 / 0 / 0 |

Immediately after collapse the groups diverge:

| Group | Median peak speed | Median no-safe cycles | Median min collision clearance |
|---|---:|---:|---:|
| PASS | 4.08 m/s | 2 | 1.50 m |
| FAIL | 7.55 m/s | 545 | 0.0126 m |

Answers to the section-10 questions:

1. **Where does `N_safe` collapse?** At turn entry, t ≈ 2–7 s, at moderate
   speed (1.8–4.0 m/s), before any large cross-track error. It is a transient
   even in runs that pass (8/15 passes entered it, median first episode 2
   cycles).
2. **Dominant rejection reason?** At the first collapse, the
   `predicted_stopping_clearance` predicate is usually the binding one
   (`reject_stopping_distance` = 60–80 of 80 samples), with
   `swept_collision` dominant in the collapse window of 19/30 runs and after
   divergence. The collapse is therefore not an actual geometric collision at
   onset; the required stopping clearance at the corner speed is.
3. **Which proposal families remain feasible?** None. Gaussian, reference,
   braking, recovery and specific-action safe counts are all zero at collapse.
   This is a geometry/stopping constraint, not proposal insufficiency.
4. **Speed at turn entry** is 1.8–4.0 m/s for both groups; entry speed alone
   does not separate pass from fail.
5. **Heading error at turn entry** is large in both groups (−2.4 … +0.3 rad);
   it does not separate pass from fail.
6. **Stopping clearance at collapse** sits at the 1.5 m configured
   `safety.stopping_clearance_m` boundary for both groups.
7. **Warm-start U does differ systematically.** Median warm-start command
   magnitude at collapse: PASS 3.86, FAIL 5.26 (strong correlation, sample
   limited, causality not established).
8. **Not correlated with callback timing.** Scheduling delay is within a few
   milliseconds in both groups, and max compute is 28.7 ms (pass median) versus
   35.0 ms (fail median). No deadline event occurred in the failing runs except
   one (section 7). Failures are not a timing artifact.

Failing runs accumulate `reject_swept_collision` ≈ 4.4×10^4 versus ≈ 5.7×10^2
in passing runs: divergence drives the vehicle to within 0.0126 m of the
obstacle cloud. This is a **recovery-after-transient-infeasibility** failure,
not a tracking or classification failure.

## 5. S05 findings

Result: 8/10 pass. One controller failure (seed 27) and one interface failure
(seed 87, 1086 `INVALID_INPUT` cycles and zero solved cycles).

- 7/8 passes entered `N_safe = 0`; the minimum collision clearance over passes
  is 0.0208 m. `GOAL_REACHED` is not a healthy margin, matching M7.2.
- At first collapse the binding predicate alternates between
  `reject_stopping_distance` (4 runs) and `reject_swept_collision` (4 runs),
  and again all proposal families are simultaneously infeasible.
- The seed-27 failure shows the same divergence signature as S03: 651 no-safe
  cycles, peak speed 7.13 m/s, stuck near 26.4 m of the ~32 m path.

Answer to "why does `N_safe` reach zero": at the gate the combination of
vehicle speed and the required stopping clearance (1.5 m) makes every sampled
trajectory violate the stopping predicate; once the controller holds with no
safe command, speed and cross-track error grow until the swept-collision
predicate also rejects everything. The main cause is **stopping-distance
constrained infeasibility at entry speed**, aggravated by lack of recovery;
excess speed alone is a contributing, not the sole, cause.

## 6. S08 findings

Result: 9/10 pass. The single failure (seed 77) is an interface failure: the
`parameter_bridge` `/clock` process died with `mutex lock failed`, so obstacle
TF lookups failed with "extrapolation into the future" and the node reported
`INVALID_INPUT` continuously. The vehicle never became flight-ready and no goal
was published.

The versioning join in `s08_handoff.csv` shows, for every run:

- monotonic `goal_id` and `global_path_id` matching a monotonically increasing
  local `local_active_path_id` (1→1, 2→2);
- the first MPPI solve occurs within one control cycle of path receipt
  (0–60 ms for nearly all handoffs, one 416 ms outlier);
- no repeated, stale or mismatched path id.

There is **no path-handoff race, no stale old path, no version mismatch and no
warm-start contamination** in the evidence. The failures seen in M7.2 and here
are simulator/vehicle-response or startup failures (old S08 failures showed a
commanded ~5 m/s while odometry stayed frozen at the start position), not
MPPI tracking failures.

## 7. Planner-timeout findings

The focused campaign recorded exactly one `PLANNER_TIMEOUT` cycle:

| timeout_id | scenario | seed | cycle | scheduling_delay | rollout | cost | safety | total | classification |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | S03 | 27 | 7 | −0.20 ms | 0.72 ms | 14.38 ms | 105.69 ms | 125.50 ms | `C_safety` |

The deadline was consumed by the safety checker, not by MPPI rollout, cost or
scheduler latency. `TrajectorySafetyChecker::Evaluate` is
O(samples × segments × obstacle points): with 80 samples, 30 steps and up to
`obstacle_max_points = 800`, one cycle evaluates ≈ 1.9×10^6 point-segment
distances, and it rebuilds the filtered cloud for every sample.

The 53 M7.2 baseline timeouts cannot be attributed from the old logs because
the pre-M7.3 build published zeroed timing on a deadline hold. The M7.2
distribution (p99 41.9 ms, max 99.7 ms) combined with this measured
safety-bound overrun indicates the tails are **controller-computation**, not
scheduler/executor latency, but this is a strong inference from one attributed
sample plus the old distribution, not a complete classification of all 53.

Explicit answer: the previous baseline timeouts were **not mainly
scheduler/executor latency**. At least the reproduced case is safety-evaluation
compute; the old timing schema simply could not show it.

## 8. Confirmed root causes

1. **Transient all-sample infeasibility at S03/S05 entry** driven primarily by
   the predicted-stopping-clearance predicate at the configured 1.5 m margin,
   with all proposal families simultaneously infeasible. Confirmed by
   instrumented counts.
2. **Recovery failure after that transient.** Some runs do not re-enter a safe
   state; speed and cross-track error grow and the vehicle reaches within
   ~0.01 m of the obstacle cloud. Confirmed by the divergence signature and the
   4-order-of-magnitude difference in swept-collision rejections.
3. **S08 replanning has no path-id race or stale-path bug.** Confirmed by the
   monotonic id join and one-cycle handoff latency.
4. **One deadline miss was safety-evaluation-bound**, and the old deadline
   timing was unobservable (zeroed). Confirmed by the new timing fields.
5. **Two of the three focused "interface/startup" failures** are ROS/Gazebo
   bridge or readiness failures (`/clock` bridge crash, obstacle TF
   extrapolation), not controller failures. Confirmed by logs.

## 9. Hypotheses not supported

- S03/S05 failures are not caused by seed alone (seed 17/67 pass 3/3, seed 27
  fails 3/3, others mixed).
- They are not caused by callback/scheduling latency (section 4, item 8).
- They are not caused by proposal-family insufficiency (all families jointly
  infeasible).
- S08 failures are not caused by MPPI tracking, stale paths, path version
  mismatch, callback ordering or warm-start contamination.
- The 53 baseline timeouts are not shown to be scheduler/executor latency; the
  one reproduced case is safety compute.

## 10. Proposed fixes

Priority 2 — runtime engineering (evidence-supported, semantics-preserving):

- Remove the per-sample rebuild of the filtered obstacle cloud in
  `TrajectorySafetyChecker::Evaluate`; filter once per cycle and reuse it for
  all samples. This directly attacks the measured `C_safety` deadline overrun
  without changing the predicate. Validate with a focused S03 timeout
  experiment before any controller change.

Priority 3 — controller behavior (only after mentor review, separate tuning
config, `m7_baseline.yaml` preserved):

- Turn/gate entry speed management or curvature-aware reference velocity so the
  corner is approached below the speed at which the 1.5 m stopping margin is
  satisfiable.
- Recovery behavior when `N_safe = 0` (for example a braking/recovery hold
  instead of resetting to the zero command), driven by the divergence evidence.
- Warm-start handling on large path-heading change, driven by the warm-start
  correlation.

Do not tune weights blindly. Every item above needs a dedicated tuning config
and a focused re-test.

## 11. Evidence required before M8

- Focused S02/S03/S05/S08 campaigns stable with the existing baseline, or with
  a reviewed replacement baseline plus a new frozen config identity.
- No sustained unexplained `N_safe = 0`, and an understood feasibility margin
  for S03/S05 with reported distributions (no invented threshold).
- A classified baseline-timeout set with zero unexplained deadline misses after
  the safety-evaluation fix.
- All publication-boundary invariants still zero:
  `stale_command_violations = 0`, `setpoint_while_disarmed = 0`,
  `setpoint_in_wrong_mode = 0`, `late_command_accepted = 0`,
  accepted-trajectory collision = false. The focused campaign met all of these.
- Regression: Python tests, C++ Release and strict Debug builds, and the ROS
  layer tests pass.

## Focused campaign safety totals

Across the 50 focused runs the analyzer reports zero accepted-trajectory
collision indications, zero late accepted commands, zero stale/disconnected
publication attempts, zero disarmed publication attempts and zero wrong-mode
publication attempts. The separation of `GOAL_REACHED` from a healthy margin is
confirmed and remains a reliability blocker.

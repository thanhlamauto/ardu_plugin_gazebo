# M7.4 — Stabilization based on confirmed M7.3 evidence

## 1. M7.3 evidence being addressed

This phase acts only on causes confirmed in
[`M7_3_ROOT_CAUSE.md`](M7_3_ROOT_CAUSE.md):

1. S03/S05 infeasibility is dominated by the `predicted_stopping_clearance`
   predicate; at collapse all proposal families are simultaneously infeasible.
2. The later failure is primarily a recovery failure.
3. S08 has no stale-path/version/handoff race.
4. The one reproduced deadline miss was `TrajectorySafetyChecker::Evaluate`
   (cloud work), not scheduling or MPPI rollout/cost.
5. Warm-start magnitude correlates with failure but is not established causal.

The MPPI weights, sample count, horizon, cost terms and safety thresholds are
unchanged. The historical `m7_baseline.yaml` is untouched.

## 2. Performance optimization (Commit A)

`TrajectorySafetyChecker::Evaluate` repeated trajectory-independent obstacle
preprocessing for every sample. M7.4 adds `PreparedCollisionEnvironment`:

- the filtered cloud, known-map expansion and the invalid-input verdict are
  computed once per solve (`TrajectorySafetyChecker::Prepare`);
- an exact KD-tree (`CloudIndex`) answers point-to-segment clearance, so the
  per-trajectory cost no longer scans the whole cloud.

The predicate, first-failure classification and both clearance values are
bit-identical to the previous linear scan. `test_safety_prepared` compares the
new path against a faithful reimplementation of the old algorithm over 80
trajectories x {open space, S03-like corner, S05-like gate} and asserts equality
of `safe`, `reason`, `minimum_collision_clearance_m`,
`minimum_stopping_clearance_m` and safe-sample count.

## 3. Old/new safety timing (Release, 80 samples x 30 steps)

| Snapshot | Old (ms) | New (ms) | Speedup |
|---|---:|---:|---:|
| Open space (400-point cloud) | 73.87 | 67.08 | 1.10x |
| S03-like turn (800-point cloud) | 107.49 | 62.25 | 1.73x |
| S05-like gate (800-point cloud) | 129.53 | 69.72 | 1.86x |

The prepare step itself is 0.13–0.32 ms. The gain is largest where obstacles
are near the trajectory, which is exactly the S03/S05 regime that produced the
observed deadline overrun. Measurements are from
`uav_navigation_core/test/test_safety_prepared.cpp`; the test asserts only a
gross-regression bound because debug-build timings are dominated by index
overhead.

## 4. Candidate controller changes (Commit B)

### Candidate 1 — stopping/turn-aware reference speed shaping

A conservative cap is computed only from planned heading change over a
lookahead, distance to the next major turn, vehicle speed and configured
braking capability. It is applied through a new `CostContext.reference_speed_limit_m_s`
(default infinity reproduces the old speed-limit behavior exactly) and to the
proactive proposal reference speed. It never exceeds the nominal speed, never
drops below the configured minimum and contains no scenario-specific logic.
Logic lives in `uav_navigation_core/stabilization.{hpp,cpp}`.

### Candidate 2 — explicit STOPPING_RECOVERY state

When all samples are rejected and the stopping-distance predicate dominates
(`StoppingDominant`), a bounded deceleration is built along the current
velocity (`BuildBrakingSequence`). It is commanded **only if the existing
safety checker certifies the braking rollout**; otherwise the controller fails
closed to `NO_SAFE_TRAJECTORY` exactly as before. Entry/exit/cycles are tracked
by `StoppingRecoveryTracker` and logged. The conditioner limits and adapter
watchdog are unchanged and `NO_SAFE_TRAJECTORY` is never hidden.

## 5. Why each change follows from evidence

| Change | M7.3 evidence |
|---|---|
| Prepared cloud + exact KD-tree | One deadline miss consumed by `Evaluate` (safety 105.7 ms) |
| Reference speed shaping | Binding predicate is stopping clearance at entry speed; all proposals infeasible |
| STOPPING_RECOVERY | Recovery failure after stopping-dominant collapse |
| No weight/sample/horizon change | M7.3 found no evidence for those levers |

## 6. Parameters added and rationale

All under `local_navigation` in the new configs; units in parentheses.

| Parameter | Meaning |
|---|---|
| `stabilization.speed_shaping.enabled` | master switch (default false) |
| `...nominal_speed_m_s` (m/s) | cap ceiling, default = MPPI vmax; never exceeded |
| `...turn_speed_m_s` (m/s) | target speed inside a turn envelope |
| `...min_speed_m_s` (m/s) | floor of the shaped cap |
| `...lateral_accel_m_s2` (m/s^2) | curvature law `v = sqrt(a_lat * R)` |
| `...braking_accel_m_s2` (m/s^2) | slow-down-to-turn distance law |
| `...reaction_delay_s` (s) | delay added to the braking distance |
| `...turn_angle_rad` (rad) | heading change that marks a major turn |
| `...lookahead_m` (m) | curvature lookahead window |
| `stabilization.stopping_recovery.enabled` | master switch (default false) |
| `...deceleration_m_s2` (m/s^2) | bounded braking ramp |

`nominal_speed_m_s`, `turn_speed_m_s`, `min_speed_m_s`, `lateral_accel_m_s2`
and `braking_accel_m_s2` default to values consistent with the frozen baseline
(vmax 10, accel 3). New configs: `m7_4_speed_shaping.yaml`,
`m7_4_stopping_recovery.yaml`, `m7_4_stabilization.yaml`; each records a
distinct config hash.

## 7–9. S03/S05 A/B and regression-control results

**The focused A/B campaign could not be completed on this host.** The simulation
began dropping the ArduPilot Gazebo FDM stream mid-run ("No JSON sensor message
received, resending servos" in the SITL logs). In an instrumented arm-D probe
(`S03`, seed 7) the vehicle's altitude collapsed from 5.19 m to 0.22 m between
t=33 s and t=37 s while the controller published only `NO_SAFE_TRAJECTORY` (no
descent command), i.e. the estimator/simulator failed, not the controller. Two
further campaign runs failed at startup (FCU connection timeout / not
flight-ready). Such runs cannot be used for a controller comparison.

The valid baseline arm **A** is the M7.3 focused campaign
([`results/m73_root_cause_10467cb_20260919/`](../results/m73_root_cause_10467cb_20260919/)),
which ran the same config hash with the same emergency semantics and is
unchanged by M7.4 when both new features are disabled.

The probe is retained as evidence, not as a result:
[`results/m74_stabilization_c29392e_20260919/probe/`](../results/m74_stabilization_c29392e_20260919/probe/).
Two observations from it are recorded for the next experiment:

- speed shaping activated (minimum cap 1.47 m/s) even though the run is invalid;
- `STOPPING_RECOVERY` never verified a safe braking rollout (0 cycles), i.e.
  under the current predicate the recovery gate is rarely satisfied. This must
  be re-measured on healthy runs before drawing conclusions.

The A/B protocol and analyzer are ready: `scripts/analyze_m74_ab.py` consumes a
root of `A/B/C/D` arm directories, reuses the M7.3 metrics and emits per-arm
scenario, S03 turn-entry and S05 margin tables, labelling passes as
`clean_pass` or `functional_pass_margin_concern`.

## 10. S08 / clock-bridge findings and fix (Commit D)

M7.3 attributed the S08 failure to a `/clock` bridge crash. The underlying cause
is the Fast DDS shared-memory transport failing to initialise on this harness
(`RTPS_TRANSPORT_SHM ... mutex lock failed: Invalid argument`), which killed the
`parameter_bridge` processes for `/clock`, odometry, TF and the obstacle cloud.
The runner now sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` unless the caller
overrides it, records the value in the manifest, and a focused S03 smoke with
the fix recorded zero bridge crashes and reached `GOAL_REACHED`. No controller
code changed.

## 11. Startup/interface findings

- The bridge-crash startup class (S08 seed 37, S09 stale-obstacle, S10 disarm)
  is addressed by the Fast DDS transport fix above; these runs failed before
  their injected fault and must not be counted as fault-handling failures.
- A second, distinct startup class remains: intermittent loss of the Gazebo FDM
  stream. It is host/simulator-level and outside the controller; it blocked the
  A/B campaign. Bounded readiness diagnostics and supervised bridge lifecycle
  are the recommended next robustness work.

## 12. Remaining limitations and gate

- No valid B/C/D controller comparison exists yet; the A/B must be rerun on a
  host with a stable Gazebo/SITL FDM stream.
- `STOPPING_RECOVERY`'s verified-braking gate may be too strict to trigger in
  the confirmed collapse state; this is an open question the next focused
  experiment must answer.
- A new full 115-run campaign is **not** yet justified. The project remains
  **NOT READY FOR M8**.

## Regression status

- Core C++: 9/9 in strict Debug and Release (`-Wall -Wextra -Wpedantic
  -Werror`).
- ROS C++: 2/2 in strict Debug and Release.
- Python: 105 passed + 3 subtests after restoring the pinned `pytorch-mppi`
  0.9.1 (with `scipy`) into the test interpreter. The previous 40 failures were
  entirely `ModuleNotFoundError: pytorch_mppi`, now resolved; no test was
  modified to avoid the import.
- Safety invariants: the M7.3 focused campaign (arm A) recorded zero
  stale/disconnected publish violations, zero setpoints while disarmed, zero
  outside GUIDED, zero late accepted outputs and zero accepted-trajectory
  collision indications. M7.4 does not relax any threshold.

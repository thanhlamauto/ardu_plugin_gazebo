# M7.2 stable-start full-regression report

## Frozen experiment

The campaign ran 115 cases with controller/runtime
`c1e187fe5428bb9af4d515324817848aff3671f0`, harness
`6e35400e53be88ec34a1acdb945cdef7e0a65a23`, and `m7_baseline.yaml` SHA-256
`5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`.
Every manifest records the same two commits and config hash. The vehicle had to
remain armed, in `GUIDED`, above 4 m, and at total speed no greater than 0.3 m/s
for 2 s before goal publication. No controller setting or seed changed after
an outcome was observed, and no failed run was repeated.

The host was an Apple M2 MacBook Air with 8 CPU cores and 16 GB RAM, macOS
26.1, ROS 2 Jazzy, Fast DDS and Gazebo Sim 8.15.0. All runs were performance
runs with RViz, MPPI markers and live plotting disabled.

## Functional results

The campaign passed 102/115 cases (88.7%). Of the 13 recorded failures, nine
were scenario timeouts after the test started and four were startup/interface
failures that never reached the stable flight-ready gate. All 37 fault tests
that reached their injection point observed the required safety state.

| Scenario/variant | Pass | Median goal time (s) | Median path (m) | Median efficiency | Median RMS CTE (m) | Median p95 CTE (m) | Minimum clearance (m) | Peak speed (m/s) | Minimum `N_safe` | Minimum ESS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| S01/base | 10/10 | 13.298 | 33.913 | 0.885 | 0.472 | 0.986 | 4.276 | 8.324 | 80 | 1 |
| S02/base | 9/10 | 12.738 | 40.962 | 0.729 | 2.266 | 4.914 | 0.007 | 6.738 | 0 | 0 |
| S03/base | 7/10 | 10.569 | 22.896 | 0.711 | 0.680 | 1.645 | 0.009 | 9.192 | 0 | 0 |
| S04/base | 10/10 | 14.386 | 38.351 | 0.787 | 0.876 | 1.626 | 0.055 | 7.254 | 0 | 0 |
| S05/base | 7/10 | 15.321 | 38.079 | 0.631 | 2.085 | 4.744 | 0.005 | 8.741 | 0 | 0 |
| S06/base | 10/10 | 26.181 | 155.527 | 0.965 | 0.890 | 2.224 | 3.797 | 10.006 | 80 | 1 |
| S07/base | 5/5 | — | — | — | — | — | — | — | — | — |
| S08/base | 7/10 | 14.992 | 47.969 | 0.769 | 1.248 | 3.176 | 4.036 | 8.617 | 80 | 1 |
| S09/stale_odometry | 5/5 | — | — | — | — | — | — | — | 80 | 1 |
| S09/stale_obstacle | 3/5 | — | — | — | — | — | — | — | 80 | 1 |
| S09/planner_deadline | 5/5 | — | — | — | — | — | — | — | — | — |
| S09/no_safe_trajectory | 5/5 | — | — | — | — | — | — | — | 0 | 0 |
| S10/fcu_disconnect | 5/5 | — | — | — | — | — | — | — | 80 | 1 |
| S10/local_planner_death | 5/5 | — | — | — | — | — | — | — | 80 | 1 |
| S10/wrong_mode | 5/5 | — | — | — | — | — | — | — | 80 | 1 |
| S10/disarm | 4/5 | — | — | — | — | — | — | — | 0 | 0 |

The failed scenario runs were S02 seed 87; S03 seeds 17, 27 and 67; S05 seeds
7, 37 and 87; and S08 seeds 17 and 67. Startup/interface failures occurred in
S08 seed 37, S09 stale-obstacle seeds 37 and 47, and S10 disarm seed 37. The
S09 seed 47 recorder ended during external shutdown without a `run_result`;
its manifest, event log and launch log remain in the raw set.

## Controller timing

Across 14,259 MPPI solves in normal reachable S01-S06/S08 runs, total compute
time was p50 10.014 ms, p95 28.952 ms, p99 41.908 ms and maximum 99.698 ms.
For 11,290 accepted `ACTIVE` outputs, the corresponding figures were 8.943,
22.767, 32.975 and 93.926 ms. No accepted result exceeded 100 ms.

Normal reachable runs nevertheless recorded 53 `PLANNER_TIMEOUT` cycles. The
deliberate S09 planner-deadline fault added five expected timeout cycles. All
58 late/deadline outcomes were rejected, and the late-accepted count was zero.
This fails the M7 runtime criterion of zero baseline deadline misses even
though the measured solve-time maximum remained below 100 ms. It indicates an
end-to-end scheduling/result-availability problem rather than an accepted
over-budget solve.

## Publication and trajectory safety invariants

The primary evidence is the C++ counters checked at the adapter publication
boundary. Totals over all 115 runs were:

| Invariant | Count |
|---|---:|
| accepted-trajectory collision indication | 0 |
| accepted output later than 100 ms | 0 |
| publication after stale/disconnected command | 0 |
| publication while disarmed | 0 |
| publication outside `GUIDED` | 0 |

The collision value is the shared accepted-trajectory safety predicate. The
harness does not record a Gazebo physics contact topic, so it is not a physical
contact-sensor claim.

## S03 stable-start result

The focused stable-start experiment had passed 10/10, but the full frozen
regression passed only 7/10. The three failures accumulated 1,481
`NO_SAFE_TRAJECTORY` cycles, had median RMS CTE 4.999 m, median peak speed 7.694
m/s, median feasible-sample count zero and minimum clearance 0.009 m. The seven
passes accumulated 76 no-safe cycles, with median RMS CTE 0.499 m, median peak
speed 4.056 m/s and median feasible-sample count 78. Stable start removed the
known takeoff-transient confound but did not make the corner reliable.

The full per-run S03 fields are retained in `summary.csv`. Representative
plots are `plots/s03_pass_run01_seed7.png` and
`plots/s03_fail_run02_seed17.png`.

## S05 narrow-passage margin

S05 passed only 7/10. Six of the seven functional passes still entered
`NO_SAFE_TRAJECTORY`; two passing runs reached recorded clearance below 0.1 m.
Across all ten runs the minimum clearance was 0.0049 m and 1,118 no-safe cycles
were recorded. The three failures had median RMS CTE 5.538 m, median peak speed
8.400 m/s and median feasible-sample count zero. This is a functional and
margin failure, not a clean safety pass.

The five-panel plots include clearance, `N_safe`, velocity, best feasible cost
and ESS. Representative files are `plots/s05_pass_run02_seed17.png` and
`plots/s05_fail_run01_seed7.png`.

## Known limitations and decision

Peak acceleration still uses callback-time finite differences and is not a
synchronized vehicle-dynamics measurement. Failures are not deterministic
from MPPI seed alone because simulator timing, sensor delivery and warm-start
history are not captured as replayable state. Three fault cases and one S08
case were not functionally exercised because the simulator/FCU did not become
flight-ready; Fast DDS shared-memory initialization warnings preceded part of
that startup cluster.

Regression verification passed 100 Python tests plus 3 subtests. The C++ core
passed 7/7 tests in both Release and Debug builds, and the ROS adapter layer
passed 2/2 tests in both builds. All four C++ builds used `-Wall -Wextra
-Wpedantic -Werror`.

**Decision: NOT READY FOR M8.** Blocking evidence is S02 9/10, S03 7/10, S05
7/10 and S08 7/10; the severe `N_safe=0`/clearance margin in S03 and S05; 53
baseline planner-timeout cycles; and four startup/interface failures. The
adapter safety boundary behaved correctly in every exercised fault case, but
functional reliability, planning margin and end-to-end deadline reliability
must be resolved and revalidated before edge/HIL/hardware work begins.

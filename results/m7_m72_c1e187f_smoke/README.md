# M7.2 publication-invariant smoke test

Runtime commit `c1e187f` moves the adapter safety counters to the exact C++
setpoint-publication boundary. The controller parameters remain
`m7_baseline.yaml` with SHA-256
`5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`.

Five stable-start smoke runs were executed:

- S01 straight: 1/1 `GOAL_REACHED`;
- S10 `fcu_disconnect`, `local_planner_death`, `wrong_mode`, `disarm`: 1/1
  each reached the expected adapter safety state;
- stale/disconnected, disarmed and wrong-mode publication counters: zero in
  all five runs.

`manifests.jsonl` records the exact runtime/harness/config/world revisions.
The CSV and generated reports are included here; raw events remain under
`/tmp/m7_c1e_smoke_20260918` and `/tmp/m7_c1e_s10_smoke_20260918` on the test
machine.

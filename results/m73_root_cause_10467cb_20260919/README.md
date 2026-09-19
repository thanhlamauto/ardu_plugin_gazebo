# M7.3 focused root-cause campaign

Instrumented focused campaign collected on 2026-09-19 to explain the S03, S05,
S08 and baseline planner-timeout blockers before any controller change.

- Controller/runtime: `10467cba8307530b88f1fb3417ecd8597d9753f6`
- Instrumentation commits: `0356f11`, `10467cb` (on top of `91fab63`)
- Config SHA-256: `5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`
  (unchanged `m7_baseline.yaml`)
- S03: 10 seeds x 3 repetitions; S05: 10 seeds; S08: 10 seeds
- Stable start: total speed at or below 0.3 m/s for 2 s; visualization disabled

Files:

- `run_metrics.csv`: one row per run with turn, margin, rejection and timing
  metrics;
- `planner_timeouts.csv`: every `PLANNER_TIMEOUT` cycle with its timing
  classification;
- `s08_handoff.csv`: `goal_id` / `global_path_id` / local active-path joins;
- `m73_summary.md`: generated summary tables;
- `summary.csv`, `REPORT.md`: the standard M7 analyzer output;
- `manifests.jsonl`: all 50 immutable run manifests with their raw result
  directory names;
- `campaign.log`: the run driver log;
- `plots/`: representative instrumented S03/S05 pass and fail time series.

The 50 focused runs total 15/30 S03, 8/10 S05 and 9/10 S08 passes, with three
interface/startup failures and 15 controller/feasibility failures. Raw
`events.jsonl` and `launch.log` files remain on the campaign machine at
`/tmp/m73_root_cause_10467cb_20260919` (about 30 MB) and are intentionally not
committed.

Findings and proposed fixes are in
[`docs/M7_3_ROOT_CAUSE.md`](../../docs/M7_3_ROOT_CAUSE.md).

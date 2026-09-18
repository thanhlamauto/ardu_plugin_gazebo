# M7.1 frozen campaign artifacts

This directory contains the review-sized artifacts from the M7.1 campaign run
on 2026-09-18. It does not contain the 90 MB raw per-cycle logs.

- Controller/runtime: `7e47663a80c3ef990a2223ed665819035b01cccb`
- Config: `uav_navigation_bringup/config/m7_baseline.yaml`
- Config SHA-256: `5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`
- Performance mode: RViz, candidate markers and live plotting disabled
- Campaign size: 115 runs
- Result: 108/115 terminal expectations met; all safety invariants met; M8
  blocked by recurrent reachable-scenario failures

Files:

- `summary.csv`: one aggregate row per M7.1 run;
- `manifests.jsonl`: reproducibility metadata for all 115 runs, with bulky
  launch command arrays removed;
- `aggregate_report.md`: generated scenario/variant safety summary;
- `m72_exact_rerun_summary.csv`: exact rerun of the seven reachable failures;
- `m72_exact_rerun_report.md`: generated rerun summary;
- `plots/`: representative pass/fail plots for S03 and S05.

The source raw results are retained on the campaign machine at
`/tmp/m7_campaign_7e47663_20260918_combined2`; exact rerun raw results are at
`/tmp/m7_m72_exact_rerun_20260918_v2`. Conclusions and limitations are in
[`docs/M7_VALIDATION.md`](../../docs/M7_VALIDATION.md).

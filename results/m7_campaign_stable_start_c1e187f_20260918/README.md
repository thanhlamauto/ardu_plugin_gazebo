# M7.2 stable-start full regression

This directory contains the committed audit artifacts for the 115-run M7.2
campaign collected on 2026-09-18. The controller and validation configuration
were frozen throughout the campaign.

- Controller/runtime: `c1e187fe5428bb9af4d515324817848aff3671f0`
- Harness: `6e35400e53be88ec34a1acdb945cdef7e0a65a23`
- Config SHA-256: `5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451`
- Stable start: total speed at or below 0.3 m/s continuously for 2 s
- Visualization and live plotting: disabled

Files:

- `summary.csv`: one analyzer row per run;
- `manifests.jsonl`: all 115 immutable run manifests plus their raw result
  directory names;
- `aggregate_report.md`: campaign metrics, failure classification and M8 gate;
- `plots/`: representative S03 and S05 pass/fail time series.

The raw `events.jsonl` and `launch.log` files occupy 118 MB and remain on the
campaign machine at
`/tmp/m7_campaign_stable_start_c1e187f_20260918_v1`. They are intentionally
not committed. The original failures were preserved and were not rerun.

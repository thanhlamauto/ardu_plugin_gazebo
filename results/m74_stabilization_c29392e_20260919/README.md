# M7.4 stabilization — artifacts

Implementation commit: `c29392e` (stabilization), `53f311a` (safety
performance), `b13d53d` (Fast DDS transport fix).

## What is here

- `probe/s03_seed7_armD_cycles.csv`: per-cycle diagnostics from the one
  instrumented arm-D probe run (`S03`, seed 7).
- `probe/manifest_s03_seed7_armD.json`: its immutable manifest.

The probe is **evidence that the host simulator failed**, not a controller
result: the vehicle's altitude collapsed from 5.19 m to 0.22 m between t=33 s
and t=37 s while the controller published only `NO_SAFE_TRAJECTORY` and the
SITL log repeated `No JSON sensor message received, resending servos`. The
focused A/B campaign (arms B, C, D) could not be completed.

The valid baseline arm **A** is the M7.3 focused campaign in
`results/m73_root_cause_10467cb_20260919/` (same `m7_baseline.yaml` hash
`5d6948f7...`; M7.4 is inert with both features disabled).

## Config hashes

| Config | SHA-256 |
|---|---|
| `m7_baseline.yaml` (unchanged) | `5d6948f725483d63d8088c67e367a10828e1776fa606ca60c2b43d1e2f44d451` |
| `m7_4_speed_shaping.yaml` (B) | `578f53566660792ecad4c2b26efaa836b083dbcffb43d9031ee71941a1562798` |
| `m7_4_stopping_recovery.yaml` (C) | `30de989dda7edc8632fda403073873eeaa84d57496923a8573e5b7364c1d6a7d` |
| `m7_4_stabilization.yaml` (D) | `1031ad9da0daaac921b7b08d3ea6f08af5b50c63b7e3cf3e843005724ced19f0` |

Findings are in
[`docs/M7_4_STABILIZATION.md`](../../docs/M7_4_STABILIZATION.md). The A/B
analyzer is `scripts/analyze_m74_ab.py`; it was not run on a complete matrix
because of the blocked campaign.

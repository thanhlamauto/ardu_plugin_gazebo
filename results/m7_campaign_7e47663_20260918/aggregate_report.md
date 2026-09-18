# M7 validation result summary

Generated from immutable per-run `events.jsonl` and `manifest.json` files.

| Scenario | Runs | Success | Collision | Deadline misses | Late accepted | Stale setpoint | Disarmed setpoint | Wrong-mode setpoint | Compute p99 worst (ms) | Min clearance (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| S01/base | 10 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 12.7629 | 4.53088 |
| S02/base | 10 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 14.2127 | 0.091289 |
| S03/base | 10 | 50.0% | 0 | 0 | 0 | 0 | 0 | 0 | 19.047 | 0.003736 |
| S04/base | 10 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 9.84774 | 0.832948 |
| S05/base | 10 | 90.0% | 0 | 0 | 0 | 0 | 0 | 0 | 15.7492 | 0.005928 |
| S06/base | 10 | 90.0% | 0 | 0 | 0 | 0 | 0 | 0 | 12.2992 | 4.01815 |
| S07/base | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 |  |  |
| S08/base | 10 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 6.52549 | 4.60695 |
| S09/no_safe_trajectory | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 5.33996 | 16.158 |
| S09/planner_deadline | 5 | 100.0% | 0 | 5 | 0 | 0 | 0 | 0 |  |  |
| S09/stale_obstacle | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 7.51531 | 3.34421 |
| S09/stale_odometry | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 5.7615 | 4.67957 |
| S10/disarm | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 16.015 | 0.043205 |
| S10/fcu_disconnect | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 7.30266 | 4.60752 |
| S10/local_planner_death | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 5.67575 | 4.51501 |
| S10/wrong_mode | 5 | 100.0% | 0 | 0 | 0 | 0 | 0 | 0 | 9.62032 | 4.54193 |

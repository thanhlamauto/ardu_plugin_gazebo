# Experiment 7A results — 2026-09-16

Offline replay của 20 exact planner snapshots: 12 `N_safe=0` và 8 control.
Mỗi snapshot chạy 20 RNG realizations tại `K=80,160,320,640`, tổng 1.600 solves.

- Failure snapshots: `P_hit=0` ở cả bốn K.
- Control snapshots: `P_hit=1` ở cả bốn K.
- `summary.json`: hit rate và solve time theo nhóm/K.
- `analysis.json`: ESS, cost gap và clearance trên feasible controls.
- `manifest.json`: state selection metadata.
- `solves.csv`: một dòng cho mỗi solve.

Raw NPZ snapshots và Gazebo logs không commit vì dung lượng lớn. Tạo lại bằng
các lệnh trong README gốc hoặc `docs/MPPI_MENTOR_CHECKPOINT_AND_NEXT_PHASE_VI.md`.

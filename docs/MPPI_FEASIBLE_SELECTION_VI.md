# Feasibility mask trước MPPI weighting

Tiếp theo: [checkpoint mentor và Experiment 7A với 1.600 replay](MPPI_MENTOR_CHECKPOINT_AND_NEXT_PHASE_VI.md).

Ngày 16/09/2026. Thay đổi này xử lý mismatch đã thấy ở seed 7/cycle 160: sample không brake-safe từng thắng cost, trong khi 148/160 sample safe nhận tổng weight bằng 0.

## Thay đổi kiến trúc

Pipeline thử nghiệm hiện là:

```text
sample U → conditioner trong dynamics → rollout
         → shared swept/cloud/prior-SDF/stopping predicate
         → unsafe sample có weight = 0
         → normalize weight trên feasible set
         → weighted nominal
         → final gate dùng lại cùng predicate
         → nếu weighted nominal fail nhưng N_safe>0: chọn best feasible sample và gate lại
```

`trajectory_safety.py` là predicate dùng chung cho sample mask, final nominal và analyzer. Nó kiểm tra mọi predicted state. Prior SDF dùng phép giao liên tục giữa segment và solid được nở theo clearance thay vì chỉ hỏi các điểm rollout. Cloud point không thuộc prior map vẫn được kiểm tra theo đoạn–điểm. Với cloud point đã khớp prior SDF trong tolerance 0.1 m, map được nở thêm bằng residual lớn nhất quan sát được; cách này tránh tính trùng hàng triệu cặp nhưng vẫn mang sai số cloud vào clearance.

Cycle 160 lịch sử vẫn cho 148/160 sample feasible. Snapshot lưu weight cũ nên mass của safe set vẫn bằng 0 và sample 73 unsafe thắng; regression mới chạy lại weighting trên cùng pool và yêu cầu:

- `N_safe = 148`;
- weight mass trên safe set = 1;
- sample nhận weight lớn nhất phải feasible;
- best-feasible fallback phải qua cùng predicate.

Không dùng stopping penalty cực lớn để giả lập constraint. Soft cost cũ vẫn tồn tại như performance term; quyền tham gia weighting do feasibility mask quyết định.

## Kết quả headless

Profile 160 sample đúng logic nhưng vượt budget khi chạy đồng thời Gazebo: một lượt seed 7 có compute trung bình≈104 ms và 142 `hold-timeout`. Kết quả đó chỉ dùng để phát hiện bottleneck, không đánh giá navigation. Cache segment global-path và bỏ kiểm tra nominal trùng giúp giảm thời gian, nhưng 160 sample vẫn có tail cao trên máy này.

Screening hiệu năng sau đó chỉ đổi `samples: 160 → 80`; objective, horizon, `lambda`, dynamics và safety predicate giữ nguyên.

| Seed | Reached / LAND | Sim time | Peak XY | Min center clearance | N_safe=0 holds | Timeout holds | Compute mean / p95 / max |
|---:|---|---:|---:|---:|---:|---:|---:|
| 7 | Có / Có | 19.55 s | 9.19 m/s | 1.853 m | 21 | 0 | 42.7 / 50.2 / 88.7 ms |
| 17 | Có / Có | 18.70 s | 8.98 m/s | 2.355 m | 11 | 0 | 37.0 / 46.0 / 65.2 ms |

Trong cả hai lượt:

- Không có cycle nào `N_safe>0` nhưng final gate vẫn gửi hold-invalid.
- Weight mass trên feasible set bằng 1 ở mọi cycle có nghiệm.
- Không có `hold-brake` hoặc optimizer timeout.
- Các hold-invalid còn lại đều đúng lúc `N_safe=0`.
- Feasible ESS vẫn tập trung mạnh: median 1.0; p95 seed 7≈1.003, seed 17≈1.00003. Đây mới là bằng chứng hợp lệ để điều tra cost scaling/`lambda` sau này, nhưng chưa tune trong phase này.

Hai lượt đạt đích không chứng minh safety thực. Tốc độ đỉnh chỉ 8.98–9.19 m/s và không giữ ≥9.5 m/s; `collision_radius_m=1.5` vẫn là center clearance chưa hiệu chuẩn rotor/estimator. Prediction phanh live chưa được chứng minh bảo thủ dù test offline acceleration-memory từng bảo thủ trên 40 pha.

## Phần chưa hoàn thành

Recovery khi `N_safe=0` chưa được bật. Hiện hệ thống vẫn gửi zero và ghi `hold-invalid-stopping-trajectory`; zero setpoint không phải verified emergency trajectory. Cần thiết kế và kiểm chứng backup rollout riêng trước khi thay hành vi này.

Do đó trạng thái DoD mới là:

| Điều kiện | Trạng thái |
|---|---|
| Cycle 160: safe samples không còn zero weight | PASS bằng regression weighting |
| Optimizer và final gate dùng cùng predicate | PASS ở code/test và hai flight |
| Có feasible sample thì output cuối feasible | PASS trong hai flight |
| Không có feasible sample thì có verified recovery | Chưa đạt |
| Live stopping predictor bảo thủ | Chưa đạt |
| Body/rotor + estimator + latency margin rõ ràng | Chưa đạt |

## Dữ liệu và tái lập

- [Seed 7 summary](../output/benchmark/yard_progress_feasible80_20260916_v1/summary.json)
- [Seed 17 summary](../output/benchmark/yard_progress_feasible80_20260916_seed17_v1/summary.json)
- [Cycle 160 unified analysis](../output/benchmark/yard_stopguard_pool_cycle160_20260916_v1/v10.0_seed7/mppi_sample_pool_cycle160_analysis.json)
- [Profile 80 sample](../config/experiments/mppi_yard_progress_feasible80.yaml)
- [Regression](../tests/test_nominal_stopping_guard.py)

```bash
python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_progress_feasible80.yaml \
  --params config/experiments/mppi_yard_high_accel.parm --timeout 45 \
  --output output/benchmark/yard_progress_feasible80_repeat
```

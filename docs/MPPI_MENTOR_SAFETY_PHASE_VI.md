# Kiểm chứng tầng an toàn MPPI trước khi tune tiếp

Tiếp theo: [feasibility mask trước MPPI weighting và hai lượt headless](MPPI_FEASIBLE_SELECTION_VI.md).

Ngày 16/09/2026. Mục tiêu: kiểm tra interface **prediction → feasibility → execution** ở bài cua bãi container, retiming tắt. Đây là cấu hình nghiên cứu, chưa phải profile bay 10 m/s đã xác nhận an toàn.

## Kết luận hiện có

- Đã thêm hard stopping guard sau conditioner và collision gate. Nó kiểm tra đoạn dừng tại **mọi state** của nominal rollout bằng cùng transition `_dynamics/_advance_applied` với MPPI. Regression từ log seed 7/cycle 70: collision cloud và SDF vẫn PASS, nhưng stopping clearance 1.357 m < ngưỡng 1.5 m; lệnh nominal nay bị từ chối và gửi zero.
- Hard guard ngăn lệnh đó nhưng **không sửa được navigation**: hai lượt headless seed 7/17 đều TIMEOUT ở x≈47 m, có 247/205 chu kỳ `hold-invalid-stopping-trajectory`. Không có optimizer timeout. Việc chặn một nominal không tạo ra một phương án phanh/rẽ khả thi; lặp zero cũng chưa được chứng nhận là safe backup.
- Thử phanh độc lập Gazebo + ArduPilot, đường trống, 10 lần cho mỗi mức 4/6/8/10 m/s. Công thức guard `v×0.25 + v²/(2×3)` **dự đoán thiếu** quãng dừng ở cả 40 lượt, mức thiếu lớn nhất theo tốc độ là 0.482/0.379/1.015/1.302 m. Vì vậy chưa thể xem `hard_brake_m=1.5` là safety margin được hiệu chuẩn.
- Surrogate có acceleration memory, khi khởi tạo bằng gia tốc từ ground truth và giả định delay 0.25 s, dự đoán quãng dừng dài hơn thực tế ở 40/40 lượt. Đây là phép tính offline với thông tin tốt hơn estimator live; **chưa chứng minh** guard live bảo thủ.

## Phép thử phanh độc lập

World thẳng 4000 m, một lần boot SITL/Gazebo và 40 pha cruise→zero liên tiếp; 10 lần mỗi tốc độ **không phải** 10 lần boot độc lập. Gửi setpoint 10 Hz, giữ gần steady ít nhất 1 s, rồi gửi zero đến khi vận tốc <0.25 m/s. Ground truth odometry và target echo MAVLink được ghi theo timestamp. Vận tốc/gia tốc phân tích từ vị trí đã nội suy 0.02 s và đạo hàm trung tâm cửa sổ 0.2 s; các số này có sai số lấy mẫu và lọc.

| Cruise yêu cầu | Vận tốc tại t0, median | Quãng dừng thực, median | Underprediction công thức guard, median / max | Underprediction surrogate, median |
|---:|---:|---:|---:|---:|
| 4 m/s | 3.94 m/s | 4.00 m | +0.428 / +0.482 m | −0.892 m |
| 6 m/s | 5.99 m/s | 7.81 m | +0.325 / +0.379 m | −1.453 m |
| 8 m/s | 7.97 m/s | 13.48 m | +0.888 / +1.015 m | −1.479 m |
| 10 m/s | 9.92 m/s | 20.10 m | +1.217 / +1.302 m | −1.999 m |

Underprediction = `actual_stop_distance − predicted_stop_distance`; số dương là phía nguy hiểm. Mọi pha đều quan sát được mốc <0.25 m/s. Echo target có độ trễ quan sát thường 0.02–0.10 s sau send, nhưng telemetry echo **không phải** timestamp motor bắt đầu thực thi. Giảm tốc đo được bắt đầu sau khoảng 0.19–0.25 s. Khoảng này gộp gửi lệnh, ArduPilot, động lực học và phương pháp phát hiện gia tốc; chưa tách được receive/execute latency nội bộ. Không dùng percentile của 40 pha này làm margin cuối cùng cho tình huống rẽ/có vật cản.

## Collision và giới hạn hard guard

Final collision gate dùng khoảng cách cloud tới **toàn đoạn** nối hai điểm rollout, không chỉ điểm đầu/cuối. Known SDF được sample mỗi tối đa 0.1 m trên đoạn và trừ nửa khoảng cách mẫu theo tính chất 1-Lipschitz để lấy lower bound. Stopping segment dùng cùng cách lấy bound cho SDF, và khoảng cách hình học đoạn–điểm chính xác với cloud. `collision_radius_m=1.5` là clearance của **tâm** tới vật cản; chưa xác nhận đây là envelope rotor/pose uncertainty đầy đủ. Không thấy vật cản trong cloud cũng không chứng minh free space; prior SDF chỉ bao phủ vật cản đã biết của world.

Hard guard hiện dùng `stopping_guard_uncertainty_m=0` và mô hình đoạn dừng thẳng với `a=3 m/s²`, `delay=0.25 s`. Vì kết quả phanh thực cho thấy công thức này thiếu tới 1.302 m, profile hard guard chỉ là regression/diagnostic. Thêm margin phải dựa trên dữ liệu đáp ứng, estimator và hình học body; tăng số 1.5 tùy ý có thể khiến loop giữ zero nhiều hơn mà vẫn chưa có backup hợp lệ.

## Sample pool và bước quyết định tiếp

Debug snapshot tại một cycle ghi toàn bộ raw sampled actions, conditioned sample actions, predicted states, cost tổng, weights, RNG trước solve, state/memory, cloud, path, nominal và lệnh sent. `scripts/analyze_mppi_sample_pool.py` tính cost thành phần và kiểm tra collision/stopping cho **từng sample** offline. Điều này nhằm phân biệt không có safe sample, safe sample có weight thấp, hay weighted nominal trở thành unsafe. Dù đã có kết quả ở hai cycle, chưa thay `lambda`, noise, horizon hoặc reward trước khi hoàn tất kiểm chứng model phanh.

Ở lượt baseline mới seed 7/cycle 70 (x=41.35 m, **khác state** của regression lịch sử x=40.11 m), 11/160 sample qua cả cloud/SDF/stopping check; sample tốt nhất nắm gần toàn bộ weight (ESS=1) và là sample an toàn. Nominal conditioned cũng an toàn, min stopping clearance≈1.966 m, được gửi. Đây là bằng chứng rằng tại state này **có** safe proposal, không phải bằng chứng cho mọi trạng thái trước cua. [Snapshot và phân tích cycle 70](../output/benchmark/yard_progress_pool_cycle70_20260916_v1/v10.0_seed7/mppi_sample_pool_cycle70_analysis.json).

Ở lượt hard-guard mới seed 7/cycle 160 (x=39.23 m, v≈0.56 m/s), nominal bị từ chối: stopping clearance≈1.264 m. **148/160 sample** qua cả ba kiểm tra, nhưng tổng normalized weight của tập safe bằng 0 sau khi underflow; ESS=1. Sample có cost thấp nhất chiếm gần toàn bộ weight và không an toàn. Sample safe tốt nhất có optimizer cost cao hơn **1052.37**, chủ yếu do progress cost kém hơn **1014.02**. Soft stopping cost của cả hai bằng **0** vì nó chỉ xét cloud, còn hard guard thấy vật cản trong prior SDF. Đây là bằng chứng cụ thể cho nhánh “có safe sample nhưng selection/cost không chọn”, **ở state này**. Không suy ra toàn bộ flight chỉ do `lambda`: objective soft và hard feasibility không cùng hình học, đồng thời stopping model/margin chưa được hiệu chuẩn. [Snapshot và phân tích cycle 160](../output/benchmark/yard_stopguard_pool_cycle160_20260916_v1/v10.0_seed7/mppi_sample_pool_cycle160_analysis.json).

Chưa đạt Definition of Done đầy đủ của mentor: cycle70 regression PASS; braking envelope, effective execution latency và body-aware safety certificate còn mở. Hai seed timeout với guard là bằng chứng rõ rằng cần thiết kế candidate/backup khả thi hoặc tích hợp feasibility vào selection, nhưng hướng sửa phải dựa trên sample pool và mô hình phanh đã hiệu chuẩn.

## Tái lập

```bash
cd ~/Projects/ardupilot_gazebo
PY=/opt/miniconda3/envs/ardupilot-rviz/bin/python
$PY scripts/run_brake_primitive.py --speeds 4 6 8 10 --repeats 10 --output output/benchmark/brake_primitive_repeat
$PY scripts/analyze_brake_primitive.py output/benchmark/brake_primitive_repeat
```

[Log 40 pha phanh](../output/benchmark/brake_primitive_20260916_10rep/analysis.json), [summary hai seed hard guard](../output/benchmark/yard_progress_stopguard10_20260916_v1/summary.json), [config hard guard](../config/experiments/mppi_yard_progress_stopguard10.yaml), [regression test](../tests/test_nominal_stopping_guard.py).

# Tuning MPPI-only: reference 10 m/s, lấy đà 60 m

## Phạm vi

Tắt retiming (`reference_accel_m_s2=0`), giữ reference 10 m/s, world/đường
lấy đà 60 m, brake và giới hạn điều khiển cũ. Ba ứng viên đầu giữ collision
radius 1.5 m; hai lượt bổ sung thử cost radius 2.5 m.
Không gọi kết quả có retiming là MPPI-only. Giữ bo hình học của đường như
baseline; không thêm lịch tốc độ theo cua.

Baseline: `mppi_yard_free10.yaml`, hai seed 7/17 đạt peak khoảng 9.84 m/s
nhưng đều abort vùng container. Đây là mốc thất bại để so sánh.

## Các ứng viên đã screening seed 7

| Profile | H / N | Lambda | Noise XY | w_path / w_reference_velocity | Warm start reference | Phạt đổi lệnh XY |
|---|---|---|---|---|---|---|
| Baseline | 30 / 350 | 1 | 0.8 | 400 / 40 | Có | 100 / 100 |
| tune10_explore | 40 / 500 | 1000 | 3 | 400 / 40 | Có | 100 / 100 |
| tune10_balanced | 40 / 500 | 100 | 3 | 40 / 4 | Không | 100 / 100 |
| tune10_cautious | 40 / 500 | 100 | 3 | 10 / 1 | Không | 300 / 300 |

Các file là `config/experiments/mppi_yard_<profile>.yaml` cho ba ứng viên.
Đây là screening thay nhiều biến, không phải ablation từng hyperparameter.
Không khẳng định các giá trị này tối ưu. H dài và N lớn có thể tăng thời gian
compute; phải đo deadline thực tế cùng kết quả bay.

## Cách đánh giá

Chạy tuần tự seed 7 để sàng lọc. Ứng viên tốt cần xác nhận seed 17 và so với
baseline. Báo riêng: tới đích/LAND, brake, clearance, peak/giữ gần 10 m/s,
tốc độ vùng cua, deadline và timeout. Bay chậm hết đường không đồng nghĩa
thành công ở 10 m/s; abort không phải lượt bay thành công.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 \
  --config config/experiments/mppi_yard_tune10_explore.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 90 --output output/benchmark/yard_tune10_explore_my_run
```

Dùng output mới cho mỗi nhóm. Phiên GUI/SITL cũ đã được dừng theo yêu cầu
của người dùng, sau đó đã chạy tuần tự các lượt bên dưới.


## Kết quả screening ngày 15/09/2026

**Chưa tìm được cấu hình thành công.** Năm ứng viên, seed 7, đều kết thúc
`ABORT_GEOMETRIC_CLEARANCE_LT_1M`, không tới đích. Chưa có ứng viên để xác nhận
seed 17. Kết quả không chứng minh mọi bộ tham số MPPI đều thất bại.

Hai ứng viên bổ sung sau ba lượt đầu:

- `tune10_margin`: giữ balanced, chỉ tăng collision cost radius 1.5 → 2.5 m.
- `tune10_tau`: giữ margin, chỉ đổi tau mô hình 0.5 → 1.5 s. Đây là độ nhạy,
  không phải nhận dạng động lực học. Không thay giới hạn ArduPilot.

| Profile | Peak XY (m/s) | Brake cycles | Compute TB (ms) | LAND/disarm |
|---|---|---|---|---|
| explore | 9.285 | 15 | 18.49 | Xác nhận |
| balanced | 9.626 | 17 | 18.95 | Xác nhận |
| cautious | 8.904 | 16 | 17.98 | Xác nhận |
| margin | 9.358 | 10 | 18.48 | Xác nhận |
| tau | 9.468 | 10 | 18.32 | Không xác nhận trong 35 s |

Không optimizer/cycle deadline miss hoặc timeout-hold trong năm lượt. Các
peak là trước abort, không phải bay hoàn tất hay giữ 10 m/s xuyên cua.
Guard footprint nới theo trục không tương đương Euclid 1 m; không có contact
sensor. Lượt tau không được báo hạ cánh thành công. Các process đã dừng.

Tất cả lượt đọc lại WP_SPD=10/WP_ACC=3. Mỗi thư mục giữ log, config, world,
source snapshot và manifest đã đối chiếu SHA-256. Không thay profile demo
retiming bằng ứng viên thất bại, cũng không coi demo retiming là MPPI-only.

[Số đo tổng hợp](../output/benchmark/yard_tune10_screening_20260915.json).
Các thư mục là `output/benchmark/yard_tune10_<profile>_20260915_v1`, có
`summary.csv`, `cruise_check.json` và `comparison.png` riêng.

Thử tăng lấy mẫu/horizon, đổi temperature/noise, hạ tracking weights, tăng
phạt đổi lệnh, tăng collision margin và đổi tau chưa đủ. Trước khi tiếp tục
quét rộng, cần kiểm tra đường nominal hợp lệ và mô hình đáp ứng thực; chưa
có cơ sở để gọi một bộ setting là hợp lý cho 10 m/s trong bài này.

# Sửa reset/proposal sau rejection — 15/09/2026

Đã hết tình trạng đứng trước cua đến timeout trong **2/2 lượt headless**
seed 7 và 17. Giữ timeout **45 s**, reference **10 m/s**, retiming **tắt**.
Vẫn dùng prior map SDF và mô hình gia tốc/phanh của cấu hình map-response.

## Sửa gì

- Khi gate từ chối, ghi nhận lệnh zero đã gửi, không reset về warm-start
  reference 10 m/s ở mỗi chu kỳ nữa.
- Chỉ khởi tạo U bằng zero một lần khi bắt đầu recovery; các lần từ chối
  tiếp theo giữ chuỗi U đã tối ưu để tiếp tục cải thiện.
- Bổ sung proposal zero và đi theo path ở 0.5, 1, 2, 4 m/s vào cùng ngân sách
  mẫu MPPI. Đây là các ứng viên điều khiển, không thay reference hay retiming.
- MPPI vẫn chọn bằng objective hiện có; quỹ đạo cuối vẫn phải qua gate cloud
  và SDF. Recovery được giữ đến khi đặt route mới.

## Kết quả Gazebo/SITL

| Chỉ số | Seed 7 | Seed 17 |
|---|---:|---:|
| Kết thúc | Tới đích, LAND/disarm | Tới đích, LAND/disarm |
| Thời gian bay mô phỏng | 19.856 s | 21.590 s |
| Tốc độ XY đỉnh | 9.615 m/s | 9.708 m/s |
| Khoảng cách tâm UAV tới vật cản nhỏ nhất | 1.624 m | 1.793 m |
| Chu kỳ hold-invalid-trajectory | 23 | 20 |
| Chu kỳ hold-brake | 0 | 14 |
| Chu kỳ recover-brake | 0 | 18 |
| Thời gian liên tục trong ±5% của 10 m/s | 0.8 s | 1.9 s |

Tất cả dòng `command` đều có final validation hợp lệ; các lần rejection
đều gửi zero. Không có deadline miss hay timeout-hold trong hai lượt này.
Cả hai có một `hold-stale` lúc khởi động.

**Giới hạn:** mới thử hai seed, chưa chứng minh độ tin cậy rộng hơn. Vẫn giảm
tốc mạnh và có giữ/phanh; chưa đạt tiêu chí cruise gần 10 m/s liên tục 5 s.
Khoảng cách trên là khoảng cách hình học từ tâm UAV, không phải contact sensor.
Đây là MPPI có bổ sung proposal recovery, prior map và surrogate đáp ứng,
không phải cấu hình MPPI nguyên bản.

Kiểm thử trong lượt sửa: **55 passed** (49 core, 4 final validation,
2 rejection/proposal). Đã đối chiếu source snapshot của hai lượt.

## Chạy lại

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_map_response10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 45 \
  --output output/benchmark/yard_rejection_recovery10_repeat
```

[Kết quả](../output/benchmark/yard_rejection_recovery10_20260915_v1/summary.json)
· [Đồ thị](../output/benchmark/yard_rejection_recovery10_20260915_v1/comparison.png)
· [Kiểm tra gate](../output/benchmark/yard_rejection_recovery10_20260915_v1/gate_check.json)
· [Chẩn đoán trước sửa](MPPI_HOLD_LOOP_DIAGNOSIS_VI.md).

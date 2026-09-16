# MPPI chủ động thử giảm tốc–rẽ, retiming tắt

## Thay đổi

`proactive_proposals: true` đưa ứng viên vào MPPI trước khi gate từ chối:
zero, các mức đi chậm theo path, và 10 lịch tăng/giảm tốc từ vận tốc đo hiện
thời tới 1/2/4/6/10 m/s với gia tốc proposal 1.5 hoặc 3 m/s². Vị trí lấy mẫu
trên path được dùng để tạo hướng lệnh qua cua. Không sửa reference thời gian
10 m/s và không gửi trực tiếp proposal xuống ArduPilot; tất cả vẫn qua model,
conditioner, objective và final gate. Tổng ngân sách vẫn 350 samples.

Cấu hình thử tăng `paper_r_delta_u` XY từ 100 lên 400 để phạt thay đổi lệnh
đã qua conditioner, gồm bước đầu so với lệnh thực chu kỳ trước.

Ứng viên `mppi_yard_proactive_margin10.yaml` thêm `collision_cost_buffer_m=0.55`:
cost tránh vùng cách vật cản <=2.05 m, còn gate giữ radius 1.5 m. Đây là
khoảng đệm tối ưu thử nghiệm, không phải chứng minh tương đương swept check
và không bảo đảm mọi proposal qua gate. Mặc định buffer=0, proactive=false
nên cấu hình recovery cũ giữ nguyên hành vi.

## Thử nghiệm

Cùng world lấy đà 60 m, reference 10 m/s, prior map, model gia tốc/phanh,
retiming tắt, timeout 45 s, seeds 7 và 17.

Ứng viên chỉ proactive + tăng phạt đổi lệnh (`yard_proactive10_20260915_v1`)
tới đích cả hai nhưng rejection tăng 23/20 → 46/63. Không chọn làm mặc định.
Seed 17 được khởi động khi thêm field buffer mặc định 0 vào source; thay đổi
này không bật buffer cho ứng viên đầu, nhưng snapshot đầu đợt không hoàn toàn
trùng source tại thời điểm khởi động seed 17. Đợt margin sau dùng source cố định.

## Kết quả cấu hình có buffer 0.55 m

| Chỉ số | Recovery cũ seed 7 / 17 | Proactive + buffer seed 7 / 17 |
|---|---:|---:|
| Tới đích và LAND/disarm | 2/2 | 2/2 |
| Thời gian mô phỏng (s) | 19.856 / 21.590 | 19.312 / 18.054 |
| Tốc độ đỉnh (m/s) | 9.615 / 9.708 | 9.645 / 9.680 |
| RMS thay đổi lệnh (m/s²) | 8.200 / 8.817 | 6.333 / 6.141 |
| Hold-invalid | 23 / 20 | 11 / 11 |
| Hold-brake | 0 / 14 | 12 / 12 |
| Recover-brake | 0 / 18 | 0 / 0 |
| Clearance tâm tới vật cản (m) | 1.624 / 1.793 | 2.017 / 2.084 |

RMS thay đổi lệnh giảm 23–30%, nhưng tổng chu kỳ gửi zero do invalid/brake
ở seed 7 vẫn 23; seed 17 giảm từ 34 xuống 23 (18 recover-brake cũ là trạng
thái riêng). Không thể nói đã loại bỏ stop–go. RMS gia tốc thực không giảm
đồng đều: mới 2.231/2.276 so với cũ 2.242/2.217 m/s².

Đường bay mới dài 87.35/84.56 m: khoảng đệm có thể làm đường vòng rộng hơn.
Mới thử 2 seed ở 10 m/s; chưa kiểm chứng 5 m/s, GUI hoặc ngang độ mượt của
retiming. Seed 7 có 3 cycle deadline misses, seed 17 không có; cả hai không
có optimizer deadline miss hoặc timeout-hold. Tất cả `command` qua final gate.

Kiểm thử: 58 test khác nhau đã qua (49 core, 4 final validation, 5 proposal).
Không thay mặc định quickstart; cấu hình mới là lựa chọn thử riêng.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --gui --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_proactive_margin10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 45 \
  --output "output/benchmark/yard_proactive_gui_$(date +%Y%m%d_%H%M%S)"
```

Bỏ `--gui` để chạy headless. Trong quy trình 5 terminal, thay riêng `--config`
bằng file proactive-margin trên và đổi tên log để tránh lẫn kết quả.

[Kết quả](../output/benchmark/yard_proactive_margin10_20260915_v1/summary.json)
· [Đồ thị](../output/benchmark/yard_proactive_margin10_20260915_v1/comparison.png)
· [Gate](../output/benchmark/yard_proactive_margin10_20260915_v1/gate_check.json).

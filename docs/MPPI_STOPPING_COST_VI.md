# Thử sửa brake bằng cost quãng dừng — 16/09/2026

## Nguyên nhân và thay đổi

Ở bản proactive-margin, seed 7 lần đầu brake tại x=54.31 m, vận tốc 6.52 m/s.
Khoảng cách hiện tại tới cloud 2.65 m nhưng đoạn dừng dài 8.72 m chỉ cách
cloud 1.17 m, dưới ngưỡng brake 1.5 m. Cost va chạm trên quỹ đạo không trực
tiếp phạt trạng thái sẽ kích hoạt guard quãng dừng này.

Thêm cost `w_stopping * max(0, stopping_margin_m - clearance)^2` cho các
trạng thái dự báo cách nhau 5 bước (0.5 s với dt=0.1). Clearance là khoảng
cách cloud tới đoạn dừng theo vận tốc dự báo, cùng công thức guard:
`length = speed * delay + speed² / (2 * max_accel_xy)`.

Cấu hình thử `mppi_yard_stopping10.yaml`: trọng số 1e6, margin 2.5 m,
delay 0.25 s; không đổi guard 1.5 m, retiming vẫn tắt. Cost này không đảm
bảo tất cả trạng thái đều an toàn; gate/brake vẫn hoạt động mỗi chu kỳ.
Prior map vẫn dùng trong cost va chạm và final gate, không dùng trong cost
quãng dừng cuối cùng. `w_stopping=0` mặc định giữ hành vi cấu hình cũ.
Diagnostics có mục `cost.stopping` trong tổng nominal objective.

## Các đợt thử

- v1: tính quãng dừng với cloud + prior map ở mọi bước quá nặng (~136 ms).
  Dừng có LAND/disarm; harness ghi SETUP_FAILED vì bị ngắt, không phải lỗi
  cài đặt. UAV bị hold-timeout tại điểm đầu.
- v2: cloud + map mỗi 5 bước, hai seed tới đích nhưng có 90/49 cycle deadline
  misses và seed 7 vẫn 21 hold-brake. Không chọn cấu hình này.
- v3: chỉ cloud cho cost quãng dừng, tăng margin/trọng số, chạy lại hai seed.

Các đợt đều timeout 45 s, reference 10 m/s, seeds 7/17, lấy đà 60 m.
Source/config snapshot nằm trong từng thư mục benchmark. Các sửa source
cho v3 thực hiện sau khi planner seed 17 của v2 đã khởi động.

## Kết quả v3 (cloud, horizon 3 s)

| Chỉ số | Seed 7 | Seed 17 |
|---|---:|---:|
| Tới đích + LAND/disarm | Có | Có |
| Hold-invalid | 0 | 9 |
| Hold-brake | 12 | 15 |
| RMS thay đổi lệnh m/s² | 4.678 | 5.675 |
| Peak m/s | 9.599 | 9.496 |
| Clearance tâm nhỏ nhất m | 1.946 | 2.268 |
| Cycle deadline misses | 8 | 15 |

So với proactive-margin cũ (11 rejection và 12 brake mỗi seed), cải thiện
không đồng đều: seed 7 hết rejection nhưng seed 17 tăng brake. Chưa chọn làm
mặc định quickstart. Không coi việc giảm rejection là đã sửa xong brake.

Ở seed 7 v3, lệnh giảm từ 7.93 m/s tại x=42.60 xuống 4.78 m/s tại x=53.31,
nhưng vận tốc thực tương ứng 9.58 và 6.67 m/s. Đợt brake bắt đầu x=53.93,
v=6.36, đoạn dừng 8.36 m, clearance 1.36 m. Có bằng chứng lệnh đã giảm trước
cua, nhưng tốc độ thực chưa giảm đủ để tránh guard. Chưa đủ dữ liệu để quy
riêng lỗi cho surrogate; cần so rollout với đáp ứng qua toàn bộ pha phanh.

[Summary v3](../output/benchmark/yard_stopping10_20260916_v3/summary.json)
· [Các đợt brake](../output/benchmark/yard_stopping10_20260916_v3/brake_check.json).

Kiểm thử source cuối: 61 passed (+3 subtests). Gate/brake không bị tắt,
không tăng timeout hay bật retiming. Các cấu hình mới là thử nghiệm.

## Thử horizon 4 s / 200 samples

`mppi_yard_stopping_long10.yaml`, cùng cost cloud của v3.
Cả hai tới đích và LAND/disarm, không hold-timeout, mọi command qua final gate.

| Chỉ số | Seed 7 | Seed 17 |
|---|---:|---:|
| Hold-invalid | 1 | 1 |
| Hold-brake | 15 | 11 |
| RMS thay đổi lệnh m/s² | 4.296 | 4.393 |
| Peak m/s | 9.096 | 9.319 |
| Thời gian mô phỏng s | 18.70 | 19.89 |
| Clearance tâm nhỏ nhất m | 2.167 | 2.156 |
| Cycle deadline misses | 6 | 7 |

Có cải thiện RMS lệnh 28–32% so với proactive-margin, nhưng brake chưa giảm
đồng đều và không đạt peak >=9.5 m/s. Không thay mặc định quickstart.
Chưa chạy 5 m/s hay GUI với cấu hình này. Các kết quả chỉ gồm 2 seed mỗi cấu
hình, không chứng minh hết brake hay độ tin cậy tổng quát.

[Summary horizon 4 s](../output/benchmark/yard_stopping_long10_20260916_v1/summary.json)
· [Đồ thị](../output/benchmark/yard_stopping_long10_20260916_v1/comparison.png).

Để lặp đúng cấu hình thử (không phải cấu hình đã sửa hết brake):

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_stopping_long10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 45 \
  --output "output/benchmark/yard_stopping_repeat_$(date +%Y%m%d_%H%M%S)"
```

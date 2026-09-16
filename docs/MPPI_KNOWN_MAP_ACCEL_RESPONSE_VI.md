# Bổ sung vật cản ngoài cloud và mô hình gia tốc/phanh — 15/09/2026

## Đã làm, retiming vẫn tắt

Thêm profile thử nghiệm `config/experiments/mppi_yard_map_response10.yaml`,
dùng đúng world `iris_mppi_yard_runup60.sdf`. Giữ weights MPPI của baseline,
reference 10 m/s, conditioning và các ngưỡng brake. Không tune cost để che lỗi.

### Vật cản ngoài cloud

Code downsample giữ điểm gần nhất theo ô góc rồi lấy tối đa 200 điểm gần nhất.
Điều đó có thể loại điểm xa; chưa có cloud gốc để tách với che khuất/FOV.
Không giả định phần ngoài cloud là free space nữa đối với map đã biết này:

- `known_obstacle_sdf` cung cấp hình học box/cylinder 3D từ SDF cho collision
  cost MPPI và gate cuối, bổ sung cho LiDAR (không thay LiDAR).
- Gate xét đoạn quỹ đạo với SDF bằng lấy mẫu tối đa 0.1 m rồi trừ nửa khoảng
  mẫu để có cận dưới clearance theo tính Lipschitz của hàm khoảng cách.
- Log ghi đường dẫn/hash bản đồ; nguồn này là **prior map đã biết**, không
  phải vật cản vừa được LiDAR quan sát. World/map trong đợt này trùng nhau.
- Không giải quyết vật cản mới/di động hoàn toàn chưa quan sát trong môi trường
  chưa biết. Không tự xây occupancy map từ LiDAR hoặc khẳng định toàn bộ vùng
  chưa quan sát đã an toàn. Chỉ dùng profile này với đúng map được chỉ định.

### Mô hình gia tốc/phanh

Thêm nhánh opt-in `response_accel_model=true`, tăng state 11 → 14 bằng bộ nhớ
acceleration. Gia tốc XY mục tiêu `(u-v)/tau` bị giới hạn độ lớn và thay đổi
bởi jerk; do đó acceleration không đảo dấu tức thời khi gửi zero. Z vẫn giữ
first-order lag. Bộ nhớ gia tốc được cập nhật từ chênh vận tốc state và
simulation timestamp, kể cả khi đang hold. Không thay code/parameter ArduPilot.

Fit offline từ log baseline seed 7, giữ seed 17 để kiểm tra. Các lựa chọn
là tau [.3,.5,.8,1,1.5], acceleration [2,3,4], jerk [2,4,6,8,12]. Bộ được chọn:
**tau=.5 s, acceleration XY=3 m/s², jerk XY=4 m/s³**.

RMSE dự đoán XY velocity sau 1 s trên các cửa sổ trượt:

| Mô hình | Seed 7 dùng fit | Seed 17 kiểm tra |
|---|---|---|
| First-order tau=.5 | 2.863 m/s | 2.911 m/s |
| Acceleration memory | 0.280 m/s | 0.274 m/s |

[Fit JSON](../output/benchmark/accel_response_fit_20260915/fit.json).
Đây là surrogate, không phải mô hình ArduPilot đầy đủ. Initial acceleration
trong fit lấy đạo hàm ground truth (offline), khác estimator filtered lúc bay;
các cửa sổ tương quan và chỉ có hai lượt baseline. Timestamp gửi chưa phải
receipt; không tách riêng trễ truyền lệnh và tầng điều khiển. Kết quả offline
không chứng minh sai số live sẽ bằng 0.274 m/s hay phanh luôn an toàn.

## Kiểm chứng code

4 test mới đạt: cost SDF phát hiện container bị thiếu trong cloud, gate từ
chối đường đó, khoảng cách NumPy/Torch khớp nhau, acceleration còn dương khi
vừa brake và được đổi theo jerk, observer dùng timestamp/reset khi có gap.
49 test core và 4 test gate/diagnostics trước đó cũng đạt (tổng 57 test lượt này).

Các cấu hình cũ mặc định không bật model/map mới. Profile thử nghiệm bật đủ
map + model + final validation; các kết quả cũ không bị ghi đè.

## Headless: 2 seed, chưa hoàn tất navigation

Chạy cùng bài 60 m, reference 10, timeout 45 s wall mỗi lượt. Cả hai **TIMEOUT**,
không tới đích, nhưng **không abort vùng đệm và đều LAND/disarm**.

| Seed | Peak XY | X khi từ chối lần đầu | X dừng cuối | Clearance tâm min | Số hold-invalid |
|---|---|---|---|---|---|
| 7 | 9.734 m/s | 32.994 m | 53.624 m | 3.007 m | 369 |
| 17 | 9.658 m/s | 34.545 m | 54.448 m | 2.688 m | 368 |

Góc cua đầu ở x=60 m. Gate bắt nguy cơ trước khi cloud tự thấy đủ vật cản:
clearance cloud lần đầu còn 2.518/1.931 m nhưng cận dưới SDF còn 1.465/1.484 m,
thấp hơn margin 1.5 m. Tất cả command phát ra đều pass gate; mọi reject gửi
zero. Không có `hold-brake` theo gate cũ, **nhưng có nhiều hold-invalid gửi
zero**, nên không gọi đây là bay không phanh.

Seed 7 có 2 cycle deadline miss, seed 17 không có; không optimizer deadline
miss hoặc timeout-hold. Mỗi lượt có 1 hold-stale khởi động. Compute trung bình
khoảng 20 ms; toàn cycle gồm map validation/logging cao hơn compute đơn thuần.
Đây là TIMEOUT toàn nhiệm vụ, khác `hold-timeout` vì optimizer quá hạn.

Bổ sung map và model cùng lúc nên chưa tách đóng góp từng phần vào flight.
Kết quả xác nhận gate chặn nguy cơ/dừng trước cua trong hai lượt này, **không
xác nhận MPPI tìm được quỹ đạo khả thi qua cua ở 10 m/s**. Clearance là tâm tới
SDF, chưa trừ rotor, không có contact sensor. Tất cả process đã dừng.

[Summary](../output/benchmark/yard_map_response10_20260915_v1/summary.csv),
[kiểm tra gate](../output/benchmark/yard_map_response10_20260915_v1/gate_check.json),
[biểu đồ](../output/benchmark/yard_map_response10_20260915_v1/comparison.png).
Manifest/source snapshot khớp hash. Không chuyển profile này thành demo thành công.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_map_response10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 45 --output output/benchmark/yard_map_response10_my_run
```

Output phải mới. Chạy khi Gazebo/SITL khác đã dừng. Việc còn lại là giúp
optimizer tìm chuỗi điều khiển được gate chấp nhận, thay vì chỉ từ chối rồi giữ.

# Độ trễ pipeline RGB + IMU trước khi chỉnh planner

Ngày đo: 29/09/2026, Gazebo/SITL trên macOS. Hai lượt probe bay thẳng dùng RGB
640×360 ở 10 Hz, IMU 100 Hz, OpenVINS và triangulation của project. Không có
LiDAR/depth input (`lidar_topic_messages=0`, `depth_topic_messages=0`). Planner
**không chạy**; MAVLink nhận lệnh vận tốc mở vòng để chỉ đo cảm nhận tại tốc độ
thực. Gazebo odometry chỉ dùng cho giới hạn an toàn của lượt bay và tính sai số
sau thử nghiệm, không đưa vào OpenVINS hay perception.

| Lượt | Vận tốc đặt | Vận tốc thực p50 / p95 | Pose OpenVINS p95 | RGB callback → cloud p95 | Cloud / frame | Sai số pose p95 |
|---|---:|---:|---:|---:|---:|---:|
| [`openvins_latency_5`](../results/monocular_research/openvins_latency_5/latency_summary.json) | 5 m/s | 4,72 / 5,04 m/s | 4,45 ms | 93,14 ms | 45/54 | 0,90 m |
| [`openvins_latency_10_cruise`](../results/monocular_research/openvins_latency_10_cruise/latency_summary.json) | 10 m/s | 8,48 / 9,99 m/s | 4,38 ms | 121,49 ms | 53/65 | 1,83 m |

Đoạn gần vận tốc đặt (≥4,5 và ≥9 m/s) có lần lượt 35 và 31 frame ra cloud;
độ trễ RGB→cloud p95 trên các frame này là 93,89 và 124,44 ms. Khoảng cách
giữa hai cloud liên tiếp p95 là 106,96 và 109,00 ms; lớn nhất quan sát là
141,54 và 157,89 ms. `openvins_latency_10` là lượt ngắn hơn (9 frame gần
10 m/s), lưu để đối chiếu nhưng không dùng làm kết luận chính. Một lượt 10 m/s
trước đó bị lỗi cổng Gazebo 9002 và chưa bay, lưu tại `openvins_latency_10_bind_failed`.

Các mốc thời gian dùng chung đồng hồ monotonic: callback RGB của bridge,
bắt đầu xử lý OpenVINS, kết thúc VIO, xuất pose, perception nhận pose, callback
RGB của perception và xuất cloud. `scripts/report_monocular_latency.py` ghép
log theo timestamp của ảnh. Trên macOS bridge dùng `CLOCK_UPTIME_RAW` để cùng
nguồn thời gian với Python `time.monotonic()`. Độ trễ pose xuất→perception nhận
p95 là 0,24/0,20 ms. Giá trị 4,45/4,38 ms **chỉ là OpenVINS**; gần như toàn bộ
độ trễ đến cloud nằm ở xử lý ảnh và triangulation của perception.

Ở 5 m/s, một chu kỳ ảnh 100 ms cộng độ trễ cloud p95 93,14 ms tương ứng
ít nhất **0,97 m** chuyển động trước khi planner có cloud mới. Ở 10 m/s, con
số là **2,21 m** (100+121,49 ms). Đây là phép tính khoảng cách từ nhịp camera
và độ trễ quan sát, chưa cộng thời gian camera tạo ảnh→callback, độ trễ planner,
truyền MAVLink, động học phanh, cũng như sai số pose. Chưa đo được độ trễ
ảnh→lệnh điều khiển vì planner chưa chạy với pipeline này.

Hệ quả để thiết kế bước tiếp theo: planner phải ghi tuổi của cloud tại thời
điểm ra lệnh, dùng pose dự đoán đến thời điểm đó, và chỉ cho phép vận tốc khi
khoảng trống quan sát được vượt **quãng đường già hóa ảnh + sai số định vị +
quãng đường phanh đã đo**. Với dữ liệu hiện tại chưa đủ cơ sở xác nhận an toàn
ở 5 hay 10 m/s; cần đo tiếp cloud→lệnh, phanh thực và xác nhận vùng trống camera
trước khi chạy tránh vật cản kín vòng.

Chạy lại bằng `scripts/run_monocular_sim.py --latency-probe-speed 5` với world
và config highspeed 5, hoặc `--latency-probe-speed 10 --probe-stop-x 45` với
world và config highspeed 10, kèm `--openvins-bridge` và
`--perception-config config/monocular_openvins_perception.json`. Dùng
`scripts/report_monocular_latency.py <thư_mục_kết_quả>` để tái tạo bảng số.

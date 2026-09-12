# Đánh giá tiến độ: MPPI local planner trên companion

Ngày đánh giá: 09/09/2026
Phạm vi: đối chiếu yêu cầu mentor với code, tài liệu và kết quả kiểm thử trong
repo `ardupilot_gazebo`.

## 1. Kết luận ngắn

Phần việc hiện tại đã đạt mức **prototype SITL end-to-end cho vanilla MPPI**
và đã có **PA-MPPI v0 chạy được trên cùng pipeline**.
Node phía companion nhận point cloud LiDAR 3D, tối ưu lệnh vận tốc và gửi
`SET_POSITION_TARGET_LOCAL_NED` để ArduPilot bám trong `GUIDED`. Cấu hình đã
tắt avoidance của ArduPilot, vì vậy pipeline không còn phụ thuộc vào ID obstacle
của `AP_Avoidance`.

Chưa nên kết luận “đã hoàn thành trên quad thật”. Nhánh dành cho edge computer
mới dừng ở giao diện MAVLink và phép đổi frame cơ bản; chưa có tích hợp driver
LiDAR/camera thật, kiểm thử HIL hoặc flight test. Nhánh rigid-body experimental
đã có state `[p,q,v,omega]`, thrust/body-rate output và offline tests, nhưng
chưa qua hover gate trong Gazebo.

**Đánh giá tổng thể: 8.5/10 cho mục tiêu prototype tuần tới; khoảng 60% cho mục
tiêu triển khai trên phần cứng thật.**

## 2. Mức hoàn thành theo yêu cầu mentor

| Yêu cầu | Trạng thái | Bằng chứng |
|---|---|---|
| Không phụ thuộc `AP_Avoidance`/ID obstacle | Hoàn thành | `config/mppi_velocity.parm` đặt `OA_TYPE=0`, `AVOID_ENABLE=0`, `PRX1_TYPE=0`; LiDAR đi thẳng vào MPPI |
| Local planner sinh quỹ đạo/lệnh điều khiển | Hoàn thành ở mức vanilla MPPI | `mppi_controller.py`: state 7 chiều, action `[vx, vy, vz, yaw_rate]`, horizon 30, 500 rollout |
| Gửi lệnh để ArduPilot bám | Hoàn thành trong SITL | `mavlink_interface.py`: mask 1479, frame `LOCAL_NED`, tốc độ gửi mặc định 10 Hz |
| Tiền xử lý LiDAR 3D | Hoàn thành cho Gazebo | lọc/downsample, đổi FLU sang FRD rồi đưa về ENU/NED |
| Tuyến global hỗ trợ local planner | Mức tối thiểu | waypoint nhập tay qua `--goal`; chưa có thuật toán global planner |
| PA-MPPI | v0 chạy được; rigid-body experimental | occupancy 3 trạng thái, perception cost; nhánh `[p,q,v,omega]` sinh thrust/body rates |
| Tích hợp edge computer/quad thật | Đang chuẩn bị | có `--state-source mav`, nhưng nhánh này mới xoay LiDAR theo yaw, chưa dùng full attitude |

## 3. Kiểm chứng đã chạy lại

Ngày 09/09/2026, kiểm tra trực tiếp trên workspace hiện tại:

- build plugin Gazebo: thành công;
- kiểm tra cú pháp toàn bộ package Python: thành công;
- đối chiếu `TYPE_MASK_VEL_YAWRATE` với enum `pymavlink`: **1479**, thành công;
- kiểm tra phép đổi ENU/NED và quaternion đơn vị: thành công;
- offline MPPI simulation: `reached=True`, sai số đích `0.91 m`, khoảng cách
  nhỏ nhất tới vật cản `6.09 m`, lớn hơn ngưỡng yêu cầu `margin + 1 = 5 m`;
- PA-MPPI v0 simulation: `reached=True`, sai số đích `0.86 m`, clearance nhỏ
  nhất `5.85 m` với 128 rollout và horizon 20;
- test suite trong repo: **18/18 test đạt**, gồm MAVLink mask, đổi frame,
  stale/brake/reached, occupancy, PA-MPPI, rigid hover và closed-loop clear-air;
- rigid-body optimizer CPU với 512 rollout, horizon 20: mean `7.47 ms`, p95
  `8.39 ms`, worst `9.24 ms`, 0/27 lần vượt deadline 20 ms;
- khởi động lại Gazebo warehouse và ArduCopter SITL, arm, cất cánh, giữ cao độ
  20 m và hạ cánh: thành công. Lần chạy này dùng để kiểm tra môi trường, chưa
  phải video MPPI hoàn chỉnh.

Handoff còn ghi nhận một chuyến bay MPPI trước đó: tới đích với sai số `0.84 m`
và khoảng cách nhỏ nhất `6.51 m`. Log của lần đó từng nằm trong `/tmp`, nên cần
quay lại và lưu log/video vào `output/` trước khi dùng làm bằng chứng chính thức.

## 4. Điểm kỹ thuật làm tốt

1. Kiến trúc bám đúng quyết định mentor: companion chịu trách nhiệm perception
   và local planning; ArduPilot giữ tầng ổn định và velocity tracking.
2. Đã tìm và sửa lỗi MAVLink quan trọng: mask cũ bỏ nhầm velocity; mask 1479
   giữ velocity và yaw rate, đồng thời bỏ position, acceleration và yaw.
3. Code đã tách thành controller, tiền xử lý LiDAR, MAVLink interface và node
   điều phối. Cấu trúc này phù hợp để thay vanilla MPPI bằng PA-MPPI sau này.
4. Có cơ chế an toàn cơ bản: dữ liệu stale hoặc obstacle quá gần thì gửi vận
   tốc 0; khi stream mất, `GUID_TIMEOUT` của ArduPilot đưa vehicle về hold.
5. Có hai nguồn state: Gazebo odometry cho SITL và MAVLink telemetry cho hướng
   triển khai trên edge computer.
6. Đã thêm predicted path, top sampled rollouts, cost breakdown, ESS và rolling
   benchmark mean/p95/worst/deadline miss để demo có số liệu kiểm chứng.

## 5. Hạn chế và rủi ro cần nói rõ trong báo cáo

- `--state-source mav` chỉ dùng yaw để quay point cloud. Khi quad roll/pitch,
  obstacle sẽ bị đặt sai trong frame local. Cần attitude quaternion đầy đủ.
- Waypoint nhập tay đang đóng vai trò global reference. Planner có thể kẹt trước
  vật cản rộng hoặc đối xứng nếu không có reference path phù hợp.
- Rigid branch dùng first-order body-rate tracking thay cho mô hình torque/motor
  đầy đủ, 512 rollout thay vì 17,500 và chưa bay live; vì vậy chưa được gọi là
  reproduction paper-faithful.
- Mapper hiện giả thiết vùng gần LiDAR 360 độ đã được quan sát và đánh dấu free.
  Khi dùng sensor thật phải tích hợp cả hit và max-range/miss ray đúng timestamp.
- Chưa đo cycle time, deadline miss, CPU/GPU, RAM và nhiệt độ trên edge computer.
- Chưa kiểm tra dropout LiDAR/MAVLink, timestamp lệch, obstacle động, nhiễu state,
  saturation của velocity controller hoặc chuyển mode ngoài ý muốn.
- `wx, wy` chưa được điều khiển và điều này phù hợp với phạm vi hiện tại. Nếu đi
  xuống tầng attitude control thì cần một bài toán an toàn và xác minh khác.

## 6. Tiêu chí hoàn thành vòng tiếp theo

1. Quay lại demo SITL với Gazebo, RViz Path, sampled rollouts, log node và
   `POSITION_TARGET_LOCAL_NED`; lưu video và log trong `output/`.
2. Chạy cùng seed/kịch bản cho vanilla và PA-MPPI, so sánh success rate,
   clearance, path length, control smoothness và compute p95.
3. Dùng attitude đầy đủ cho nhánh MAVLink hoặc nhận odometry chuẩn từ estimator
   trên edge computer.
4. Đóng gói node bằng ROS 2 package/service, thêm health monitoring và launch
   file cho sensor driver, planner và MAVLink.
5. Benchmark trên edge computer ở 10 Hz; báo cáo mean, p95 và worst-case cycle
   time cùng tỷ lệ deadline miss.
6. Sau khi v0 ổn định, port dynamics thrust/body-rate và occupancy backend từ
   sensor thật nếu mục tiêu là tái lập đầy đủ PA-MPPI.

## 7. Kịch bản video nên dùng

Video 60–90 giây, quay một lần liên tục:

1. Gazebo warehouse và RViz hiển thị PointCloud2, Odometry, predicted Path.
2. Xác nhận `OA_TYPE=0`, `AVOID_ENABLE=0`, `GUID_OPTIONS=0`.
3. Cất cánh 20 m, chạy MPPI qua tuyến `16,10,20;30,0,20`.
4. Đặt cạnh nhau log `pos`, `nearest`, `u`, `yaw_rate` và MAVProxy
   `watch POSITION_TARGET_LOCAL_NED`.
5. Cho thấy UAV vòng qua stack container, tới waypoint, tới đích rồi `mode land`.
6. Kết video ở trạng thái `DISARMED` và giữ lại log cùng video trong `output/`.

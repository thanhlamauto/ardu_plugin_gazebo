# Báo cáo gửi mentor: UAV tránh vật cản bằng một camera RGB + IMU

Ngày 30/09/2026. Repo cá nhân: <https://github.com/thanhlamauto/ardu_plugin_gazebo>.
Nhánh `main` trên remote hiện ở commit `865466c1149932508854116dd99159c67059158f`.
**Các sửa đổi và kết quả bên dưới đang ở working tree cục bộ, chưa được push lên
remote.** Gói snapshot đính kèm chứa mã hiện tại và dữ liệu lượt chạy cuối.

## Mục tiêu và kiến trúc đang thử

Mục tiêu là tránh vật cản kín vòng với một RGB monocular camera, IMU, ước lượng
pose từ OpenVINS và MPPI, không dùng LiDAR/depth camera/Gazebo odometry làm
đầu vào perception hay planner. Một mốc cao độ hover 3 m đã biết cung cấp gốc
tọa độ; RGB + IMU cung cấp chuyển động tương đối có thang mét. Gazebo odometry
chỉ ở runner để kiểm tra an toàn và đánh giá, còn ArduPilot SITL vẫn sử dụng
cảm biến điều hướng mô phỏng nội bộ. Vì vậy đây **không phải thử nghiệm phần
cứng camera-only tuyệt đối**.

Pipeline: Gazebo RGB 640×360 @10 Hz và IMU @100 Hz → C++ OpenVINS bridge →
`/perception/visual_odometry`; RGB + pose VIO → optical-flow keyframe
triangulation → occupied local cloud; cloud + pose VIO → MPPI → setpoint
ArduPilot. Camera nhìn về phía trước. SDF định nghĩa camera ở
`(0.09, 0, 0) m` so với IMU và `(0.17, 0, 0.16) m` so với body (hệ FLU).

## Đã sửa và kiểm chứng

- Bridge ghép IMU gần timestamp ảnh nhất, tối đa 20 ms; thêm ablation ZUPT
  sau chuyển động và log khoảng cách timestamp.
- Audit so SDF, thông số bridge và offset dùng để triangulate đều đạt. Trong
  227 ảnh của lượt cuối, độ lệch timestamp ảnh–IMU trong mô phỏng là 0 ms.
  Điều này **chỉ chứng minh tính nhất quán cấu hình trong mô phỏng**, chưa là
  calibration độc lập của camera/IMU hoặc ước lượng time offset phần cứng.
- Bản đồ ghép bằng chứng qua voxel lân cận bán kính 0,3 m, tối đa một hit mỗi
  frame; tuổi lưu 3 s, tuổi điểm được phát 1,5 s. Cloud gửi lại cho subscriber
  mới vẫn giữ timestamp ảnh gốc; planner từ chối dữ liệu quá 1 s.
- Planner được khởi tạo và subscribe trước warmup, nhưng chỉ phát lệnh sau
  cổng start. Không giảm ngưỡng 12 điểm, không tắt gate stale/pose.
- 27 kiểm thử liên quan qua. Audit xác nhận 0 LiDAR/depth message; bridge,
  perception và planner không subscribe Gazebo odometry.

## Kết quả định lượng của lượt cuối

Cảnh một hộp phía trước, goal `(12,0,3) m`, seed 7, giới hạn planner 0,5 m/s,
thời lượng đặt 30 s. Sau warmup chéo để có parallax, cloud có 42 điểm khi
planner bắt đầu. MPPI phát lệnh 5 chu kỳ, rồi `hold-stale` 48 chu kỳ vì không
có triangulation mới đủ đều. Tốc độ ngang thực tế p95 là 0,343 m/s; quãng
đường trong pha điều khiển là 0,609 m, x lớn nhất 1,239 m. Sai số pose VIO
so với ground truth đánh giá: p50 0,389 m, p95 1,352 m, lớn nhất 1,518 m.
Runner ngắt ở ngưỡng 1,5 m với trạng thái `LOCALIZATION_DIVERGED`. Khoảng hở
tối thiểu đo được 5,76 m và không có collision proxy trên quãng ngắn này;
UAV **không tới đích, không chứng minh được tránh vật cản thành công**.
Độ trễ perception lúc pha điều khiển p95 là 88,3 ms; con số này không bao gồm
toàn bộ độ trễ camera → điều khiển và không chứng minh an toàn ở 5/10 m/s.

Lượt ZUPT trước khi sửa map có 14 điểm và planner chỉ phát 1 chu kỳ trước khi
VIO vượt 1,5 m. Lượt cuối cải thiện phần cloud/startup nhưng không giải quyết
drift. Ở thời điểm hiện tại **chưa có tốc độ bay tránh vật cản an toàn nào được
xác nhận** cho pipeline OpenVINS này, kể cả 0,5 m/s; 5 và 10 m/s chưa phù hợp
để chạy kín vòng với các gate hiện có.

## Hai câu hỏi cần mentor giúp ưu tiên

1. VIO: Với RGB 10 Hz, IMU 100 Hz, mốc hover 3 m và SDF/clock đã đối chiếu,
   nên kiểm tra gì tiếp để tách lỗi `IMU noise/time offset/extrinsics`, sai
   quy ước orientation trong bridge, độ kích thích chuyển động, và chất lượng
   feature track? Nên đổi sang dataset/calibration và estimator nào để có
   baseline độc lập trước khi tuning MPPI?
2. Depth/map: Khi UAV dừng hoặc chạy chậm, triangulation keyframe theo baseline
   0,4 m không cho độ sâu mới liên tục. Nên theo dõi landmark 2D và cập nhật
   inverse-depth uncertainty qua nhiều frame, hay thêm một nguồn đo metric
   khác (stereo, depth camera, LiDAR)? Với camera đơn + IMU, làm thế nào xác
   nhận vùng *free/unknown* đủ tin cậy để planner được phép tiến?

## Tệp để xem nhanh và tái lập

- Kết quả: `results/monocular_research/openvins_fused_gate_05_20260930/`
  gồm `analysis.json`, `openvins_audit.json`, `calibration_audit.json`,
  `planner.jsonl`, `openvins.csv`, `trajectory.png`, `manifest.json`, và
  `source_snapshot/` ghi mã đúng thời điểm chạy.
- Tài liệu chi tiết/lệnh chạy:
  `docs/MONOCULAR_OPENVINS_VI.md`, `tools/openvins_gz_bridge/README.md`.
- Mã chính hiện tại: `tools/openvins_gz_bridge/main.cpp`,
  `scripts/monocular_geometry_gz.py`, `mppi_ardupilot/camera_local_map.py`,
  `mppi_ardupilot/monocular_point_tracker.py`,
  `scripts/run_monocular_sim.py`, `config/monocular_openvins_fused_perception.json`.
- OpenVINS upstream phải build riêng ở revision
  `69488123ed9362dd44b6f28e7f4680abbff1442b` theo README của bridge.
  Gói snapshot không chứa binary hoặc dependency đó.

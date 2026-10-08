# MPPI không dùng LiDAR

**Cập nhật:** cấu hình trong tài liệu này chỉ dùng RGB để nhìn vật cản nhưng còn lấy pose từ Gazebo. [Thử nghiệm visual odometry mới](MONOCULAR_VISUAL_ODOMETRY_VI.md) đã thay pose đầu vào perception/MPPI bằng ước lượng RGB + IMU + mốc kích thước biết trước; ở mức đặt 5/10 m/s hiện vẫn thất bại, nên không được dùng kết quả cũ để tuyên bố bay chỉ bằng camera.

Cấu hình cải tiến hiện dùng **chuỗi ảnh RGB + triangulation theo pose có metric scale**,
không dùng pretrained depth trong flight. Xem `MONOCULAR_RESEARCH_VI.md` cho kết quả
lặp lại và các baseline thất bại. Chạy cấu hình geometry bằng lệnh:

```bash
/tmp/uav-monocular-env/bin/python scripts/run_monocular_sim.py \
  --world worlds/iris_monocular_textured.sdf \
  --config config/monocular_geometry_mppi.yaml \
  --perception-config config/monocular_geometry_perception.json \
  --duration 180 --seed 9 --output results/monocular_geometry_new
```

Kết quả cấu hình geometry:9/10 lượt tới đích trên2 layout; các lượt đạt giữ
clearance ít nhất1.20 m và goal error<=0.50 m. Ngắt camera5 s đưa lệnh về0
rồi resume tới đích. Một lượt mất odometry được tính thất bại. Chi tiết audit
và giới hạn nằm trong `MONOCULAR_RESEARCH_VI.md`. Tham số `lidar_topic` là tên
interface cũ, hiện trỏ tới cloud camera; model không có sensor LiDAR/depth.

Phần bên dưới ghi lại **baseline learned-depth ban đầu** để giữ evidence lịch sử.

Model `iris_with_monocular_camera` bỏ hẳn sensor `gpu_lidar` và `depth_camera`
khỏi model sensor suite. Các include kế thừa cũng không có LiDAR/depth sensor.
Obstacle duy nhất của MPPI đến từ `/sensor_suite/rgb` qua pretrained metric depth,
phát `/perception/obstacles_camera`. Không cấp ground truth depth, không fallback
LiDAR, không cấp SDF obstacle map/A* cho planner trong runner.

Odometry và IMU/GPS vẫn đảm nhiệm trạng thái bay. Đây là **RGB-only obstacle
perception**, chưa phải hệ thống localization/navigation chỉ dùng camera.
Model main hiện dùng passive gimbal hierarchy riêng; chỉ còn camera RGB ở sensor suite.
Các model gimbal gốc của project giữ nguyên.

## Chạy thử closed-loop

Dùng venv inference đã chuẩn bị theo `MONOCULAR_BASELINE_VI.md`. Thêm dependency
cho MPPI và MAVLink nếu chưa có:

```bash
uv pip install --python /tmp/uav-monocular-env/bin/python \
  'pymavlink==2.4.50' 'pytorch-mppi==0.9.1' pyyaml
/tmp/uav-monocular-env/bin/python scripts/run_monocular_sim.py \
  --output results/monocular_only_new_run --duration 45
```

Runner tự mở Gazebo + SITL, đọc lại OA_TYPE/AVOID_ENABLE/PRX1_TYPE = 0, chờ EKF,
arm GUIDED, takeoff 3 m, rồi chạy Python MPPI hiện có với config
`config/monocular_mppi.yaml`: seed 7, vmax 1 m/s, 120 rollouts, horizon 30,
margin 1.5 m, final/stopping trajectory validation bật. Không sửa optimizer.
Goal `(12,0,3)` sau khối chắn tại `(8,0,3)`, kích thước `(2,3,6)` m.
Khi kết thúc, runner dừng planner và hạ cánh rồi dừng các tiến trình sở hữu.
Output phải là thư mục mới; cần để trống các cổng SITL 5760/5762.

`manifest.json` và `source_snapshot/` ghi input/config/source; `planner.jsonl`
ghi các quyết định MPPI; `perception/frames.jsonl` ghi suy luận; `ground_truth.jsonl`
chỉ dùng để đánh giá và readiness. `result.json` đếm message LiDAR/depth (phải 0),
cloud camera, final position và clearance. Camera cloud frame FLU sensor_suite_link;
planner thêm offset link so với body `(0.08,0,0.16)` m, không thêm offset LiDAR.
Offset camera +0.09 m đã nằm trong cloud.

Collision proxy: khoảng cách tâm UAV tới bề mặt box <0.5 m. Đây không phải
contact telemetry và không đủ để chứng nhận collision-free. Run kết thúc khi
đạt goal, hết thời gian wall-clock, proxy bị vi phạm hoặc lỗi tiến trình.

## ROS C++ launch tùy chọn

Sau build/source ROS workspace:

```bash
ros2 launch uav_navigation_bringup monocular_sim.launch.xml \
  python_executable:=/tmp/uav-monocular-env/bin/python device:=mps
```

Launch dùng model không LiDAR/depth, tắt LiDAR bridge, local navigation subscribe
camera cloud, tắt global planner đọc map SDF. Cần cấp một `nav_msgs/Path` theo odom
cho `/planning/global_path`; nếu chưa có, node chờ path. Autopilot adapter mặc định
tắt; `enable_autopilot_adapter:=true` dùng khi SITL đã ready và UAV đã takeoff.
Launch này là integration tùy chọn; runner closed-loop dùng Python MPPI trực tiếp
qua Gazebo/MAVLink và không cần ROS bridge.

FOV forward và metric scale vẫn là giới hạn của thử nghiệm này. Không có local
map/coverage guard cho unknown space; final/stopping validators chỉ thấy cloud
camera đã dự đoán. Recovery của planner mặc định có thể lùi vào vùng chưa nhìn.
Những hạn chế này được giữ nguyên để quan sát hành vi baseline khi bỏ LiDAR.

## Kết quả chạy thật ngày 27/09/2026

Lượt có planner hoạt động: `results/monocular_only_20260927_r3/`.

| Metric | Kết quả |
|---|---:|
| LiDAR messages / depth-camera messages | 0 / 0 |
| Camera cloud nhận trong lượt chạy | 446 |
| Kết quả | TIMEOUT, không đạt goal |
| Thời gian điều khiển wall / simulation | 44.91 / 17.31 s |
| Vị trí cuối (ENU m) | (-0.695, -0.118, 3.206) |
| X tiến xa nhất | 0.012 m |
| Altitude khi điều khiển | 3.184–3.301 m |
| Khoảng cách còn lại tới goal | 12.697 m |
| Planner events command / hold-timeout / hold-stale | 190 / 48 / 16 |
| Nhận RGB → publish p95 khi control | 299.67 ms |
| Khoảng cách tâm UAV tới box nhỏ nhất | 6.988 m |

Không thấy vi phạm collision proxy; không thể gọi tránh vật cản thành công vì
UAV gần như không tiến về vật cản. Đã hạ cánh và dừng tiến trình thử nghiệm.
Model depth, optimizer và tải compute đều cần phân tích tiếp; 48 hold-timeout,
16 hold-stale và real-time factor khoảng 0.39 là yếu tố ảnh hưởng rõ rệt.
Đây là một lượt ở tốc độ đặt tối đa 1 m/s, không phải benchmark success rate.
Không có dense GT depth để chấm RMSE trong run này vì sensor đã bỏ hoàn toàn.

Evidence: `result.json`, `analysis.json`, `trajectory.png`, `planner.jsonl`,
`perception/frames.jsonl` và `source_snapshot/`. Lượt đầu
`monocular_only_20260927` cất cánh nhưng planner exit vì recovery speed=0;
lượt r2 không qua arming gate (GPS/accelerometer readiness khi sim chậm), có lỗi
cleanup process group. Hai lượt đó không được tính làm kết quả closed-loop.
Runner đã sửa recovery, tăng thời gian chờ arming và fallback signal theo PID;
r3 chạy đủ và cleanup xong.

Tái tạo biểu đồ/tổng hợp từ log mà không chạy lại flight:

```bash
/tmp/uav-monocular-env/bin/python scripts/analyze_monocular_sim.py \
  results/monocular_only_20260927_r3
```

Validation: 7 tests pass, XML launch parse được, SDF Valid. Tests kiểm tra
đệ quy các model include để không còn LiDAR/depth, ngoài các contract geometry,
timestamp và metric đã có. ROS C++ camera launch chưa được chạy trong lượt này.

## Lượt kiểm tra bổ sung: RGB và bản đồ cục bộ

Đã thử indoor metric depth, lọc sky từ RGB, ghép cloud theo pose tại thời điểm
chụp, lọc dải độ cao 3 m và cost collision/progress, vmax 0.5 m/s.
Run OpenResearch `1b1d75d0-2650-43b2-b132-2e5722c02d70`, commit `9d30548`.
Không có LiDAR/depth camera: 0/0 messages; nhận 175 camera clouds.
Kết quả INFRA_FAILURE do odometry mô phỏng quá cũ, chưa đạt đích.
Vị trí cuối (0.142, -0.079, 3.199) m; khoảng cách tâm tới box nhỏ nhất 6.853 m.
Receive-to-publish p95 959.61 ms. Không coi đây là thử nghiệm tránh vật cản
thành công hoặc dùng nó để tính success rate. Các tiến trình thử đã dừng.
Evidence tổng hợp: `results/monocular_research/planar_rgb_only/analysis.json`.
Cấu hình này được giữ trong nhánh nghiên cứu, chưa thay baseline mặc định.
Validation hiện tại: 10 unit tests pass; fixed-axis command constraint kiểm tra
được việc vận tốc đứng luôn bằng 0.

## Cấu hình đã có lượt đạt đích

Xem `MONOCULAR_RESEARCH_VI.md`, mục “Lượt đạt đích đầu tiên”.
Ứng viên mới dùng temporal RGB triangulation + scaled odometry và MPPI motion
proposals, đã có một pilot đạt đích với clearance1.421 m. Batch10 lượt/two-layout
và camera-blackout đang chạy; chưa thể công bố success rate. Cách chạy ứng viên
được ghi trong tài liệu nghiên cứu, với config geometry được tách khỏi baseline
Depth Anything phía trên. Main model chỉ còn1 RGB camera; integration replay
của passive hierarchy sẽ được kiểm chứng sau batch.

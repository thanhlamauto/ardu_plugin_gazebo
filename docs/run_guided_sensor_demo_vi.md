# Demo warehouse: click MAVProxy Map + tránh vật cản lidar 3D

Demo này dùng cùng một mốc địa lý cho cả hai phía:

- Gazebo: warehouse Depot thực tế (mesh Fuel, không phải box lý tưởng), UAV `iris_with_sensor_suite` có RGB camera, depth camera, LiDAR 3D 640×16, IMU, từ kế, khí áp và NavSat;
- MAVProxy Map: ảnh vệ tinh MicrosoftSat tại CMAC, Australia;
- RViz: RobotModel, TF, odometry trajectory và toàn bộ dữ liệu sensor;
- mốc WGS84: `-35.363262, 149.165237`, cao độ `584 m`.

Vì ArduPilot SITL và Gazebo dùng chung origin, một điểm chọn trên bản đồ được ArduPilot đổi thành quãng dịch chuyển tương ứng trong world Gazebo.

> Map warehouse thay cho `iris_sensor_runway.sdf` / `iris_avoidance_20m.sdf` lý tưởng theo yêu cầu mentor: dùng `Depot` từ `fuel.gazebosim.org` (mesh thực tế) thay vì pillar/box hoàn hảo. Lần đầu chạy cần internet để Gazebo tải Fuel models.

World: [`worlds/iris_warehouse_sensor.sdf`](../worlds/iris_warehouse_sensor.sdf) (biến thể realistic của `iris_warehouse.sdf`, chỉ thay drone thành `iris_with_sensor_suite`).

## Terminal 1 — Gazebo server

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_warehouse_demo
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 -r "$PWD/worlds/iris_warehouse_sensor.sdf" -s
```

## Terminal 2 — Gazebo GUI

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_warehouse_demo

gz sim -v1 -g \
  --gui-config "$PWD/config/gazebo_runway_camera.config"
```

## Terminal 3 — ArduPilot SITL và MAVProxy Map

```bash
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"
cd ~/Projects/ardupilot

MAP_SERVICE=MicrosoftSat python3 Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f JSON \
  -N \
  -w \
  --map \
  -A "--serial1=tcp:2" \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/avoidance_20m.parm"
```

`-w` xóa EEPROM SITL cũ để chắc chắn nạp lại toàn bộ file parameter; chỉ dùng
cho mô phỏng. `-A "--serial1=tcp:2"` mở TCP `5762` riêng cho bridge. File
`avoidance_20m.parm` đặt `SERIAL1_PROTOCOL=2` (MAVLink 2), baud logic
`SERIAL1_BAUD=921` (921600 bps), `PRX1_TYPE=2`
(MAVLink proximity), `AVOID_ENABLE=2` (chỉ proximity) và `OA_TYPE=1`
(BendyRuler ngang). `GUID_OPTIONS=64` bật bit 6 để mọi position target của
Guided đi qua `AC_WPNav_OA`; nếu thiếu bit này, MAVProxy vẫn làm UAV bay tới
đích nhưng bỏ qua BendyRuler. Queue OA được đặt 200 phần tử để chứa trọn một
frame LiDAR đã rút gọn 180 điểm. Bản runway cũ dùng
`gazebo-iris-gimbal.parm` không có avoidance.

> Mỗi lần chỉ chạy 1 Gazebo + SITL: plugin `ArduPilotPlugin` bind cố định UDP `127.0.0.1:9002`. Mở run mới khi run cũ còn sống sẽ báo `SocketUDP Bind failed: Address already in use` và plugin tự abort. Dừng hẳn run cũ (Ctrl+C từng terminal) trước khi chạy lại.

Chờ MAVProxy báo nhận đủ parameter và EKF đang dùng GPS. Kiểm tra:

```text
param show PRX1_TYPE  # =2
param show SERIAL1_PROTOCOL  # =2
param show OA_TYPE    # =1
param show AVOID_ENABLE  # =2
param show GUID_OPTIONS  # =64
param show OA_DB_QUEUE_SIZE  # =200
```

## Terminal 4 — ROS 2 bridge và RViz

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_warehouse_demo
export ROS_DOMAIN_ID=45

./scripts/run_sensor_rviz.sh
```

Script tự tìm `conda` trong `PATH`; trên đúng máy đã quay video, nó cũng tự dùng `/opt/miniconda3/bin/conda` nếu shell hiện tại chưa nạp Conda.

RViz dùng `odom` làm fixed frame. Gazebo publish ground-truth pose qua `/iris/odometry`; bridge đổi `gz.msgs.Odometry` thành `nav_msgs/msg/Odometry`. Gazebo cũng publish transform `odom -> base_link` lên `/tf`, còn `robot_state_publisher` cung cấp các transform cố định từ `base_link` tới rotor và `sensor_suite_link`.

Có thể kiểm tra độc lập ba đầu vào chính của RViz trong terminal khác:

```bash
export PATH="/opt/miniconda3/bin:$PATH"
export ROS_DOMAIN_ID=45

conda run -n ardupilot-rviz ros2 topic echo /iris/odometry --once
conda run -n ardupilot-rviz ros2 run tf2_ros tf2_echo odom base_link
conda run -n ardupilot-rviz ros2 topic echo /robot_description --once
```

Trong RViz, vệt mũi tên đỏ là lịch sử Odometry; mô hình quadrotor là RobotModel. Khung `RGB Camera` và `Depth Camera` vẫn là dữ liệu ảnh trực tiếp từ sensor Gazebo.

## Terminal 5 — Bridge lidar 3D → MAVLink avoidance

SITL JSON chỉ gửi `rng_1..rng_6` (6 tia đơn), không đủ cho lidar 3D. Bridge phải fusion ngoài qua MAVLink `OBSTACLE_DISTANCE_3D` → `AP_Proximity_MAV` → `Boundary_3D` → `AC_Avoid`/`OA`:

```
Gazebo gpu_lidar (640×16, 10Hz, 30m) → GZ Transport /sensor_suite/lidar/points
  → lidar_to_mavlink_avoidance.py (giữ frame mới nhất, gom yaw/pitch 5°,
     tối đa 180 điểm, mục tiêu gửi 5Hz)
  → MAVLink 2 OBSTACLE_DISTANCE_3D (BODY_FRD) → SITL → avoidance
```

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_warehouse_demo
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/lidar_to_mavlink_avoidance.py \
  --gz --mav tcp:127.0.0.1:5762
```

`--gz` là backend khuyến nghị cho đường điều khiển vì callback chỉ giữ scan
mới nhất. Nếu xử lý chậm hơn LiDAR 10 Hz, log sẽ hiện `dropped=N`; đây là chủ
đích để bỏ scan cũ và giữ độ trễ thấp. RViz vẫn nhận PointCloud2 qua
`ros_gz_bridge` độc lập ở Terminal 4. Có thể thay `--gz` bằng `--ros` và export
`ROS_DOMAIN_ID=45` khi muốn bridge avoidance dùng chính topic ROS 2.

`5762` là TCP port của `SERIAL1` do SITL mở. Script chờ heartbeat để xác nhận
đúng peer, ép dialect MAVLink 2 và gửi `OBSTACLE_DISTANCE_3D` (message ID
11037). Tất cả các điểm của cùng một frame dùng chung `time_boot_ms` để
`AP_Proximity_MAV` commit chúng vào cùng một `Boundary_3D`. Không dùng
`udpout:14550`: đó chỉ là địa chỉ output của MAVProxy và không phải cổng nhận
trực tiếp của SITL trong setup này.

Mặc định tối đa 180 điểm mỗi scan và mục tiêu 5 Hz giữ lưu lượng dưới khả năng
của cổng 921600 bps. Ở độ cao bay thử, sau lọc thường còn khoảng 40–75 điểm.
Backend GZ tự bỏ frame trung gian nếu cần để không phát dữ liệu quá hạn và vẫn
cập nhật nhanh hơn timeout của `AP_Proximity_MAV`. Không tăng đồng thời
`--max-points` và `--send-hz` nếu chưa đo lại độ trễ ở phía ArduPilot.

Chi tiết thuật toán ArduPilot: `libraries/AP_Proximity/AP_Proximity_MAV.cpp`, `libraries/AC_Avoidance/AC_Avoid.cpp`, `libraries/AC_Avoidance/AP_OADatabase.cpp`, `AP_OABendyRuler.cpp`/`AP_OADijkstra.cpp` + `AP_OAPathPlanner.cpp` (OA params nằm ở group `OA_*`: `OA_TYPE`, `OA_MARGIN_MAX`, `OA_BR_*`, `OA_DB_*`). Xem thêm `config/avoidance_20m.parm`.

## Cất cánh 20 m

Nhập trong MAVProxy:

```text
map zoom 500
param set WP_SPD 3
mode guided
arm throttle
takeoff 20
```

`WP_SPD=3` giới hạn tốc độ ngang ở khoảng `3 m/s`. World warehouse xếp bốn
container thành chướng ngại cao 24 m trên đường bay, vì vậy `takeoff 20` vẫn
đặt vật cản ngang tầm LiDAR và buộc thuật toán tìm đường vòng. Kiểm tra dữ liệu
đã vào ArduPilot bằng `watch DISTANCE_SENSOR`; hướng trước là `orientation=0`
và `current_distance` dùng đơn vị cm. Nhấn `Ctrl+C` để thoát chế độ watch.

## Click bản đồ để bay và né

Sau khi UAV giữ độ cao 20 m:

1. Đưa chuột tới một điểm phía sau Depot/container trên cửa sổ `Map`.
2. Nhấn chuột phải và chọn `Fly To`.
3. Nhập altitude `20`, chọn frame `AboveHome`, rồi xác nhận.
4. Quan sát UAV bẻ đường vòng (Dijkstra/BendyRuler) và `AC_Avoid` kẹp velocity khi cách vật < `AVOID_MARGIN=4 m`.
5. Chờ UAV đến điểm đó rồi mới chọn điểm tiếp theo.

Nút `2D Goal Pose` của RViz chỉ publish `/goal_pose`; setup này không dùng nút đó để điều khiển ArduPilot. Lệnh bay theo bản đồ vẫn phải đi từ `Fly To` của MAVProxy Map để tạo MAVLink `MAV_CMD_DO_REPOSITION`.

MAVProxy dùng tọa độ của điểm vừa click để gửi `MAV_CMD_DO_REPOSITION` trong
một MAVLink `COMMAND_INT`. Nhờ `GUID_OPTIONS=64`, ArduCopter chuyển target này
sang nhánh waypoint (`AC_WPNav_OA::update_wpnav`) để BendyRuler xử lý. Khi
ArduPilot chấp nhận, console hiển thị:

```text
Got COMMAND_ACK: DO_REPOSITION: ACCEPTED
```

Có thể kiểm tra cùng đường xử lý mà không dùng chuột bằng lệnh sau. Điểm này cách home xấp xỉ 30 m về phía đông, UAV sẽ phải vòng qua container:

```text
guided -35.363262 149.165600 20
```

Lệnh này giữ nguyên latitude và bay thẳng về phía đông, xuyên qua vị trí stack
container tại `x=12 m` nếu avoidance bị tắt. Khi avoidance hoạt động, đường
Odometry trong RViz và chuyển động trong Gazebo phải lệch sang một bên rồi trở
lại đích. Có thể xác minh nhánh điều khiển sau chuyến bay trong DataFlash log:
phải có message `GUIP` với `Type=1` (WP) và các message `OABR`; `GUIP Type=2`
nghĩa là target đã đi qua position controller trực tiếp và BendyRuler không
được gọi.

## Hạ cánh và dừng

```text
mode land
```

Chỉ tắt các terminal sau khi MAVProxy báo `DISARMED`. Dừng theo thứ tự: MAVProxy/SITL, bridge lidar, RViz/bridge, Gazebo GUI, rồi Gazebo server.

## Quay video xác nhận gửi mentor

Trên macOS, nhấn `Shift+Command+5`, chọn **Record Entire Screen** rồi bắt đầu
quay. Một video đủ bằng chứng nên hiển thị theo thứ tự:

1. Gazebo warehouse và RViz PointCloud2 đang cập nhật.
2. Terminal bridge có các dòng `frame=... points=... nearest=...m`.
3. MAVProxy `watch DISTANCE_SENSOR` cho thấy khoảng cách thay đổi khi UAV tiến
   gần stack container; thoát watch bằng `Ctrl+C`.
4. Chạy `guided -35.363262 149.165600 20`, quay rõ UAV bẻ hướng vòng qua stack
   và vệt Odometry cong trong RViz.
5. Cho UAV đến đích, chạy `mode land`, chờ `DISARMED`, sau đó mới dừng quay.

Nên lưu bản nộp vào
`output/video/gazebo_warehouse_lidar_avoidance_3d.mp4`. Trước khi gửi, mở lại
video và kiểm tra chữ trong terminal đọc được, có đoạn tiếp cận vật cản, đổi
hướng và đến đích; chỉ hiện point cloud mà không có chuyển động né chưa đủ để
xác nhận object avoidance.

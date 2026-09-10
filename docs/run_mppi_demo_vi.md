# Demo warehouse: MPPI local planner trên companion, điều khiển ArduPilot bằng velocity MAVLink

Demo này thay BendyRuler (thuật toán avoidance mặc định của ArduPilot) bằng
**MPPI** ([UM-ARM-Lab/pytorch_mppi](https://github.com/UM-ARM-Lab/pytorch_mppi))
chạy phía companion, đúng hướng kiến trúc đã chốt với mentor:

- ArduPilot **tắt hoàn toàn** avoidance riêng (`OA_TYPE=0`, `AVOID_ENABLE=0`,
  `PRX1_TYPE=0`, `GUID_OPTIONS=0`);
- node Python tự tiêu thụ point cloud LiDAR 3D trực tiếp từ Gazebo (không qua
  `OBSTACLE_DISTANCE_3D`, khỏi cần đánh ID obstacle cho `AP_avoidance`);
- node chạy MPPI sinh `u = [vx, vy, vz, yaw_rate]` và truyền
  `SET_POSITION_TARGET_LOCAL_NED` (mask 1479: velocity + yaw_rate) cho
  ArduPilot ở chế độ GUIDED bám theo;
- nếu node ngừng gửi quá `GUID_TIMEOUT` (3 s), ArduPilot tự brake và hold —
  đó là lưới an toàn khi Ctrl+C node.

Kiến trúc:

```
Gazebo gpu_lidar /sensor_suite/lidar/points ──┐
Gazebo /iris/odometry  ───────────────────────┤→ mppi_ardupilot (MPPI rollouts,
  hoặc MAVLink telemetry LOCAL_POSITION_NED /    cost obstacle, waypoint route)
  ATTITUDE (--state-source mav) ──────────────┘
                                                      │
                   MAVLink SET_POSITION_TARGET_LOCAL_NED, mask 1479
                        (vx, vy, vz NED + yaw_rate)
                                                      ▼
                                 ArduPilot SITL GUIDED (OA đã tắt)
```

Code nằm trong package `mppi_ardupilot/` (đúng cấu trúc mentor chốt):

```text
mppi_ardupilot/
├── mppi_controller.py        dynamics/cost/command, predict_trajectory()
├── mavlink_interface.py      get_state() / send_velocity_ned() (mask 1479)
├── lidar_preprocess.py       filter/downsample + BODY_FRD -> LOCAL frame
├── mppi_local_planner_node.py lidar callback, control loop, RViz publishers
└── config.yaml               horizon, samples, vmax, safety_radius, cost weights
```

`scripts/mppi_velocity_avoidance.py` chỉ còn là wrapper CLI mỏng
(`--sim-test`, `--goal`, `--state-source`, `--rviz-traj-topic`, `--config`).

Thế giới, mốc địa lý, model UAV giống hệt demo
[`run_guided_sensor_demo_vi.md`](run_guided_sensor_demo_vi.md): warehouse Depot,
`iris_with_sensor_suite` (LiDAR 3D 640×16, 30 m), WGS84 `-35.363262, 149.165237`,
cao độ 584 m.

## Cài thêm (một lần) torch + pytorch-mppi vào env đã có pymavlink/gz

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/pip install torch pytorch-mppi
```

Node dùng chung env `ardupilot-rviz` (đã có `gz.transport13`, `gz.msgs10`,
pymavlink). CPU là đủ: bài toán 500 rollout × horizon 30 chỉ mất vài ms mỗi
chu kỳ.

Kiểm tra planner hoàn toàn offline (không cần Gazebo/MAVLink, obstacle giả
theo đúng geometry stack container `x=12, y∈[-6,6], z 0–24`):

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test
```

Kết quả mong đợi: `reached=True`, `min_clearance ≥ margin + 1 m`, dòng cuối
`[sim-test] PASSED`.

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
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/mppi_velocity.parm"
```

Khác demo BendyRuler ở đúng file parm: `config/mppi_velocity.parm` đặt
`OA_TYPE=0`, `AVOID_ENABLE=0`, `PRX1_TYPE=0`, `GUID_OPTIONS=0`, `WP_SPD=3`,
`WPNAV_SPEED=300` (3 m/s). `SERIAL1` vẫn là MAVLink 2 trên TCP 5762 cho node.
`-A "--serial1=tcp:2"` và quy tắc "mỗi lần một run Gazebo + SITL" giữ nguyên
như demo cũ.

Kiểm tra:

```text
param show OA_TYPE      # =0 ( avoidance ArduPilot đã tắt)
param show AVOID_ENABLE # =0
param show GUID_OPTIONS # =0
param show WPNAV_SPEED  # =300
```

## Terminal 4 — ROS 2 bridge và RViz

Giống hệt demo cũ (xem
[`run_guided_sensor_demo_vi.md`](run_guided_sensor_demo_vi.md#terminal-4--ros-2-bridge-và-rviz)):

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_warehouse_demo
export ROS_DOMAIN_ID=45

./scripts/run_sensor_rviz.sh
```

RViz để xem Odometry/PointCloud2; nút `2D Goal Pose` vẫn không dùng để điều
khiển (goal MPPI vào bằng `--goal` của node).

## Cất cánh 20 m

Trong MAVProxy:

```text
map zoom 500
mode guided
arm throttle
takeoff 20
```

## Terminal 5 — Node MPPI (chạy SAU khi đã takeoff, đang GUIDED)

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_warehouse_demo

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --mav tcp:127.0.0.1:5762
```

Mặc định `--goal "16,10,20;30,0,20"` (m, world ENU Gazebo): bay vòng phía bắc
stack container tại `x=12` rồi tới đích phía đông ~30 m — tương đương lệnh
`guided -35.363262 149.165600 20` của demo cũ. Node in mỗi chu kỳ:

```text
[mppi] cycle=42 pos=(  3.12,  1.44,20.05) goal_dist=13.10 nearest= 7.42m u=( 2.41, 1.62, 0.03) yaw_rate=+0.12
[mppi] tới waypoint 1/1: [30. 0. 20.]
[mppi] ĐÃ TỚI ĐÍCH sau ... chu kỳ; giữ nguyên vị trí.
```

`u` giờ có 4 chiều `[vx, vy, vz, yaw_rate]`; `yaw_rate` đổi dấu khi qua
`enu_to_ned` trước lúc gửi MAVLink (NED yaw quay thuận chiều kim đồng hồ).

Terminal là một **vùng chấp nhận**, không phải đòi sai số bằng 0. Mặc định
SITL hiện tại là `--goal-radius 0.25`. Khi vào vùng này, terminal được
latch, MPPI không tối ưu lại và node stream velocity-zero để ArduPilot hold.
Vì vậy process vẫn còn chạy là hành vi an toàn có chủ ý. Muốn trả terminal
shell ngay sau khi tới đích, thêm `--exit-on-goal`; sau đó phải chủ động
chuyển mode/land vì stream Guided setpoint đã dừng.

Lệnh velocity từng chu kỳ được low-pass và giới hạn slew-rate bởi
`command_alpha`, `max_accel_xy`, `max_accel_z`, `max_yaw_accel`. Đây là ràng
buộc interface do project bổ sung để giảm quỹ đạo khấp khuỷu, không phải
một kết quả Gazebo hay một thành phần được quy cho bài báo PA-MPPI.

### State source: odom hay MAVLink telemetry?

- `--state-source odom` (mặc định SITL): pose/velocity/yaw từ Gazebo
  ground truth — cùng frame với LiDAR và `--goal` tuyệt đối nên không lệch
  origin; transform LiDAR dùng full attitude quaternion.
- `--state-source mav`: pos/vel từ `LOCAL_POSITION_NED`, yaw từ `ATTITUDE`
  (đúng pipeline mentor vẽ: edge computer chỉ nghe autopilot). Lúc này
  `--goal` hiểu là ENU tương đối home, transform LiDAR dùng yaw-only
  `body_frd_to_ned()`; trong các world demo home ≈ gốc Gazebo nên vẫn
  dùng được bộ goal mặc định.

### Vẽ predicted trajectory lên RViz

Thêm `--rviz-traj-topic /mppi/predicted_path` để node publish nominal
rollout (`predict_trajectory()`) ra `nav_msgs/Path` (frame `odom`). Trong
RViz thêm display Path trỏ vào topic này: mentor nhìn một lần là thấy
ngay planner của mình đã thay BendyRuler — point cloud ở trước, vệt Path
tối ưu được chọn, rồi UAV bám theo.

`nearest` là khoảng cách tới điểm LiDAR gần nhất (m). Khi thấy
`ĐÃ TỚI ĐÍCH`, node chỉ còn gửi velocity 0 — drone hold; chạy `mode land` để
hạ cánh. Đổi đích/tuyến khác thì Ctrl+C node rồi chạy lại với `--goal` mới
(khi node dừng, ArduPilot hold sau `GUID_TIMEOUT`).

### Vì sao cần waypoint trung gian?

MPPI là planner **local**: nó tối ưu quỹ đạo trong horizon 3 s quanh nominal
trajectory. Nếu đứng yên trước một bức tường rộng, mọi rollout "vòng trái" và
"vòng phải" đối xứng có chi phí bằng nhau nên hành động trung bình triệt tiêu
— planner đứng yên. Cung waypoint (dù chỉ 1–2 điểm chọn tay, như `Fly To` của
MAVProxy Map) chính là phần "global planner" trong kiến trúc global + local;
MPPI lo phần né vật cản cục bộ quanh tuyến đó. Cùng lý do này PA-MPPI cũng
dùng reference path. Đặt waypoint cách obstacle tối thiểu `margin` (4 m) để
node không kẹt ở biên chi phí.

### Kiểm tra ArduPilot nhận đúng velocity

```text
watch POSITION_TARGET_LOCAL_NED
```

`vx, vy, vz` phải khớp (sau đổi trục ENU→NED) với `u=...` trong log node. Có
thể chạy node với `--no-mav` để chỉ xem log MPPI mà không điều khiển.

## Tham số đáng chỉnh

| Cờ | Mặc định | Ý nghĩa |
|---|---|---|
| `--goal` | `16,10,20;30,0,20` | tuyến waypoint `x,y,z;...` (m, ENU; mode `mav` thì tương đối home) |
| `--state-source` | `odom` | `odom`: Gazebo truth; `mav`: telemetry `LOCAL_POSITION_NED`+`ATTITUDE` |
| `--hz` | 10 | tần số vòng điều khiển; giữ ≥ 5 để không vượt `GUID_TIMEOUT` |
| `--vmax/--vzmax` | 2 / 1 | giới hạn tốc ngang/đứng (demo đầu cứ chậm) |
| `--yaw-rate-max` | 0.6 | giới hạn yaw rate [rad/s] |
| `--margin` | 4 | khoảng cách an toàn tới obstacle trong cost (≈ `AVOID_MARGIN` cũ) |
| `--horizon` | 30 | bước lookahead × `--dt 0.1` = 3 s quy hoạch |
| `--samples` | 500 | số rollout MPPI mỗi chu kỳ (GPU: 1024–2048) |
| `--noise-xy/--noise-z/--noise-yaw` | 0.8 / 0.3 / 0.3 | sigma nhiễu sampling |
| `--lambda` | 1.0 | temperature MPPI |
| `--tau` | 0.5 | hằng số bám tốc của mô hình điểm-mass (giả lập giới hạn gia tốc) |
| `--device` | cpu | đổi `cuda` nếu companion có GPU |
| `--rviz-traj-topic` | (tắt) | publish predicted rollout ra `nav_msgs/Path`, vd `/mppi/predicted_path` |
| `--config` | (không) | file yaml ghi đè mặc định (mẫu: `mppi_ardupilot/config.yaml`) |

Mô hình quy hoạch: state `[p, v, yaw]` (7 chiều), action
`u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]` (4 chiều);
`v' = v + (u − v)·dt/τ`, `p' = p + v'·dt`, `yaw' = yaw + yaw_rate·dt`.
Cost mỗi bước: khoảng cách tới waypoint đang bám + softplus(margin −
khoảng cách obstacle gần nhất, đã trừ offset để bằng 0 ngay tại biên
margin) + norm điều khiển + yaw bám hướng bay (tỉ lệ tốc ngang); cost
cuối: bình phương khoảng cách tới đích. Chi tiết trong
`mppi_ardupilot/mppi_controller.py` (`QuadMPPI`).

### Vì sao mask phải là 1479?

`SET_POSITION_TARGET_LOCAL_NED.type_mask` (bit 0-2 bỏ position, 6-8 bỏ
accel, bit 10 bỏ yaw, giữ velocity + yaw_rate) = `7 + 448 + 1024 = 1479`
— đã đối chiếu với `common.xml` của MAVLink, handler
`GCS_MAVLINK_Copter::handle_message_set_position_target_local_ned` và
enum của chính dialect pymavlink đang cài
(`mavlink_interface.check_mask_against_dialect()`). Bẫy đã gặp: nhầm bit
3-5 thành "position ignore" (thực ra là velocity ignore) khiến ArduPilot
bỏ velocity, đọc position (0,0,0), lại vô tình set bit FORCE_SET trong khi
accel không ignore → rơi vào nhánh `hold_position()`: drone đứng yên dù
log node vẫn đẹp. Mọi lệnh gửi đi đều dùng hằng dựng từ bit có tên
(`TYPE_MASK_VEL_YAWRATE`), không còn magic number.

## An toàn

- Lidar hoặc odometry cũ hơn 1 s → node tự gửi velocity 0 (hold).
- Điểm LiDAR gần hơn 1 m → zero velocity (brake).
- Ctrl+C node → ArduPilot hold sau `GUID_TIMEOUT`; hạ cánh bằng `mode land`.
- Không chạy node khi UAV chưa cất cánh: velocity target bị ArduPilot từ chối.

## Hạ cánh và dừng

```text
mode land
```

Chờ `DISARMED`, dừng node MPPI trước, rồi lần lượt MAVProxy/SITL, RViz/bridge,
Gazebo GUI, Gazebo server.

## Quay video xác nhận gửi mentor

Tương tự demo BendyRuler, video nên theo thứ tự:

1. Gazebo warehouse + RViz PointCloud2 cập nhật, drone đã takeoff 20 m.
2. Terminal node MPPI hiện các dòng `cycle=... pos=... nearest=...m u=...`.
3. MAVProxy `watch POSITION_TARGET_LOCAL_NED` cho thấy velocity ArduPilot
   đang bám khớp `u` của node; thoát watch bằng `Ctrl+C`.
4. UAV vòng qua stack container (vệt Odometry cong trong RViz), log node có
   dòng `tới waypoint 1/1` rồi `ĐÃ TỚI ĐÍCH`.
5. `mode land`, chờ `DISARMED`, dừng quay.

Lưu vào `output/video/gazebo_warehouse_mppi_velocity_avoidance.mp4`.

## Diagnostics, sampled rollouts và benchmark

Để quay demo có đủ bằng chứng thuật toán, chạy node với predicted path, 20
rollout tốt nhất và JSONL diagnostics:

```bash
mkdir -p output/log
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --mav tcp:127.0.0.1:5762 \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --rviz-top-k 20 \
  --diag-every 10 \
  --diag-jsonl output/log/mppi_demo.jsonl
```

Trong RViz, thêm `MarkerArray` với topic `/mppi/sampled_trajectories`. Log
`[diag]` hiển thị compute time, rolling mean/p95/worst, deadline miss, ESS và
các thành phần goal/obstacle/smoothness/terminal cost. File JSONL lưu đầy đủ
mỗi chu kỳ để vẽ bảng và so sánh planner sau demo.

Test suite lưu trong repo, không còn phụ thuộc file `/tmp`:

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python -m unittest \
  discover -s tests -p 'test_mppi*.py' -v
```

## PA-MPPI v0

`--planner pa-mppi` bật bước port đầu tiên của Perception-Aware MPPI. Bản v0
giữ dynamics velocity-level và MAVLink interface hiện tại, đồng thời bổ sung:

- occupancy grid 3D với ba trạng thái unknown/free/occupied;
- ray integration từ LiDAR: ray là free, endpoint là occupied;
- collision cost cho occupied và unknown voxel;
- perception cost tại endpoint rollout: hướng camera về goal, thưởng ray đi
  tới unknown frontier, phạt ray đụng occupied voxel;
- tắt perception cost khi đã có line-of-sight trực tiếp tới goal.

Chạy thử:

```bash
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner pa-mppi \
  --mav tcp:127.0.0.1:5762 \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --diag-jsonl output/log/pa_mppi_demo.jsonl
```

Đây là **port v0**, chưa phải reproduction đầy đủ của paper. Paper dùng full
rigid-body dynamics, control `[collective_thrust, body_rates]`, ROG-Map và
17,500 rollout ở 50 Hz. Baseline trong repo vẫn dùng state 7D và velocity
setpoint để ArduPilot giữ tầng attitude control. Giai đoạn kế tiếp cần benchmark
v0, sau đó mới quyết định port full dynamics hay giữ kiến trúc velocity-level.
Nguồn đối chiếu: [Zhai, Reiter và Scaramuzza, IEEE RA-L
2026](https://rpg.ifi.uzh.ch/docs/RAL26_Zhai.pdf).

## Rigid-body PA-MPPI experimental trong Gazebo

Nhánh `--planner rigid-pa-mppi` là bước port tiếp theo. Planner dùng state
`[p_W, q_WB, v_W, omega_B]` và control
`[collective_thrust_N, p_rate, q_rate, r_rate]`. Quaternion ánh xạ body FLU
của Gazebo sang world ENU. Trước khi gửi MAVLink, body rate được đổi FLU sang
FRD. ArduPilot vẫn đóng vòng body-rate và motor.

Kiểm tra dynamics hoàn toàn offline trước:

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner rigid-pa-mppi \
  --config mppi_ardupilot/rigid_pa_mppi_config.yaml \
  --sim-test
```

Kết quả phải có `position_error=0`, `velocity_error=0`, quaternion norm bằng
1 và `[rigid-test] PASSED`. Test suite còn chạy một closed-loop clear-air dài
80 chu kỳ để kiểm tra planner tiến tới goal mà vẫn giữ cao độ.

### Tham số SITL bắt buộc

ArduCopter chỉ hiểu `SET_ATTITUDE_TARGET.thrust` là thrust khi
`GUID_OPTIONS` bit 3 được bật. Chạy SITL với hai file tham số theo đúng thứ tự:

```bash
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON -N -w \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/mppi_velocity.parm" \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/mppi_attitude_sitl.parm" \
  -A "--serial1=tcp:2"
```

Xác nhận trong MAVProxy trước khi chạy node:

```text
param show GUID_OPTIONS     # 8
param show MOT_THST_HOVER   # 0.38
```

Giá trị 0.38 lấy từ flight log SITL hiện có, trong đó ArduPilot từng học
`MOT_THST_HOVER` khoảng 0.36--0.386. Nếu đổi mass/model/propeller thì phải
calibrate lại.

### Gate thử live đầu tiên

Chỉ chạy sau khi UAV đã takeoff 20 m và đang GUIDED. Lần đầu dùng goal ngắn
1 m trong không gian trống, chưa chạy warehouse obstacle:

```bash
MAVLINK20=1 KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner rigid-pa-mppi \
  --config mppi_ardupilot/rigid_pa_mppi_config.yaml \
  --experimental-attitude-control \
  --state-source odom \
  --goal "1,0,20" \
  --mav tcp:127.0.0.1:5762 \
  --diag-jsonl output/log/rigid_hover_gate.jsonl
```

Node từ chối chạy live nếu thiếu cờ xác nhận, dùng state MAVLink hoặc tần số
dưới 20 Hz. Khi stale, brake, reached hoặc Ctrl+C, node chuyển về velocity-zero
GUIDED target thay vì gửi zero thrust. Có người trực MAVProxy để chuyển
`mode loiter` hoặc `mode land` ngay khi altitude/attitude lệch.

Nhánh này đã qua offline dynamics và optimizer tests nhưng **chưa bay live
trong Gazebo**. Chỉ sau khi hover gate và goal 1 m đạt mới bật perception cost
với stack container.

## So sánh với demo BendyRuler (để báo cáo)

| | BendyRuler (demo cũ) | MPPI (demo này) |
|---|---|---|
| Nơi chạy avoidance | Trong ArduPilot (`AP_OABendyRuler` qua `AC_WPNav_OA`) | Companion (node Python + torch) |
| Dữ liệu LiDAR vào | `OBSTACLE_DISTANCE_3D` → `AP_Proximity` → boundary/database; spec cho phép `obstacle_id=UINT16_MAX` khi không biết ID | Point cloud thẳng vào map/cost MPPI (world ENU) |
| Lệnh điều khiển | Position target (`MAV_CMD_DO_REPOSITION`, GUID_OPTIONS bit 6) | Velocity + yaw_rate (`SET_POSITION_TARGET_LOCAL_NED`, mask 1479) |
| Global direction | BendyRuler tự bẻ quanh database object | Waypoint route từ `--goal` |
| Ghép nối | Phải chuyển map/perception nội bộ sang representation của AP avoidance | Planner giữ trực tiếp map ba trạng thái nhưng tự chịu trách nhiệm safety/local optimum |

Việc chạy planner ở companion là **lựa chọn kiến trúc của project**, không phải
do ArduPilot bắt buộc persistent obstacle ID. Ở target ArduPilot commit
`f808f78ce5a518ca96f2fb36420608b2b6254367`, handler
`AP_Proximity_MAV::handle_obstacle_distance_3d_msg` không đọc trường
`obstacle_id`; xem audit đầy đủ tại `docs/SOURCE_AUDIT.md`.

Lưu ý riêng cho `SET_ATTITUDE_TARGET`: trang “Copter Commands in Guided Mode”
hiện nói body-rate fields không được hỗ trợ, nhưng source target commit trên nhận
đủ cả ba rate và đưa về hold nếu chỉ một phần rate bị ignore. Project theo source
đúng commit, dùng mask 128, đồng thời giữ discrepancy này trong báo cáo thay vì
coi tài liệu chung là bằng chứng duy nhất.

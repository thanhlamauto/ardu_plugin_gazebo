# Handoff: MPPI local planner trên companion + ArduPilot GUIDED

Ngày: 2026-09-08. Người nhận: agent tiếp tục dự án (đọc file này là đủ).

## 1. Bối cảnh

Repo `ardupilot_gazebo` (Gazebo Harmonic + ArduPilot SITL + warehouse Depot).
Tuần 3: avoidance bằng `OBSTACLE_DISTANCE_3D` → `AP_Proximity` → BendyRuler
(`docs/run_guided_sensor_demo_vi.md`). Mentor chốt kiến trúc mới: **không nhét
MPPI vào ArduPilot**; edge computer/ROS 2 làm tầng local planner, ArduPilot chỉ
bám velocity setpoint do MPPI sinh ra.

Pipeline đã triển khai:

```text
LiDAR 3D → preprocess → local obstacle cloud ─┐
MAVLink telemetry / Gazebo odometry ───────────┤→ MPPI → u=[vx,vy,vz,yaw_rate]
                                               → SET_POSITION_TARGET_LOCAL_NED
                                               → ArduPilot GUIDED → quadrotor
```

## 2. Cấu trúc code (mới, đã xong)

```text
mppi_ardupilot/
├── mppi_controller.py         QuadMPPI 7-state [p,v,yaw] / 4-action, predict_trajectory()
├── pa_mppi_controller.py      PA-MPPI v0: perception/collision cost, two-phase objective
├── rigid_body_pa_mppi.py      experimental [p,q,v,omega], thrust/body-rate control
├── occupancy_grid.py          voxel map unknown/free/occupied + ray integration
├── mavlink_interface.py       ArduPilotInterface: setup_telemetry, get_state, send_velocity_ned
├── lidar_preprocess.py        downsample week 3 + body_frd_to_ned() + helper ENU↔NED/yaw
├── mppi_local_planner_node.py control loop + waypoint + an toàn + RViz Path publisher
└── config.yaml                horizon 30, samples 500, vmax 2.0, vzmax 1.0, yaw_rate_max 0.6,
                               noise 0.8/0.3/0.3, margin 4.0 (xem file)
scripts/mppi_velocity_avoidance.py  wrapper CLI (--planner mppi|pa-mppi, diagnostics,
                               sampled rollouts, --config, --mav)
config/mppi_velocity.parm      OA_TYPE=0, AVOID_ENABLE=0, PRX1_TYPE=0, GUID_OPTIONS=0
docs/run_mppi_demo_vi.md       doc demo đã cập nhật đầy đủ
```

`--state-source odom` (mặc định SITL): pose từ Gazebo ground truth, goal ENU
tuyệt đối, LiDAR dùng full attitude quaternion. `--state-source mav`: pos/vel
từ `LOCAL_POSITION_NED` + yaw từ `ATTITUDE`, goal ENU tương đối home, LiDAR
dùng yaw-only `body_frd_to_ned()`.

## 3. Bug quan trọng đã sửa (đọc kỹ trước khi đụng MAVLink)

Bản node cũ dùng mask tự tính **15928**: bit 3–5 tưởng là "bỏ position" nhưng
thực ra là **bỏ velocity** → ArduPilot bỏ qua `u`, đọc position (0,0,0), lại
dính bit FORCE_SET trong khi accel không ignore → rơi vào `hold_position()`.
Triệu chứng: drone đứng yên dù log node đẹp.

Đã đối chiếu 3 nguồn (MAVLink `common.xml` trong `~/Projects/ardupilot`,
`GCS_MAVLINK_Copter::handle_message_set_position_target_local_ned`,
enum dialect pymavlink trong env): mask đúng cho **velocity + yaw_rate là 1479**
(bỏ pos + accel + yaw). Code dùng hằng dựng từ bit có tên
`TYPE_MASK_VEL_YAWRATE` + `assert == 1479` + `check_mask_against_dialect()`.

## 4. Đã verify (có số liệu)

- `--sim-test`: `reached=True`, `min_clearance≈6.2m ≥ margin+1`, yaw bám hướng
  bay. PASSED.
- `/tmp/test_mppi_package.py` (27 check, chạy lại được): mask bit-by-bit,
  `body_frd_to_ned`, đổi dấu yaw_rate ENU↔NED, `parse_state`, logic
  hold/brake/reached. ALL PASSED.
- **Bay SITL thật** (Gazebo server có sẵn + arducopter tự chạy + node live):
  takeoff 20m → `(-6,0,20)` vòng đầu bắc stack → `(29.6,-0.7,20.2)`,
  goal_dist 0.84m → hold → land → DISARMED. `nearest` min 6.51m, không
  brake/stale lần nào. Log: `/tmp/mppi_live.log` (1331 chu kỳ).
  Chuyển động + tới đích chính là bằng chứng mask 1479 có tác dụng.

## 5. Việc dở + bẫy môi trường (quan trọng)

- PA-MPPI v0 đã chạy nhưng chưa paper-faithful. Bản này giữ velocity-level
  dynamics và MAVLink output. Nhánh `rigid-pa-mppi` đã thêm rigid-body rollout
  và `SET_ATTITUDE_TARGET`, nhưng chưa qua hover gate live trong Gazebo.
- `GUID_OPTIONS=8` là bắt buộc cho thrust-as-thrust; dùng override
  `config/mppi_attitude_sitl.parm`. Không dùng nhánh này trên quad thật.
- Test chính thức nằm ở `tests/test_mppi_core.py`: hiện 18/18 đạt trong env
  `/opt/miniconda3/envs/ardupilot-rviz`.

- Gazebo server của user vẫn chạy (PID 98156): world `iris_warehouse_sensor`,
  **partition `ardupilot_mppi_e2e`** (không phải `ardupilot_warehouse_demo`
  như doc cũ). Mọi lệnh gz/node phải export partition này.
- `sim_vehicle.py` lỗi trên máy này (`MAVProxy exited` → kill SITL). Đã bay
  bằng cách chạy `arducopter` trực tiếp (xem lệnh mục 6). Khi cần MAVProxy
  console thì điều tra riêng.
- Script takeoff phải có `request_data_stream_send` nếu không
  `GLOBAL_POSITION_NED`/`GLOBAL_POSITION_INT` không chảy về connection mới
  (từng timeout giả). Đọc `COMMAND_ACK` phải trên cùng connection đã gửi lệnh.
  Script mẫu: `/tmp/mppi_takeoff.py`, `/tmp/mppi_arm.py`, `/tmp/mppi_land.py`,
  `/tmp/mppi_diag.py` (throwaway, chưa commit).
- Sandbox của session trước đã siết `bind()` (mọi port EPERM) và shell không
  thấy display → **không mở được Gazebo GUI, không chạy được SITL từ agent**.
  User phải tự mở GUI + SITL trên Terminal của mình; agent chỉ hỗ trợ node/log.
- Drone đang nằm ở `(29.6,-0.7)` mặt đất sau lần land. Route bay về đã chuẩn
  bị: `--goal "16,-10,20;-6,0,20"` (né phía nam stack).
- Chưa quay video cho mentor (lần bay trước headless). Cần: GUI + RViz display
  Path `/mppi/predicted_path` qua `--rviz-traj-topic`.

## 6. Lệnh chạy demo (user tự mở terminal)

```bash
# T1: Gazebo server (đang chạy sẵn, giữ nguyên)
# T2: GUI
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_e2e
gz sim -g --gui-config "$PWD/config/gazebo_runway_camera.config"

# T3: SITL
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"
cd ~/Projects/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON -N -w \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/mppi_velocity.parm" \
  -A "--serial1=tcp:2"
# fallback nếu MAVProxy thoát: chạy arducopter trực tiếp
./build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --serial1=tcp:2 \
  --defaults "$HOME/Projects/ardupilot_gazebo/config/mppi_velocity.parm" \
  --sim-address=127.0.0.1 -I0 --home -35.363262,149.165237,584.0,0.0

# MAVProxy: mode guided → arm throttle → takeoff 20

# T5: node MPPI (sau takeoff, đang GUIDED)
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_e2e
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --mav tcp:127.0.0.1:5762 \
  --goal "16,-10,20;-6,0,20" --rviz-traj-topic /mppi/predicted_path
# hạ cánh: mode land, chờ DISARMED
```

## 7. Quyết định đã chốt với mentor (đừng đảo lại nếu chưa hỏi)

Vanilla MPPI làm baseline, PA-MPPI v0 chạy song song để so sánh; không dùng `nav2_mppi_controller` (xe đất);
output `u=[vN,vE,vD,yaw_rate]`, chưa đụng `wx,wy` (tầng `SET_ATTITUDE_TARGET`);
giữ cao độ 20m cố định ở demo đầu; waypoint tay làm global direction vì MPPI
local kẹt trước obstacle rộng đối xứng.

Chạy PA-MPPI offline:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --planner pa-mppi \
  --config mppi_ardupilot/config.yaml --sim-test --samples 128 --horizon 20
```

Kết quả gần nhất: `reached=True`, `final_goal_dist=0.86 m`,
`min_clearance=5.85 m`, `PASSED`.

Rigid-body offline gate:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --planner rigid-pa-mppi \
  --config mppi_ardupilot/rigid_pa_mppi_config.yaml --sim-test
```

Kết quả: hover error bằng 0, quaternion norm bằng 1, `PASSED`. Benchmark CPU
512 rollout/H20 đạt mean 7.47 ms, p95 8.39 ms và worst 9.24 ms trên máy hiện
tại. Lệnh hover map sang normalized thrust 0.38 dựa trên flight log SITL.

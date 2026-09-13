# Quickstart: chạy 3 map MPPI + Gazebo + RViz samples

File này dành để copy/paste nhanh. Chỉ chạy **một map tại một thời điểm**.
Mỗi lần đổi map phải dừng node MPPI, land, rồi reset Gazebo và SITL.

## Ba map

| Map | World | Route chính |
|---|---|---|
| 1. Slalom | `iris_mppi_slalom.sdf` | Luồn qua bốn cột xen kẽ |
| 2. Narrow gate | `iris_mppi_narrow_gate.sdf` | Qua cửa lớn rồi vòng phía bắc cụm cột |
| 3. Right angle | `iris_mppi_right_angle.sdf` | Đi trong hành lang và cua 90 độ |

Profile dùng trong các lệnh demo dưới:

```text
Slalom:                   config/experiments/mppi_slalom_demo_smooth.yaml
Narrow gate/right-angle: config/experiments/mppi_demo_smooth.yaml
```

Đây vẫn là `cost_profile: paper`: effort `R`, input-change `R_delta`,
position/velocity reference theo thời gian trên global path và collision
indicator trên point-cloud đã inflate. So với profile audit
`mppi_paper_cost_only.yaml`, profile demo giảm reference speed `1.0 -> 0.60
m/s`, giới hạn vận tốc `1.5 -> 1.0 m/s`, giới hạn gia tốc ngang `1.5 -> 0.6
m/s²`, tăng làm mượt `command_alpha: 0.45 -> 0.30`, và dùng arrival gate
`0.35 m`. Đây là engineering tune cho demo, không phải tham số của paper.
Cấu hình chậm này là engineering candidate để quay video. Kết quả Gazebo
headless mới nhất được ghi riêng bên dưới; chạy kèm Gazebo GUI và RViz vẫn
phải kiểm tra lại vì tải đồ họa có thể ảnh hưởng deadline.

Để giảm gãy ở waypoint, profile demo còn bo góc reference với bán kính `0.8
m` (`reference_corner_samples: 6`). Cost đổi lệnh tính cả bước nối từ lệnh đã
gửi ở cycle trước tới lệnh đầu của rollout và dùng
`paper_r_delta_u: [100,100,50,20]`. Hai weight ngang bằng nhau để không thiên
lệch theo trục ENU. Đây là project tune; profile audit vẫn giữ đúng weight đối
chiếu paper.

A/B offline, seed 7, cùng reduced dynamics cho thấy thay đổi mới vẫn đạt
terminal ở cả ba case và giảm tổng biến thiên lệnh: right-angle `32.95 ->
31.46`, slalom `51.25 -> 50.37`, narrow-gate `51.38 -> 48.62`. Đây chỉ là
`VERIFIED OFFLINE`; phải tạo log Gazebo mới trước khi dùng các con số này để
kết luận chất lượng bay 3D.

Ba lệnh bên dưới truyền `--global-path` tương ứng với route
clearance-checked của từng map. Profile này là adaptation velocity-level,
không phải reproduction đầy đủ của paper. `mppi_my_test.yaml` vẫn được
giữ làm baseline project-cost khi cần.

File YAML không hot-reload. Sau khi sửa config phải dừng và chạy lại Terminal 5.

## Kết quả kiểm chứng hiện có

### Profile demo làm mượt — Gazebo headless

Ngày 2026-09-13, `mppi_demo_smooth.yaml` được kiểm tra trên right-angle và
narrow-gate; slalom dùng profile riêng `mppi_slalom_demo_smooth.yaml`. Cả ba
dùng ArduPilot SITL commit `f808f78c`. Slalom đã chạy từ trạng thái reset với
hai seed độc lập:

| Map / seed | Terminal | Chu kỳ | Dài quỹ đạo | Min point-cloud clearance | Compute mean / p95 / worst | Deadline miss / timeout / collision-cost |
|---|---:|---:|---:|---:|---:|---:|
| Right-angle / 7 | đạt, 0.282 m | 178 | 22.87 m | 2.40 m | 15.7 / 17.6 / 53.1 ms | 0 / 0 / 0 |
| Narrow-gate / 7 | đạt, 0.243 m | 311 | 35.00 m | 1.84 m | 15.2 / 16.9 / 77.1 ms | 0 / 0 / 0 |
| Slalom tuned / 7 | đạt, 0.350 m | 311 | 34.63 m | 1.96 m | 10.7 / 11.5 / 51.5 ms | 0 / 0 / 0 |
| Slalom tuned / 19 | đạt, 0.323 m | 313 | 34.75 m | 1.95 m | 10.4 / 11.3 / 45.7 ms | 0 / 0 / 0 |

Hai run slalom tuned không kích hoạt hard brake. Đây là kết quả live headless,
không phải bằng chứng cho hiệu năng khi đồng thời mở Gazebo GUI và RViz.

### Profile audit cũ

Ngày 2026-09-13, ba route bên dưới đã được chạy trực tiếp với
`mppi_paper_cost_only.yaml`, seed 7, Gazebo Harmonic và ArduPilot SITL commit
`f808f78c`. Bảng này là bằng chứng cho profile audit cũ, **không được
gán sang profile `mppi_demo_smooth.yaml` khi chưa ghi log Gazebo mới**:

| Map | Terminal | Chu kỳ | Dài quỹ đạo | Min point-cloud clearance | Cross-track mean / p95 | Compute mean / p95 / worst |
|---|---:|---:|---:|---:|---:|---:|
| Right angle | đạt, 0.235 m | 128 | 23.15 m | 2.19 m | 0.17 / 0.47 m | 14.5 / 14.9 / 51.9 ms |
| Narrow gate | đạt, 0.230 m | 192 | 35.50 m | 2.07 m | 0.14 / 0.39 m | 13.3 / 14.7 / 53.9 ms |
| Slalom | đạt, 0.220 m | 193 | 37.67 m | 1.30 m | 0.44 / 1.14 m | 13.6 / 15.5 / 50.7 ms |

Không run nào có deadline miss, `hold-brake` hay `recover-brake`. Clearance là
khoảng cách từ tâm odometry tới điểm LiDAR gần nhất, **không phải** khoảng hở
hình học đã trừ bán kính UAV. Slalom xuống 1.30 m dù rollout collision radius
là 1.5 m, nên vẫn phải kiểm tra nhiều seed và hiệu chuẩn footprint/sensor
trước khi kết luận safety.

Log gốc nằm trong `output/log/*_paper_v2_live_20260913.jsonl`; plot nằm trong
`output/plots/paper_global_path_v2_live_20260913/`.

## Thứ tự 5 terminal

```text
Terminal 1: Gazebo server
Terminal 2: Gazebo GUI
Terminal 3: ArduPilot SITL + MAVProxy
Terminal 4: ROS 2 bridge + RViz
Terminal 5: MPPI planner + MAVLink + RViz trajectory publisher
```

## Terminal 1 — Gazebo server

Chọn đúng **một** trong ba block sau.

### Map 1 — Slalom

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 -r "$PWD/worlds/iris_mppi_slalom.sdf" -s
```

### Map 2 — Narrow gate

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 -r "$PWD/worlds/iris_mppi_narrow_gate.sdf" -s
```

### Map 3 — Right angle

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 -r "$PWD/worlds/iris_mppi_right_angle.sdf" -s
```

## Terminal 2 — Gazebo GUI

Lệnh giống nhau cho cả ba map:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge

gz sim -v1 -g \
  --gui-config "$PWD/config/gazebo_runway_camera.config"
```

## Terminal 3 — ArduPilot SITL + MAVProxy

Lệnh giống nhau cho cả ba map:

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

Kiểm tra trong MAVProxy:

```text
param show OA_TYPE
param show AVOID_ENABLE
param show GUID_OPTIONS
```

Kỳ vọng cho velocity-level MPPI:

```text
OA_TYPE      = 0
AVOID_ENABLE = 0
GUID_OPTIONS = 0
```

## Terminal 4 — ROS 2 bridge + RViz

Lệnh giống nhau cho cả ba map:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

./scripts/run_sensor_rviz.sh
```

Lúc này RViz chỉ thấy sensor/odometry. Topic MPPI chỉ xuất hiện sau khi Terminal
5 được chạy với các cờ `--rviz-*`.

## Takeoff trước khi chạy Terminal 5

Trong MAVProxy:

```text
mode guided
arm throttle
takeoff 20
```

Chờ UAV hover ổn định quanh 20 m rồi mới chạy MPPI.

## Terminal 5 — chọn lệnh theo map

### Map 1 — MPPI slalom

Slalom dùng profile riêng để giữ deadline 10 Hz trong run dài. Các waypoint
được đặt cùng hoành độ với tâm từng cột và lệch sang phía đối diện. Sau khi
bo góc `0.8 m`, khoảng cách hình học nhỏ nhất từ reference tới bề mặt cột xấp
xỉ `2.03 m` (chưa tính sai số bám thực tế), lớn hơn
`collision_radius_m: 1.5`. Route cũ đặt waypoint sớm hơn cột `1.5 m` chỉ có
khoảng hở hình học xấp xỉ `1.14 m` nên không còn dùng cho demo.

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_slalom_demo_smooth.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '7,-2.2,20;13,2.2,20;19,-2.2,20;25,2.2,20;30,0,20' \
  --global-path '0,0,20;7,-2.2,20;13,2.2,20;19,-2.2,20;25,2.2,20;30,0,20' \
  --seed 7 \
  --diag-every 1 \
  --diag-jsonl output/log/slalom_demo_smooth_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --rviz-top-k 5 \
  --exit-on-goal
```

### Map 2 — MPPI narrow gate

Route chính bên dưới đi qua cửa tường lớn rồi vòng phía bắc cụm hai cột.
Đây là route demo; nó **không đi xuyên khe nhỏ giữa hai cột**. Không
dùng route stress-test phía dưới khi quay demo.

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_demo_smooth.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '13,0,20;12.5,5.8,20;23,5.8,20;24,0,20' \
  --global-path '0,0,20;13,0,20;12.5,5.8,20;23,5.8,20;24,0,20' \
  --seed 7 \
  --diag-every 1 \
  --diag-jsonl output/log/narrow_gate_demo_smooth_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --rviz-top-k 5 \
  --exit-on-goal
```

Route stress test cũ, chỉ dùng để tái lập failure/trade-off trong SITL:

```text
13,0,20;18,-0.2,20;24,0,20
```

Không dùng route stress-test này để chứng minh safety.

### Map 3 — MPPI right-angle corridor

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_demo_smooth.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '5.5,0,20;10,0,20;10,5,20;10,13,20' \
  --global-path '0,0,20;5.5,0,20;10,0,20;10,5,20;10,13,20' \
  --seed 7 \
  --diag-every 1 \
  --diag-jsonl output/log/right_angle_demo_smooth_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --rviz-top-k 5 \
  --exit-on-goal
```

## Thêm MPPI visualization vào RViz

Sau khi Terminal 5 in:

```text
[rviz] publish predicted trajectory -> /mppi/predicted_path
[rviz] publish top-5 samples -> /mppi/sampled_trajectories
```

Trong RViz:

```text
Global Options → Fixed Frame = odom

Add → By topic → /mppi/predicted_path → Path
Add → By topic → /mppi/sampled_trajectories → MarkerArray
```

Ý nghĩa:

```text
/mppi/predicted_path       = nominal trajectory MPPI đang chọn
/mppi/sampled_trajectories = top-K rollout có cost thấp nhất
```

Nếu không thấy topic:

```bash
export ROS_DOMAIN_ID=45
export PATH="/opt/miniconda3/envs/ardupilot-rviz/bin:$PATH"

ros2 topic list | grep mppi
```

## Click goal trực tiếp trong RViz

File `config/sensor_suite.rviz` đã có tool `2D Goal Pose`, publish
`geometry_msgs/PoseStamped` lên `/goal_pose`. Để điều khiển tương tác, dùng
lệnh Terminal 5 sau cho bất kỳ world nào:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_demo_smooth.yaml \
  --mav tcp:127.0.0.1:5762 \
  --rviz-goal-topic /goal_pose \
  --rviz-goal-frame odom \
  --rviz-goal-altitude 20 \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories \
  --rviz-top-k 5 \
  --diag-every 1 \
  --diag-jsonl output/log/rviz_goal_demo_smooth_seed7.jsonl
```

Không thêm `--exit-on-goal`: node phải tiếp tục chạy để nhận lần click tiếp
theo. Sau khi Terminal 5 báo `chờ goal`, trong RViz:

1. Chọn `2D Goal Pose` trên toolbar.
2. Click vị trí đích trên mặt phẳng XY và kéo mũi tên theo hướng bất kỳ.
3. Terminal phải báo `goal#N accepted ENU=(x,y,20)`.
4. Có thể click goal mới khi UAV đang bay hoặc sau khi đã tới goal cũ; route
   cũ và MPPI warm start sẽ được reset.

RViz `2D Goal Pose` thường gửi `z=0`; node cố ý không dùng giá trị này.
`--rviz-goal-altitude 20` giữ target tại 20 m. Nếu bỏ cờ altitude, node giữ
độ cao UAV tại đúng thời điểm click. Orientation của mũi tên hiện chưa dùng để
đặt yaw goal.

Mỗi click tạo reference thẳng `current position -> clicked goal`; đây là
interactive waypoint interface, **không phải global planner**. Trên slalom,
U-wall hoặc góc khuất, hãy click lần lượt các waypoint có clearance thay vì
click xuyên vật cản. MAVProxy vẫn cần cho `arm throttle`, `takeoff 20`,
`mode brake` khẩn cấp và `mode land`; không còn cần `Fly To` để điều hướng.

## Điều chỉnh global path trong paper profile

Trong profile demo, `w_path=400`, `w_reference_velocity=40` và
`reference_speed_m_s=0.60` đã bật. `--goal` là route/waypoint; `--global-path`
được time-parameterize thành chuỗi `(p_ref[j], v_ref[j])` cho horizon.
Khi thay map hoặc route, phải cập nhật cả hai tham số:

```bash
--global-path 'x0,y0,z0;x1,y1,z1;x2,y2,z2;...'
```

Polyline phải cùng frame ENU, cùng đơn vị mét, và đã kiểm tra clearance; path
cost không thay thế collision cost. Có thể tune `reference_speed_m_s`,
`w_path`, `w_reference_velocity`, `paper_r_u`, `paper_r_delta_u`, `lambda`,
`reference_corner_radius_m` và `collision_radius_m` trong
`config/experiments/mppi_demo_smooth.yaml`, hoặc
`config/experiments/mppi_slalom_demo_smooth.yaml` khi chạy slalom; sau mỗi lần
sửa phải khởi động lại Terminal 5. Nếu cần lặp lại đúng kết quả audit ngày
2026-09-13 thì
dùng `mppi_paper_cost_only.yaml`. Nếu muốn chạy baseline project-cost, đổi config về
`mppi_my_test.yaml`; khi đó `w_path: 0` và có thể bật path cost bằng
`--w-path`.

Trong dòng `[diag]`, trường `smooth=` của paper profile chính là cost
`input_change`, đã gồm độ nhảy từ lệnh gửi ở cycle trước tới lệnh đầu của
rollout. Nếu tăng weight mà `smooth` vẫn gần như không đổi và acceleration liên
tục chạm hard limit, nguyên nhân còn lại thường là corner reference, weight
collapse (ESS thấp) hoặc giới hạn slew, không phải thiếu weight đơn thuần.

## Dừng và reset trước khi đổi map

Thực hiện theo thứ tự:

1. Terminal 5: `Ctrl+C` để dừng MPPI.
2. Terminal 3/MAVProxy: `mode land`.
3. Chờ disarm rồi `Ctrl+C` SITL.
4. Terminal 4: `Ctrl+C` bridge/RViz.
5. Terminal 2: `Ctrl+C` Gazebo GUI.
6. Terminal 1: `Ctrl+C` Gazebo server.
7. Khởi động lại Terminal 1 tới Terminal 4 với world mới.
8. Takeoff 20 m, sau đó mới chạy Terminal 5 tương ứng.

Không chạy hai world hoặc hai node MPPI đồng thời trên cùng partition/MAVLink.

## Đổi seed mà không ghi đè log

Ví dụ seed 19:

```bash
--seed 19 \
  --diag-jsonl output/log/slalom_paper_seed19.jsonl
```

Logger mở file theo chế độ append. Mỗi lần chạy phải dùng tên JSONL mới, nếu
không dữ liệu của nhiều run sẽ nối vào cùng file.

## Vẽ log sau từng map

Ví dụ slalom:

```bash
cd ~/Projects/ardupilot_gazebo

/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/plot_mppi_experiment.py \
  --run slalom=output/log/slalom_demo_smooth_seed7.jsonl \
  --output-dir output/plots/slalom_demo_smooth_seed7
```

Đổi label/path/output-dir tương ứng cho `narrow_gate` và `right_angle`.

## Điều kiện dừng thử nghiệm

Trong MAVProxy dùng `mode brake` hoặc `mode land` nếu:

- predicted path hoặc sample fan xuyên obstacle;
- `hold-timeout`, `hold-brake` hoặc `hold-stale` lặp lại;
- clearance xuống dưới margin;
- lệnh rung mạnh hoặc UAV đi ngược route;
- Gazebo real-time factor giảm mạnh;
- frame LiDAR/odometry không khớp visual map.

Ba world/profile này là môi trường thử nghiệm SITL, chưa được xác nhận cho
edge computer hoặc hardware.

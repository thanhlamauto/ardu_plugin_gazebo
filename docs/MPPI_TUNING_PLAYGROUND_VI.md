# MPPI tuning playground: map, địa hình và thí nghiệm tái lập

Tài liệu này là sổ tay thực hành để tự thay đổi:

- hình học world Gazebo;
- route/waypoint;
- dynamics prediction của MPPI;
- sampling, cost và giới hạn control;
- seed và số lần lặp;
- quy trình chuyển từ offline sang Gazebo/SITL.

Bản lệnh rút gọn để copy/paste chạy cả ba challenge world nằm tại
[`RUN_3_MPPI_MAPS_QUICKSTART_VI.md`](RUN_3_MPPI_MAPS_QUICKSTART_VI.md).

Mục tiêu là mỗi thay đổi đều trả lời được: **đã đổi gì, vì sao đổi, metric nào
tốt/xấu hơn, kết quả thuộc offline hay Gazebo?**

## 1. Các file cần biết

| Việc muốn thay đổi | File |
|---|---|
| Cấu hình MPPI live mặc định | `mppi_ardupilot/config.yaml` |
| Giá trị mặc định trong Python | `mppi_ardupilot/mppi_controller.py` |
| Logic waypoint, arrival và conditioner | `mppi_ardupilot/mppi_local_planner_node.py` |
| CLI chạy planner | `scripts/mppi_velocity_avoidance.py` |
| Benchmark offline nhiều scenario | `scripts/benchmark_mppi_tuning.py` |
| Map slalom | `worlds/iris_mppi_slalom.sdf` |
| Map khe hẹp | `worlds/iris_mppi_narrow_gate.sdf` |
| Map cua chữ L | `worlds/iris_mppi_right_angle.sdf` |
| Vẽ log Gazebo sau chuyến bay | `scripts/plot_mppi_experiment.py` |

`config.yaml` chỉ được đọc khi lệnh có:

```bash
--config mppi_ardupilot/config.yaml
```

File không hot-reload. Sau khi sửa YAML phải dừng và khởi động lại node.

Thứ tự ưu tiên cấu hình:

```text
CLI > file YAML được truyền bằng --config > MPPIConfig trong Python
```

## 2. Tạo một profile riêng để vọc

Không nên thay nhiều lần trực tiếp vào config gốc mà không lưu lịch sử. Tạo
một bản sao có tên nói rõ mục đích:

```bash
cd ~/Projects/ardupilot_gazebo
cp mppi_ardupilot/config.yaml config/experiments/mppi_my_test.yaml
```

Nếu thư mục chưa có:

```bash
mkdir -p config/experiments
```

Sau đó chạy bằng:

```bash
--config config/experiments/mppi_my_test.yaml
```

Tên profile gợi ý:

```text
mppi_slalom_h40_n500_seed7.yaml
mppi_gate_wobs24_seed19.yaml
mppi_corner_vmax10_seed31.yaml
```

Mỗi thí nghiệm chỉ nên thay **một nhóm tham số**. Không đồng thời đổi map,
route, seed, cost, tốc độ và horizon rồi kết luận nguyên nhân.

## 3. Bản đồ tham số MPPI

### 3.1 Prediction model

```yaml
dt: 0.1
tau: 0.5
horizon: 30
```

- `dt`: thời gian giữa hai bước prediction.
- `tau`: hằng số bám velocity của mô hình closed-loop bậc một.
- `horizon * dt`: thời gian nhìn trước. Ví dụ `30 * 0.1 = 3 s`.

Tăng horizon giúp thấy góc cua sớm hơn nhưng tăng compute và tổng cost tích
lũy. Nếu cost chưa được chuẩn hóa theo horizon, thay horizon cũng làm thay đổi
cân bằng tương đối giữa running cost và terminal cost.

Khoảng dừng tối thiểu để tham khảo:

```text
d_stop ≈ v²/(2*a) + v*latency
```

Prediction phải thấy quyết định trước khi UAV đi vào vùng không còn đủ khoảng
dừng.

### 3.2 Sampling

```yaml
samples: 500
lambda: 1.0
seed: 7
noise_xy: 0.8
noise_z: 0.3
noise_yaw: 0.3
```

- `samples`: số rollout mỗi chu kỳ.
- `noise_*`: độ rộng phân bố control perturbation.
- `lambda`: temperature của trọng số MPPI.
- `seed`: giúp tái lập cùng chuỗi sampling.

Theo dõi ESS trong log. Với `N=500` mà ESS chỉ khoảng `1--2`, update gần như
do một vài rollout quyết định. Tăng samples đơn thuần không chắc giải quyết cost
concentration.

### 3.3 Cost

```yaml
w_goal: 1.0
w_terminal: 5.0
w_obstacle: 300.0
w_u: 0.05
w_du: 0.2
w_yaw: 0.2
w_path: 0.0
path_scale_m: 1.0
margin: 4.0
```

- `w_goal`: kéo prediction về active waypoint ở mọi bước.
- `w_terminal`: kéo trạng thái cuối horizon về waypoint.
- `w_obstacle`: phạt proximity tới point cloud.
- `margin`: khoảng cách bắt đầu vùng phạt theo mô hình hiện tại.
- `w_u`: phạt độ lớn velocity command.
- `w_yaw`: ưu tiên yaw theo hướng bay.
- `w_du` hiện phạt `u - v`, **không phải** `u[k] - u[k-1]`; do đó không được
  diễn giải nó là jerk cost thực.

#### 3.3.1 Bám global path theo reference-tracking cost

Paper Minařík et al., *Model Predictive Path Integral Control for Agile
Unmanned Aerial Vehicles* (arXiv:2407.09812, Eq. 16--18), tách riêng cost
input/độ đổi input và một metric tracking theo reference state. Trong code hiện
Project-cost profile giữ cost legacy theo khoảng cách tới polyline:

\[
  J_{path}=w_{path}\sum_{k=0}^{H-1}
    \left(\frac{d(p_k,\mathcal P)}{s_{path}}\right)^2,
\]

trong đó $d(p,\mathcal P)$ là khoảng cách ngắn nhất từ vị trí rollout tới
polyline global path (projection lên từng segment), còn `path_scale_m` là
scale chuẩn hóa theo mét. Đây là **PROJECT DESIGN DECISION/adaptation**, không
phải tuyên bố tái tạo nguyên cost của paper. Riêng
`cost_profile: paper` không còn dùng công thức nearest-polyline này; xem
mục 3.3.2. Obstacle cost vẫn có ưu tiên an
toàn cao hơn; path reference không được coi là collision certificate.

Bật thử bằng YAML:

```yaml
w_path: 1.0
path_scale_m: 1.0
global_path: "0,0,20;8,3,20;13,3,20;18,0,20;30,0,20"
```

Có sẵn profile tối giản `config/experiments/mppi_global_path_reference.yaml`
với `w_path: 0.5`; vẫn phải truyền polyline phù hợp với world đang chạy.

hoặc bằng CLI:

```bash
... scripts/mppi_velocity_avoidance.py \
  --config config/experiments/mppi_my_test.yaml \
  --w-path 1.0 --path-scale-m 1.0 \
  --global-path '0,0,20;8,3,20;13,3,20;18,0,20;30,0,20'
```

`--goal` vẫn là các waypoint nhiệm vụ và quyết định khi chuyển waypoint;
`--global-path` là polyline dày hơn để tối ưu bám. Hãy cung cấp path đã kiểm
tra clearance trong map. Nếu bỏ `--global-path` khi `w_path>0`, node dùng
`--goal` làm fallback và in cảnh báo; fallback này chỉ phù hợp để debug hình
học, không phải path an toàn đã được chứng minh.

Quy trình tune: giữ nguyên seed, horizon, samples và obstacle weights; chạy
`w_path=0`, `0.25`, `0.5`, `1.0`, `2.0`, rồi so sánh `path` trong cost breakdown,
minimum clearance, goal distance và compute time. Nếu UAV cắt góc, tăng `w_path`
hoặc làm path dày hơn; nếu bị kéo vào vật cản hoặc đứng trước khe, giảm
`w_path`, giảm `path_scale_m` chỉ khi hiểu rõ scale, và kiểm tra lại path có
thực sự đi qua vùng an toàn hay không. Không dùng path cost để thay thế
collision cost.

#### 3.3.2 Profile chỉ dùng nhóm cost của paper Minařík

Để kiểm tra ảnh hưởng của các cost project, dùng
`config/experiments/mppi_paper_cost_only.yaml` với `--planner mppi`. Profile
này dùng augmented velocity dynamics và objective chỉ còn các nhóm có trong
paper: input effort với ma trận chéo `R` (ánh xạ vào `[vx,vy,vz,yaw_rate]`),
time-indexed position/velocity reference và collision indicator. Mỗi cycle,
global polyline được chiếu thành progress không lùi, sau đó sample
`p_ref[j]`, `v_ref[j]` theo `reference_speed_m_s`. `w_terminal` project-only
được tắt.

| Thành phần hiện tại | Đối chiếu paper | Profile paper-only |
|---|---|---|
| `w_goal * ||p-goal||` mỗi bước | Paper dùng reference state metric, không phải active-waypoint heuristic | Tắt (`w_goal=0`) |
| `w_terminal * ||p_T-goal||²` | Không có trong Eq. 16--18 | Tắt (`w_terminal=0`) |
| softplus obstacle proximity | Không phải Eq. 22 | Tắt (`w_obstacle=0`) |
| `w_collision * 1{x∈C_obs}` | Paper Eq. 22 | Bật, `w_collision=1e6`; point cloud inflate 1.5 m là adapter project |
| scalar effort `w_u` | Paper Eq. 16 dùng `R` diagonal | Dùng `paper_r_u=[0.01,0.05,0.05,0.10]` |
| input-change `R_Δ` | Paper Eq. 16 dùng `Σ ΔuᵀR_ΔΔu`, `Δu=u[j+1]-u[j]` | Dùng `paper_r_delta_u=[0.05,0.10,0.10,0.30]` trên cả chuỗi rollout |
| `w_du * (u-v)`, `w_yaw` | Không tương ứng trực tiếp Eq. 16–18 | Tắt |
| `w_path` | Position term của Eq. 18 | `400 * ||p[j]-p_ref[j]||²` |
| `w_reference_velocity` | Velocity term của Eq. 18 | `40 * ||v[j]-v_ref[j]||²` |

`R_\Delta` của Eq. 16 được tính từ chuỗi action khả thi sau
low-pass/slew limit trong
terminal callback của `QuadMPPI`; phần breakdown tương ứng có khóa
`input_change`. Đây là cách triển khai phù hợp với chuỗi
`\Delta u_j=u_{j+1}-u_j` của paper, nhưng vị trí callback là chi tiết của thư
viện `pytorch-mppi`. Profile vẫn **không** phải reproduction đầy đủ vì state,
dynamics rigid-body và collision module vẫn khác paper. Có thể tune
`paper_r_delta_u=[0.05,0.10,0.10,0.30]` trong YAML nếu muốn phạt thay đổi
velocity/yaw-rate mạnh hơn hoặc nhẹ hơn.

Chạy offline trước:

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --sim-test --planner mppi \
  --config config/experiments/mppi_paper_cost_only.yaml \
  --goal '16,10,20;30,0,20' \
  --global-path '0,0,20;16,10,20;30,0,20' \
  --collision-radius-m 2.0 --margin 0.5 --seed 7
```

`--margin` trong lệnh smoke-test chỉ phục vụ điều kiện pass của harness; paper
profile không dùng softplus margin. Khi chuyển sang Gazebo, phải thay
`collision-radius-m` bằng bán kính/geometry đã hiệu chuẩn và đánh dấu kết quả
PENDING cho đến khi có log.

Dấu hiệu `w_obstacle` quá lớn:

- UAV dừng trước một khe hình học vẫn có thể đi qua;
- clearance lớn nhưng không chuyển waypoint;
- tăng horizon/samples vẫn không tiến;
- terminal/goal cost không thắng được obstacle cost tích lũy.

Dấu hiệu `w_obstacle` quá nhỏ:

- success tăng nhưng clearance nhỏ hơn margin;
- kết quả thay đổi mạnh theo seed;
- selected trajectory cắt sát endpoint point-cloud.

### 3.4 Control và làm mượt interface

```yaml
vmax: 2.0
vzmax: 1.0
yaw_rate_max: 0.6
command_alpha: 0.45
max_accel_xy: 1.5
max_accel_z: 0.8
max_yaw_accel: 1.2
```

- Giảm `vmax` giúp tăng thời gian phản ứng nhưng không tự phá local minimum.
- Giảm `command_alpha` làm lệnh mượt hơn nhưng tăng trễ.
- `max_accel_*` là slew limit của setpoint gửi ra, không phải acceleration
  constraint nằm trong toàn bộ rollout.

### 3.5 Waypoint và terminal

```yaml
goal: "16,10,20;30,0,20"
wp_radius: 2.5
goal_radius: 0.25
goal_slowdown_radius: 4.0
goal_approach_gain: 0.5
goal_min_speed: 0.10
```

- `wp_radius`: chỉ dựa vào khoảng cách vị trí để đổi waypoint trung gian.
- `goal_radius`: terminal gate cuối cùng.
- `goal_slowdown_radius`: chỉ áp dụng cho goal cuối, không cho waypoint giữa.
- Tăng slowdown radius hoặc giảm approach gain giúp giảm overshoot.

Waypoint trung gian phải nằm trong vùng có clearance rõ ràng. Không đặt
waypoint ngay sát góc hoặc để đoạn thẳng giữa hai waypoint xuyên qua inflated
obstacle rồi kỳ vọng MPPI luôn tìm đúng homotopy.

## 4. Chạy lại insight offline đã đo

### 4.1 Kiểm tra môi trường và unit test

```bash
cd ~/Projects/ardupilot_gazebo

/opt/miniconda3/envs/ardupilot-rviz/bin/python -m unittest \
  discover -s tests -v
```

### 4.2 Chạy toàn bộ profile với một seed

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --seeds 7 \
  --output output/benchmark/my_seed7
```

Scenario mặc định:

```text
multi_obstacle_slalom
narrow_gate
right_angle_corridor
```

Profile mặc định nằm trong dictionary `PROFILES` của
`scripts/benchmark_mppi_tuning.py`.

### 4.3 Tái lập trade-off đã phát hiện

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --profiles tight_passage,safe_tight \
  --seeds 7,19,31 \
  --output output/benchmark/reproduce_tradeoff
```

Insight cần tái lập:

- `tight_passage`: thường tới đích nhưng có seed vi phạm `margin=2 m`;
- `safe_tight`: giữ margin tốt hơn nhưng kẹt khe hẹp;
- ESS vẫn rất thấp so với 500 rollout;
- chỉ tăng horizon/samples/noise không tự giải quyết local minimum.

Kết quả tạo ra:

```text
output/benchmark/reproduce_tradeoff.csv
output/benchmark/reproduce_tradeoff.json
```

Không ghi đè dữ liệu cũ nếu muốn so sánh; đổi tên `--output` cho mỗi run.

### 4.4 Chỉ chạy một scenario/profile

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --scenarios narrow_gate \
  --profiles baseline,tight_passage,safe_tight \
  --seeds 7,19,31 \
  --output output/benchmark/gate_weight_sweep
```

### 4.5 Tạo profile benchmark mới

Mở `scripts/benchmark_mppi_tuning.py`, tìm `PROFILES` và thêm:

```python
"my_profile": {
    "horizon": 35,
    "samples": 500,
    "lambda_": 2.0,
    "noise_xy": 0.7,
    "vmax": 1.0,
    "max_accel_xy": 0.6,
    "w_goal": 1.5,
    "w_terminal": 8.0,
    "w_obstacle": 40.0,
},
```

Sau đó:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --profiles my_profile \
  --seeds 7,19,31 \
  --output output/benchmark/my_profile_v1
```

Đây chỉ là ví dụ cú pháp, không phải bộ weight đã được xác nhận an toàn.

### 4.6 Nhìn lại trade-off trên map 3D Gazebo

Hai profile Gazebo tương ứng đã được chuẩn bị:

```text
config/experiments/mppi_tight_passage_gazebo.yaml
config/experiments/mppi_safe_tight_gazebo.yaml
```

Đây là profile **SITL-only**. `tight_passage` từng vi phạm margin offline;
`safe_tight` chỉ là tên profile, không phải chứng nhận an toàn. Gazebo replay
giữ `hard_brake_m=0.5`, trong khi benchmark point-mass đặt hard brake bằng 0,
do đó hai phép thử không bitwise-identical.

#### Chọn insight dễ nhìn nhất

Dùng map khe hẹp:

```text
worlds/iris_mppi_narrow_gate.sdf
```

Kỳ vọng cần kiểm chứng, không phải kết quả đã đo trong Gazebo:

- `tight_passage`: có xu hướng tiến qua khe nhưng clearance có thể sát margin;
- `safe_tight`: có thể giữ xa hơn nhưng dừng/local-minimum trước khe.

Hai lượt phải dùng cùng world, route, seed và điều kiện ban đầu. Sau lượt A,
phải reset cả Gazebo và SITL rồi mới chạy lượt B; không chạy B từ vị trí cuối A.

#### Terminal 1 — challenge world server

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 -r "$PWD/worlds/iris_mppi_narrow_gate.sdf" -s
```

#### Terminal 2 — Gazebo GUI

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge

gz sim -v1 -g \
  --gui-config "$PWD/config/gazebo_runway_camera.config"
```

#### Terminal 3 — SITL

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

#### Terminal 4 — ROS 2 bridge và RViz

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

./scripts/run_sensor_rviz.sh
```

Trong RViz thêm hai display nếu config chưa có:

```text
Add → By topic → /mppi/predicted_path → Path
Add → By topic → /mppi/sampled_trajectories → MarkerArray
```

Hai topic được bật bởi ba cờ trong lệnh chạy node:

```bash
--rviz-traj-topic /mppi/predicted_path \
--rviz-samples-topic /mppi/sampled_trajectories \
--rviz-top-k 20
```

Ý nghĩa:

| Topic | Nội dung |
|---|---|
| `/mppi/predicted_path` | Nominal trajectory sau lần cập nhật MPPI gần nhất |
| `/mppi/sampled_trajectories` | `K` rollout có total cost thấp nhất trong chu kỳ gần nhất |

Mỗi sample bắt đầu từ state đo hiện tại của UAV. Chiều dài thời gian hiển thị
là `horizon * dt`; với `horizon=30`, `dt=0.1 s` thì mỗi đường dự đoán 3 giây.
Các đường này là prediction của mô hình, không phải cam kết UAV sẽ bay đúng
theo chúng.

Nếu Gazebo/RViz nặng hoặc planner hay `hold-timeout`, giảm phần hiển thị trước:

```bash
--rviz-top-k 5
```

`top-k` chỉ thay số marker publish, không thay số rollout mà optimizer dùng.
Muốn thay số rollout thật phải chỉnh `samples` trong YAML hoặc dùng `--samples`.

Kiểm tra topic nếu RViz không thấy đường:

```bash
ros2 topic list | grep mppi
ros2 topic type /mppi/predicted_path
ros2 topic type /mppi/sampled_trajectories
```

Kết quả mong đợi:

```text
nav_msgs/msg/Path
visualization_msgs/msg/MarkerArray
```

Trong RViz, `Fixed Frame` phải là `odom`. Có thể tăng `Line Width`, đổi màu
nominal path và giảm alpha của sampled trajectories để phân biệt đường được
chọn với đám rollout.

Cách đọc sample fan để debug:

| Quan sát trong RViz | Khả năng cần kiểm tra |
|---|---|
| Không sample nào đi vào khe | Local minimum, obstacle cost quá trội hoặc horizon chưa thấy cửa ra |
| Nhiều sample đi qua nhưng nominal không chọn | Cost breakdown, weight update và ESS |
| Sample xuyên visual obstacle | Point cloud/frame/collision cost không khớp map |
| Fan quá rộng, đổi mạnh mỗi chu kỳ | `noise_xy` lớn hoặc temperature/cost scale chưa cân bằng |
| Các sample gần như trùng nhau | Noise nhỏ hoặc nominal sequence đã hội tụ |
| Chỉ một vài sample tốt hoàn toàn chi phối | ESS thấp; không kết luận robust chỉ từ nominal path đẹp |

Ảnh RViz phải được đọc cùng JSONL. Sample fan đẹp không thay thế các metric
`minimum_clearance_m`, `compute_ms`, `deadline_miss`, `ess` và event
`hold-brake/hold-timeout`.

#### Takeoff bằng controller chuẩn

Trong MAVProxy:

```text
mode guided
arm throttle
takeoff 20
```

Chờ UAV hover ổn định rồi mới mở node MPPI.

#### Lượt A — `tight_passage`

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_tight_passage_gazebo.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '13,0,20;18,-0.2,20;24,0,20' \
  --seed 7 \
  --diag-every 1 \
  --diag-jsonl output/log/gate_tight_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories
```

Khi kết thúc: `Ctrl+C` node, `mode land`, sau đó dừng SITL, Gazebo GUI/server
và khởi động lại bốn terminal trên để trả UAV về `(0,0,0.35)`.

#### Lượt B — `safe_tight`

Sau khi reset đầy đủ và takeoff lại 20 m:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_safe_tight_gazebo.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '13,0,20;18,-0.2,20;24,0,20' \
  --seed 7 \
  --diag-every 1 \
  --diag-jsonl output/log/gate_safe_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories
```

#### Vẽ và so sánh hai log

```bash
cd ~/Projects/ardupilot_gazebo

/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/plot_mppi_experiment.py \
  --run tight=output/log/gate_tight_seed7.jsonl \
  --run safe=output/log/gate_safe_seed7.jsonl \
  --output-dir output/plots/gate_tradeoff_seed7
```

So sánh ít nhất:

- có chuyển qua waypoint sau khe hay không;
- minimum clearance;
- XY trajectory và path length;
- compute mean/p95/worst và deadline miss;
- ESS theo từng vùng trước/trong/sau khe;
- saturation/hard-brake/stale-state;
- final goal distance.

Muốn kiểm tra robustness, lặp toàn bộ cặp A/B với `--seed 19` và `--seed 31`,
đổi cả tên JSONL để không nối dữ liệu của hai run vào cùng file. Không chỉ thay
seed rồi tiếp tục từ state cuối của run trước.

## 5. Đọc metric

| Metric | Câu hỏi trả lời |
|---|---|
| `reached` | Có vào terminal gate trong time budget không? |
| `final_error_m` | Khi dừng còn cách goal bao xa? |
| `min_clearance_m` | Điểm gần obstacle nhất là bao nhiêu? |
| `margin_violated` | Có nhỏ hơn margin cấu hình không? |
| `path_length_m` | Tổng chiều dài quỹ đạo |
| `path_ratio` | Đường thực tế / khoảng cách thẳng start-goal |
| `command_accel_rms_mps2` | Mức thay đổi lệnh điển hình |
| `command_accel_max_mps2` | Spike lệnh lớn nhất |
| `compute_p95_ms` | 95% chu kỳ tính nhanh hơn giá trị này |
| `ess_mean`, `ess_p05` | Mức phân tán trọng số rollout |
| `waypoint_switches` | Planner đã vượt qua bao nhiêu waypoint giữa? |

Ở `hz=10`, period là 100 ms. `compute_p95` phải thấp hơn period với headroom
cho sensor parsing, logging và MAVLink. Kết quả compute trên máy này không được
suy rộng sang edge computer.

Một case không đạt nếu xảy ra ít nhất một trong các điều sau:

```text
reached = false
min_clearance < margin
NaN/Inf
deadline miss liên tiếp
hard-brake hoặc stale-state kéo dài
quỹ đạo phụ thuộc mạnh vào một seed
```

## 6. Tự thiết kế scenario offline

Trong `scripts/benchmark_mppi_tuning.py`, mỗi scenario gồm:

```python
Scenario(
    name="ten_scenario",
    start=(x, y, z),
    waypoints=((x1, y1, z1), (x2, y2, z2)),
    obstacles=point_cloud_numpy,
    max_steps=450,
)
```

Helper hiện có:

```python
_line((x1, y1), (x2, y2), spacing=0.35, z=20.0)
_circle((cx, cy), radius=1.0, count=28, z=20.0)
```

Khi so sánh thuật toán, giữ nguyên:

- start, goal và waypoint;
- obstacle cloud;
- dynamics và safety limits;
- horizon nếu không chủ đích sweep horizon;
- seed;
- time budget và success condition.

## 7. Tự sửa map Gazebo SDF

### 7.1 Hệ tọa độ

Các world challenge dùng world ENU:

```text
+X: East
+Y: North
+Z: Up
pose: x y z roll pitch yaw
```

Chiều dài box trong SDF:

```xml
<size>size_x size_y size_z</size>
```

### 7.2 Box tĩnh

```xml
<model name="my_wall">
  <static>true</static>
  <pose>10 4 12 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.6 8 24</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.6 8 24</size></box></geometry>
    </visual>
  </link>
</model>
```

### 7.3 Cylinder tĩnh

```xml
<model name="my_column">
  <static>true</static>
  <pose>15 -2 12 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><cylinder><radius>1.2</radius><length>24</length></cylinder></geometry>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>1.2</radius><length>24</length></cylinder></geometry>
    </visual>
  </link>
</model>
```

Giữ collision và visual cùng geometry để physics/render/LiDAR không mô tả hai
vật khác nhau. Tên model không được trùng trong cùng world.

### 7.4 Tính pose từ hai đầu tường

Với tường dọc từ `y_min` tới `y_max`:

```text
center_y = (y_min + y_max)/2
length_y = y_max - y_min
```

Ví dụ từ `-8` tới `-2.75`:

```text
center_y = -5.375
length_y = 5.25
```

### 7.5 Validate SDF

```bash
cd ~/Projects/ardupilot_gazebo

SDF_PATH="$PWD/models" gz sdf -k worlds/my_world.sdf
```

Không chạy MPPI nếu world chưa báo `Valid.`

### 7.6 Biến thể địa hình 3D

#### Ramp/dốc bằng box quay pitch

```xml
<model name="ramp">
  <static>true</static>
  <!-- pitch = -0.1745 rad ≈ -10 độ -->
  <pose>12 0 1.0 0 -0.1745 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>12 8 0.4</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>12 8 0.4</size></box></geometry>
    </visual>
  </link>
</model>
```

Một mặt dốc chỉ có ý nghĩa với planner nếu route/goal altitude buộc UAV tương
tác với nó và sensor nhìn được bề mặt. Không đặt ramp xuyên qua ground/UAV.

#### Trần thấp hoặc cầu ngang

```xml
<model name="low_ceiling">
  <static>true</static>
  <pose>15 0 21.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>8 8 1</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8 8 1</size></box></geometry>
    </visual>
  </link>
</model>
```

Ở goal altitude 20 m, đáy box trên nằm tại 21 m. Phải cộng kích thước UAV và
margin theo trục Z trước khi coi khe đứng là khả thi.

#### Obstacle nổi để ép quyết định lên/xuống

Đặt box không chạm đất, ví dụ tâm `z=20`, cao 4 m. Sau đó tạo hai route cạnh
tranh: một route qua trên, một route qua dưới. Đừng cho waypoint biết sẵn lời
giải nếu mục tiêu là kiểm tra khả năng exploration của planner.

LiDAR hiện chỉ có vertical field of view khoảng `±15°`. Ở độ cao lớn, ground
hoặc trần nằm ngoài vùng nhìn có thể không xuất hiện trong point cloud. Trước
khi kết luận planner 3D an toàn, phải kiểm tra point cloud trong RViz và không
coi vùng sensor không thấy là free.

#### Vật cản động

Đổi `<static>true</static>` thành false chưa đủ tạo obstacle chuyển động có kiểm
soát; cần trajectory/actor hoặc system plugin. Planner hiện dùng obstacle cloud
tức thời và chưa dự đoán velocity của obstacle. Vì vậy bài test động phải được
ghi riêng là **reactive avoidance**, không được diễn giải là dynamic-obstacle
prediction.

## 8. Visualize map trước khi bay

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 "$PWD/worlds/iris_mppi_narrow_gate.sdf"
```

Kiểm tra bằng mắt:

- UAV có ở đúng phía trước obstacle không;
- kích thước khe đúng theo thiết kế không;
- obstacle cao qua flight altitude không;
- ground không xuyên vào UAV;
- tên entity không trùng;
- camera nhìn được toàn scenario.

## 9. Quy trình đưa một scenario sang Gazebo/SITL

### Gate A — world và sensor

1. Validate SDF.
2. Mở Gazebo nhưng chưa arm.
3. Kiểm tra `/sensor_suite/lidar/points` và `/iris/odometry`.
4. Xác nhận point cloud trùng visual obstacle.

### Gate B — ArduPilot chuẩn

1. Khởi động SITL bằng `config/mppi_velocity.parm`.
2. Takeoff 20 m bằng GUIDED controller chuẩn.
3. Hover ổn định trước khi chạy planner.
4. Xác nhận ENU position khớp vị trí trong world.

### Gate C — MPPI tốc độ thấp

Ví dụ map khe hẹp:

```bash
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_my_test.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '13,0,20;18,-0.2,20;24,0,20' \
  --margin 2.0 \
  --vmax 1.0 \
  --max-accel-xy 0.6 \
  --diag-every 1 \
  --diag-jsonl output/log/gate_my_test_seed7.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories
```

Không dùng `--no-mav` để đánh giá closed-loop flight: khi không gửi MAVLink,
UAV không đi theo prediction nên đó chỉ là kiểm tra node/topic/rollout.

### Abort condition

Chuyển `mode brake` hoặc `mode land` nếu:

- trajectory dự đoán xuyên obstacle;
- clearance tiến gần hard-brake threshold;
- odometry/LiDAR stale;
- lệnh có NaN/Inf hoặc saturation kéo dài;
- planner deadline miss liên tiếp;
- frame/sign có biểu hiện ngược hướng.

## 10. Route cho ba challenge world

```text
Slalom:
5.5,-2.2,20;11.5,2.2,20;17.5,-2.2,20;23.5,2.2,20;30,0,20

Khe hẹp:
13,0,20;18,-0.2,20;24,0,20

Cua chữ L:
5.5,0,20;10,0,20;10,5,20;10,13,20
```

Hướng dẫn mở từng world nằm ở
[`run_mppi_challenge_worlds_vi.md`](run_mppi_challenge_worlds_vi.md).

## 11. Thứ tự tune khuyến nghị

```text
1. Frame và sensor đúng
2. Hình học scenario khả thi
3. dt/tau khớp response đo được
4. Horizon đủ thấy quyết định
5. Cân bằng goal/terminal/obstacle
6. Kiểm tra ESS, rồi tune lambda/noise/samples
7. Tune vmax và slew-rate
8. Tune waypoint transition/arrival
9. Sweep nhiều seed offline
10. Lặp lại Gazebo nhiều run và đọc log
11. Sau cùng mới thử edge/hardware
```

Chỉ thay một nhóm mỗi lần và giữ một baseline không đổi.

## 12. Mẫu nhật ký thí nghiệm

Sao chép block này cho mỗi lần chạy:

```markdown
### EXP-YYYYMMDD-01

- Status: OFFLINE / GAZEBO / EDGE / HARDWARE
- Git commit:
- World/scenario:
- Config file:
- Goal/route:
- Seeds:
- Thay đổi so với baseline:
- Giả thuyết:
- Success condition:
- Abort condition:
- Log/output:

Kết quả:
- Success rate:
- Final error:
- Minimum clearance:
- Path length/ratio:
- Command accel RMS/max:
- Compute mean/p95/worst:
- Deadline misses:
- ESS mean/p05:

Kết luận:
- Giả thuyết được ủng hộ hay bác bỏ?
- Trade-off mới:
- Thay đổi tiếp theo duy nhất:
- Điều gì vẫn chưa được xác minh?
```

## 13. Các lỗi diễn giải cần tránh

- Offline pass không đồng nghĩa Gazebo pass.
- Gazebo pass một lần không đồng nghĩa robust.
- `margin` trong cost mềm không phải hard collision guarantee.
- Nhiều samples hơn không tự động tốt hơn nếu ESS collapse.
- Đường mượt trong RViz không chứng minh frame hoặc dynamics đúng.
- Planner dừng xa obstacle không nhất thiết an toàn hơn; có thể là local minimum.
- Tới goal không đủ nếu clearance/deadline/control smoothness thất bại.
- Không tune riêng mỗi thuật toán bằng điều kiện thuận lợi khác nhau khi làm
  MPPI vs PA-MPPI ablation.

# Quickstart: bãi container, lấy đà 60 m, cruise 5 và 10 m/s

[Checkpoint mentor và Experiment 7A — 1.600 replay](MPPI_MENTOR_CHECKPOINT_AND_NEXT_PHASE_VI.md).
[Báo cáo tổng hợp để trao đổi mentor — 16/09/2026](MENTOR_REVIEW_YARD_MPPI_VI.md).
[Kiểm chứng safety mới nhất](MPPI_MENTOR_SAFETY_PHASE_VI.md) và
[nhánh feasible-selection](MPPI_FEASIBLE_SELECTION_VI.md) đã đưa cùng một
điều kiện an toàn vào cả bước chọn sample lẫn gate cuối. Profile 80 sample đã
tới đích 2/2 ở yêu cầu 10 m/s và hiện là cấu hình tốt nhất để chạy thử.

Quy trình 5 terminal như `RUN_3_MPPI_MAPS_QUICKSTART_VI.md`, dành riêng cho
world `iris_mppi_yard_runup60.sdf`. Chạy từng tốc độ trong một phiên mới.
Không chạy đồng thời với harness/sweep hoặc Gazebo/SITL khác.

Nếu MAVProxy báo đã arm/takeoff nhưng UAV trong GUI vẫn đứng yên, kiểm tra:

```bash
ps -axo pid,command | grep '[g]z sim -v2'
lsof -nP -iUDP:9002
```

Phải chỉ có một Gazebo server và tiến trình đó phải giữ UDP `9002`. Hai server
có thể làm SITL điều khiển world cũ trong khi GUI đang hiển thị world mới.
Dừng cả Gazebo server và SITL, sau đó khởi động lại Terminal 1 trước Terminal 3.

Dùng `config/experiments/mppi_yard_progress_feasible80.yaml`: **retiming tắt**,
objective path-progress, prior map SDF, mô hình đáp ứng gia tốc/phanh, proposal
giảm tốc–rẽ chủ động và kiểm tra stopping trajectory. Sample không an toàn bị
loại trước khi chuẩn hóa trọng số; gate cuối dùng đúng cùng điều kiện an toàn.
80 sample giữ thời gian tính gần ngân sách 10 Hz tốt hơn bản 160/350 sample.

Hai lượt đặt cả cruise request và `vmax` thành 5 hoặc 10 m/s; đây là tốc độ ngang,
**cao độ bay vẫn 5 m**. UAV cất cánh rồi tăng tốc từ hover; reference 10 m/s
không có nghĩa vận tốc thực lập tức bằng 10 m/s. MPPI và các lớp kiểm tra/
giới hạn điều khiển vẫn có thể làm UAV giảm tốc hoặc giữ trước vật cản.

Cấu hình gốc đã chạy headless ở yêu cầu 10 m/s, seed 7 và 17: **2/2 tới đích
và LAND/disarm**, peak **8.98–9.19 m/s**, không có optimizer timeout hoặc
emergency brake guard. Vẫn còn 11–21 chu kỳ gửi zero khi toàn bộ sample không
hợp lệ (`N_safe=0`), nên chưa thể khẳng định bay hoàn toàn mượt hoặc giữ ổn
định 10 m/s.

Profile GUI đã được chạy xác nhận ngày 16/09 với seed 7: cả 5 và 10 m/s đều
tới đích; peak lần lượt **4.98** và **8.68 m/s**. Phép thử này không bật RViz.

Proposal giảm tốc–rẽ chủ động và khoảng đệm cost đã được tích hợp trong profile
mặc định này; không cần đổi sang profile proactive cũ.

**Kiểm tra ngày 16/09:** [thử cost quãng dừng và horizon dài hơn](MPPI_STOPPING_COST_VI.md)
đã giảm rejection nhưng chưa loại brake; chưa thay cấu hình mặc định bên dưới.

**Nhánh objective mới:** [hình học đường + reward tiến độ](MPPI_PATH_PROGRESS_VI.md)
đã bỏ cost đuổi reference 10 m/s theo thời gian. Có lệnh thử riêng trong tài liệu;
đã giữ gần 10 m/s trên đường thẳng khoảng 22 s ở hai seed. Các lệnh mặc định
bên dưới dùng nhánh feasible-selection mới nhất.

## Chạy tự động bằng harness (có Gazebo 3D)

Chạy block này độc lập, không mở quy trình 5 terminal bên dưới cùng lúc.
Harness tự chạy lần lượt 5 rồi 10 m/s, khởi động phiên mới, cất cánh, bay,
LAND/disarm và lưu ground truth/diagnostics cho mỗi lượt.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/build_yard_runup60.py
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --gui \
  --speeds 5 10 --seeds 7 \
  --config config/experiments/mppi_yard_progress_feasible80_gui.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 90 \
  --output "output/benchmark/yard_feasible80_gui_$(date +%Y%m%d_%H%M%S)"
```

Profile `_gui` chỉ tăng deadline tính toán từ 90 lên 150 ms để bù tải render;
objective và safety predicate không đổi. Khi bỏ `--gui` để chạy headless, dùng
lại `mppi_yard_progress_feasible80.yaml`. Muốn chạy riêng một tốc độ, dùng
`--speeds 5` hoặc `--speeds 10`. Lệnh mặc định dùng seed 7 để chạy nhanh hai
tốc độ; đổi thành `--seeds 7 17` nếu muốn lặp lại mỗi tốc độ bằng hai seed.

## Chuẩn bị

Dùng môi trường Gazebo, ArduPilot và `ardupilot-rviz` đã cài cho quickstart
ba map. Không cần build lại nếu các bài trước đang chạy được.

```bash
cd ~/Projects/ardupilot_gazebo
mkdir -p output/log
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/build_yard_runup60.py
```

Cấu hình đọc prior map từ `worlds/iris_mppi_yard_runup60.sdf`, vì vậy chạy
planner từ thư mục repo như lệnh bên dưới.

Đường giữ cố định: `(0,0,5) → (60,0,5) → (60.5,-4,5) → (73,-4,5)
→ (77.1,-1.9,5)`. Đỉnh cua đầu cách điểm xuất phát 60 m; reference bo bắt
đầu trước đỉnh. Không dùng A* tự tính lại trong lượt GUI này.

## Terminal 1 — Gazebo server

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_yard_5_10
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"
gz sim -v2 -r "$PWD/worlds/iris_mppi_yard_runup60.sdf" -s
```

## Terminal 2 — Gazebo GUI 3D

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_yard_5_10
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"
gz sim -v1 -g --gui-config "$PWD/config/gazebo_runway_camera.config"
```

GUI nối vào server Terminal 1. Camera cũ có thể không bao trọn đường 60 m;
chọn model `iris` và điều chỉnh góc nhìn/zoom để theo dõi UAV.

## Terminal 3 — ArduPilot SITL + MAVProxy

```bash
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"
cd ~/Projects/ardupilot
MAP_SERVICE=MicrosoftSat python3 Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f JSON -N -w \
  -A "--serial1=tcp:2" \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/experiments/mppi_yard_high_accel.parm"
```

Trong MAVProxy kiểm tra:

```text
param show WP_SPD
param show WP_ACC
param show OA_TYPE
param show AVOID_ENABLE
param show GUID_OPTIONS
```

Kỳ vọng `WP_SPD=10`, `WP_ACC=3`, `OA_TYPE=0`, `AVOID_ENABLE=0`,
`GUID_OPTIONS=0`. Giữ nguyên cả khi thử cruise 5 m/s để so sánh cùng setting.

## Terminal 4 — ROS 2 bridge + RViz (tùy chọn)

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_yard_5_10
export ROS_DOMAIN_ID=45
./scripts/run_sensor_rviz.sh
```

Sensor/odometry xuất hiện trước; trajectory MPPI xuất hiện khi chạy Terminal 5.

## Cất cánh — nhập trong MAVProxy Terminal 3

Đợi EKF/pre-arm sẵn sàng rồi nhập:

```text
mode guided
arm throttle
takeoff 5
```

Chờ hover ổn định gần `(0,0,5)` trước khi chạy planner. Nếu arming bị từ chối,
đọc lỗi và chờ sensor sẵn sàng. Không dùng `takeoff 20` của quickstart ba map.

## Terminal 5 — chọn 5 hoặc 10 m/s

Chọn **một** block dưới đây, sau đó chạy block planner chung trong cùng terminal.

### Lượt 5 m/s

```bash
YARD_SPEED=5
```

### Lượt 10 m/s

```bash
YARD_SPEED=10
```

### Planner chung — giữ nguyên cấu hình

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_yard_5_10
export ROS_DOMAIN_ID=45
YARD_SEED=7
YARD_RUN_TAG=$(date +%Y%m%d_%H%M%S)
: "${YARD_SPEED:?Chon YARD_SPEED=5 hoac YARD_SPEED=10 truoc}"
mkdir -p output/log

MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config config/experiments/mppi_yard_progress_feasible80_gui.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '77.1,-1.9,5' \
  --global-path '0,0,5;60,0,5;60.5,-4,5;73,-4,5;77.1,-1.9,5' \
  --reference-speed-m-s "$YARD_SPEED" \
  --vmax "$YARD_SPEED" \
  --seed "$YARD_SEED" \
  --diag-every 20 \
  --diag-jsonl "output/log/yard_runup60_feasible80_gui_v${YARD_SPEED}_seed${YARD_SEED}_${YARD_RUN_TAG}.jsonl" \
  --exit-on-goal
```

Không bật RViz trong lượt đo/demo tốc độ cao. Bridge camera/depth, RViz và việc
publish sampled trajectories đã làm compute tăng từ khoảng 84 lên 143 ms và
gây 300 timeout trong 413 cycle ở phép thử ngày 16/09. Terminal 4 chỉ dùng cho
lượt quan sát riêng; khi đó kết quả thời gian chạy không còn so trực tiếp với
harness một lệnh.
Để thử seed thứ hai, đổi `YARD_SEED=17` và chạy từ một phiên mới.

## Kết thúc và đổi tốc độ

Planner báo tới đích rồi thoát; **lệnh planner này không tự LAND**.
Trong MAVProxy Terminal 3 nhập:

```text
mode land
```

Đợi xác nhận UAV đã hạ cánh và disarm. Sau đó dừng các terminal bằng Ctrl+C,
khởi động lại Terminal 1–4, cất cánh lại và chọn tốc độ còn lại ở Terminal 5.
Phải xuất phát lại gần `(0,0,5)`, không chạy lượt 10 ngay từ đích của lượt 5.

Nếu cần dừng sớm: dừng planner ở Terminal 5 rồi nhập `mode land` trong
MAVProxy. Không tiếp tục planner đồng thời với thao tác LAND thủ công.

Đây là quy trình GUI thủ công: giữ brake, feasible-selection, gate quỹ đạo cuối
và recovery proposal của planner nhưng không có guard
abort footprint/envelope và cleanup LAND tự động của harness. Log JSONL
chứa chẩn đoán planner; không thay thế bộ ground truth/summary của sweep.
Lệnh trong tài liệu đã kiểm tra cú pháp; chưa chạy xác nhận GUI/RViz trực tiếp.

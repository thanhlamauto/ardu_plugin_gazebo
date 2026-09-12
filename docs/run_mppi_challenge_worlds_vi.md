# Visualize các challenge map MPPI trong Gazebo

Muốn copy/paste nhanh toàn bộ 5 terminal cho cả ba map, xem
[`RUN_3_MPPI_MAPS_QUICKSTART_VI.md`](RUN_3_MPPI_MAPS_QUICKSTART_VI.md).

Ba world này là bản 3D của các scenario trong
[`MPPI_TUNING_EXPERIMENTS.md`](MPPI_TUNING_EXPERIMENTS.md):

Muốn tự sửa map, tạo profile và chạy lại sweep nhiều seed, xem
[`MPPI_TUNING_PLAYGROUND_VI.md`](MPPI_TUNING_PLAYGROUND_VI.md).

| Map | File SDF | Route ENU ở cao độ 20 m |
|---|---|---|
| Slalom nhiều obstacle | `worlds/iris_mppi_slalom.sdf` | `5.5,-2.2,20;11.5,2.2,20;17.5,-2.2,20;23.5,2.2,20;30,0,20` |
| Khe hẹp | `worlds/iris_mppi_narrow_gate.sdf` | `13,0,20;18,-0.2,20;24,0,20` |
| Hành lang cua 90 độ | `worlds/iris_mppi_right_angle.sdf` | `5.5,0,20;10,0,20;10,5,20;10,13,20` |

Tất cả obstacle cao 24 m nên LiDAR vẫn nhìn thấy ở cao độ bay 20 m. Iris sinh
tại `(0,0,0.35)`; route dùng world ENU tương đối cùng origin đó. World chỉ dùng
geometry local, không cần tải Fuel model.

## Chỉ mở để xem 3D

Phải dừng Gazebo server/GUI cũ trước vì mỗi lần chỉ chạy một world cho cùng
partition. Chọn đúng một file:

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v2 "$PWD/worlds/iris_mppi_slalom.sdf"
```

Thay file cuối bằng một trong hai file còn lại để xem map khác:

```text
worlds/iris_mppi_narrow_gate.sdf
worlds/iris_mppi_right_angle.sdf
```

Lệnh trên mở cả server và GUI. Có thể orbit/pan camera bằng chuột; nhấn `F` sau
khi chọn một entity trong Entity Tree để focus.

## Chạy tách server và GUI như demo hiện tại

Terminal server:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

export MPPI_WORLD=iris_mppi_slalom.sdf
gz sim -v2 -r "$PWD/worlds/$MPPI_WORLD" -s
```

Terminal GUI:

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge

gz sim -v1 -g \
  --gui-config "$PWD/config/gazebo_runway_camera.config"
```

ROS bridge/RViz và node MPPI cũng phải dùng đúng partition:

```bash
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45
```

SITL, takeoff và ROS bridge giữ nguyên theo
[`run_mppi_demo_vi.md`](run_mppi_demo_vi.md). Khởi động lại SITL với `-w` sau
mỗi lần đổi world để reset trạng thái UAV.

## Goal tương ứng khi chạy node

Ví dụ cho slalom:

```bash
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi \
  --config mppi_ardupilot/config.yaml \
  --mav tcp:127.0.0.1:5762 \
  --goal '5.5,-2.2,20;11.5,2.2,20;17.5,-2.2,20;23.5,2.2,20;30,0,20' \
  --margin 2.0 \
  --vmax 1.0 \
  --max-accel-xy 0.6 \
  --diag-every 1 \
  --diag-jsonl output/log/mppi_slalom_gazebo_01.jsonl \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories
```

Thay chuỗi `--goal` theo bảng đầu tài liệu cho hai map còn lại.

## Cảnh báo về cấu hình

`margin=2 m` được dùng để khớp benchmark hình học; nó không phải margin đã xác
nhận an toàn cho UAV thật. Kết quả offline cho thấy cấu hình weight mặc định có
thể dừng trước khe, trong khi giảm mạnh `w_obstacle` có thể đi qua nhưng vi phạm
margin. Vì vậy:

1. chạy chậm `vmax=1.0`;
2. quan sát predicted rollouts trước;
3. giữ sẵn lệnh `mode brake` hoặc `mode land` trong MAVProxy;
4. không dùng các map/config thử nghiệm này trên hardware;
5. không ghi nhận pass cho đến khi file JSONL và log SITL chứng minh được.

## Kiểm tra SDF

```bash
cd ~/Projects/ardupilot_gazebo

for f in worlds/iris_mppi_slalom.sdf \
         worlds/iris_mppi_narrow_gate.sdf \
         worlds/iris_mppi_right_angle.sdf; do
  SDF_PATH="$PWD/models" gz sdf -k "$f"
done
```

Kết quả mong đợi cho từng file: `Valid.` Các warning `gz_frame_id` đến từ model
sensor hiện có; chúng không phải lỗi của geometry challenge.

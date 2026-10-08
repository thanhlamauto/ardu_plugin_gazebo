# Baseline monocular trong Gazebo

**Tài liệu lịch sử của vòng shadow perception đầu tiên.** Cấu hình LiDAR được
nhắc bên dưới chỉ thuộc baseline đó. Model và runner không LiDAR hiện tại nằm
trong `MONOCULAR_ONLY_VI.md`; nghiên cứu và kết quả mới nằm trong
`MONOCULAR_RESEARCH_VI.md`.

Đã tích hợp theo survey `uav_monocular_depth.pdf`: RGB → Depth Anything V2
Metric Outdoor Small → back-projection → cloud FLU → ROS PointCloud2 tại
`/perception/obstacles_camera`. MPPI tiếp tục dùng LiDAR tại
`/perception/obstacles`. Depth Gazebo chỉ chấm điểm, không scale/calibrate prediction.

## Kết quả chạy thật, 26/09/2026

World: `iris_sensor_arena.sdf`, UAV đứng yên ở mặt đất, camera 640×360,
80° HFOV. Model revision `2fd93bd764b15eea94dcf7763bba7ddc25007d0f`,
PyTorch MPS trên máy Mac hiện tại. Đây là smoke test perception, không phải
benchmark bay, không phải xác nhận model dùng được trên UAV.

| Metric | Run 100 frame |
|---|---:|
| Cloud phát thành công | 100/100 |
| Điểm/cloud (stride 8, axial depth 0.2–25 m) | 3600 |
| Inference p50 / p95 / p99 | 64.60 / 68.96 / 73.73 ms |
| Nhận RGB → gọi publish p50 / p95 / p99 | 71.03 / 76.79 / 84.32 ms |
| AbsRel | 3.2594 |
| RMSE | 6.5119 m |
| δ1 | 0.01653 (1.65%) |

GT là camera depth đồng trục 320×180. Prediction được resize về GT để chấm
điểm, không fit scale; bỏ pixel GT không hữu hạn hoặc ngoài 0.2–25 m.
RGB/GT ghép bằng acquisition timestamp, lệch tối đa 60 ms. Camera đứng yên,
100 ảnh gần như giống nhau: kết quả chỉ cho một cảnh, không phải 100 scene độc lập.
Run 20 frame đầu có cold start khoảng 2.2 s; một cloud trễ bị loại.
Run 100 frame gồm các frame đầu của lần chạy đó; không loại warmup khỏi bảng.
Độ trễ host ở đây bắt đầu từ callback RGB, chưa bao gồm capture/transport trước
callback hay ROS/planner sau publish. `simulation_age_s` ghi riêng bằng `/clock`.

Ảnh prediction nhìn có cấu trúc nhưng scale sai rõ rệt: depth median khoảng
9.22 m trong khi GT median khoảng 2.29 m. Chưa xác định được toàn bộ nguyên nhân;
scene thiếu texture/domain khác training là một giả thuyết cần kiểm chứng.
**Baseline chưa đạt R1; không bật điều khiển monocular.** R3 reliable range,
R4 coverage/stability, thin-obstacle recall và quyết định safety chưa được chứng minh.
Không dùng việc có 3600 điểm làm chứng cứ vùng chưa quan sát là free.

Evidence:

- `results/monocular_baseline_100/summary.json`, `frames.jsonl`
- `results/monocular_baseline_100/comparison.png`, RGB, depth GT/prediction và cloud `.npy`
- `results/monocular_baseline/ros_cloud_smoke.json`: cloud nhận thực tế qua ROS bridge
- `results/monocular_baseline/`: log Gazebo, bridge và cold-start run

## Chạy lại trên máy hiện tại

Môi trường inference tách riêng vì PyTorch trong conda `ardupilot-rviz` bị
xung đột OpenMP. Môi trường đã thử tại `/tmp/uav-monocular-env`; để giữ lâu dài,
đổi đường dẫn này thành đường dẫn venv tùy chọn khi tạo và khi chạy.

```bash
cd ~/Projects/ardupilot_gazebo
uv venv --python /opt/miniconda3/envs/ardupilot-rviz/bin/python /tmp/uav-monocular-env
uv pip install --python /tmp/uav-monocular-env/bin/python -r config/monocular_requirements.txt
# Dùng Python bindings của Gazebo Harmonic đang cài trên máy; chỉ tạo nếu chưa có.
ln -s /opt/miniconda3/envs/ardupilot-rviz/lib/python3.12/site-packages/gz \
  /tmp/uav-monocular-env/lib/python3.12/site-packages/gz
```

Terminal 1:

```bash
export GZ_PARTITION=monocular_baseline
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"
gz sim -s -r -v 3 worlds/iris_sensor_arena.sdf
```

Terminal 2 (output mới để giữ evidence):

```bash
export GZ_PARTITION=monocular_baseline
/tmp/uav-monocular-env/bin/python scripts/monocular_depth_gz.py \
  --device mps --gt-topic /sensor_suite/depth --frames 100 \
  --output-dir results/monocular_repeat
```

Terminal 3, tùy chọn để xem cloud trong ROS/RViz:

```bash
export GZ_PARTITION=monocular_baseline
/opt/miniconda3/envs/ardupilot-rviz/lib/ros_gz_bridge/parameter_bridge \
  '/perception/obstacles_camera@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
```

Cloud có acquisition stamp gốc; frame FLU `sensor_suite_link`, gồm offset camera
(+0.09,0,0) m. TF hiện có đưa cloud sang odom. Camera K suy từ HFOV của model,
fx=fy; nếu đổi sensor, cập nhật calibration. Không thay frame_id optical mà
không đổi phép chiếu. Script chỉ publish nếu simulation age trong [0,1] s,
không tích queue inference. GT option có thể bỏ hoàn toàn khi chạy RGB-only.

Trong workspace ROS đã build/source, chạy launch shadow song song với
`sim.launch.xml` và cùng GZ_PARTITION:

```bash
ros2 launch uav_navigation_bringup monocular_shadow.launch.xml \
  python_executable:=/tmp/uav-monocular-env/bin/python device:=mps
```

Launch mới cần build lại package bringup. Launch XML và luồng bridge đã được
kiểm tra riêng; chưa chạy full ROS launch + SITL trong thử nghiệm này.

## Các bước để đủ điều kiện đóng vòng

So sánh thêm metric backends trên cảnh ở cao độ bay, turn, gate và yard; giữ
intrinsics/GT alignment, đo obstacle-distance theo range và cold start. Sau khi
R1 đạt, shadow cùng chuyến bay LiDAR để so nearest obstacle/safety decisions.
Trước khi swap planner input cần local map có observation coverage hoặc guard
trajectory theo FOV, stopping range và latency; planner hiện tại không hiểu
unknown/free của camera. Chỉ remap cloud chưa giải quyết được R4. Sau đó mới
đo collision, clearance, success ở tốc độ 5/10 m/s.

Model source: https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf

Validation: 5 unit tests cho frame axes, range/invalid filtering, metric scale,
padded RGB rows và stamp/cloud serialization. Smoke test Gazebo → inference →
cloud → ROS đã nhận 3 messages hợp lệ.

Thử nghiệm bỏ LiDAR hoàn toàn và runner closed-loop được bổ sung tại
[`MONOCULAR_ONLY_VI.md`](MONOCULAR_ONLY_VI.md).

# Demo cho mentor trước khi có Jetson

**Chạy trực tiếp trên chiếc Mac hiện tại:** từ gốc repo, mở Terminal và chạy:

```bash
bash scripts/demo_cpp_depth_mac.sh
```

Cửa sổ phát lại 16 khung RGB đã ghi trong Gazebo, suy luận từng khung bằng
**C++/ONNX đang chạy ngay trên Mac**, hiện depth màu, sai lệch với LiDAR mô phỏng,
độ trễ và CPU. Nhấn **Space** để tạm dừng/tiếp tục; đóng cửa sổ để dừng. Nếu
muốn kiểm tra không mở GUI: `bash scripts/demo_cpp_depth_mac.sh --headless --frames 3`.
Máy này đã có model ONNX và executable C++ nên không cần Docker hay Jetson để
chạy demo. Python chỉ phát lại RGB, vẽ cửa sổ và đọc LiDAR để chấm; suy luận
depth do executable C++ `depth_anything_onnx_stream` thực hiện. Nếu cần build
lại sau khi sửa C++:

```bash
source /opt/miniconda3/envs/ardupilot-rviz/setup.bash
cmake -S uav_navigation_ros -B build/uav_navigation_ros_cpp_migration
cmake --build build/uav_navigation_ros_cpp_migration --target depth_anything_onnx_stream -j 2
```

**Mục tiêu 3–5 phút:** cho thấy RGB → depth bằng C++ trên ảnh Gazebo,
đối chiếu LiDAR chỉ **sau** suy luận, rồi trình bày độ trễ CPU và giới hạn hiện
tại. Đây là demo phát lại perception, **chưa phải** bay tránh vật cản closed-loop.

1. Mở [ảnh C++ mẫu](mentor_depth_evidence/cpp/demo_frame_000220.png). Ảnh bên
   trái là RGB từ Gazebo; bên phải là depth từ `depth_anything_onnx_probe`
   (Depth Anything V2 Metric Outdoor Small, ONNX, C++/OpenCV DNN). Ở vùng giữa
   ảnh, C++ cho 4,44 m; LiDAR mô phỏng cho 5,06 m; sai lệch −0,62 m. LiDAR
   **không** là đầu vào của model.
2. Nếu cần xem thay đổi theo thời gian, phát [video 19 giây](../artifacts/depth_demo.mp4)
   và [đồ thị đánh giá](mentor_depth_evidence/comparison.png). Nói rõ video và
   MAE 1,445 m là từ benchmark **Python trước đây** trên cùng model/scene; video
   không chứng minh pipeline C++ bay closed-loop. [Báo cáo gốc](MONOCULAR_MENTOR_DEPTH_BENCHMARK_VI.md).
3. Cho xem phép đo C++ trên Mac M2: 640×360, một luồng CPU, p50 790,6 ms,
   p95 804,3 ms, khoảng 1,27 frame/s, gần 100% một lõi
   ([log 20 frame](mentor_depth_evidence/cpp/cpu_reference_20261006.json)).
   Camera mô phỏng 10 Hz có chu kỳ 100 ms; CPU hiện tại không theo kịp.
   Số này **không** phải hiệu năng Jetson Nano hay độ trễ toàn pipeline ROS.
4. Nếu mentor hỏi Docker: [image ROS 2 Humble ARM64](MENTOR_DOCKER_HUMBLE_VI.md)
   đã [build và kiểm tra executable trong CI](https://github.com/thanhlamauto/ardu_plugin_gazebo/actions/runs/37891051497);
   chưa chạy inference trong container hoặc đo trên Jetson. Cần model ONNX
   gắn ngoài image, và máy demo có Docker.

## Chạy lại một frame bằng C++

Từ gốc repo, sau khi [xuất ONNX model](MENTOR_CPP_QUICKSTART_VI.md), đặt:

```bash
export UAV_DEPTH_ONNX="$PWD/results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx"
```

Trên Mac đã build C++ native (ROS Jazzy), dùng probe hiện có:

```bash
build/uav_navigation_ros_cpp_migration/depth_anything_onnx_probe \
  "$UAV_DEPTH_ONNX" docs/mentor_depth_evidence/cpp/sample_rgb_000220.rgb \
  /tmp/uav_depth_000220.f32 20
```

Trên máy có Docker, chạy cùng mẫu bằng image Humble (image cần build/nạp
trước theo [hướng dẫn Docker](MENTOR_DOCKER_HUMBLE_VI.md)):

```bash
docker run --rm \
  -v "$UAV_DEPTH_ONNX:/models/depth.onnx:ro" \
  -v "$PWD/docs/mentor_depth_evidence/cpp:/demo:ro" \
  -v "$PWD/artifacts:/out" \
  uav-monocular:humble \
  ros2 run uav_navigation_ros depth_anything_onnx_probe \
    /models/depth.onnx /demo/sample_rgb_000220.rgb /out/uav_depth_000220.f32 20
```

Lệnh trên in latency/FPS/CPU và lưu depth trong `artifacts/`. Dùng
`scripts/render_cpp_depth_demo.py` để vẽ; Python ở bước này chỉ dùng để vẽ,
không suy luận. Nếu chạy probe native ở trên, thay đường dẫn `.f32` bằng
`/tmp/uav_depth_000220.f32`:

```bash
python3 scripts/render_cpp_depth_demo.py \
  docs/mentor_depth_evidence/cpp/sample_rgb_000220.png \
  artifacts/uav_depth_000220.f32 /tmp/uav_depth_demo.png \
  --lidar-center-m 5.061122417449951
```

Script vẽ cần `numpy`, `Pillow`, `matplotlib`. Nếu không có sẵn, chỉ cần mở
[ảnh đã chuẩn bị](mentor_depth_evidence/cpp/demo_frame_000220.png).

**Câu nói ngắn khi demo:** “Em đang dùng Depth Anything V2 Metric Outdoor
Small từ một camera RGB. Đây là kết quả C++ trên ảnh Gazebo; LiDAR chỉ để chấm
sai số. Model C++ cho vật giữa ảnh 4,44 m so với LiDAR 5,06 m. Trên Mac chạy
CPU khoảng 0,8 giây một ảnh, nên chưa đủ 10 Hz. Image Humble ARM64 đã build,
nhưng em cần đo lại trên Jetson trước khi nói đến tránh vật khi bay.”

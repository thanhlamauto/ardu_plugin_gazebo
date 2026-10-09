# Demo cho mentor trước khi có Jetson

**Chạy video RGB + depth MPS trực tiếp trên chiếc Mac hiện tại:** từ gốc repo,
mở Terminal và chạy:

```bash
bash scripts/demo_mps_depth_mac.sh
```

Cửa sổ phát lại **131 khung RGB ở 10 Hz**, hiện FPS RGB, FPS depth xử lý thực
đo và số frame bị bỏ. Backend này dùng **PyTorch trên GPU Apple MPS**, cùng
checkpoint Depth Anything V2 Metric Outdoor Small; Python thực hiện suy luận.
LiDAR chỉ dùng để đối chiếu offline. Nhấn **Space** để tạm dừng/tiếp tục; đóng
cửa sổ để dừng. Kiểm tra không mở GUI:
`bash scripts/demo_mps_depth_mac.sh --headless --frames 3`.

Muốn thử phát nhanh hơn 10 Hz: `bash scripts/demo_mps_depth_mac.sh --camera-hz 12`.
Đây là phát lại nhanh hơn tốc độ 10 Hz của dữ liệu camera gốc.
Ở lượt thử trên Mac này, RGB và depth thường quanh 12 FPS; một lần suy luận
vọt lên ~145 ms khiến 1 frame RGB bị bỏ. Số FPS trên cửa sổ là thông lượng
đo thực, không phải `1000 / inference_ms` của một ảnh riêng lẻ.

Để so với **đường triển khai C++**, chạy `bash scripts/demo_cpp_depth_mac.sh`.
Lệnh này dùng OpenCV DNN **CPU**, không dùng MPS; cửa sổ hiện cùng các chỉ số
FPS và frame bị bỏ. Mặc định 4 luồng CPU; thêm `--threads 1` để đo lại mốc cũ
một luồng. Máy này đã có model ONNX và executable C++, không cần Docker hay
Jetson để chạy. Nếu cần build lại sau khi sửa C++:

```bash
source /opt/miniconda3/envs/ardupilot-rviz/setup.bash
cmake -S uav_navigation_ros -B build/uav_navigation_ros_cpp_migration
cmake --build build/uav_navigation_ros_cpp_migration --target depth_anything_onnx_stream -j 2
```

**Mục tiêu 3–5 phút:** cho thấy video RGB → depth trên ảnh Gazebo,
đối chiếu LiDAR chỉ **sau** suy luận, rồi trình bày FPS/độ trễ và giới hạn hiện
tại. Đây là demo phát lại perception, **chưa phải** bay tránh vật cản closed-loop.

Trên đoạn thử trực tiếp tại máy này, MPS sau warmup xử lý khoảng 67–70 ms/ảnh;
cả RGB lẫn depth hiển thị khoảng **10 FPS** và không bỏ frame. Cùng video đó,
C++ OpenCV CPU 4 luồng xử lý khoảng **3,5 FPS** khi RGB phát 10 FPS, nên phải
bỏ ảnh cũ để giữ đầu ra mới nhất. MPS là GPU Apple qua PyTorch/Python, **không
phải backend của runtime C++ và không có trên Jetson**. Kết quả MPS chỉ chứng
minh model depth có thể theo kịp 10 Hz trên Mac với backend GPU này.

Trên Mac M2, phép thử một luồng 4 ảnh mất khoảng 0,78–0,81 s/ảnh. Với 4 luồng
trên 20 ảnh khác nhau, p50 là 256 ms, p95 là 315 ms, CPU trung vị khoảng
341% của một lõi (tức 3,4 lõi). Đây là suy luận
OpenCV DNN **CPU**, không dùng GPU Apple MPS. Dù đã nhanh hơn, 4 luồng vẫn
chậm hơn chu kỳ 100 ms của camera 10 Hz; phải tối ưu backend/model và đo lại
trên Jetson trước khi dùng để bay.

1. Chạy `bash scripts/demo_mps_depth_mac.sh`; chỉ vào nhãn **RGB FPS**,
   **Depth FPS** và số **RGB bỏ qua** trên cửa sổ. Nếu muốn so trực tiếp,
   đóng cửa sổ rồi chạy `bash scripts/demo_cpp_depth_mac.sh`.
2. Mở [ảnh C++ mẫu](mentor_depth_evidence/cpp/demo_frame_000220.png). Ảnh bên
   trái là RGB từ Gazebo; bên phải là depth từ `depth_anything_onnx_probe`
   (Depth Anything V2 Metric Outdoor Small, ONNX, C++/OpenCV DNN). Ở vùng giữa
   ảnh, C++ cho 4,44 m; LiDAR mô phỏng cho 5,06 m; sai lệch −0,62 m. LiDAR
   **không** là đầu vào của model.
3. Nếu cần bản video dự phòng, phát [video 19 giây](../artifacts/depth_demo.mp4)
   và [đồ thị đánh giá](mentor_depth_evidence/comparison.png). Nói rõ video và
   MAE 1,445 m là từ benchmark **Python trước đây** trên cùng model/scene; video
   không chứng minh pipeline C++ bay closed-loop. [Báo cáo gốc](MONOCULAR_MENTOR_DEPTH_BENCHMARK_VI.md).
4. Cho xem phép đo C++ trên Mac M2: 640×360, một luồng CPU, p50 790,6 ms,
   p95 804,3 ms, khoảng 1,27 frame/s, gần 100% một lõi
   ([log 20 frame](mentor_depth_evidence/cpp/cpu_reference_20261006.json)).
   Camera mô phỏng 10 Hz có chu kỳ 100 ms; CPU hiện tại không theo kịp.
   Số này **không** phải hiệu năng Jetson Nano hay độ trễ toàn pipeline ROS.
5. Nếu mentor hỏi Docker: [image ROS 2 Humble ARM64](MENTOR_DOCKER_HUMBLE_VI.md)
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

**Câu nói ngắn khi demo:** “Em dùng Depth Anything V2 Metric Outdoor Small từ
camera RGB, còn LiDAR chỉ để chấm sai số. Trên Mac, backend MPS của PyTorch
theo kịp video 10 FPS; bản C++ OpenCV CPU chỉ khoảng 3,5 FPS và bỏ frame.
MPS là cách demo nhanh trên GPU Apple, chưa phải backend C++ để đưa lên Jetson.
Em sẽ đo lại và tối ưu trên đúng Jetson trước khi thử tránh vật khi bay.”

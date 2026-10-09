# Chạy nhánh C++ monocular

Nếu host không cài ROS 2 Humble, dùng
[Docker runtime C++](MENTOR_DOCKER_HUMBLE_VI.md) trước. Docker runtime không
đóng gói Gazebo/SITL.

Trên Ubuntu 22.04 có ROS 2 Humble, Gazebo Harmonic, `ros_gz_bridge`, MAVROS
và OpenCV DNN. Humble mặc định đi với Gazebo Fortress, nhưng plugin trong repo
cần Harmonic (`gz-sim8`). Cài Harmonic từ OSRF cùng gói
`ros-humble-ros-gzharmonic`; tránh trộn với bộ `ros-humble-ros-gz*` mặc định
cho Fortress ([hướng dẫn Gazebo](https://gazebosim.org/docs/garden/ros_installation/)).
Chạy từ gốc repo. Đây là smoke test **RGB → depth C++ →
cloud** và khởi động MPPI C++; launch monocular tắt LiDAR và adapter, không
tạo global path nên chưa phải closed-loop tránh vật cản.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . uav_navigation_core uav_navigation_ros uav_navigation_bringup \
  --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

File ONNX khoảng 98 MB không nằm trong Git. Xuất một lần trên máy phát triển
có `torch`, `transformers`, `onnx`, rồi dùng đường dẫn tuyệt đối tới file đó
(hoặc chép file sang máy chạy):

```bash
python3 scripts/export_depth_anything_onnx.py \
  --output results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx
export UAV_DEPTH_ONNX="$PWD/results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx"
sha256sum "$UAV_DEPTH_ONNX"
```

SHA-256 của file đã kiểm thử: `1a7cb9d9c31d60afc9cbe96c6a401ac27c8cebcfce78cfc4ca1c2587691dc4a8`.
Khởi động ArduPilot SITL theo [setup hiện có](../README.md#ros-2-architecture-workflow);
launch dưới đây mở Gazebo, bridge RGB, node depth và planner C++ nhưng không
bật adapter:

```bash
ros2 launch uav_navigation_bringup monocular_sim.launch.xml \
  model_file:="$UAV_DEPTH_ONNX" depth_backend:=cpu \
  enable_gazebo_gui:=false enable_rviz:=false
```

Ở terminal khác:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /perception/monocular_depth/diagnostics --once
ros2 topic hz /perception/obstacles_camera
```

Mặc định inference/tổng xử lý phải ≤100 ms và ảnh ≤250 ms tuổi; kết quả trễ
không phát cloud. Để **chỉ kiểm tra đường dữ liệu trên CPU chậm**, chạy lại
launch với thêm `max_inference_ms:=1500 max_processing_ms:=1500 max_image_age_s:=2.0`;
không dùng các ngưỡng nới này để đánh giá bay.
`depth_backend:=cuda` hoặc `cuda_fp16` chỉ hoạt động nếu OpenCV DNN trên máy
có target tương ứng. Đo lại latency/CPU trên đúng Jetson trước khi chọn backend.

Mã: [MPPI core](../uav_navigation_core/src/mppi/),
[depth ONNX](../uav_navigation_ros/src/depth_anything_onnx.cpp),
[node RGB](../uav_navigation_ros/src/monocular_depth_node.cpp),
[planner ROS](../uav_navigation_ros/src/local_navigation_node.cpp).
[Kết quả và giới hạn hiện tại](JETSON_CPP_RUNTIME_VI.md).

Trên Mac Apple Silicon, đã build/smoke test nhánh này với ROS 2 **Jazzy qua
RoboStack**, chưa kiểm thử Humble. ROS 2 Humble không liệt kê macOS arm64 trong
[nền tảng hỗ trợ chính thức](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst);
RoboStack có bản Humble cộng đồng cho macOS, nhưng cần xác nhận đủ MAVROS,
`ros_gz_bridge` và Gazebo Harmonic trước khi chạy toàn bộ launch. Để tái lập
đúng môi trường của mentor, ưu tiên Ubuntu 22.04 x86_64 trên máy Linux với cặp
Humble + Harmonic ở trên. VM Ubuntu 22.04 arm64 trên Mac có thể dùng để thử
Humble, nhưng Gazebo trên arm64 chỉ được hỗ trợ best-effort
([nền tảng Gazebo](https://gazebosim.org/docs/harmonic/install/)). Bản build
Jazzy trên Mac không chứng minh stack Humble đã chạy.

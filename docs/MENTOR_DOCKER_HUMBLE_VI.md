# Docker ROS 2 Humble cho monocular C++

Image này chứa **ROS 2 Humble + hai package C++ `uav_navigation_core` và
`uav_navigation_ros`** và OpenCV 5 CPU biên dịch từ source. OpenCV 4.5 mặc
định của Ubuntu 22.04 chưa hỗ trợ `LayerNormalization` trong model ONNX này.
Không cần cài Humble trên Mac/host. Model ONNX được
mount lúc chạy; image không chứa Gazebo, SITL, MAVROS node hay CUDA. Launch
runtime khởi động depth và, nếu bật, MPPI; **không tự arm hoặc điều khiển UAV**.

Từ gốc repo, build image đúng kiến trúc máy chạy:

```bash
# Mac Apple Silicon / Jetson Nano: ARM64; máy Ubuntu x86_64: linux/amd64.
docker build --platform linux/arm64 -f docker/Dockerfile.humble \
  -t uav-monocular:humble .
docker run --rm uav-monocular:humble ros2 pkg executables uav_navigation_ros
```

Build OpenCV trong image tốn thời gian và nhiều GB dung lượng Docker. Kiểm tra
import ONNX và một lượt suy luận CPU với model đã xuất theo
[quickstart C++](MENTOR_CPP_QUICKSTART_VI.md):

```bash
export UAV_DEPTH_ONNX="$PWD/results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx"
docker run --rm -v "$UAV_DEPTH_ONNX:/models/depth.onnx:ro" \
  uav-monocular:humble bash -lc \
  'dd if=/dev/zero of=/tmp/black.rgb bs=691200 count=1 status=none && \
   ros2 run uav_navigation_ros depth_anything_onnx_probe \
     /models/depth.onnx /tmp/black.rgb /tmp/depth.f32 1'
```

Sau khi probe qua, chạy perception CPU:

```bash
docker run --rm --network host --ipc host \
  -v "$UAV_DEPTH_ONNX:/models/depth.onnx:ro" \
  uav-monocular:humble \
  ros2 launch uav_navigation_ros monocular_runtime.launch.xml \
    model_file:=/models/depth.onnx params_file:=/opt/uav/monocular.yaml \
    depth_backend:=cpu enable_local_navigation:=false
```

Lệnh `--network host --ipc host` ở trên dành cho **Linux/Jetson** để ROS 2
nhận topic từ host. Trên macOS, có thể build và kiểm tra package trong
container; truyền topic DDS qua Docker Desktop cần cấu hình mạng riêng và
chưa được kiểm thử ở repo này. Khi có `/localization/odometry` từ EKF,
`/planning/global_path`, TF và ảnh `/sensor_suite/rgb`, đổi
`enable_local_navigation:=true` để thử MPPI, giữ adapter bay tắt. Profile
`/opt/uav/monocular.yaml` hiện giới hạn 1 m/s và vẫn chỉ có occupied cloud;
đây chưa phải stack tránh vật an toàn để bay.

Để chuyển **cùng image ARM64** sang Jetson, dùng `docker save` / `docker load`
hoặc registry. Chạy CPU trước và đo latency/CPU trên Nano. CUDA/TensorRT
trong container phụ thuộc JetPack, driver host và bản OpenCV được build với
CUDA; image này biên dịch OpenCV 5 **CPU**. Docker không tự bổ sung
GPU backend. Cần xác nhận JetPack và thử tương thích kernel/runtime trên đúng
Nano trước khi dùng image; không coi build trên Mac là xác nhận Jetson.

Gazebo Harmonic + `ros_gz` Humble được tách khỏi image ARM64: gói
`ros-humble-ros-gzharmonic` của OSRF hiện chỉ có binary Ubuntu x86_64.
Muốn chạy cả SITL/Gazebo theo repo, dùng máy mô phỏng Ubuntu 22.04 x86_64;
runtime Docker có thể nhận topic ROS từ máy mô phỏng khi cấu hình mạng DDS.
MAVROS Humble node cũng cần kiểm tra riêng trước khi nối adapter; image này
chỉ mang `mavros_msgs` để biên dịch node C++.

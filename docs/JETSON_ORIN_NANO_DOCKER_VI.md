# Chạy perception C++ trong Docker trên Jetson Orin Nano

Máy đích đã báo `R36 (release), revision: 5.2, EABI: aarch64` (Jetson Linux
36.5.2). Image trong repo dùng ROS 2 Humble/Ubuntu 22.04 ARM64, phù hợp nhánh
R36 về hệ điều hành. Container chỉ chứa node C++ và thư viện của dự án; model
ONNX được mount chỉ đọc. Nó nhận ảnh từ một ROS 2 camera publisher đang chạy
trên host hoặc trong container khác, không cần truy cập trực tiếp `/dev/video*`.

**Image hiện tại chỉ suy luận bằng OpenCV DNN trên CPU.** `--runtime nvidia`
không làm image này dùng GPU. Kết quả 12 FPS trên Mac là PyTorch/MPS của Apple,
không phải tốc độ Docker này trên Orin. Trước khi nối planner, cần đo inference,
tuổi ảnh và số cloud được publish trên chính Orin. Đường chạy dưới đây chỉ bật
perception; không chạy MPPI, MAVROS hay adapter điều khiển bay.

Từ gốc repo trên Orin, chuẩn bị image theo một trong hai cách:

```bash
# Nếu đã tải artifact uav-monocular-humble-arm64.tar.gz từ CI:
gunzip -c uav-monocular-humble-arm64.tar.gz | docker load

# Hoặc build tại chỗ (biên dịch OpenCV 5 có thể tốn thời gian và nhiều bộ nhớ):
docker build --platform linux/arm64 --build-arg OPENCV_BUILD_JOBS=2 \
  -f docker/Dockerfile.humble -t uav-monocular:humble .
```

Chỉ chạy **một** trong hai lệnh trên. Sau đó chỉ đường dẫn tuyệt đối tới file
ONNX đã xuất theo [quickstart C++](MENTOR_CPP_QUICKSTART_VI.md):

```bash
export UAV_DEPTH_ONNX="$(realpath results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx)"
bash scripts/run_cpp_depth_orin.sh check
bash scripts/run_cpp_depth_orin.sh probe
```

`check` xác nhận R36, Docker, kiến trúc `linux/arm64` và model. `probe` chạy
một inference CPU trên khung RGB đen 640×360 để kiểm tra model được nạp; ảnh
đen không dùng để đánh giá độ chính xác depth. Để nhận ảnh thật qua ROS:

```bash
export ROS_DOMAIN_ID=0  # đặt cùng domain với camera publisher
bash scripts/run_cpp_depth_orin.sh run
```

Camera publisher cần gửi `sensor_msgs/Image` 640×360, encoding `rgb8` hoặc
`bgr8`, có timestamp hiện tại và `frame_id`, tới `/sensor_suite/rgb`.
Node xuất `sensor_msgs/PointCloud2` ở `/perception/obstacles_camera` và
`diagnostic_msgs/DiagnosticArray` ở
`/perception/monocular_depth/diagnostics`. Nếu inference quá 100 ms hoặc
ảnh cũ quá 250 ms, node **không xuất cloud** và ghi lý do trong diagnostics.
Vì vậy `probe` thành công chỉ chứng minh image/model nạp được, chưa chứng minh
theo kịp camera 10 Hz.

Script dùng Docker `runc` (CPU), chia sẻ mạng/IPC host để ROS 2 DDS thấy các topic, nhưng
root filesystem chỉ đọc, model chỉ đọc, không mount source hay camera device,
không thêm quyền đặc biệt, và tự xóa container khi dừng. Docker vẫn tạo image,
container và dữ liệu quản lý Docker trên host; nó không cài ROS/OpenCV vào môi
trường hệ điều hành của mentor. Nếu camera publisher nằm trong container khác,
đặt `ROS_DOMAIN_ID` giống nhau và kiểm tra DDS discovery.

Lượt chạy đầu trên Orin cần ghi `inference_ms`, `image_age_ms`, tỉ lệ cloud
được publish, CPU/RAM và nhiệt độ khi camera phát 10 Hz. Với backend CPU hiện
tại, chưa có số đo nào xác nhận đạt hạn 100 ms trên Orin. Nếu không đạt, bước
tiếp theo là image GPU/TensorRT riêng, được build và đo trên đúng R36.5.2;
không tăng deadline rồi coi đó là đủ để bay.

Tham chiếu: [NVIDIA Jetson Linux R36.5.2](https://docs.nvidia.com/jetson/archives/r36.5.2/DeveloperGuide/index.html),
[NVIDIA Docker Setup cho Orin Nano](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html).

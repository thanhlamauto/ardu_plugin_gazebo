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

Trên Orin, clone repo để có script:

```bash
git clone https://github.com/thanhlamauto/ardu_plugin_gazebo.git
cd ardu_plugin_gazebo
```

Tải artifact `uav-monocular-humble-arm64`
từ [run CI ARM64](https://github.com/thanhlamauto/ardu_plugin_gazebo/actions/workflows/docker-humble-runtime.yml)
(GitHub tải về một file ZIP; copy ZIP sang Orin rồi giải nén để lấy `.tar.gz`).
Copy file ONNX từ Mac sang Orin vì model không nằm trong Git/Docker image:

```bash
# Chạy trên Mac; thay user/IP của Orin.
ssh user@JETSON_IP 'mkdir -p ~/models'
scp ~/Downloads/uav-monocular-humble-arm64.zip \
  user@JETSON_IP:~/ardu_plugin_gazebo/
scp results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx \
  user@JETSON_IP:~/models/depth.onnx
```

Từ gốc repo trên Orin, chuẩn bị image theo một trong hai cách:

```bash
# Nếu đã tải artifact dạng ZIP từ CI:
unzip uav-monocular-humble-arm64.zip -d ./uav-image
gunzip -c ./uav-image/uav-monocular-humble-arm64.tar.gz | docker load

# Hoặc build tại chỗ (biên dịch OpenCV 5 có thể tốn thời gian và nhiều bộ nhớ):
docker build --platform linux/arm64 --build-arg OPENCV_BUILD_JOBS=2 \
  -f docker/Dockerfile.humble -t uav-monocular:humble .
```

Chỉ chạy **một** trong hai lệnh trên. Sau đó chỉ đường dẫn tuyệt đối tới file
ONNX đã xuất theo [quickstart C++](MENTOR_CPP_QUICKSTART_VI.md):

```bash
export UAV_DEPTH_ONNX="$(realpath ~/models/depth.onnx)"
sha256sum "$UAV_DEPTH_ONNX"
# SHA-256 đã kiểm thử: 1a7cb9d9c31d60afc9cbe96c6a401ac27c8cebcfce78cfc4ca1c2587691dc4a8
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

## Kiểm tra GPU bằng TensorRT (model riêng, chưa phải node ROS)

Trên Orin, kiểm tra Docker đã có NVIDIA runtime:

```bash
docker info --format '{{json .Runtimes}}'
```

Nếu không thấy `nvidia`, làm theo [NVIDIA Docker Setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html)
để cài NVIDIA Container Toolkit và cấu hình runtime; bước đó thay đổi cấu hình
Docker của host nên cần người quản trị máy thực hiện. Khi runtime đã sẵn sàng:

```bash
export UAV_DEPTH_ONNX="$(realpath ~/models/depth.onnx)"
bash scripts/probe_depth_tensorrt_orin.sh
```

Script dùng image NVIDIA `l4t-jetpack:r36.4.0` với `--runtime nvidia`, mount
model chỉ đọc, build engine FP16 trên chính Orin rồi benchmark 10 giây. Nó lưu
`build.log`, `benchmark.log` và `depth_fp16.plan` trong
`artifacts/orin_tensorrt/`. NVIDIA hiện công bố tag container `r36.4.0`; host
đang ở R36.5.2. Probe C++ bên dưới đã nạp engine TensorRT 10.3 tạo trên host
và chạy thành công trong image này; nếu đổi engine/image vẫn phải thử lại.
Có thể đặt `UAV_JETPACK_IMAGE`
để dùng tag khác nếu NVIDIA phát hành bản phù hợp hơn.

`trtexec` dùng tensor ngẫu nhiên để đo riêng suy luận TensorRT; tốc độ đó
không gồm camera, tiền/hậu xử lý, ROS hay chất lượng depth. Engine `.plan`
không được node C++ hiện tại nạp. OpenCV 5 `ENGINE_NEW` hiện chỉ chạy CPU;
muốn perception ROS thật sự dùng GPU cần thêm backend TensorRT C++ đọc engine,
so kết quả depth với bản CPU và đo lại toàn pipeline trước khi bật planner.

### Kết quả đo trực tiếp trên Orin R36.5.2 (09/10/2026)

Với ONNX SHA-256 ở trên, `trtexec` TensorRT 10.3 tạo engine FP16 50 MiB
trong 443,8 giây. Benchmark 20 giây tại power mode 15 W: **62,48 inference/s**,
host latency trung bình **16,19 ms**, p95 **16,22 ms**. Đây là phép đo mô hình
với đầu vào giả, không phải FPS camera hay node ROS.
Chính engine này cũng nạp và chạy trong image `l4t-jetpack:r36.4.0` trên host
R36.5.2: phép đo 5 giây đạt **61,89 inference/s**, p95 **16,27 ms**.

Probe C++ riêng đã chạy một ảnh RGB 640×360 từ Gazebo qua tiền xử lý, engine
TensorRT và nội suy ra depth 640×360 trên Orin: 30 lần lặp có p50 **44,76 ms**,
p95 **54,76 ms**, **23,25 FPS** và CPU tiến trình **39,44% của một lõi**.
So với OpenCV DNN CPU trên Mac cho đúng frame/model đó, sai khác depth FP16
trung bình tuyệt đối **0,019 m** trên 230.400 pixel (p95 **0,055 m**). Đây là
độ lệch giữa hai implementation, không phải sai số so với LiDAR ground truth.
[Ảnh RGB, depth TensorRT và sai khác CPU](mentor_depth_evidence/orin/tensorrt_cpp_20261009.png)
cho mentor xem trực tiếp; thang màu depth được cắt tại 25 m.
Probe C++ đã được chạy lại **trong image JetPack** trên cùng Orin, không cần
OpenCV trong container: 30 lần lặp đạt p50 **37,27 ms**, p95 **51,52 ms**,
**26,01 FPS**, CPU **37,36% của một lõi**. Depth từ container khác bản chạy
trực tiếp trên host trung bình **0,000002 m** do bước nội suy C++ thay OpenCV.
Lệnh tái lập:

```bash
export UAV_TRT_ENGINE="$(realpath ~/uav_deploy/depth_fp16.plan)"
bash scripts/probe_cpp_depth_tensorrt_orin.sh
```

Probe C++ này dùng file RGB mẫu trong repo, không đăng ký topic ROS, không nối
planner và chưa đo camera thời gian thực. File depth float32 và log lưu ở
`artifacts/orin_cpp_tensorrt/`.

Tham chiếu: [NVIDIA Jetson Linux R36.5.2](https://docs.nvidia.com/jetson/archives/r36.5.2/DeveloperGuide/index.html),
[NVIDIA Docker Setup cho Orin Nano](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html),
[OpenCV 5 DNN engine selection](https://docs.opencv.org/5.0/main_modules/dnn_engine_selection.html),
[NVIDIA L4T JetPack container](https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/l4t-jetpack/-).

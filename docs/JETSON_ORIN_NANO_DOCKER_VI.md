# Chạy perception C++ trong Docker trên Jetson Orin Nano

Máy đích đã báo `R36 (release), revision: 5.2, EABI: aarch64` (Jetson Linux
36.5.2). Image trong repo dùng ROS 2 Humble/Ubuntu 22.04 ARM64, phù hợp nhánh
R36 về hệ điều hành. Container chỉ chứa node C++ và thư viện của dự án; model
ONNX được mount chỉ đọc. Nó nhận ảnh từ một ROS 2 camera publisher đang chạy
trên host hoặc trong container khác, không cần truy cập trực tiếp `/dev/video*`.

**Image `uav-monocular:humble` dưới đây chỉ suy luận bằng OpenCV DNN trên CPU.**
`--runtime nvidia` không làm image CPU này dùng GPU. Image
`uav-monocular:humble-gpu` ở cuối tài liệu dùng TensorRT trong node ROS C++.
Kết quả 12 FPS trên Mac là PyTorch/MPS của Apple, không phải tốc độ Docker
trên Orin. Cả hai đường chạy ở đây chỉ bật perception, không chạy MPPI,
MAVROS hay adapter điều khiển bay.

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

Image CPU đã nạp và chạy `check`/`probe` thành công trên Orin R36.5.2.
Với **cùng frame RGB mẫu**, 5 lần suy luận sau warmup đạt p50 **3.129,67 ms**,
p95 **3.135,08 ms**, **0,319 FPS** và CPU tiến trình **99,69% một lõi**.
Nó không đạt hạn 100 ms hay camera 10 Hz. Đây là phép đo file ảnh bằng probe,
chưa có publisher camera/ROS để đo `image_age_ms`, tỉ lệ cloud, RAM hay nhiệt
độ của toàn pipeline. Không tăng deadline rồi coi đó là đủ để bay.

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
không gồm camera, tiền/hậu xử lý, ROS hay chất lượng depth. Image CPU ở phần
trên vẫn chỉ dùng OpenCV DNN CPU. Image GPU ở phần cuối tài liệu có backend
TensorRT C++ nạp engine `.plan` vào chính node ROS perception; cần dùng image
đó để nhận ảnh ROS bằng GPU.

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

[Video RGB cạnh depth TensorRT trên Orin](mentor_depth_evidence/orin/orin_tensorrt_depth_video.mp4)
cho thấy 131 frame Gazebo liên tiếp, phát lại ở 10 FPS. Probe C++ xử lý offline
toàn chuỗi bằng engine FP16 trên Orin: p50/p95 **42,56/53,47 ms**, thông lượng
**22,29 frame/s**, CPU **46,42% một lõi**. FPS ghi trên video là tốc độ phát
lại, còn 22,29 frame/s là thông lượng suy luận của probe. Đây là chuỗi offline
khác với bài kiểm tra node ROS ở cuối tài liệu (100 lần phát cùng một frame);
chưa phải video camera thật hoặc closed-loop.

Probe C++ đã được chạy lại **trong image JetPack** trên cùng Orin, không cần
OpenCV trong container: 30 lần lặp đạt p50 **37,27 ms**, p95 **51,52 ms**,
**26,01 FPS**, CPU **37,36% của một lõi**. Depth từ container khác bản chạy
trực tiếp trên host trung bình **0,000002 m** do bước nội suy C++ thay OpenCV.
Image NVIDIA `l4t-jetpack:r36.4.0` sau khi tải chiếm khoảng **15,5 GB** theo
`docker images`; engine FP16 chiếm khoảng **50 MiB**. Lần đầu tạo engine mất
khoảng 7 phút trên Orin, các lần chạy probe dùng lại file `.plan`.
Lệnh tái lập:

```bash
export UAV_TRT_ENGINE="$(realpath ~/uav_deploy/depth_fp16.plan)"
bash scripts/probe_cpp_depth_tensorrt_orin.sh
```

Probe C++ này dùng file RGB mẫu trong repo, không đăng ký topic ROS, không nối
planner và chưa đo camera thời gian thực. File depth float32 và log lưu ở
`artifacts/orin_cpp_tensorrt/`.

## Node ROS C++ dùng GPU trên Orin

Image GPU build từ image Humble CPU ở trên và các thư viện TensorRT/CUDA từ
`l4t-jetpack:r36.4.0`; image runtime `uav-monocular:humble-gpu` đo được khoảng
2,7 GB. Node `monocular_depth_node` nhận `backend:=tensorrt`, nạp engine FP16
đã tạo trên chính Orin, và giữ nguyên topic ảnh/cloud/diagnostics. ROS và
OpenCV nằm trong image; driver GPU được NVIDIA Container Runtime gắn lúc chạy.

```bash
cd ~/ardu_plugin_gazebo
docker build --platform linux/arm64 -f docker/Dockerfile.humble-gpu \
  -t uav-monocular:humble-gpu .
# Mặc định: ~/uav_deploy/depth_fp16.plan; chỉ cần export UAV_DEPTH_ENGINE nếu đặt nơi khác.
bash scripts/run_cpp_depth_orin_gpu.sh check
ROS_DOMAIN_ID=73 bash scripts/run_cpp_depth_orin_gpu.sh run
```

Trong terminal thứ hai, kiểm tra với ảnh RGB Gazebo phát ở 10 Hz qua ROS 2:

```bash
cd ~/ardu_plugin_gazebo
ROS_DOMAIN_ID=73 UAV_REPLAY_FRAMES=100 \
  bash scripts/probe_cpp_depth_ros_orin_gpu.sh
```

Hai container dùng domain thử nghiệm 73; đổi thành domain của camera khi nối
camera thật. Node GPU hiện chỉ chạy perception, không bật MPPI/MAVROS hay lệnh
bay. Lệnh replay là C++ và báo nhịp publish thật, số cloud, inference,
processing và tuổi ảnh; file RGB đầu vào là frame mẫu trong repo.

**Đo trên Orin R36.5.2:** lượt cuối phát 100 ảnh với publish interval
p50/p95 **99,99/100,79 ms**, nhận **97 cloud/97 diagnostics OK**, inference
p50/p95 **60,23/61,08 ms**, processing p95 **61,21 ms**, image age p95
**62,77 ms**. CPU của tiến trình depth đạt trung vị **27,80% một lõi** và p95
**30,14% một lõi**. Hai lượt trước nhận 59/60 và 99/100 cloud; mọi ảnh được
xử lý đều qua deadline 100 ms, nhưng có **1–3 ảnh/lượt không tới cloud**.
Chưa xác định chính xác ảnh rơi ở DDS, hàng đợi hay khâu phát ảnh. Đây là
replay của **một frame lặp lại**, chưa kiểm tra camera thật, độ chính xác khi
scene đổi, nhiệt/nguồn dài hạn hoặc closed-loop an toàn.

Tham chiếu: [NVIDIA Jetson Linux R36.5.2](https://docs.nvidia.com/jetson/archives/r36.5.2/DeveloperGuide/index.html),
[NVIDIA Docker Setup cho Orin Nano](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html),
[OpenCV 5 DNN engine selection](https://docs.opencv.org/5.0/main_modules/dnn_engine_selection.html),
[NVIDIA L4T JetPack container](https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/l4t-jetpack/-).

# Báo cáo cho mentor: depth monocular so với LiDAR 3D mô phỏng

Ngày 30/09/2026.

## Tóm tắt để trao đổi với mentor

Hiện em dùng **Depth Anything V2 Metric Outdoor Small** để ước lượng khoảng
cách vật cản từ một ảnh camera. Đây là model có sẵn, em chưa huấn luyện lại
cho môi trường mô phỏng. Em cho UAV bay thử hai lượt ở 0,5 m/s, rồi dùng
LiDAR mô phỏng để kiểm tra kết quả sau khi bay; LiDAR không giúp model dự
đoán. Sai số độ sâu trung bình khoảng **1,45 m**, còn thời gian từ lúc nhận
ảnh đến lúc có kết quả ở mức p95 là **77–92 ms**. Em cũng thử tính độ sâu từ
nhiều ảnh liên tiếp với pose EKF: các điểm theo dõi đủ tốt có sai số khoảng
**0,10–0,11 m**, nhưng UAV phải tiến khoảng **1,44 m** mới có kết quả đầu
tiên và cách này chưa nhìn đủ mọi vùng phía trước. Vì vậy hiện em mới có kết
quả đo độ sâu, **chưa chứng minh được khả năng tránh vật cản tự động**.

## Bối cảnh và cách chạy thử

Theo góp ý của mentor, lần này em tập trung trả lời câu hỏi:
**chỉ với camera, UAV ước lượng vật cản cách bao xa và kết quả đến chậm bao
lâu?** Em giữ phần định vị của ArduPilot để bay thử, chưa làm tiếp OpenVINS
hay chỉnh MPPI. ArduPilot cung cấp pose local từ EKF qua MAVLink và log có
`GPS_RAW_INT` fix type 6. Đây là ước lượng của hệ thống bay, **không phải pose
do camera tính ra**, cũng không có nghĩa GNSS một mình cho đủ pose 6-DoF.
Gazebo odometry chỉ dùng để giới hạn an toàn và đối chiếu sau chuyến bay.

World [`iris_monocular_lidar_gt_benchmark.sdf`](../worlds/iris_monocular_lidar_gt_benchmark.sdf)
dùng cùng RGB camera 640×360, 10 Hz và hộp textured ở x=8 m. Sensor LiDAR
3D 640×32, 10 Hz được thêm **chỉ cho đánh giá**, đặt đồng vị trí với RGB;
không có depth camera. [`run_monocular_sim.py`](../scripts/run_monocular_sim.py)
chạy chế độ `--mentor-depth-benchmark`: hover, tiến thẳng theo EKF khoảng
2,56 m với lệnh 0,5 m/s, rồi hover; không bật planner hay OpenVINS.
LiDAR được [`log_lidar_gt_gz.py`](../scripts/log_lidar_gt_gz.py) ghi vào thư
mục riêng. Predictor RGB không subscribe LiDAR, depth GT, Gazebo pose hoặc
OpenVINS.

## Mô hình depth đang dùng

Em đang thử **Depth Anything V2 Metric Outdoor Small**. Mỗi lần camera gửi
một ảnh màu, model dự đoán khoảng cách tới các bề mặt trong ảnh. Đây là model
đã được huấn luyện sẵn; em **chưa huấn luyện lại hay hiệu chuẩn** bằng ảnh
Gazebo. LiDAR không tham gia dự đoán, chỉ được đem ra so sánh sau khi bay.
Vì thế, sai số ở phần kết quả là sai số của model khi áp dụng trực tiếp vào
scene mô phỏng hiện tại.

Để chạy lại đúng thí nghiệm: checkpoint là
`depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf`, revision
`2fd93bd764b15eea94dcf7763bba7ddc25007d0f`. Code dùng Hugging Face
Transformers, input 518×518, Apple MPS và đưa kết quả về ảnh 640×360. Hai
lượt dùng cùng checkpoint: [log lượt 1](mentor_depth_evidence/r3/summary.json),
[log lượt lặp](mentor_depth_evidence/repeat/summary.json).
Cách theo dõi landmark qua nhiều ảnh được chấm thêm bên dưới là **một phương
pháp hình học khác**: nó dùng ảnh và pose EKF, không dùng model Depth Anything.

## Hai nhánh depth được chấm trên cùng chuyến bay

1. **Metric depth học máy trực tiếp từ một RGB.**
   [`monocular_depth_gz.py`](../scripts/monocular_depth_gz.py) chạy
   checkpoint nêu trên. Predictor chạy online và lưu cả ảnh/depth với
   timestamp. Chỉ sau chuyến bay,
   [`evaluate_monocular_lidar_depth.py`](../scripts/evaluate_monocular_lidar_depth.py)
   chiếu điểm LiDAR lên ảnh, so sánh ở những pixel LiDAR thực sự đo được.
2. **Triangulation landmark nhiều frame với pose EKF.**
   [`evaluate_multiframe_ekf_lidar.py`](../scripts/evaluate_multiframe_ekf_lidar.py)
   replay cùng RGB đã lưu, dùng `LOCAL_POSITION_NED` và `ATTITUDE` của
   ArduPilot, đồng bộ theo `time_boot_ms`. Bộ theo dõi không dùng Gazebo pose
   hay LiDAR. Chỉ kết quả cuối mới ghép với điểm LiDAR lân cận pixel feature;
   vùng có depth discontinuity bị loại. `processing_ms` của nhánh này là thời
   gian **replay offline**, chưa phải end-to-end online.

Hai lượt hoàn tất ở cùng scene. Repo chứa [summary lượt 1](mentor_depth_evidence/r3/)
và [summary lượt lặp](mentor_depth_evidence/repeat/); raw log và snapshot mã
nguồn đúng lúc chạy nằm trong thư mục `results/monocular_research/` trên máy
chạy benchmark, không đưa lên GitHub vì dung lượng lớn.
Nhãn `--seed` của runner chỉ điều khiển planner khi planner chạy; trong bài
benchmark này không phải seed Gazebo được kiểm soát. Đây là hai repetition,
chưa phải kiểm định nhiều scene/seed.

| Phép đo | Lượt 1 | Lượt lặp |
| --- | ---: | ---: |
| RGB–LiDAR frame ghép timestamp | 131 | 136 |
| Sai lệch stamp p95 | 0 ms | 0 ms |
| Quãng tiến EKF / Gazebo truth | 2,560 / 2,557 m | 2,560 / 2,558 m |
| Sai số pose EKF p95 / orientation p95 so với truth (chỉ audit) | 0,016 m / 0,150° | 0,017 m / 0,121° |
| RGB inference p95 | 78,8 ms | 68,0 ms |
| Nhận ảnh → publish p95 | 91,6 ms | 76,6 ms |
| Tuổi ảnh theo `/clock` lúc output p95 | 102 ms | 84 ms |
| Metric model: MAE trên mọi LiDAR return nhìn thấy | 1,445 m | 1,472 m |
| Metric model: MAE median depth vùng 50×50 px giữa ảnh | 1,214 m | 1,240 m |
| Metric model: bias vùng giữa khi vật ở 5–7 m | −1,531 m | −1,543 m |
| Metric model: bias vùng giữa khi vật ở 3–5 m | +0,639 m | +0,618 m |
| Metric model: precision / recall ngưỡng 5 m trên LiDAR return giữa ảnh | 0,448 / 0,773 | 0,435 / 0,788 |
| Multi-frame: frame có depth / frame replay | 63 / 131 | 62 / 136 |
| Multi-frame: landmark LiDAR match ổn định | 256 | 241 |
| Multi-frame: MAE trên **các landmark được chọn** | 0,115 m | 0,101 m |
| Multi-frame: median absolute error trên các landmark đó | 0,071 m | 0,072 m |
| Multi-frame: xử lý replay p95 | 53,6 ms | 54,8 ms |

Số liệu đầy đủ: [learned-depth lượt 1](mentor_depth_evidence/r3/lidar_depth_analysis.json),
[learned-depth lượt lặp](mentor_depth_evidence/repeat/lidar_depth_analysis.json),
[multi-frame lượt 1](mentor_depth_evidence/r3/multiframe_ekf_lidar_analysis.json),
[multi-frame lượt lặp](mentor_depth_evidence/repeat/multiframe_ekf_lidar_analysis.json),
và [đồ thị hai lượt](mentor_depth_evidence/comparison.png).

Một kiểm tra hình học độc lập tại tâm ảnh: khi UAV ở x≈0, mặt trước hộp dự
kiến cách camera ~6,84 m, LiDAR đo 6,84 m nhưng mô hình RGB cho ~5,07 m.
Khi UAV tới x≈2,8 m, LiDAR đo ~4,0 m còn mô hình cho ~4,8 m. Tức mô hình
không chỉ có một hệ số scale cố định: sai số đổi dấu theo cự ly. Ở ngưỡng
5 m, precision ~0,44 trên những LiDAR return ở giữa ảnh, nên không thể dùng
`predicted depth <5 m` trực tiếp như một quyết định tránh vật cản đáng tin.

Tracker hình học có sai số thấp **trên subset được chấp nhận** chứ không có
dense depth. Nó cần UAV đã tiến khoảng 1,44 m và mất ~3,5 s kể từ đầu pha
chuyển động mới tạo depth đầu tiên; khi đó hộp đã gần hơn. Landmark được
chấm chỉ khi ít nhất ba điểm LiDAR lân cận trong vòng 12 px có khoảng cách
ổn định và điểm gần nhất ≤6 px. Những pixel không có LiDAR, cạnh hộp, vùng
không texture và khoảng thời gian chưa có parallax không được tính vào MAE
0,10–0,11 m. Vì vậy không thể suy ra khả năng tránh vật cản từ con số MAE đó.
Pose EKF được đối chiếu Gazebo truth **chỉ trong bộ chấm offline**; p95 vị trí
~1,6–1,7 cm trong scene SITL này không đại diện cho nhiễu GNSS thực tế.

Các kiểm toán [`lượt 1`](mentor_depth_evidence/r3/mentor_benchmark_audit.json)
và [`lượt lặp`](mentor_depth_evidence/repeat/mentor_benchmark_audit.json)
xác nhận: không có OpenVINS/planner, predictor chỉ RGB, topic LiDAR runtime
cũ và depth camera đều 0 message, topic LiDAR đánh giá có 451 message,
GPS có 3D fix, và quãng dịch EKF khớp truth trong <0,01 m. LiDAR chỉ là
ground truth trên các tia đo được, không chứng nhận toàn ảnh hay vùng unknown
là free. Quãng đường tương ứng tuổi ảnh p95 84–102 ms là ~0,42–0,51 m ở
5 m/s, ~0,84–1,02 m ở 10 m/s, **chưa cộng** delay planner, truyền lệnh,
đáp ứng UAV hoặc khoảng phanh. Đây không phải kết luận tốc độ an toàn.

**Kết luận kỹ thuật hiện tại:** benchmark depth và latency mà mentor hỏi đã
có, nhưng chưa đạt gate để dùng làm tránh vật cản tự động. Không chạy
closed-loop 5–10 m/s từ các số này. Bước tiếp theo nên thử cải thiện metric
depth theo miền ảnh Gazebo/địa hình thực hoặc tạo representation chỉ công bố
occupied/free khi có bằng chứng, chấm thêm vật ít texture/mỏng và nhiều
cự ly; đồng thời thử lịch tạo parallax an toàn cho nhánh hình học. Chỉ sau
khi định nghĩa và vượt gate precision/recall, coverage và end-to-end delay
mới nối sang planner dùng EKF pose.

Chạy lại (thư mục `--output` phải mới):

```bash
PYTHON=python3  # môi trường có Gazebo bindings và dependencies trong README
"$PYTHON" scripts/run_monocular_sim.py \
  --output results/monocular_research/mentor_depth_new_trial \
  --world worlds/iris_monocular_lidar_gt_benchmark.sdf \
  --config config/monocular_speed_visual_0.5.yaml \
  --eval-scene config/monocular_scene_visual_ground_pilot.json \
  --goal 12 0 3 --duration 35 --mentor-depth-benchmark
"$PYTHON" scripts/evaluate_monocular_lidar_depth.py \
  results/monocular_research/mentor_depth_new_trial
"$PYTHON" scripts/evaluate_multiframe_ekf_lidar.py \
  results/monocular_research/mentor_depth_new_trial
"$PYTHON" scripts/audit_mentor_depth_benchmark.py \
  results/monocular_research/mentor_depth_new_trial
```

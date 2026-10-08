# Thử OpenVINS thay bộ ước lượng chuyển động ảnh tự viết

Thử nghiệm dùng [OpenVINS](https://github.com/rpng/open_vins) revision
`69488123ed9362dd44b6f28e7f4680abbff1442b`, biên dịch thành thư viện
không cần ROS trên macOS và nối trực tiếp Gazebo Transport qua
[`tools/openvins_gz_bridge`](../tools/openvins_gz_bridge/README.md). Bridge
đọc **một RGB camera 640×360, 10 Hz** và **IMU gia tốc/con quay 100 Hz**.
Tại hover, nó khởi tạo OpenVINS bằng trung bình IMU, vận tốc đứng yên và
mốc `(0,0,3 m)` đã khai báo. Đây là mốc độ cao/vị trí ban đầu, không phải
Gazebo odometry. Sau đó bridge publish `/perception/visual_odometry` cho
perception; MPPI được cấu hình dùng cùng topic nếu qua được bước setup.
Không module nào trong hai phần này đọc `/iris/odometry` ở chế độ OpenVINS.
Runner vẫn dùng Gazebo odometry để xác nhận hover, kiểm tra an toàn và chấm
điểm. Bridge dùng trường orientation trong IMU mô phỏng để khởi tạo/căn hệ
trục; chưa kiểm chứng bộ lọc attitude từ IMU thô ngoài đời. ArduPilot SITL
vẫn dùng cảm biến điều hướng nội bộ mô phỏng.

| Cảnh / mức đặt | Pose OpenVINS so với ground truth chấm điểm, p95 | Kết quả | Ghi chú |
| --- | ---: | --- | --- |
| 0,5 m/s, [`openvins_trial_05_r7`](../results/monocular_research/openvins_trial_05_r7/openvins_audit.json) | 7,47 m | `SETUP_FAILED` | VIO xuất 179 pose nhưng cloud camera không đủ điểm sau warmup. |
| 5 m/s, [`openvins_trial_5`](../results/monocular_research/openvins_trial_5/openvins_audit.json) | 22,22 m | `SETUP_FAILED` | VIO xuất 170 pose; cloud camera bằng 0. |
| 10 m/s, [`openvins_trial_10`](../results/monocular_research/openvins_trial_10/openvins_audit.json) | 2,18 m | `SETUP_FAILED` | VIO xuất 76 pose; warmup bị dừng khi pose vượt giới hạn chuyển động. |
| 0,5 m/s, [warmup tiến 30/09](../results/monocular_research/openvins_closed_loop_05_forward_20260930/openvins_audit.json) | 0,20 m | `SETUP_FAILED` | Dừng ở x≈2,2 m; 0 cloud, 0 điểm triangulation hợp lệ từ mặt vật cản nhìn chính diện. |
| 0,5 m/s, [warmup chéo 30/09](../results/monocular_research/openvins_closed_loop_05_diagonal_20260930/openvins_audit.json) | 0,93 m | `SETUP_FAILED` | Có tới 317 điểm triangulation ứng viên trong một frame, nhưng 0 cloud ổn định; VIO vượt ngưỡng sai số giám sát 1 m. |

Audit từng lượt kiểm tra topic đầu vào, command perception/planner, mã bridge,
hash binary và số message: cả ba lượt đều **không có LiDAR/depth camera**, và
Gazebo odometry không đi vào OpenVINS, perception hay MPPI. Sai số được tính
theo timestamp OpenVINS và nội suy ground truth chỉ trong evaluator. Pose p95
của lượt 10 m/s chỉ tính đến khi setup dừng nên không thể đọc là VIO hoạt động
ở vận tốc 10 m/s.

**Chưa có lượt bay điều hướng OpenVINS ở 5 hoặc 10 m/s:** các mức đó chỉ là
giới hạn đặt cho planner, nhưng cổng an toàn chưa cho planner khởi động. So
với bộ ảnh+IMU tự viết, OpenVINS đã xuất state đều hơn sau khởi tạo nhưng
không cải thiện kết quả hoàn thành trong các cảnh này. Đây là một cấu hình
thử, chưa phải benchmark rộng của OpenVINS. Cảnh có các ô nền lặp lại,
camera chỉ 10 Hz, và tham số camera–IMU mới là giá trị danh định từ SDF;
cần kiểm tra quality của feature track, frame/extrinsics và đồng bộ thời gian
trước khi quy lỗi cho bản thân thuật toán VIO. Hiện perception dùng phép
tam giác hóa ảnh theo thời gian cũng không tạo đủ cloud từ pose đầu ra.

Các lượt `openvins_trial_05` đến `_r6` là chẩn đoán tích hợp trước khi cấu
hình IMU scale, khởi tạo hover và ghép timestamp được sửa. Chúng **không**
được tính là các lần lặp độc lập của kết quả cuối.

Hai lượt ngày 30/09 dùng cùng revision bridge sau khi thêm timestamp đo độ trễ.
Lượt thứ hai tăng baseline keyframe monocular từ 0,18 lên 0,40 m và dùng
warmup chéo để tạo thị sai ngang. Cả hai dừng **trước khi MPPI phát lệnh**;
cột 0,5 m/s là **giới hạn cấu hình của planner**, không phải vận tốc kín vòng
đã đạt. Warmup chỉ gửi thành phần vận tốc 0,25 m/s theo trục được chọn;
cổng cloud và sai số pose hoạt động theo thiết kế. Ground truth chỉ ở runner
để chặn bài thử và đánh giá, không làm đầu vào OpenVINS/perception/planner.
Do đó hiện vẫn chưa có vận tốc dương nào được xác nhận cho tránh vật cản kín
vòng bằng RGB + IMU/OpenVINS trong cấu hình này. Không nên bỏ cổng để ép bay.

Chẩn đoán bổ sung [`openvins_diag_map_05_20260930`](../results/monocular_research/openvins_diag_map_05_20260930/openvins_audit.json)
ghi số điểm trước/sau bộ lọc: frame tốt nhất có 455 điểm triangulation hợp lệ,
79 điểm trong dải cao độ 3±0,65 m, nhưng 0 voxel được thấy hai lần ở thời
điểm đó. Sau đó bản đồ chỉ xác nhận tối đa 2 voxel; publisher phát lại hai
điểm cũ trong 147 message. Điều kiện `min_obstacle_points=12` vẫn ngăn planner
khởi động. Trong 15 giây chờ cloud, VIO trôi đến sai số 33,4 m ở cuối log;
runner hiện cũng kiểm tra sai số VIO trong pha chờ này để dừng sớm. Đây là
hai lỗi **riêng biệt**: chuyển động/đồng bộ/hiệu chuẩn của VIO và tích hợp
điểm sâu vào bản đồ. Không có bằng chứng rằng giảm ngưỡng 12 hay cho phép
voxel chỉ một hit sẽ tạo bản đồ vật cản đúng.

Hướng đối chiếu từ mã nguồn gốc: [hướng dẫn hiệu chuẩn OpenVINS](https://docs.openvins.com/gs-calibration.html)
yêu cầu kiểm tra intrinsics, extrinsics, nhiễu IMU, time offset và chuyển động
khởi tạo đủ kích thích; [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
có ước lượng online cho extrinsics/time offset;
[SVO depth filter](https://github.com/uzh-rpg/rpg_svo/blob/master/svo/include/svo/depth_filter.h)
duy trì độ sâu từng feature với bất định qua nhiều ảnh thay vì đếm voxel khớp
chính xác; [Voxblox](https://github.com/ethz-asl/voxblox/blob/master/docs/pages/The-Voxblox-Node.rst)
tích hợp đo sâu theo trọng số rồi tạo TSDF/ESDF, nhưng cần đầu vào depth đáng
tin trước khi áp dụng. [Issue Gazebo của OpenVINS](https://github.com/rpng/open_vins/issues/263)
cũng mô tả drift sau vài giây, song không phải bằng chứng cùng nguyên nhân.

## Sửa và chạy lại ngày 30/09/2026

Bridge nay chọn orientation IMU gần timestamp ảnh nhất (giới hạn 20 ms), ghi
`imu_alignment_ms` và cho bật thử nghiệm `--openvins-zupt-after-motion`.
[`audit_monocular_calibration.py`](../scripts/audit_monocular_calibration.py)
so hình học camera–IMU trong SDF với hằng số triangulation và cấu hình bridge.
Ở [lượt cuối](../results/monocular_research/openvins_fused_gate_05_20260930/calibration_audit.json),
227 ảnh có lệch timestamp IMU bằng 0 ms trong mô phỏng và các phép đối chiếu
SDF đều đạt. Đây chỉ là kiểm tra nhất quán cấu hình mô phỏng, chưa phải
calibration độc lập của camera, nhiễu IMU hay time offset thực tế.
OpenVINS log có ZUPT được chấp nhận, nhưng VIO vẫn trôi.

Bản đồ local mới có tùy chọn ghép voxel trong bán kính 0,3 m, tối đa một hit
mỗi frame, giữ bằng chứng 3 s nhưng chỉ xuất điểm được quan sát trong 1,5 s.
Perception chỉ tạo cloud mới khi ảnh đó có triangulation trong dải cao độ;
cloud gần nhất được gửi lại cho subscriber khởi động muộn với timestamp ảnh
gốc để kiểm tra stale không bị qua mặt. Runner khởi tạo planner và đăng ký
topic trước warmup, khóa phát lệnh cho đến khi warmup kết thúc. Cấu hình:
[`monocular_openvins_fused_perception.json`](../config/monocular_openvins_fused_perception.json).
Đây là phép ghép không gian có giới hạn, chưa phải bộ lọc độ sâu theo feature
với bất định như SVO.

Lệnh chạy lại:

```sh
python scripts/run_monocular_sim.py \
  --output results/monocular_research/openvins_fused_gate_05_20260930 \
  --world worlds/iris_monocular_visual_ground_pilot.sdf \
  --config config/monocular_speed_visual_0.5.yaml \
  --perception-config config/monocular_openvins_fused_perception.json \
  --eval-scene config/monocular_scene_visual_ground_pilot.json \
  --goal 12 0 3 --seed 7 --duration 30 \
  --openvins-bridge /path/to/openvins_gz_bridge \
  --openvins-zupt-after-motion
```

[Kết quả](../results/monocular_research/openvins_fused_gate_05_20260930/analysis.json):
`LOCALIZATION_DIVERGED`, không tới đích. Cloud có 42 điểm khi planner bắt đầu;
MPPI phát 5 chu kỳ lệnh rồi `hold-stale` 48 chu kỳ do không có phép triangulation
mới. Tốc độ ngang thực tế p95 = 0,343 m/s, không đạt vận tốc đặt 0,5 m/s.
Sai số VIO p95 = 1,352 m, cực đại = 1,518 m; cổng an toàn ngắt ở 1,5 m.
Không có collision proxy trong quãng ngắn này, khoảng hở tối thiểu 5,76 m,
nhưng đây không phải hoàn thành bài toán tránh vật cản. [Audit cách ly](../results/monocular_research/openvins_fused_gate_05_20260930/openvins_audit.json)
xác nhận không có LiDAR/depth message và pose Gazebo chỉ dùng để chấm điểm,
giới hạn bài thử. ArduPilot SITL vẫn sử dụng cảm biến điều hướng mô phỏng
nội bộ. Các bài kiểm thử bản đồ và perception liên quan: 27 pass.

Vấn đề còn lại là VIO drift và độ sâu mới quá thưa khi chuyển động chậm hoặc
dừng. Mây điểm hiện chỉ biểu diễn vật cản đã quan sát, không xác nhận khoảng
không gian chưa quan sát là trống. Vì vậy chưa xác nhận vận tốc bay tránh vật
cản an toàn nào cho cấu hình RGB + IMU này; không nên thử 5 hay 10 m/s bằng
cách bỏ điều kiện stale/pose.

**Cập nhật sau góp ý mentor:** phát hiện bridge cũ che toàn bộ ảnh OpenVINS
bằng mask `255` và publish orientation Gazebo thay vì orientation của VIO.
Bridge, IMU noise model và keyframe tracker đã sửa; bài VIO-only cải thiện,
nhưng lượt closed-loop mới vẫn dừng do cloud hết hạn. Xem số liệu và lệnh
chạy trong [`MONOCULAR_VIO_MENTOR_FOLLOWUP_VI.md`](MONOCULAR_VIO_MENTOR_FOLLOWUP_VI.md).

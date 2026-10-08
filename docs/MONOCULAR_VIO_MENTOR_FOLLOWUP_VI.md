# Đối chiếu góp ý mentor: kiểm tra VIO trước MPPI

Ngày 30/09/2026. Đây là follow-up cho
[`MENTOR_MONOCULAR_HANDOFF_VI.md`](MENTOR_MONOCULAR_HANDOFF_VI.md).
Các lượt motion-stop dưới đây chỉ chạy OpenVINS, không chạy perception, map hay
planner. Gazebo pose chỉ ra lệnh đoạn dịch ngang khoảng 1 m, chặn vùng bay và
chấm điểm. Sau hai lượt VIO-only đạt, có một lượt closed-loop 0,5 m/s riêng.

## Lỗi tích hợp tìm được và đã sửa

1. `/perception/visual_odometry` trước đây publish position của OpenVINS nhưng
   orientation của IMU Gazebo. Bridge nay biến đổi `q_GtoI` từ OpenVINS sang
   orientation IMU→ENU, rồi publish cả position và orientation từ cùng state
   VIO. Gazebo IMU orientation vẫn chỉ dùng để neo hướng ban đầu và khởi tạo
   hover; không đưa vào pose downstream ở các frame sau.
2. Bridge đã truyền mask ảnh toàn giá trị `255`. Theo [mã TrackKLT của
   OpenVINS](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/TrackKLT.cpp),
   `255` nghĩa là pixel bị che. Do đó các lượt cũ thực chất không có feature
   update trong OpenVINS: drift chủ yếu là tích phân IMU. Mask được đổi thành
   `0` như ví dụ trong [OpenVINS](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/test_webcam.cpp),
   và số feature khởi tạo được đặt rõ là 150. External KLT tracker của
   perception trước đây vẫn báo nhiều match; đó là pipeline khác, không chứng
   minh OpenVINS đang dùng feature.
3. SDF IMU đã khai báo orientation reference `ENU` rõ ràng. Hai lượt đầu sau
   sửa mask vẫn dùng IMU lý tưởng và covariance floor quá nhỏ, khiến gần như
   mọi ZUPT accept qua nhánh disparity dù chi² in ra vượt ngưỡng. SDF hiện
   thêm nhiễu Gaussian từng mẫu ở 100 Hz: gyro std 0,0016968 rad/s, accel
   std 0,02 m/s²; OpenVINS dùng density tương ứng 0,00016968 rad/s/√Hz và
   0,002 m/s²/√Hz. Bias random walk vẫn chỉ là numerical floor, chưa mô phỏng
   bias động. Audit xác nhận giá trị SDF/bridge và timestamp, không thay
   calibration thực địa.
4. Bridge ghi `v`, gyro/accel bias, số active track, số MSCKF feature update,
   cùng `q` published vào CSV. Chế độ debug ghi ZUPT accept/reject, disparity
   và chi² vào `openvins.log`.

## Bài test đứng → dịch ngang 1 m → đứng

Mỗi lượt hover khoảng 6 s, dịch ngang tới khi Gazebo truth đi ~1 m, rồi gửi
vận tốc 0 khoảng 10 s. Phân tích bỏ 1 s đầu của mỗi pha hover để tránh giai
đoạn chuyển tiếp. Tiêu chí đặt trước: drift VIO trong mỗi pha hover <0,2 m,
vận tốc VIO p95 sau dừng <0,1 m/s, sai số độ dài đoạn dịch <0,2 m.

| Lượt | Đoạn dịch VIO / truth | Drift VIO / truth sau dừng | Vận tốc VIO p95 sau dừng | Kết luận |
| --- | ---: | ---: | ---: | --- |
| [Mask lỗi, ZUPT bật](../results/monocular_research/vio_motion_stop_enu_20260930/vio_motion_stop_analysis.json) | 2,343 / 0,977 m | 13,713 / 0,019 m | 2,481 m/s | Fail |
| [Mask sửa, ZUPT bật, lượt 1](../results/monocular_research/vio_motion_stop_unmasked_20260930/vio_motion_stop_analysis.json) | 1,035 / 0,985 m | 0,210 / 0,014 m | 0,035 m/s | Fail sát ngưỡng drift |
| [Mask sửa, ZUPT bật, lượt 2](../results/monocular_research/vio_motion_stop_unmasked_repeat_20260930/vio_motion_stop_analysis.json) | 1,002 / 0,998 m | 0,101 / 0,019 m | 0,017 m/s | Pass lượt này |
| [Mask sửa, không cho ZUPT sau motion](../results/monocular_research/vio_motion_stop_no_post_zupt_20260930/vio_motion_stop_analysis.json) | Filter diverge | Filter diverge | Filter diverge | Fail; không dùng như ước lượng vật lý |
| [IMU noise khớp, lượt 1](../results/monocular_research/vio_motion_stop_noisy_20260930/vio_motion_stop_analysis.json) | 0,923 / 1,005 m | 0,122 / 0,016 m | 0,016 m/s | Pass |
| [IMU noise khớp, lượt 2](../results/monocular_research/vio_motion_stop_noisy_repeat_20260930/vio_motion_stop_analysis.json) | 0,928 / 0,984 m | 0,064 / 0,015 m | 0,012 m/s | Pass |

Hai lượt mask sửa nhưng IMU lý tưởng cho **1 pass / 1 fail**. Hai lượt có noise
model khớp cho **2 pass / 2 lượt**, nhưng vẫn chỉ là một kiểu chuyển động ở
khoảng 0,25 m/s, không phải bằng chứng bay tránh vật cản an toàn. Sai số orientation
VIO p95 khi hover sau dừng lần lượt ~0,30° và ~0,24°. Sai số vị trí tuyệt đối
bao gồm khoảng 0,2 m bias cao độ vì mốc VIO được khai báo 3 m trong khi tâm
vehicle ở hover của Gazebo ~3,2 m; tiêu chí drift ở trên dùng **thay đổi vị
trí trong từng pha**, không dùng bias tuyệt đối đó. Trong lượt mask sửa thứ
hai, MSCKF dùng 122 feature update ở pha dịch và ZUPT có 196 lần accept;
trước khi sửa mask, MSCKF không cập nhật feature và pose trôi nhiều mét.

Lượt không cho ZUPT sau motion có divergence phi vật lý từ pha hover trong
cấu hình IMU lý tưởng cũ. Log cho thấy nhánh disparity có thể accept ZUPT dù
chi² in ra vượt ngưỡng; với noise model khớp, số lần này giảm còn 3/263 và
5/274 ZUPT accept trong hai lượt. Vẫn cần kiểm tra nguyên nhân các override đó.
Nó không chứng minh estimator sẽ ổn định ở cấu hình thật.

Lệnh tái lập bài test:

```sh
python scripts/run_monocular_sim.py \
  --output results/monocular_research/vio_motion_stop_new \
  --world worlds/iris_monocular_visual_ground_pilot.sdf \
  --config config/monocular_speed_visual_0.5.yaml \
  --perception-config config/monocular_openvins_fused_perception.json \
  --eval-scene config/monocular_scene_visual_ground_pilot.json \
  --openvins-bridge /path/to/openvins_gz_bridge \
  --openvins-zupt-after-motion --vio-only-motion-stop
python scripts/analyze_vio_motion_stop.py \
  results/monocular_research/vio_motion_stop_new
```

Sau hai lượt VIO-only có noise khớp đạt, tracker được sửa để chỉ thay keyframe
khi có ít nhất 12 điểm triangulation hợp lệ. [Lượt closed-loop 0,5 m/s](../results/monocular_research/openvins_corrected_closed_loop_05_20260930/analysis.json)
không diverge pose: sai số VIO p95 0,512 m, max 0,530 m. Cloud có 67 điểm lúc
planner bắt đầu; planner phát 9 chu kỳ lệnh rồi `hold-stale` 141 chu kỳ và
`TIMEOUT` ở x≈2,4 m, chưa tới vật cản hay goal. Tốc độ ngang thực tế p95
0,315 m/s, không đạt mức đặt 0,5 m/s. Không có LiDAR/depth message;
[audit cách ly](../results/monocular_research/openvins_corrected_closed_loop_05_20260930/openvins_audit.json)
đạt. Sau khi cloud hết hạn, log có 287 frame `low-baseline` liên tiếp: UAV
đang hold nên không tạo thị sai mới. Đây là vòng kẹt active perception:
không đủ bằng chứng free-space để planner tiến, nhưng đứng yên cũng không
triangulate được độ sâu mới.

Giữ `min_obstacle_points=12`, stale timeout và pose safety gate. Lượt này
không chứng minh tốc độ 0,5 m/s là an toàn. Bước kế tiếp nên ưu tiên
multi-frame inverse-depth với uncertainty, quan sát/free-space raycasting và
chính sách active perception có giới hạn an toàn; chưa thử 5–10 m/s.

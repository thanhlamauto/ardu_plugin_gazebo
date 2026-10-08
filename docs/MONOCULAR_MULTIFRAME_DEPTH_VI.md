# Monocular multi-frame depth: mốc perception độc lập

Ngày 30/09/2026. Mục tiêu của lượt này là giữ landmark qua nhiều ảnh và đo
inverse-depth cùng uncertainty trước khi nối dữ liệu sang map/MPPI. Bộ thử
[`monocular_multiframe_gz.py`](../scripts/monocular_multiframe_gz.py) chỉ đọc
RGB và pose từ OpenVINS (RGB + IMU + mốc cao độ ban đầu 3 m). Nó **không**
publish obstacle cloud, không khởi động planner và không đọc Gazebo odometry,
LiDAR hoặc depth camera. Gazebo truth và kích thước hộp chỉ có trong harness
điều khiển đoạn bay và bộ chấm điểm offline.

## Triển khai

[`monocular_multiframe_depth.py`](../mppi_ardupilot/monocular_multiframe_depth.py)
giữ ID/track KLT qua nhiều frame. Mỗi quan sát gồm pixel, tâm camera, tia nhìn
trong ENU và rotation của pose VIO. Track cần ít nhất ba vị trí camera, có
parallax ít nhất 1°, reprojection error mỗi view không quá 1,5 px và độ sâu
trong khoảng 0,3–20 m. Các pose dùng làm ràng buộc hình học cách nhau tối
thiểu 0,12 m; quan sát neo đầu được giữ lại khi cửa sổ tối đa 16 quan sát
đầy. Vì thế các ảnh lặp khi hover không được tính thành nhiều bằng chứng độc
lập.

Điểm 3D được fit bằng giao nhiều tia theo least-squares, rồi lưu dưới dạng
inverse depth theo ray neo. `depth_sigma_m` xấp xỉ từ nhiễu pixel giả định
0,7 px và sai số pose 0,15 m; landmark chỉ được đánh dấu đủ điều kiện khi
sigma ≤0,75 m và sigma/depth ≤0,25. Đây là **heuristic có điều kiện trên pose
VIO**, chưa phải covariance đã hiệu chuẩn và chưa phải Bayesian inverse-depth
filter. Nếu VIO sai tỉ lệ dịch chuyển, cả độ sâu lẫn uncertainty có thể lệch.

## Bài thử và cách chấm

Gazebo bay hover 6 s → dịch ngang ở lệnh 0,25 m/s → hover 10 s; khoảng dịch
thử 1 m và 1,5 m. World có một hộp ở x=8 m, kích thước 2×3×6 m nên mặt
trước ở x=7 m. Bộ
[`analyze_monocular_multiframe_depth.py`](../scripts/analyze_monocular_multiframe_depth.py)
dùng **pixel + camera pose Gazebo chỉ sau khi bay xong** để tìm các tia thật
gặp mặt hộp. Nó lấy estimate cuối của mỗi feature ID, tránh đếm cùng track
nhiều lần. Sai số mặt trước là `abs(x_est - 7)`. Sai số 3D còn chứa bias
cao độ ban đầu khoảng 0,2 m từ mốc 3 m so với hover thật ~3,2 m.

Các lượt thăm dò ban đầu cho thấy cửa sổ trượt bỏ quan sát neo làm mất
baseline, còn uncertainty dùng pose floor 0,1 m có trường hợp undercoverage:

| Lượt thăm dò | Khoảng dịch | Landmark mặt hộp | Sai số x trung vị / p95 | Kết quả |
| --- | ---: | ---: | ---: | --- |
| [Bản đầu](../results/monocular_research/multiframe_depth_probe_20260930/multiframe_depth_analysis.json) | 1 m | 239 | 0,057 / 0,159 m | Một lượt tốt, không ổn định giữa các lần chạy |
| [Lặp bản đầu](../results/monocular_research/multiframe_depth_probe_seed11_20260930/multiframe_depth_analysis.json) | 1 m | 220 | 0,570 / 0,707 m | Baseline estimate cuối co xuống 0,284 m |
| [Giữ anchor, floor 0,1 m](../results/monocular_research/multiframe_step12_distance15_20260930/multiframe_depth_analysis.json) | 1,5 m | 158 | 0,512 / 0,574 m | Chỉ 31,6% sai số dọc ray trong ±1 sigma |

Lượt xác nhận cuối với pose floor **0,15 m** và trần sigma **0,75 m**:

| Lượt | Khoảng dịch | Depth xuất hiện sau | Landmark mặt hộp | Sai số x trung vị / p95 | Tỷ lệ sai số dọc ray trong ±1 sigma |
| --- | ---: | ---: | ---: | ---: | ---: |
| [1,5 m](../results/monocular_research/multiframe_posefloor15_distance15_20260930/multiframe_depth_analysis.json) | 1,5 m | 1,465 m | 157 | 0,669 / 0,704 m | 24,8% |

Kết quả này **không đạt gate uncertainty**. VIO đo đoạn dịch 1,351 m khi truth
là 1,490 m, thấp hơn ~9,3%. Sai số scale tương ứng gần đúng 0,65 m ở mặt
hộp 7 m, phù hợp với sai số depth quan sát. Tăng một pose-noise floor duy nhất
không giải quyết được sai số scale tương quan qua toàn bộ track.

`--seed` trong harness hiện chỉ áp dụng cho planner; vì planner không chạy ở
bài VIO-only nên các nhãn seed của hai lượt đầu **không phải seed Gazebo được
kiểm soát**. Các lượt là các repetition với nhiễu IMU/Gazebo thay đổi. Không
được dùng chúng như chứng minh thống kê qua nhiều seed.

Với cấu hình hiện tại, lượt 1 m
[`multiframe_step12_seed7_20260930`](../results/monocular_research/multiframe_step12_seed7_20260930/multiframe_depth_analysis.json)
có 236 landmark trên mặt hộp, sai số x trung vị 0,436 m và p95 0,487 m,
nhưng depth đầu tiên chỉ xuất hiện sau khi đã dịch ~1,015 m, ngay ở pha dừng.
Image age p95 ~19 ms và thời gian từ callback tới xong xử lý p95 ~39 ms;
đây chưa gồm toàn bộ độ trễ điều khiển/hành động. Trước dịch không có depth,
đúng với bài toán monocular tĩnh.

## Gate và giới hạn hiện tại

- **A — VIO-only motion-stop:** pass cho các lượt hoàn tất; xem
  [`analyze_vio_motion_stop.py`](../scripts/analyze_vio_motion_stop.py) và
  analysis JSON trong từng thư mục kết quả.
- **B — landmark depth + uncertainty:** **chưa pass** vì độ sâu phụ thuộc
  scale VIO và sigma undercoverage. Hai lượt 1,5 m đo VIO dịch 1,614/1,351 m
  khi truth đều ~1,490 m, tức scale lệch khoảng +8,3% / −9,3%; chiều lệch
  depth cũng đổi theo. Một lượt giữ anchor trước đó gặp trễ evaluator
  odometry >1 s và bị loại khỏi gate.
- **C–F — free/occupied/unknown khi hover, recovery, closed-loop 0,5 m/s và
  1 m/s:** chưa chạy với tracker mới. `monocular_geometry_gz.py` vẫn dùng
  tracker cũ. Không dùng kết quả ở đây để kết luận vận tốc bay an toàn.

Việc tiếp theo là truyền uncertainty pose/scale đáng tin cậy từ VIO sang
landmark estimator, kiểm tra tương ứng feature và độ phủ sai số trên nhiều
world/run, rồi mới nối sang raycast free/occupied/unknown. Active perception
0,2–0,3 m chưa được chấp nhận: với gate này độ sâu đầu tiên cần khoảng 1 m
baseline trong cảnh hộp 7 m. Mọi dịch chuyển để lấy parallax sau này phải
được giới hạn trong free-space đã xác nhận.

Chạy lại một lượt 1,5 m:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_monocular_sim.py \
  --output results/monocular_research/my_multiframe_trial \
  --world worlds/iris_monocular_visual_ground_pilot.sdf \
  --config config/monocular_speed_visual_0.5.yaml \
  --perception-config config/monocular_openvins_fused_perception.json \
  --eval-scene config/monocular_scene_visual_ground_pilot.json \
  --goal 12 0 3 --duration 30 \
  --openvins-bridge /tmp/openvins_gz_bridge_build/openvins_gz_bridge \
  --openvins-zupt-after-motion --vio-only-motion-stop \
  --vio-only-translation-m 1.5 --multiframe-depth-probe
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/analyze_monocular_multiframe_depth.py \
  results/monocular_research/my_multiframe_trial
```

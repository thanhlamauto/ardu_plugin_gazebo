# Nghiên cứu và thử nghiệm monocular obstacle avoidance

## Điều kiện đánh giá đã chốt trước vòng cải tiến flight

Không có LiDAR/depth sensor trong vehicle; RGB camera duy nhất làm input obstacle.
Pose/IMU/GPS là subsystem trạng thái riêng. Không nạp geometry SDF/A* làm obstacle
cho planner, không fit prediction theo dense GT trong flight. GT depth và geometry
vật cản chỉ chấm điểm. Pose lý tưởng từ Gazebo được cấp riêng cho trạng thái bay
và triangulation; đây là một giả định rõ ràng của mô phỏng.

Mục tiêu chấp nhận cho demo mô phỏng: bay quanh vật cản tới goal (sai số <=0.5 m),
không vi phạm proxy footprint 0.5 m, tâm UAV cách bề mặt obstacle ít nhất 0.75 m;
kiểm tra lặp lại nhiều seed và ít nhất hai bố trí obstacle. Mốc đầu là >=80% goal
success trong 10 lượt ở vận tốc đặt 0.5–1 m/s. Thời gian tối đa/RTF phải công bố;
không tính đứng yên không va chạm là thành công. Kiểm tra mất camera phải đưa
về hold. Đây là functional simulation acceptance, không xác nhận UAV thật hay
vận tốc 5/10 m/s trong survey. Chưa đạt các điều kiện này thì goal nghiên cứu vẫn mở.

## Các công trình đã đọc và setup đáng chuyển sang repo

**MonoNav (Simon & Majumdar, ISER 2023).** RGB + onboard pose estimation,
offboard ZoeDepth, hiệu chỉnh ảnh về intrinsics Kinect, TSDF fusion; tìm motion
primitive gần goal nhất với khoảng cách tới voxel bị chiếm >=c. Bay Crazyflie
0.5 m/s; depth/map 3–4 Hz, replan 1 Hz, c=0.5 m trong phần hardware demonstration.
Trong 15 lượt ở 10 môi trường, một crash và một lượt dừng sớm do vùng bị che khuất.
Comparison riêng 15 lượt mỗi phương pháp ở 5 môi trường: MonoNav collision rate
0.13, tiến độ tới goal 47.4%; NoMaD 0.53 và 61.0%. **47.4% là progress, không
phải goal success rate.** [Paper, pp.6–9](https://www.alphaxiv.org/pdf/2311.14100?page=6)

![Bảng gốc MonoNav](../results/monocular_research/literature/mononav_table2.png)

Bài học chuyển sang đây: calibration, warmup inference, map theo acquisition
pose, lọc floor/ceiling và voxels yếu, kiểm tra primitive thay vì phạt repulsion
quá lớn. Repo cảnh báo planner hiện vẫn coi unobserved là free; không sao chép
điểm đó thành một chứng nhận an toàn. [Repo](https://github.com/natesimon/MonoNav)

**Obstacle Avoidance Using a Monocular Camera (2012.01608).** AirSim, depth
network + RL policy + collision predictor + contingency policy. Obstacle course
100 m với sáu xe, vị trí ngang random; 1000 evaluation episodes. Hybrid expert:
completion 94.9%, collision 1.8%; hybrid A*: 84.2%, 5.5%; RL đơn: 81.9%, 17.2%.
Các mạng cần training và không thể dùng các con số đó như kết quả của Gazebo này.
Bài học: tách progress policy khỏi collision/safety và đánh giá cả completion,
crash, timeout. [Paper, Table 6](https://www.alphaxiv.org/pdf/2012.01608)

**MonoMPC (2508.07387).** Depth Anything V2 + learned distribution of minimum
clearance, risk-aware MPC; real hardware benchmarking trên Jackal RGB-only,
69° HFOV, RTX3080/TensorRT. LiDAR chỉ visualization/evaluation. Learned collision
model train khoảng 6M samples, RTX5090 ~2.5 h. Paper có phần quadcopter comparison:
40 experiments; ở setting vừa, MonoNav 20% collision/80% stuck, method đề xuất
0%/0%. Không trộn kết quả Jackal, quadcopter và hardware thành một success rate.
Bài học: raw depth map có thể vừa tạo false obstacles vừa tạo false free space;
không sửa bằng giảm margin một cách tùy tiện. [Paper, Tables III–IV](https://www.alphaxiv.org/pdf/2508.07387)

**ArduMonoNav (repo cộng đồng, không phải benchmark paper độc lập).** Fork
MonoNav sang ArduPilot, Depth Anything V2 metric Hypersim, input252, TSDF,
primitive 0.5 m/s. Đây là nguồn setup sát stack hiện tại, không phải chứng cứ
success rate định lượng. README thừa nhận scale mismatch và BendyRuler chưa
hoạt động; do đó không dùng BendyRuler để giả lập một kết quả MPPI.
[Repo/config](https://github.com/GauthamMPrakash/ArduMonoNav)

Nguồn khác đã xem: Mono-Navigation dùng ORB-SLAM2 + Fast-Planner, nhấn mạnh depth
accuracy lẫn scale consistency; ROS Melodic/RTX2070. Chưa đọc được số định lượng
trong paper của họ nên không ghi số từ video/README.
[Repo](https://github.com/YongzhouPan/Mono-Navigation)

## Vòng perception đầu tiên

Cùng RGB arena/GT 320×180, không align scale; pinned revisions, MPS, 12 lần inference,
latency warm bỏ 2 lần đầu. Đây chỉ là một scene để chọn hướng, không đại diện aerial.

| Node | RMSE m | AbsRel | δ1 | warm p95 ms |
|---|---:|---:|---:|---:|
| Outdoor Small, input518 | 6.512 | 3.259 | 0.0163 | 60.65 |
| Indoor Small, input518 | 1.395 | 0.688 | 0.2221 | 60.23 |
| Indoor Small, input252 | 1.958 | 1.001 | 0.2181 | 44.54 |

Promote indoor518 để thử flight; 252 chưa đáng đổi accuracy lấy 16 ms. Indoor
checkpoint được fine-tune Hypersim, outdoor trên Virtual KITTI; việc scene Gazebo
hiện tại gần indoor metric range hơn là giả thuyết, không phải kết luận domain
chung. [Model card](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf)

Run ids/evidence logs:

- Outdoor: ec0612d5-301a-41af-8d0e-35de7fd6a327
- Indoor518: c3544db6-b0c3-407b-8e3e-ac725ba2eeb6
- Indoor252: 7d3d3d6c-d774-4702-acd7-dc401f577438

Project OpenResearch local: `45d0c18b-a3e7-4d1e-83d8-7cf3a9353a6c`, private local
snapshot repo `~/Projects/uav_monocular_research`, không publish GitHub. Mỗi
experiment branch/run là snapshot immutable. Baseline cũ giữ nguyên trong
`results/monocular_only_20260927_r3`. Nguồn retrieval/full texts/revisions nằm ở
`results/monocular_research/literature/`.

## Vòng giảm tải và kiểm tra metric scale

Các run dưới đây vẫn có 0 LiDAR/depth-camera messages và không cấp SDF obstacle
map cho controller. Cấu hình inference/controller lưu cùng evidence.

- `dbda853c-2d97-467e-b343-0fcac0959358`, commit `a50da7a`: depth518 + semantic256,
  inference2 Hz, RGB10 Hz, bỏ camera gimbal không dùng khỏi snapshot nghiên cứu.
  TIMEOUT sau 179.98 wall / 87.41 sim s, nhận345 clouds, vị trí cuối
  (3.401,-4.399,3.191) m, goal distance9.661 m. Clearance tâm nhỏ nhất4.368 m,
  độ cao3.184–3.201 m. Camera publish p95 khi control836.04 ms; 79 stale-odom
  polls, 68 hold-stale, 60 hold-timeout, 182 stopping-validation holds. Có tiến
  triển nhưng chưa đạt acceptance. Không coi TIMEOUT không collision là success.
- `6207aecb-addf-4447-a291-7f785b8a027d`, commit `3ad8d04`: thay MPS bằng CPU,
  giữ các tham số khác. SETUP_FAILED: không qua hover/camera-ready, 0 clouds
  published vì inference quá cũ; không có closed-loop controller để so success.
- Compute-only `c64ce2a1-549b-4824-b0af-74e759bcc07d`, commit `26c0b78`:
  không có Gazebo, semantic256, 8 iterations/device/size, bỏ2 warmup. CPU depth518
  warm p95399.76 ms, MPS78.88 ms. CPU252263.19 ms, MPS61.71 ms. Đây là latency
  không tải renderer; không dùng thay cho số flight latency.

Hướng tiếp theo là triangulation từ ảnh của cùng camera ở các thời điểm khác
nhau và pose có metric scale. Module `monocular_triangulation.py` kiểm tra
parallax/reprojection/range; `monocular_scale_tracker.py` dùng forward-backward
LK, loại thiếu texture và scale thiếu nhất quán. Không dùng dense GT để fit.
Hai module chưa tích hợp vào flight; unit tests chứng minh geometry trên case
synthetic có ground-truth để kiểm tra, không chứng minh hiệu quả navigation.

Diagnostic recorded flight `6528cdf3-2298-4dc2-af30-d6cb6d81462b`, commit `5d461ab`:
13 saved RGB/depth pairs matched với pose acquisition lệch<=60 ms. Không có
scale được chấp nhận: các snapshot baseline cũ cách nhau quá xa (3–11 m).
Vì vậy cần capture ảnh và pose đồng bộ dày hơn; không nới rejection gate để
tạo scale giả. Ground-truth log mới bổ sung quaternion để phục vụ tái kiểm tra.

Artifacts: `results/monocular_research/reduced_rendering`, `cpu_flight`,
`compute_metrics.json`, `temporal_metrics.json`. 49 MPPI core tests pass;
10 monocular cloud/map tests pass; 4 temporal geometry/scale tests pass.
Ở cuối vòng này chưa có lượt GOAL_REACHED và chưa đạt điều kiện 8/10 trên hai layouts.

## Lượt đạt đích đầu tiên và batch acceptance ban đầu

Run `3cf850d6-24cc-4785-84e8-04665dd98a16`, commit `154da0a`, đã GOAL_REACHED.
Depth trong nhánh này được triangulate từ nhiều ảnh của **cùng một RGB camera**,
dùng intrinsics thật và pose tại thời điểm chụp để có metric scale. Không chạy
Depth Anything trong controller này, không dùng SDF/range để sửa depth khi bay.
Motion proposals angular/side-then-forward được thêm opt-in vào MPPI, lấy cảm
hứng từ primitive selection của MonoNav; không đổi collision/stopping gates.

Cấu hình: RGB640×360 HFOV80°, RGB10 Hz, geometry10 Hz, occupied voxels0.2 m,
ít nhất2 observations, memory60 simulation seconds, altitude band3±0.65 m.
Warmup dịch ngang khoảng0.56 m ở đầu course để tạo parallax. UAV cần textured
static obstacles và scaled odometry; đây chưa phải camera-only localization.
MPPI5 Hz, dt0.2, horizon50, samples160, vmax0.5 m/s, vzmax0, collision radius1.25 m,
terminal progress, feasible-sample weighting, final/stopping guards bật.

| Metric từ raw ground-truth control trace | Kết quả |
|---|---:|
| Goal error | 0.4869 m |
| Min center clearance | 1.4211 m |
| Altitude min–max | 3.1798–3.1905 m |
| Control wall / simulation | 109.18 / 92.75 s |
| Camera p95 receive→publish khi control | 90.97 ms |
| LiDAR / depth-camera messages | 0 / 0 |
| Camera clouds | 958 |

231/420 cycles hold-timeout cho thấy latency planner còn là hạn chế. Batch runner
không import các model PyTorch không dùng ở process điều phối; cần báo số runtime
riêng của batch, không trộn với lượt pilot.

Các lần thử material mesh trắng `ae8040c8...` và `789337ad...` không đạt setup
texture và không được tính success. Lượt geometry native-SDF `2aca9242...`
TIMEOUT; min clearance0.5818 m không đạt0.75 m, nên đã tăng allowance sai số và
đổi proposal/progress setup trong child. Negative evidence được giữ nguyên.

Batch đầu đã hoàn tất: experiment `1908b1aa-ac13-4363-88f3-20d444333270`, run
`82e67c1b-076f-434e-b9b5-35a8fc2a6729`, commit `0721d9c`. Khai báo trước10 lượt
seed7–11 trên crate center(8,0,3) và(8,1,3), cộng1 lượt camera-blackout5 s.
Không retry/loại lượt lỗi khỏi denominator. Acceptance cần>=8/10 cùng clearance
>=0.75 m, goal error<=0.5 m, 0 range-sensor messages và blackout hold-zero rồi
resume đạt đích. World cloud còn kiểm tra acquisition timestamp để reject replay.

Source và raw logs của các run terminal được giữ trong
`~/.local/share/openresearch/local-runs/<run-id>/source-and-artifacts.tar.gz`.
Các log batch sau mỗi lượt được gzip lossless, có SHA256 trước/sau kiểm tra.
Không archive hay sửa run đang chạy. Artifact pilot dễ xem:
`results/monocular_research/first_goal/trajectory.png`, `analysis.json`.

Cấu hình trong project: `config/monocular_geometry_mppi.yaml`,
`config/monocular_geometry_perception.json`; kết quả acceptance mới ở cuối tài liệu.
Model main dùng passive gimbal hierarchy riêng để chỉ còn1 camera vật lý;
model gimbal gốc giữ nguyên. Replay main hierarchy seed13 đã đạt;
chi tiết audit ở cuối tài liệu.

```bash
/tmp/uav-monocular-env/bin/python scripts/run_monocular_sim.py \
  --world worlds/iris_monocular_textured.sdf \
  --config config/monocular_geometry_mppi.yaml \
  --perception-config config/monocular_geometry_perception.json \
  --duration 180 --output results/monocular_geometry_new
```

Đây là Python MPPI/Gazebo/MAVLink route đã có trong repo. C++ ROS launch optional
trong docs baseline là controller khác và chưa được dùng để hỗ trợ con số này.

## Audit batch đầu và vòng tối ưu CPU

Audit độc lập đủ11 lượt xác nhận7/10 điều hướng đạt, nên chưa đạt acceptance8/10. Ba TIMEOUT là centered seed9/10 và shifted seed11; không bỏ khỏi mẫu. Tất cả lượt đều dùng một RGB camera, không range messages hay map prior. Lượt blackout đạt đích,19 chu kỳ hold-stale với velocity0; clearance1.4875 m. Bảng và audit nằm ở `results/monocular_research/acceptance_initial/`.

Child `e98786d4-8c21-40fb-a797-3f6339401372` thử chunk16 cho swept safety để giảm temporary memory, giữ160 samples và mọi threshold. Pilot seed9 đạt trước khi chạy lại toàn bộ ma trận. Unit test boolean safety masks tương đương chunk1/16/32/256 trên trajectories an toàn và không an toàn;49 core tests đạt. Kết quả acceptance mới được ghi ở phần tiếp theo.

## Kết quả acceptance cuối: 9/10 + blackout đạt

Experiment `a172c798-2451-45db-8e98-edbbcaf80af9`, run `03e1e075-d457-458f-8af7-bbe4c2e73ade`, commit `761e2efd81a26ba7a6be6a329ad305a8b3d20aa3`. Giữ nguyên ma trận seed7–11 và hai layout,180 s tối đa/lượt; không retry hoặc bỏ lượt lỗi. Model đã dùng dedicated passive gimbal hierarchy của main project, chỉ1 RGB camera.

| Lượt | Kết quả | Clearance tâm–bề mặt (m) | Sai số goal (m) | Điều khiển wall (s) |
|---|---|---:|---:|---:|
| centered_seed7 | GOAL_REACHED | 1.414 | 0.499 | 35.440 |
| centered_seed8 | GOAL_REACHED | 1.448 | 0.495 | 37.548 |
| centered_seed9 | GOAL_REACHED | 1.433 | 0.493 | 35.058 |
| centered_seed10 | GOAL_REACHED | 1.417 | 0.496 | 36.800 |
| centered_seed11 | INFRA_FAILURE | — | — | — |
| shifted_seed7 | GOAL_REACHED | 1.291 | 0.497 | 107.307 |
| shifted_seed8 | GOAL_REACHED | 1.201 | 0.499 | 40.711 |
| shifted_seed9 | GOAL_REACHED | 1.285 | 0.486 | 42.262 |
| shifted_seed10 | GOAL_REACHED | 1.338 | 0.495 | 104.934 |
| shifted_seed11 | GOAL_REACHED | 1.287 | 0.483 | 41.625 |
| camera_blackout_seed12 | GOAL_REACHED | 1.478 | 0.494 | 52.330 |

9/10 lượt điều hướng đạt tất cả tiêu chí. Centered seed11 mất odometry >15 s và được tính thất bại; không loại khỏi denominator. Audit không gán clearance/success cho flight lỗi thiếu endpoint. Các lượt đạt có clearance nhỏ nhất1.2007 m, goal error tối đa0.4985 m, thời gian điều khiển35.06–107.31 s.

Blackout thực tế5.028 s:19 chu kỳ trong khoảng sau stale deadline đều hold-stale và velocity0; camera trở lại thì đạt goal. Đây là zero **command**, không phải chứng nhận không drift vật lý. Audit kiểm tra tất cả chu kỳ trong khoảng, không chỉ chọn các chu kỳ hold.

Audit độc lập source SHA256, physical include hierarchy, command arguments, parameter readback, gzip hashes và dense GT xác nhận:0 LiDAR/depth messages, không obstacle-map prior/GT-depth input, final/stopping/sample guards bật. GT ở các lượt đạt có max simulation timestamp gap0.034 s. Topic cloud là `/perception/obstacles_camera`; tên tham số cũ `lidar_topic` chỉ là interface cloud, không phải sensor LiDAR.

Artifact review: `results/monocular_research/acceptance_final/{summary,independent_audit,plan,run_metadata}.json`; mỗi trial có result/analysis/manifest/trajectory. Raw logs và exact source giữ trong archive ở run directory. Một số archive vòng cũ được chuyển sang `/Volumes/Extreme SSD/uav_monocular_research_archives/` vì ổ trong hết chỗ; path archive gốc là symlink tới bản đã kiểm tra SHA256. Archive đó cần SSD được mount.

Lượt integration riêng seed13: experiment `5e50ad27-d906-4d0f-abe7-a85037f32884`, run `ebbe7b22-2991-40dc-ab26-63cfc85c544d`, commit `a1b2bd4`. GOAL_REACHED, dense clearance1.4784 m, goal error0.4922 m,36.18 s điều khiển,172 command cycles và0 deadline misses. Audit xác nhận source/model đang dùng khớp main byte-for-byte và tham số YAML tương đương; chỉ1 camera vật lý,0 range messages. Artifact `results/monocular_research/integration_seed13/` có result/analysis/manifest/source_snapshot/trajectory và independent_audit. Đạt toàn bộ điều kiện demo đã chốt, goal nghiên cứu được hoàn tất.

Các checks đã chạy:49 MPPI core tests và2 tests safety chunking đều đạt; boolean safe masks giống nhau ở chunk1/16/32/256. Các tests geometry/perception trước đó giữ nguyên. Native plugin không đổi; ROS C++ launch tùy chọn chưa được build/chạy và không hỗ trợ con số nghiệm thu này.

Đây là acceptance cho **demo static textured box, scaled ideal Gazebo pose, tốc độ0.5 m/s và altitude~3 m**. Chưa có chứng cứ cho vật ít texture/mảnh/chuyển động, drift VIO/GPS, vùng chưa quan sát hay bay thật. Các lần pause/timeout cho thấy vẫn phụ thuộc tải host. Không dùng9/10 như benchmark tổng quát hoặc thay thế các kết quả hardware trong paper.

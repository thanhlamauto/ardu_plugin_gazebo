# Tăng tốc và mở rộng tình huống monocular

**Cập nhật 5/10 m/s:** xem [kết quả 6 lượt mới](MONOCULAR_SPEED_5_10_VI.md). Tốc độ đặt 10 m/s chưa đồng nghĩa UAV bay 10 m/s ổn định; một lượt đạt 5 m/s thực đã COLLISION_PROXY. Ma trận 54 lượt cũ bị ngắt sau 44 lượt có result, PID không còn và OpenResearch vẫn ghi trạng thái `running` cũ; không coi đó là thí nghiệm hoàn tất.

Mục tiêu: tăng tốc từ baseline0.5m/s và thử nhiều tình huống hơn bằng **một RGB camera, không LiDAR/depth camera**. Baseline trước đạt9/10 trên2 layout, có blackout. Gazebo odometry vẫn cung cấp pose có scale cho state và triangulation; chưa phải định vị chỉ từ camera. Không truyền obstacle/world/GT-depth vào perception hoặc planner.

## Ma trận và tiêu chí

54 lượt: mức đặt0.5/1.0/1.5m/s ×6 tình huống ×seed17/23/31. Khối giữa, khối lệch, hai khối so le, khe3.4m, đích chéo(14,4,3), khối ít texture. Gate chỉ đạt task khi thật sự qua khe. Giữ mọi thất bại trong mẫu số, không retry hoặc loại low-texture.

180s/lượt điều khiển; clearance tâm phương tiện–bề mặt mọi box≥0.75m; sai số goal≤0.5m; altitude3±1m;0 range messages; giữ final/swept/stopping/sample safety guards. Khoảng cách không phải telemetry tiếp xúc. Unknown space chưa được chứng minh trống.

Vmax giới hạn từng trụcXY, không phải norm; vận tốc thực p50/p95/p99 đo từ dense GT theo thời gian mô phỏng. Báo wall/sim time vàRTF, không dùng tên config để tuyên bố vận tốc thực. World geometry chỉ dùng evaluator, độc lập với planner. Auditor kiểm tra mọi collision box, goal CLI/seed, source/sensor hierarchy, raw/gzip hash và dense trace; gate navigation/task tách riêng.

## Evidence đã kết thúc

| Cấu hình | Thử nghiệm | Kết quả |
|---|---|---|
|160 samples,horizon50,dt0.2 |18 lượt,seed17 |0.5:5/6;1.0:5/6;1.5:1/6 |
|80 samples,horizon50,dt0.2 |centered1.5,seed17 |TIMEOUT180s,goal error5.4644m,p95 thực0.7846m/s |
|80 samples,horizon30,dt0.2 |centered1.5,seed17 pilot |GOAL22.25s,clearance1.8456m,p95 thực1.2827m/s,p99=1.5901m/s |

Screening18: experiment `e354cb55-1eec-4a80-83d7-53a8dcdc7cb3`, commit `0df86ce`, run `1c23b22a-f3a6-4c73-bf6c-ccea38d602b2`. Tất cả5 tình huống có texture đạt ở0.5/1.0; low-texture SETUP_FAILED cả3 mức, không đủ geometry để bắt đầu. Cả3 gate qua khe. Kết quả chỉ từ1 seed. [Báo cáo và biểu đồ](../results/monocular_research/speed_expansion/screening_report/REPORT_VI.md).

Pilot80/h50: experiment `c9b1dda5-9e24-47fd-bfbd-177d75d673ff`, commit `2e92f3a`, run `0f6300a2-c9a1-4a10-bbe9-bdf3ac10a23b`. [Artifact thất bại](../results/monocular_research/speed_expansion/sample80_horizon10_negative/summary.json). Không promote cấu hình này.

Pilot80/h30: experiment `4f10ccb7-de05-4c05-b279-30a64fffd30e`, commit `087994f`, run `32c32359-8e49-4430-9360-022cb817a0fd`. Independent audit đạt tất cả predicates,103commands0deadline misses,1RGB0range. [Audit pilot](../results/monocular_research/speed_expansion/horizon6_pilot/independent_audit.json). Một pilot chưa chứng minh reliability.

Raw/source của screening18 và pilot80/h50 đã archive, kiểm tra hash từng file, tại `/Volumes/Extreme SSD/uav_monocular_research_archives/<run-id>.tar.gz`; cần mount SSD để xem raw logs. Các nguồn immutable/failed outcomes giữ nguyên.

## Full54 bị ngắt sau 44/54 lượt có result

Experiment `87e400b2-0e34-49a0-8595-4830667567a8`, commit `13e854f`, run `ea6b82ca-9ee9-458b-8e57-4f077928d9c4`. Fixed80samples,horizon30,dt0.2 cho **mọi** lượt; không trộn với kết quả160/h50 hay80/h50.

Tại thời điểm run bị ngắt, **44/54** lượt đã có result; trong số đó 0.5:13/15, 1.0:13/15, 1.5:12/14 đạt tiêu chí runner. Đây chưa phải audit độc lập toàn ma trận. PID83171 không còn và không có exit_code, nguyên nhân ngắt chưa xác định. Không tự suy diễn 10 lượt thiếu là thành công hoặc thất bại. [Progress cũ](../results/monocular_research/speed_expansion/progress.json) không phải trạng thái cuối.

Live artifacts: `~/.local/share/openresearch/local-runs/ea6b82ca-9ee9-458b-8e57-4f077928d9c4/repo/results/speed_screening/`. Không restart live run vì observation timeout. Main configs vẫn160/h50; chưa promote candidate. Planner/perception/model main và immutable child khớp byte-for-byte; thay đổi candidate chỉ config samples/horizon.

## Việc còn lại trước khi kết luận

1. Nếu cần kết luận cho mức 0.5/1.0/1.5, phải lập node riêng cho lượt thiếu, giữ rõ lượt bị ngắt và audit đủ raw/source; không sửa kết quả run cũ.
2. Chạy8 fresh-seed blackout trials: centered/shifted ×seed41/43 ×mức1.0/1.5; ngắt perception5wallsec tạix2m. Child `83c784a0-c251-4b55-ac2c-0a4805082d26`, branch `orx/higher-speed-camera-dropout-safety-validation`, commit `5693463` đã chuẩn bị, **chưa launch**. Sau1.2s grace tới resume, mọi command phải hold-stale+zero velocity; yêu cầu tới goal và clearance. Không tuyên bố drift vật lý bằng0. Không chạy đồng thời2 flight.
3. Chọn cấu hình theo toàn bộ evidence, promote config vào main và kiểm tra integration replay với source/config tương đương. Báo rõ mọi giới hạn và tỷ lệ gồm failures. Goal còn active tới khi hoàn tất và kiểm tra các bước này.

3 unit tests evaluator/gate đạt trước khi launch. Các checks rộng hơn chỉ chạy khi có thay đổi/lỗi mới. Run command nghiên cứu cố định: `/tmp/uav-monocular-env/bin/python experiment.py`, launch bằng `orx exp run <id> --backend local` từ committed node; không chạy flight nghiên cứu trực tiếp.

## Phát hiện sparse-cloud và nhánh sửa riêng

Full54 chưa có evidence floor đã ghi `low_texture_v0.5_seed17` COLLISION_PROXY: clearance0.485550m, goalerror5.497860m, control15.72s. Log của71cycles chỉ dùng1point ở(7.1,-1.5,2.7); ảnh cuối gần đồng màu. Một point ở mép không thể đại diện mặt trước của cả box. Runner stop theo proxy, không phải xác nhận contact vật lý. [Raw và ảnh lỗi](../results/monocular_research/speed_expansion/low_texture_collision_seed17/). Low_texture1.0/1.5 seed17 vẫn SETUP_FAILED; mọi failures giữ trong mẫu số.

Nhánh riêng `3b688fb5-dc98-405c-aea5-175fd52c299c`, branch `orx/reject-insufficient-monocular-cloud-evidence-pil`, commit `cf7d99e`, đã chuẩn bị6 pilot: low_texture0.5/1/1.5 vàcentered/slalom/gate1.5, seed47. Thêm min_obstacle_points12 tại admission và mỗi planner step; thiếu finitepoints thì hold-insufficient-cloud, zero control và reset applied-control memory. 5 unit tests chuyển trạng thái/NaN/CLI precedence đạt. Chưa launch hoặc promote; main vẫn baseline. Đây chỉ là evidence floor, không chứng nhận visibility/coverage; có thể từ chối các scene khác có ít points.

Parent54 đã bị ngắt, không sửa source hay retry ẩn. Nếu chọn guarded method, cần ma trận riêng với cùng variant, rồi blackout và integration replay; không pool với successes trước guard. Child blackout `83c784a0...` trên nguồn preguard được đánh dấu superseded, chưa launch.

Raw/source pilot h6 và các archive baseline/integration cũng đã chuyển sang `/Volumes/Extreme SSD/uav_monocular_research_archives/<run-id>.tar.gz`, hash kiểm tra đầy đủ. Historical run directories vẫn giữ metadata; baseline/integration archive gốc là symlink sang SSD.

Đã di chuyển29 **completed** trial folders của run bị ngắt sang `/Volumes/Extreme SSD/uav_monocular_research_live/ea6b82ca-9ee9-458b-8e57-4f077928d9c4/completed_trials/`, kiểm tra mọi file source hash trước/sau copy rồi giữ symlink tại path gốc. Index hash ở `results/monocular_research/speed_expansion/completed_trial_storage.json`; audit các lượt này cần SSD mounted. Không archive repo có symlink rồi xóa external data; raw completed files nằm ngoài tar nếu tar mặc định không dereference.

Guard pilot commit mới `9a7ec6a` thêm storage trên SSD trong `experiment.py`, theo runUUID và fail trước flight nếu volume chưa mount; source/command vẫn immutable orx. Pilot chưa launch.

# Map bãi kho: A* + MPPI và ablation vận tốc

[Báo cáo tổng hợp để trao đổi mentor — 16/09/2026](MENTOR_REVIEW_YARD_MPPI_VI.md).

[Mới: sửa reset/proposal sau rejection](MPPI_REJECTION_RECOVERY_VI.md): 2/2 lượt headless tới đích, reference 10 m/s, retiming tắt, timeout giữ 45 s; vẫn có giữ/phanh.

[Trước sửa reset: prior map + mô hình gia tốc/phanh](MPPI_KNOWN_MAP_ACCEL_RESPONSE_VI.md): hai lượt dừng trước cua, chưa tới đích; retiming tắt.

[Audit chọn quỹ đạo và mô hình đáp ứng](AUDIT_MPPI_SELECTION_RESPONSE_VI.md): đã sửa diagnostics, thêm gate cloud có kiểm thử; headless vẫn thất bại do hạn chế quan sát/mô hình, chưa tune tiếp.

Tuning MPPI-only (retiming tắt): [5 ứng viên headless, chưa có cấu hình đạt](TUNE_YARD_MPPI_ONLY_10_MS_VI.md).

Quickstart GUI theo 5 terminal: [bài 60 m ở 5/10 m/s](RUN_YARD_5_10_MS_QUICKSTART_VI.md).

## Chạy Gazebo 3D GUI: lấy đà 60 m, retiming, yêu cầu 5 và 10 m/s

Giữ nguyên `mppi_yard_step10.yaml` và `mppi_yard_high_accel.parm` của đợt
headless thành công. Harness chỉ ghi đè cruise bằng `--speeds 5 10`.
Đã thêm `--gui` để mở GUI cùng partition với server, tự đóng sau mỗi lượt.
Dừng Gazebo/SITL đang chạy trước khi dùng; không cần tự mở SITL hay arm.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --gui \
  --speeds 5 10 --seeds 7 \
  --config config/experiments/mppi_yard_step10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 \
  --output "output/benchmark/yard_runup60_gui_$(date +%Y%m%d_%H%M%S)"
```

Chạy lần lượt 5 rồi 10 m/s, mỗi lượt tự khởi động world/SITL, cất cánh,
hover, bay và LAND. Để lặp hai seed, đổi `--seeds 7` thành `--seeds 7 17`.
Trong GUI chọn model `iris` để theo dõi UAV, chỉnh camera nếu cần.
Hai mức là cruise yêu cầu: retiming vẫn giảm reference trước cua.
Lệnh `--gui` đã kiểm tra CLI/syntax, chưa kiểm chứng hiển thị GUI trực tiếp;
kết quả headless không bảo đảm timing khi mở thêm GUI.

## Bật retiming trên đường lấy đà 60 m — 15/09/2026

**Cấu hình dùng cho bài 60 m hiện tại: `mppi_yard_step10.yaml`.**
Reference cruise 10 m/s, `reference_accel_m_s2=1.5`,
`reference_lateral_accel_m_s2=1.0`, `reference_ramp_from_measured=false`.
Reference đầu tiên vẫn là 10 m/s, nhưng giảm trước cua theo retiming.
World/đường và các ngưỡng an toàn giữ như bài 60 m tắt retiming.

**Hai lượt đều tới đích và LAND/disarm, 0 brake khẩn cấp**, không optimizer/
cycle deadline miss hoặc timeout-hold. Seed 17 có một `hold-stale` ban đầu
khi chờ odometry; không abort stale. WP_SPD=10, WP_ACC=3 đọc lại đúng.

| Seed | Peak XY (m/s) | Liên tục trong 9.5–10.5 m/s (s sim) | Thời gian tới đích (s sim) | Clearance tâm min (m) | Brake |
|---|---|---|---|---|---|
| 7 | 9.579 | 1.0 | 19.652 | 2.516 | 0 |
| 17 | 9.547 | 0.5 | 19.720 | 2.488 | 0 |

Tốc độ trung bình trong vùng cua khoảng 1.54/1.56 m/s, nhỏ nhất 0.84/0.64
m/s (seed 7/17). Vùng cua là bán kính XY 2 m quanh các đỉnh `(60,0)` và
`(60.5,-4)`, gồm cả phần vào/ra. Không mô tả là giữ 10 m/s xuyên cua.
Chưa đạt tiêu chí cruise ±5% liên tục 5 s. Clearance đo tâm tới solid, chưa
trừ rotor, không có xác nhận contact sensor. Tới đích theo bán kính 0.35 m,
không phải kiểm chứng settled hover.

So với tắt retiming (peak 9.84 m/s nhưng cả hai abort), bật retiming cho phép
hoàn tất hai lượt bằng giảm tốc chủ động trước cua. Không bảo đảm mọi seed
đều thành công. Process đã dừng; source snapshot khớp hash trong manifest.

[Biểu đồ](../output/benchmark/yard_runup60_retimed10_20260915_v1/comparison.png),
[summary](../output/benchmark/yard_runup60_retimed10_20260915_v1/summary.csv),
[cruise](../output/benchmark/yard_runup60_retimed10_20260915_v1/cruise_check.json),
[vùng cua](../output/benchmark/yard_runup60_retimed10_20260915_v1/turn_check.json).

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_step10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_runup60_retimed_my_run
```

Output phải mới, Gazebo/SITL khác đã dừng trước khi chạy.

## Lấy đà 60 m trước cua — headless 15/09/2026

**Hai lượt đều đạt khoảng 9.84 m/s trên đoạn lấy đà, nhưng không hoàn thành
bài rẽ container.** Giữ profile `mppi_yard_free10.yaml`: reference 10 ngay từ
đầu, không retiming, giữ conditioning/collision/brake và SITL WP_SPD=10,
WP_ACC=3 (đã đọc lại đúng cả hai lượt).

World mới `worlds/iris_mppi_yard_runup60.sdf` dịch toàn bộ bãi cũ
`(+45.1, -1.9)` m; không đổi hình dạng hoặc khoảng cách giữa container.
UAV vẫn cất cánh tại `(0,0)`; thay đoạn vào cũ bằng đoạn thẳng tới góc gắt
đầu tiên `(60,0)`. Đường thô: `(0,0) → (60,0) → (60.5,-4) → (73,-4)
→ (77.1,-1.9)`, cao 5 m. Không gọi toàn đường này là A* mới: đây là đoạn
lấy đà nối với các đỉnh A* cũ đã dịch. Góc bo bắt đầu trước đỉnh 60 m
(trim 1.5 m). Sàn được kéo dài để bao phủ đường lấy đà.

Đã kiểm tra các đoạn thô với inflation 2.6 m và toàn bộ reference bo với
inflation 1.5 m; đều đạt. Kiểm tra đường không bảo đảm quỹ đạo bay thực.

| Seed | Peak XY (m/s) | Liên tục trong 9.5–10.5 m/s (s sim) | Brake cycles | Kết quả | LAND/disarm |
|---|---|---|---|---|---|
| 7 | 9.835 | 3.3 | 14 | Abort vùng đệm | Xác nhận |
| 17 | 9.844 | 1.6 | 17 | Abort vùng đệm | Không xác nhận trong 35 s |

Cả hai trạng thái `ABORT_GEOMETRIC_CLEARANCE_LT_1M`, không tới đích.
Không optimizer/cycle deadline miss hoặc timeout-hold. Peak và các số đo
chỉ thuộc đoạn planner hoạt động trước abort. Chưa đạt tiêu chí giữ cruise
±5% liên tục 5 s. Seed 17 không được ghi là hạ cánh thành công; harness đã
kết thúc Gazebo/SITL sau thời hạn chờ LAND. Không kết luận va chạm vật lý
vì không có contact sensor. Guard dùng footprint box nới theo trục; summary
clearance dùng khoảng cách Euclid và cửa sổ dữ liệu khác, nên không diễn giải
tên trạng thái thành một phép đo clearance chính xác tại thời điểm abort.

Kết quả phân biệt rõ: **đường lấy đà dài giúp UAV lên gần 10 m/s; phần tránh
container/rẽ ở tốc độ đó vẫn chưa giải quyết được bằng tuning hiện tại.**

[Biểu đồ](../output/benchmark/yard_runup60_free10_20260915_v1/comparison.png),
[summary](../output/benchmark/yard_runup60_free10_20260915_v1/summary.csv),
[cruise](../output/benchmark/yard_runup60_free10_20260915_v1/cruise_check.json),
[kiểm tra đường bo](../output/benchmark/yard_runup60_free10_20260915_v1/rounded_reference_check.json).
Giữ manifest, source snapshot, nguồn tạo world và hash bổ sung trong output.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/build_yard_runup60.py
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_free10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_runup60_my_run
```

Output phải mới, Gazebo/SITL khác đã dừng trước khi chạy.

## Chẩn đoán thất bại reference 10 m/s — đối chiếu log 15/09/2026

Đã phân tích hai log `yard_free10_20260915_v1`, không thay tuning hoặc chạy lại.
**Không có dấu hiệu quá hạn tính toán**: trung bình MPPI khoảng 12 ms,
cao nhất dưới 40 ms, chu kỳ 100 ms; không deadline miss/timeout-hold.

### Các phát hiện có bằng chứng

1. **Lệnh tránh hướng thô khác đáng kể lệnh gửi thực.** Ví dụ seed 7, chu kỳ
   32, raw XY `(9.266, 9.806)` m/s thành sent XY `(8.171, -0.583)` m/s.
   Giới hạn thay đổi lệnh khiến hướng ngang chưa đảo kịp trong chu kỳ đó.
   Tuy nhiên `_dynamics()` đã mô phỏng conditioner trong rollout, nên không
   thể nói planner hoàn toàn bỏ qua bộ lọc lệnh. Raw vector có thể lớn hơn
   10 về norm vì bounds hiện tại theo từng thành phần; reference 10 không
   đồng nghĩa hard cap norm mọi raw action là 10.
2. **Có nominal mang chi phí collision dương nhưng vẫn gửi command.** Seed 7
   có ở chu kỳ 28/29 (2e6/1e6), seed 17 có cả chu kỳ 31 ngay trước brake.
   Collision ở đây là chi phí phạt, không phải chứng nhận đường được chọn
   chắc chắn hợp lệ. ESS có lúc bằng 1, tức trọng số tập trung mạnh vào một
   mẫu; chưa đủ bằng chứng để quy nguyên nhân riêng cho số mẫu hoặc lambda.
3. **Đáp ứng brake thực khác xa mô hình vận tốc đơn giản.** Khi bắt đầu brake,
   lệnh gửi chuyển về zero; UAV tiếp tục tăng tốc rồi mới giảm. Số đo dưới đây
   chỉ trong cửa sổ trước abort, không gồm LAND:

| Seed | Cycle brake đầu | Vận tốc state lúc brake (m/s) | Peak Gazebo sau brake (m/s) | Thời gian tới peak (s sim) | Dịch chuyển trong 1 s (m) | Vận tốc Gazebo sau 1 s (m/s) |
|---|---|---|---|---|---|---|
| 7 | 33 | 6.314 | 7.785 | 0.592 | 7.471 | 7.182 |
| 17 | 32 | 6.554 | 7.921 | 0.560 | 7.623 | 7.279 |

Peak trong chẩn đoán dùng nội suy 0.02 s nên khác nhẹ summary 0.1 s.
Vận tốc state đã lọc khác vận tốc lấy đạo hàm ground truth. Mốc brake dùng
simulation timestamp của state; state age khi gửi khoảng 32–35 ms wall.
Không coi thời gian tới peak là phép đo trễ thuần của ArduPilot.

Với zero đã áp dụng, mô hình rời rạc `v_next = v + 0.2*(0-v)` (`dt=.1`,
`tau=.5`) khởi tạo từ state trên cho khoảng 0.678/0.704 m/s sau 1 s.
Đây là đối chiếu minh họa khả năng mô tả đáp ứng, **không phải rollout thật
của brake gate**: brake có luật đoạn dừng riêng; chưa tách trễ truyền lệnh,
điều khiển ArduPilot, độ nghiêng và lọc vận tốc bằng một thí nghiệm nhận dạng.

### Quỹ đạo dự đoán và giới hạn diễn giải

Quỹ đạo nominal cuối trước brake đã cắt khỏi đường A* thô. Trong phần tương
lai còn quan sát được trước abort, sai lệch XY lớn nhất giữa nominal này và
Gazebo là 0.643 m (seed 7), 0.390 m (seed 17). Những con số này **không phải
sai số mô hình thuần**: hệ thống chuyển sang brake, không thực thi toàn bộ
chuỗi nominal. `selected_trajectory_enu` và cost được tái dựng sau khi
`accept_applied_control()` ghi lại U[0], không phải snapshot bất biến của
mẫu tối ưu gốc. Log không lưu toàn bộ point cloud/chuỗi raw U để tái hiện
chính xác quá trình lựa chọn. Không kết luận mất vật cản chỉ vì có chu kỳ
collision cost bằng 0.

**Kết luận:** hệ thống hiện tại có vấn đề về lựa chọn/kiểm tra quỹ đạo và khả
năng dự đoán phản ứng phanh ở tốc độ cao; chưa có bằng chứng máy tính tính
không kịp. Chưa xác định một nguyên nhân duy nhất. Bước sửa nên bắt đầu bằng
đo đáp ứng lệnh vận tốc/phanh trên đường trống, đối chiếu với mô hình và guard;
đồng thời bổ sung snapshot trước/sau conditioning và kiểm tra tính hợp lệ
quỹ đạo được chọn. Không đổi tau hoặc tăng Hz tùy ý rồi coi là đã giải quyết.

[Biểu đồ đối chiếu](../output/benchmark/yard_free10_20260915_v1/tracking_failure.png),
[số đo JSON](../output/benchmark/yard_free10_20260915_v1/tracking_failure.json).
Script phân tích tự lưu bản sao và hash trong output:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_tracking_failure.py \
  output/benchmark/yard_free10_20260915_v1
```

## Reference 10 m/s không retiming, MPPI tự tránh vật cản — 15/09/2026

Đã chạy đúng chế độ yêu cầu bằng `config/experiments/mppi_yard_free10.yaml`:
`reference_speed_m_s=10`, `vmax=10`, `reference_accel_m_s2=0`.
Không ramp reference, không giới hạn reference theo độ cong hoặc tính giảm tốc
trước cua. MPPI vẫn tối ưu tracking/collision; giữ đường bo, giới hạn lệnh,
WP_ACC=3, WP_SPD=10 và brake khẩn cấp. Hai tham số SITL đọc lại đúng.

**Cả hai lượt thất bại ở bài tránh container**, trạng thái
`ABORT_GEOMETRIC_CLEARANCE_LT_1M`; không tới đích, đều LAND/disarm sau abort.
Log xác nhận reference XY đầu tiên là **10.0 m/s**. Đây là yêu cầu reference,
không phải vận tốc thực ban đầu hay ép mọi lệnh MAVLink bằng 10.

| Seed | Peak XY đo (m/s) | Thời gian dữ liệu planner (s sim) | Brake cycles | Clearance tâm min trong dữ liệu phân tích (m) |
|---|---|---|---|---|
| 7 | 7.771 | 4.216 | 12 | 1.075 |
| 17 | 7.909 | 4.148 | 12 | 1.061 |

Không optimizer/cycle deadline miss hoặc timeout-hold. Seed 7 có một
`hold-stale` lúc khởi động. Chưa đạt dải 9.5–10.5 m/s. Các số đo chỉ bao gồm
đoạn planner hoạt động trước abort, không phải chuyến bay hoàn tất hay giai
đoạn LAND. Không dùng chúng như kết quả thành công hoặc chứng nhận không va chạm.

Tên trạng thái abort là tên có sẵn trong harness: với box, guard xét footprint
nới 1 m theo từng trục, không phải khoảng cách Euclid chính xác 1 m. Vì vậy
abort có thể xảy ra gần góc box khi clearance Euclid còn lớn hơn 1 m; cửa sổ
summary cũng kết thúc ở bản ghi planner cuối. Clearance tâm chưa trừ rotor,
không có contact sensor để kết luận va chạm vật lý.

Kết luận: hệ thống đã cho phép reference 10 ngay từ đầu, nhưng MPPI với tuning
hiện tại cùng bộ giới hạn lệnh/brake **chưa xử lý được bài này**. Không kết luận
10 m/s là bất khả thi; cần điều chỉnh và kiểm chứng controller trước khi dùng
chế độ này cho demo. Giữ profile mượt đã kiểm chứng làm phương án demo hiện tại.

Kết quả: [biểu đồ](../output/benchmark/yard_free10_20260915_v1/comparison.png),
[summary](../output/benchmark/yard_free10_20260915_v1/summary.csv),
[cruise](../output/benchmark/yard_free10_20260915_v1/cruise_check.json).
Manifest/source snapshot đã đối chiếu hash; các process thí nghiệm đã dừng.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_free10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_free10_my_run
```

Output phải mới, Gazebo/SITL khác đã dừng trước khi chạy.

## Yêu cầu reference 10 m/s ngay từ đầu — cấu hình thử nghiệm

File `config/experiments/mppi_yard_step10.yaml` đặt:

```yaml
vmax: 10.0
reference_speed_m_s: 10.0
reference_accel_m_s2: 1.5
reference_ramp_from_measured: false
```

“Ngay từ đầu” là chu kỳ planner đầu tiên sau cất cánh/hover ở 5 m.
Mỗi horizon khởi tạo tốc độ reference từ giới hạn tốc độ tại vị trí hiện tại
trên đường, thay vì ramp từ vận tốc UAV đo được. Trên đoạn thẳng đủ dài,
reference yêu cầu 10 m/s ngay cả khi UAV đang đứng yên.

Vẫn giữ giới hạn theo độ cong (`reference_lateral_accel_m_s2=1.0`), tính
ngược giảm tốc trước cua/đích, và giới hạn tăng tốc giữa các điểm trong horizon.
Nếu vị trí xuất phát đã trong vùng giảm tốc trước cua, reference ngay từ đầu
vẫn nhỏ hơn 10 m/s. Không ép 10 m/s vượt giới hạn đường vừa tính.
Setting này thay thế bản trước đặt `reference_accel_m_s2=0` vì bản đó tắt cả
việc giảm tốc reference cần giữ lại.

MPPI vẫn tối ưu lệnh; `command_alpha=0.30`, `max_accel_xy=3`, ArduPilot
`WP_ACC=3`, collision và brake khẩn cấp giữ nguyên. Đây không phải lệnh
MAVLink nhảy ngay lên 10 m/s hay vận tốc vật lý ban đầu 10 m/s.
Đã kiểm tra reference bằng unit test và chạy Gazebo headless hai seed 7/17;
kết quả bên dưới.
Không áp dụng kết quả 0 brake của profile mượt cho cấu hình này.

### Kết quả headless 15/09/2026 — bỏ ramp từ vận tốc đo, giữ retiming

Cả hai lượt yêu cầu 10 m/s đều tới đích và LAND/disarm. Không abort,
`hold-stale`, timeout-hold, optimizer hoặc cycle deadline miss. Đọc lại
`WP_SPD=10`, `WP_ACC=3` đúng ở cả hai lượt. Process đã dừng sau thí nghiệm.

| Seed | XY peak (m/s) | Thời gian (s sim) | Clearance tâm min (m) | Brake cycles | Min XY vùng cua (m/s) | RMS gia tốc XY (m/s²) |
|---|---|---|---|---|---|---|
| 7 | 5.241 | 15.062 | 2.397 | 6 | 0.576 | 2.224 |
| 17 | 5.118 | 14.382 | 2.372 | 0 | 0.969 | 2.261 |

Reference XY ngay chu kỳ đầu là **3.680 m/s** khi vận tốc đo bằng 0;
reference XY được log lớn nhất là **5.833/5.844 m/s** (seed 7/17).
Điều này xác nhận đã bỏ ramp từ trạng thái đứng yên, nhưng giới hạn do đường
và giảm tốc trước cua vẫn làm reference thấp hơn yêu cầu cruise 10 m/s.
Cả hai lượt không vào dải 9.5–10.5 m/s, chưa đạt tiêu chí giữ cruise 5 s.

So với hai lượt 10 m/s của profile mượt bo rộng (peak 3.88–4.14 m/s,
16.116 s, 0 brake), profile mới nhanh hơn nhưng RMS gia tốc cao hơn và một
lượt có brake trở lại. **Chạy hoàn thành, chưa đạt mục tiêu 10 m/s hoặc
không brake ở cả hai lượt.** Chưa thay profile demo mượt bằng bản này.
Clearance là tâm tới solid SDF, chưa trừ rotor, không phải xác nhận bằng
contact sensor. Tới bán kính đích 0.35 m chưa phải settled hover. Định nghĩa
vùng cua giữ nguyên bán kính XY 2 m quanh hai đỉnh A* gắt.

Output: [biểu đồ](../output/benchmark/yard_step10_retimed_20260915_v1/comparison.png),
[summary](../output/benchmark/yard_step10_retimed_20260915_v1/summary.csv),
[vùng cua](../output/benchmark/yard_step10_retimed_20260915_v1/turn_check.json),
[cruise](../output/benchmark/yard_step10_retimed_20260915_v1/cruise_check.json).
Manifest/source snapshot đã đối chiếu SHA-256; giữ log gốc và command từng process.

Lệnh thử headless riêng, dùng output mới và khi Gazebo/SITL khác đã dừng:

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_step10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_step10_my_run
```

Phải truyền `--speeds 10`: harness ghi đè tốc độ trong YAML bằng từng mức
`--speeds` và mặc định của harness là các mức tốc độ thấp.

## Profile mượt qua cua, giảm brake — 15/09/2026

**Khuyến nghị cho demo mượt: `config/experiments/mppi_yard_smooth_round.yaml`.**
Đã chạy headless hai profile mới, mỗi profile 4 lượt (yêu cầu 5/10 m/s,
seeds 7/17), tổng cộng 8/8 tới đích và LAND/disarm, không kích hoạt brake.
Đây là kết quả trên các lượt đã đo, không bảo đảm mọi lần chạy đều không brake.

| Chỉ số | Gia tốc cao trước đó | Giảm tốc sớm | Giảm tốc sớm + bo rộng (chọn) |
|---|---|---|---|
| Chu kỳ brake mỗi lượt | 9–12 | 0 | 0 |
| XY peak mỗi lượt (m/s) | 5.14–5.71 | 4.08–4.32 | 3.88–4.30 |
| XY thấp nhất trong vùng cua (m/s) | 0.15–0.24 | 0.26–0.44 | 0.89–1.13 |
| RMS gia tốc XY, trung bình 4 lượt (m/s²) | 2.443 | 1.721 | 1.625 |
| RMS biến thiên lệnh, trung bình 4 lượt (m/s²) | 4.782 | 3.170 | 3.151 |
| Thời gian tới đích trung bình (s sim) | 13.549 | 17.255 | 16.099 |
| Clearance tâm nhỏ nhất cả nhóm (m) | 2.003 | 2.389 | 2.248 |

Profile được chọn giảm RMS gia tốc khoảng **33.5%**, RMS biến thiên lệnh
khoảng **34.1%**, nhưng thời gian dài hơn **18.8%** so với nhóm gia tốc cao.
Bo rộng giúp bớt gần dừng ở cua so với chỉ giảm tốc sớm. Tốc độ vẫn dao động;
đây chưa phải chuyển động có jerk được giới hạn hoặc cruise 5/10 m/s.
Cả 8 lượt mới đều không đạt tiêu chí giữ tốc độ yêu cầu ±5% liên tục 5 s.

Thay đổi so với `mppi_yard_high_accel.yaml`:

- `reference_accel_m_s2`: 3 → **1.5**, để reference tính giảm tốc từ sớm.
- `reference_lateral_accel_m_s2`: 2 → **1.0**, giảm tốc độ mục tiêu theo độ cong.
- `reference_corner_radius_m`: 0.8 → **1.5**, `reference_corner_samples`: 6 → **12**.
  Tên tham số là radius nhưng thực hiện bằng khoảng cắt góc Bézier, không phải
  bán kính cung tròn chính xác. Đường A* thô giữ nguyên; reference đã bo thay đổi.
- Giữ `max_accel_xy=3`, `WP_ACC=3`, `WP_SPD=10`, `command_alpha=0.30`,
  collision radius 1.5 m, brake 1.5 m / release 2.0 m / delay 0.25 s và
  kiểm tra đoạn dừng 3D. **Không tắt brake khẩn cấp.**

Brake cũ xuất hiện khi đoạn dừng dự đoán tiến sát container trước cua;
ví dụ lượt 5 m/s seed 7 bắt đầu brake ở chu kỳ 47 khi đang gần 4.94 m/s.
Giảm tốc thông thường vẫn cần xảy ra để đổi hướng; mục tiêu ở đây là giảm tốc
có kế hoạch để tránh phải chuyển sang brake khẩn cấp.

Trước chạy, toàn bộ 53 đoạn reference bo rộng qua kiểm tra hình học với
clearance yêu cầu 1.5 m: [preflight](../output/benchmark/yard_smooth_round_preflight.json).
Kiểm tra này chỉ xét đường reference; clearance thực tế được đo riêng từ
quỹ đạo Gazebo. Không thay đổi world hoặc dời container.

| Yêu cầu | Seed | Peak XY (m/s) | Min XY vùng cua (m/s) | Thời gian (s sim) | Clearance tâm min (m) | Brake |
|---|---|---|---|---|---|---|
| 5 m/s | 7 | 4.095 | 0.927 | 16.082 | 2.248 | 0 |
| 5 m/s | 17 | 4.297 | 0.973 | 16.082 | 2.409 | 0 |
| 10 m/s | 7 | 4.142 | 1.134 | 16.116 | 2.401 | 0 |
| 10 m/s | 17 | 3.881 | 0.890 | 16.116 | 2.422 | 0 |

Nhóm bo rộng có 1 cycle deadline miss ở lượt 10 m/s seed 7, chu kỳ 139
(116.9 ms); không optimizer deadline miss hoặc timeout-hold. Hai lượt 5 m/s
mỗi lượt có 1 `hold-stale` ban đầu khi chờ odometry. Không abort stale.
Nhóm chỉ giảm tốc sớm không deadline miss hoặc timeout-hold; lượt 5 m/s
seed 7 cũng có 1 `hold-stale` ban đầu. Tham số WP_SPD/WP_ACC đọc lại đúng
10/3 ở cả 8 lượt. Tất cả process của hai đợt đã dừng sau LAND/disarm.

Vùng cua là vùng XY bán kính 2 m quanh hai đỉnh A* đổi hướng ≥60°, gồm cả
đoạn vào/ra cua. Clearance đo từ tâm tới solid SDF, chưa trừ rotor, không phải
contact sensor. Tới bán kính đích 0.35 m chưa phải dừng ổn định tại đích.

Kết quả bản chọn: [biểu đồ](../output/benchmark/yard_smooth_round_20260915_v1/comparison.png),
[summary](../output/benchmark/yard_smooth_round_20260915_v1/summary.csv),
[vùng cua](../output/benchmark/yard_smooth_round_20260915_v1/turn_check.json),
[cruise](../output/benchmark/yard_smooth_round_20260915_v1/cruise_check.json).
Bản chỉ giảm tốc sớm: [summary](../output/benchmark/yard_smooth_20260915_v1/summary.csv).
Mỗi đợt giữ manifest, source snapshot khớp SHA-256 và log gốc.

Tái chạy (output phải là thư mục mới, Gazebo/SITL khác đã dừng):

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard --speeds 5 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_smooth_round.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_smooth_round_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_speed_ablation.py \
  output/benchmark/yard_smooth_round_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_turns.py \
  output/benchmark/yard_smooth_round_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_speed_cruise.py \
  output/benchmark/yard_smooth_round_my_run
```

## Thử tốc độ cao qua các góc rẽ trong bãi container — 15/09/2026

**Đã chạy xong 4 lượt gia tốc cao hơn, cả bốn tới đích và LAND/disarm.**

| Cruise yêu cầu | Seed | XY max (m/s) | Thời gian sim (s) | Clearance tâm min (m) | Chu kỳ brake |
|---|---|---|---|---|---|
| 5 m/s | 7 | 5.142 | 13.668 | 2.003 | 9 |
| 5 m/s | 17 | 5.206 | 13.498 | 2.108 | 9 |
| 10 m/s | 7 | 5.705 | 13.600 | 2.187 | 12 |
| 10 m/s | 17 | 5.299 | 13.430 | 2.245 | 11 |

Không abort stale, timeout-hold hoặc optimizer/cycle deadline miss trong bốn
lượt này. Tuy nhiên **không đạt tiêu chí giữ cruise ±5% liên tục 5 s**:
yêu cầu 5 m/s chỉ giữ được 0.9/1.1 s liên tục; yêu cầu 10 m/s không đạt dải
9.5–10.5 m/s. Đây là tăng tốc trên các đoạn nối rồi giảm mạnh qua cua.

Trong vùng quanh hai góc gắt, XY trung bình 1.84–1.90 m/s, nhỏ nhất
0.15–0.24 m/s; có thời điểm gần dừng. Không mô tả kết quả này là “rẽ mượt
ở 5/10 m/s”. Clearance là khoảng cách tâm UAV tới solid SDF, chưa trừ rotor
và không phải log contact vật lý. Tới bán kính đích 0.35 m chưa phải settled
hover. Tham số `WP_SPD=10`, `WP_ACC=3` đã đọc lại đúng ở cả bốn lượt.

Kết quả gia tốc cao hơn: [đường bay/vận tốc/clearance](../output/benchmark/yard_high_accel_20260915_v1/comparison.png),
[summary.csv](../output/benchmark/yard_high_accel_20260915_v1/summary.csv),
[số đo vùng cua](../output/benchmark/yard_high_accel_20260915_v1/turn_check.json),
[kiểm tra giữ cruise](../output/benchmark/yard_high_accel_20260915_v1/cruise_check.json).
Mốc transfer: [summary.csv](../output/benchmark/yard_cruise_transfer_20260915_v1/summary.csv).
Hai thư mục đều giữ manifest, source snapshot và log gốc; process đã dừng.

Hai nhóm chạy headless tách biệt, cùng world bãi container và đường A* cũ
từ `(0,0,5)` tới `(32,0,5)`, không mở rộng cua hay dời container:

- **Transfer**: `mppi_yard_cruise_transfer.yaml`, giữ setting của bài cruise
  đường thẳng (gia tốc 0.6 m/s²), tham số SITL từ `mppi_speed_straight.parm`.
  Hai mức 5/10 m/s, seed 7: đều tới đích, XY max 2.548/2.536 m/s,
  thời gian 25.398/25.636 s sim, clearance tâm min 2.520/2.438 m;
  không brake, stale, timeout-hold hay deadline miss. Tốc độ trung bình trong
  vùng quanh hai góc gắt khoảng 1.0 m/s.
- **Gia tốc cao hơn**: `mppi_yard_high_accel.yaml`, budget reference dọc
  3.0 m/s², ngang 2.0 m/s², giới hạn thay đổi lệnh XY 3.0 m/s².
  `mppi_yard_high_accel.parm` đặt `WP_SPD=10`, `WP_ACC=3`; harness đọc lại
  hai tham số trước arm. Giữ lambda=1, warm-start reference, retiming,
  brake theo đoạn dừng 3D, collision radius 1.5 m và A* inflate 2.6 m.
  Đây là thay đổi nhiều setting giữa hai nhóm, không phải ablation một biến.

Lệnh tái chạy nhóm gia tốc cao hơn, chỉ khi Gazebo/SITL khác đã dừng:

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard --speeds 5 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_high_accel.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 150 --output output/benchmark/yard_high_accel_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_speed_ablation.py \
  output/benchmark/yard_high_accel_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_turns.py \
  output/benchmark/yard_high_accel_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_speed_cruise.py \
  output/benchmark/yard_high_accel_my_run
```

Output phải mới. `turn_check.json` đo trong vùng bán kính XY 2 m quanh
hai đỉnh A* có đổi hướng ≥60°: `(14.9,1.9)` và `(15.4,-2.1)`.
Vùng này gồm cả đoạn vào/ra cua, không phải phép đo tại đúng đỉnh bo cong.
Tốc độ peak toàn chuyến không được diễn giải là tốc độ giữ xuyên suốt cua.

## Kiểm chứng cruise 5/10 m/s headless — ngày 15/09/2026

**Đợt xác nhận cuối đạt tiêu chí tốc độ ở 4/4 lượt**, cùng đường thẳng 300 m,
seeds 7/17. Cả bốn tới bán kính đích 0.35 m, LAND/disarm và dừng process.

| Yêu cầu | Seed | XY max đo (m/s) | Liên tục trong ±5% (s sim) | Thời gian tới đích (s sim) |
|---|---|---|---|---|
| 5 m/s | 7 | 5.144 | 51.4 | 67.116 |
| 5 m/s | 17 | 5.176 | 51.4 | 67.184 |
| 10 m/s | 7 | 9.928 | 11.9 | 46.478 |
| 10 m/s | 17 | 9.927 | 12.1 | 46.444 |

Không brake hay abort stale trong đợt xác nhận. Hai lượt 5 m/s mỗi lượt có
1 cycle deadline miss. Lượt 10 m/s seed 17 có 1 optimizer/cycle deadline
miss và 1 timeout-hold ở cycle 2, gần điểm xuất phát (compute 188.6 ms).
Lượt 10 m/s seed 7 không có các sự kiện trên. Giữ nguyên những sự kiện này
trong log; chưa thể tuyên bố hard real-time hoặc demo GUI đã ổn định.
Sai số cao độ lớn nhất của bốn lượt dưới 0.20 m. Tới đích chưa đồng nghĩa
đã settled hover; không có kiểm chứng contact rotor. Đợt này chỉ xác nhận
cruise trên đường thoáng, không xác nhận bay 10 m/s giữa các container.

Kết quả: [đồ thị tốc độ và dải ±5%](../output/benchmark/straight_cruise_retimed_20260915_v1/cruise_speed.png),
[summary.csv](../output/benchmark/straight_cruise_retimed_20260915_v1/summary.csv),
[cruise_check.json](../output/benchmark/straight_cruise_retimed_20260915_v1/cruise_check.json).
Raw log, manifest và source snapshot ở cùng thư mục. Các chỉ số max/thời
gian chuyến bay lấy từ analyzer tổng hợp; thời gian giữ dải lấy từ analyzer
cruise theo các khoảng dịch chuyển 0.1 s nên có thể lệch 0.1 s so với số đếm
ngưỡng của analyzer tổng hợp. 69 test đạt trước đợt xác nhận.

Profile thử nghiệm mới: `config/experiments/mppi_straight_cruise_retimed.yaml`.
Chỉ dùng với đường thẳng 300 m. Giữ gia tốc reference/lệnh 0.6 m/s²,
`command_alpha=0.30`, `vmax=10`; bật lại retiming để giảm tốc trước đích.
Thay `lambda` từ 100 thành 1 và bật `reference_warm_start`: chuỗi đề xuất
ban đầu và phần đuôi horizon lấy từ reference, vẫn qua conditioner và MPPI.
Không gửi thẳng reference để thay thế đầu ra MPPI.

`brake_swept_path=true` kiểm tra clearance của **tất cả điểm cloud tới đoạn
dừng theo hướng vận tốc hiện tại trong 3D**. Giữ khoảng cách gần tối thiểu,
phát hiện vật cản phía trước và mặt đất khi đang hạ xuống; không coi điểm
mặt đất cách dưới đường bay ngang 5 m là va chạm chỉ vì nó nằm phía trước.
Đây là mô hình dừng thẳng với các điểm quan sát được, không bảo đảm khoảng
không chưa quan sát hoặc động lực học phanh thực tế. Hai cờ mới mặc định tắt
cho profile cũ.

Vòng điều khiển cũng đọc MAVLink chiều về khi dùng state odometry, tối đa
256 message/chu kỳ, để tránh bỏ mặc bộ đệm TCP. Giữ các gate stale/flight
envelope; chưa kết luận đây là nguyên nhân duy nhất của stale trước đây.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario straight --speeds 5 10 --seeds 7 17 \
  --config config/experiments/mppi_straight_cruise_retimed.yaml \
  --timeout 180 --output output/benchmark/straight_cruise_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_speed_ablation.py \
  output/benchmark/straight_cruise_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_speed_cruise.py \
  output/benchmark/straight_cruise_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/plot_speed_cruise.py \
  output/benchmark/straight_cruise_my_run
```

Dừng các phiên Gazebo/SITL khác trước; output phải mới. Tiêu chí tốc độ:
XY thực trong **±5% yêu cầu liên tục ít nhất 5 s mô phỏng**. Script cruise
tính vận tốc từ dịch chuyển ground truth trên từng khoảng 0.1 s, ngắt chuỗi
khi timestamp không tăng hoặc gap sim/wall vượt 0.5 s. `cruise_check.json`
tách việc đạt tốc độ khỏi tới đích/LAND; không dùng một đỉnh tốc độ để báo đạt.

Các đợt đối chứng đã giữ riêng:

- `straight_step_reference_20260915_v1`: bốn lượt reference tức thời;
  5 m/s tới đích nhưng XY max chỉ 4.406–4.417 m/s; 10 m/s max 4.865 m/s,
  cả hai abort stale và có 176–180 chu kỳ brake. Không lượt nào đạt cruise.
- `straight_cruise_20260915_v1`: thêm warm-start/reference tail và brake 3D,
  vẫn `lambda=100`, seed 7. Cả hai tới đích, XY max 4.418/7.725 m/s;
  không brake nhưng có timeout-hold/deadline miss. Không lượt nào đạt cruise.
- `straight_cruise_offline_lambda_probe.json`: chỉ mô hình nội bộ; cùng
  setting, đổi lambda 100→1 cho tốc độ trung bình 5 s cuối 4.31→5.03 và
  7.60→9.84 m/s. Không thay thế bằng chứng Gazebo. Một số tác vụ kiểm tra
  mô hình/test có chạy cùng đợt thăm dò; không dùng timing của đợt thăm dò
  làm benchmark compute độc lập.

## Bài thử riêng: đường thẳng 300 m để kiểm tra tốc độ thực

**Profile đặt reference ngay từ đầu (đã chạy, chưa đạt cruise; xem đối chứng trên):**
`config/experiments/mppi_straight_step_reference.yaml` đặt
`reference_accel_m_s2: 0.0`, nên reference tiến ngay theo cruise 5/10 m/s,
không khởi tạo lại ramp từ vận tốc đo. Trong lệnh đường thẳng bên dưới,
thay đường dẫn `--config` bằng profile này và dùng thư mục output mới.
Thao tác này cũng tắt giới hạn reference theo cua và giảm tốc trước đích;
chỉ dùng cho bài thử đường thẳng. Các điểm reference vẫn bị chặn ở cuối đường.
`max_accel_xy: 0.6` và `command_alpha: 0.30` vẫn giữ: lệnh gửi MAVLink
chưa phải bước nhảy 5/10 m/s và tốc độ thực không đổi tức thời.

World `worlds/iris_mppi_speed_straight.sdf` giữ Iris/sensor/physics của bãi
kho, thay các vật cản bằng mặt sân phẳng dài 400 m. A* cho đường thẳng
`(0,0,5) → (300,0,5)`. Đây là bài thử bám vận tốc trên đường thoáng,
không phải kết quả tránh vật cản của bãi kho.

Giữ nguyên profile MPPI retiming, kể cả gia tốc 0.6 m/s². File SITL riêng
`config/experiments/mppi_speed_straight.parm` đặt `WP_SPD=10` và giá trị
tương thích cũ `WPNAV_SPEED=1000`. Harness đọc lại `WP_SPD=10` trước arm;
không thay file tham số baseline. Trong source ArduPilot tại phiên chạy,
`ModeGuided::velaccel_control_run()` gọi `input_vel_accel_NE_m(..., false)`;
nhánh này không trực tiếp clip vận tốc bằng `WPNAV_SPEED`. Không được dùng
giá trị waypoint đó để tự kết luận có hard cap 3 m/s trong velocity GUIDED.

Chạy sau khi các phiên Gazebo/SITL khác đã dừng:

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario straight --speeds 5 10 --seeds 7 17 \
  --config config/experiments/mppi_industrial_retimed.yaml \
  --timeout 180 --output output/benchmark/straight_speed_my_run
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_speed_ablation.py \
  output/benchmark/straight_speed_my_run
```

Output phải là thư mục mới. Safety gate vẫn kiểm tra stale odometry,
clearance và cao độ 3.5–6.5 m; riêng tuyến dài dùng bán kính giới hạn XY
330 m và giới hạn lệch ngang `|y| ≤ 10 m`. Mỗi lượt LAND/disarm rồi reset.
Đọc thêm `time_at_or_above_95pct_request_sim_s`: tổng thời gian vận tốc XY
thực đo đạt ít nhất 95% yêu cầu, không phải chỉ chạm một đỉnh tốc độ.

### Kết quả đường thẳng ngày 14/09/2026

| Yêu cầu | Seed | Kết quả | Thời gian sim ghi nhận (s) | XY max (m/s) | Thời gian ≥95% yêu cầu (s) |
|---|---|---|---|---|---|
| 5 m/s | 7 | ABORT_STALE_ODOMETRY | 83.028 | 4.310 | 0 |
| 5 m/s | 17 | ABORT_STALE_ODOMETRY | 83.368 | 4.287 | 0 |
| 10 m/s | 7 | Tới đích | 83.028 | 4.402 | 0 |
| 10 m/s | 17 | Tới đích | 83.334 | 4.427 | 0 |

Cả bốn LAND/disarm, `WP_SPD` đọc lại đều 10 m/s. Hai lượt abort còn cách
đích lần lượt 1.905 và 2.320 m; thời gian của lượt abort không phải thời gian
hoàn thành tuyến. Ở lượt 5 m/s seed 7, khoảng ngắt odometry wall-time lớn nhất
2.095 s đi cùng sim-time chỉ tăng 0.034 s. Nguyên nhân stall chưa xác định;
không tắt gate hay loại lượt lỗi khỏi kết quả.

Không có optimizer/cycle deadline miss hoặc timeout-hold. Lượt 10 m/s seed 17
có 5 chu kỳ `hold-brake` tại x khoảng 255–262 m; ba lượt khác không có brake.
Log nearest lúc đó khoảng 15.7–16.9 m, chưa xác định nguyên nhân kích hoạt
brake. Không diễn giải bài thử này là pipeline hoàn toàn ổn định.

**Đường thẳng dài hơn vẫn chưa đủ để đạt 5/10 m/s với cấu hình ngày 14/09.**
Trong đoạn x=50–250 m, lệnh XY trung bình 3.927–4.036 m/s, vận tốc odometry
trung bình 3.922–4.028 m/s. UAV bám gần mức lệnh trung bình, còn lệnh chưa lên
đủ tốc độ yêu cầu. Điều này bổ sung cho giải thích về cua/quãng đường: cần
kiểm tra thêm scheduler reference và bộ tối ưu, chưa thể kết luận giới hạn
vật lý của UAV hoặc hard cap ở ArduPilot.

Code retiming khởi tạo lại tốc độ dự báo từ vận tốc đo mỗi chu kỳ, bước đầu
chỉ tăng tối đa 0.06 m/s; reference đầu horizon vì thế cũng quanh 4 m/s.
Thư viện MPPI đang nối lệnh zero vào cuối horizon khi dịch chuỗi nominal.
Đây là hai điểm cần kiểm chứng khi tune tiếp, **chưa phải nguyên nhân đã
được xác nhận bằng thử nghiệm đối chứng**. Đợt này không sửa lõi MPPI.

Log: [summary.csv](../output/benchmark/straight_speed_20260914_v2/summary.csv),
[đồ thị](../output/benchmark/straight_speed_20260914_v2/comparison.png),
[đối chiếu lệnh/vận tốc](../output/benchmark/straight_speed_20260914_v2/tracking_diagnostics.json).
`source_snapshot/` lưu source/config/world đã đối chiếu hash với manifest.
`straight_speed_20260914_v1` chỉ là lần setup lỗi hết dung lượng trước khi mở
Gazebo, không có dữ liệu bay. Đã giải phóng cache pip tải lại được để chạy v2;
không xóa log thí nghiệm cũ. Kiểm tra A* trả đúng hai đầu tuyến và 56 test đạt.

## Chạy lại headless 5/10 m/s — seeds 7/17, ngày 14/09/2026

Đã chạy bốn lượt Gazebo/SITL với `mppi_industrial_retimed.yaml`, timeout
150 s/lượt, giữ nguyên cấu hình và đường A*. Cả bốn tới đích, LAND/disarm;
script kết thúc thành công và không còn process Gazebo/SITL/MPPI của sweep.

| Cruise yêu cầu | Seed | Thời gian sim (s) | XY trung bình (m/s) | XY max (m/s) | Clearance tâm min (m) |
|---|---|---|---|---|---|
| 5 m/s | 7 | 28.016 | 1.270 | 2.219 | 2.176 |
| 5 m/s | 17 | 28.220 | 1.264 | 2.089 | 2.276 |
| 10 m/s | 7 | 28.186 | 1.260 | 2.186 | 2.354 |
| 10 m/s | 17 | 28.152 | 1.271 | 2.207 | 2.273 |

Không có brake, timeout-hold hoặc optimizer deadline miss. Lượt 10 m/s seed 7
có **1 cycle deadline miss**; ba lượt còn lại không có. RTF đo được
0.978–0.993. Vận tốc lấy từ vị trí ground truth, nội suy theo bước 0.1 s
mô phỏng rồi tính đạo hàm; không phải tốc độ setpoint.

**Chưa đạt vận tốc thực 5 hoặc 10 m/s.** Hai yêu cầu vẫn cho kết quả gần nhau
với profile giới hạn theo gia tốc/độ cong hiện tại. Tới đích là vào bán kính
0.35 m, không xác nhận settled hover; clearance chưa trừ rotor và không phải
kiểm tra contact vật lý. Đây là khảo sát hai seed, không chứng minh độ tin cậy.

Kết quả mới: [summary.csv](../output/benchmark/industrial_retimed_20260914_repeat1/summary.csv),
[summary.json](../output/benchmark/industrial_retimed_20260914_repeat1/summary.json),
[đồ thị](../output/benchmark/industrial_retimed_20260914_repeat1/comparison.png).
Trong cùng thư mục có manifest, bản sao config/world và log gốc từng lượt.

## Bổ sung: yêu cầu cruise 5/10 với reference retiming (headless đã thử một seed)

Không dùng lệnh cũ chỉ tăng `--reference-speed-m-s`/`--vmax` để kỳ vọng
bám được 5/10 m/s qua các góc gắt. Profile mới là:
`config/experiments/mppi_industrial_retimed.yaml`.
Trong Terminal 5 bên dưới, chọn `YARD_SPEED=5.0` hoặc `YARD_SPEED=10.0`;
block chạy chung tự chọn profile này. Mỗi lượt phải reset/start gate lại.
Config mới đã có `vmax: 10.0`; giữ các safety gate, không chạy hai controller.

Reference mới giới hạn cruise theo độ cong, chạy backward pass để giảm tốc
trước cua/đích, và tăng tốc từ vận tốc đo. Budget dọc/ngang 0.6 m/s².
Đây là scheduler reference của project, không phải chứng chỉ khả thi động lực
học: nếu xe đã vượt tốc độ profile, reference có thể chậm lại nhanh hơn xe.
MPPI và giới hạn lệnh vẫn chịu trách nhiệm bám; model/latency vẫn cần xác nhận.

Kết quả thực đo, Gazebo/SITL headless seed 7, mỗi mức một lượt:

| Cruise yêu cầu | Tới đích | Thời gian sim | Tốc độ XY max đo | Clearance tâm min |
|---|---|---|---|---|
| 5 m/s | Có | 27.914 s | 2.171 m/s | 2.305 m |
| 10 m/s | Có | 27.812 s | 2.083 m/s | 2.624 m |

Cả hai LAND/disarm, không brake/timeout-hold/deadline miss trong lượt này.
**Không phải đã bay được 5 hoặc 10 m/s**: map/gia tốc giới hạn profile nên hai
yêu cầu đều cho chuyến bay khoảng 28 s. Không gộp với ablation cũ.
Arrival chỉ là vào bán kính 0.35 m, không phải settled hover; clearance chưa trừ rotor.
Lỗi stale odometry kéo dài của phiên GUI cũ chưa xác định nguyên nhân;
fresh headless không tái hiện lỗi không có nghĩa đã sửa được nguyên nhân đó.

Log: `output/benchmark/industrial_retimed_highspeed_v1/` (manifest, cấu hình,
raw odometry, planner, summary CSV/JSON và comparison.png).
Lệnh tái chạy, chỉ khi phiên GUI/SITL cũ đã dừng:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --speeds 5 10 --seeds 7 17 \
  --config config/experiments/mppi_industrial_retimed.yaml \
  --timeout 150 --output output/benchmark/industrial_retimed_repeat
```

Thư mục output phải chưa tồn tại. Baseline cũ không đổi vì retiming mặc định tắt.
Lệnh trên chạy bốn lượt tuần tự: hai tốc độ × hai seed. Đây là thử nghiệm
**yêu cầu cruise 5/10 m/s**, tách riêng khỏi sweep 0,4–1,2 m/s bên dưới.
Để kiểm tra UAV có thực sự đạt 5/10 m/s, đọc vận tốc XY từ ground truth và
đồ thị sau chạy; không lấy `YARD_SPEED` làm vận tốc thực đo. Với gia tốc
0,6 m/s², riêng quãng đường tăng tốc lý tưởng từ đứng yên lên 5 m/s đã là
20,8 m, lên 10 m/s là 83,3 m (chưa tính giảm tốc). Tuyến bãi kho hiện tại
có nhiều cua nên không phù hợp để xác nhận cruise thực đo 10 m/s; phép thử
đạt tốc độ đó cần tuyến thẳng dài hơn hoặc một thí nghiệm tune động lực học riêng.

Kiểm tra: full discovery `python -m unittest discover -s tests -p 'test_*.py'`
đạt 56 test. Một lần chạy riêng module retiming gặp lỗi duplicate OpenMP trên
máy này; full suite sau đó chạy đạt. Đính chính ngày 15/09: CLI trên macOS
và `test_mppi_core.py` đã có sẵn `KMP_DUPLICATE_LIB_OK=TRUE`, nên việc suite
đạt không chứng minh lỗi duplicate runtime đã được xử lý. Cần sửa môi trường
OpenMP riêng; các lượt ở đây vẫn dùng môi trường project hiện có.

Không phải báo cáo tuần. Đây là hướng dẫn tái chạy thí nghiệm và đọc log.

Đã chạy 8 lượt headless ngày 14/09/2026, seeds 7 và 17. Dữ liệu:
[`industrial_speed_20260914_v2`](../output/benchmark/industrial_speed_20260914_v2/summary.csv),
[đồ thị](../output/benchmark/industrial_speed_20260914_v2/comparison.png).
0,6 / 0,8 / 1,0 m/s đều tới đích ở 2/2 lượt. Hai lượt 0,4 m/s bị
`ABORT_STALE_ODOMETRY`: sim-time chỉ tăng 0,034 s trong khoảng gián đoạn
wall-time 2,14–2,17 s; nguyên nhân stall Gazebo/SITL chưa xác định.
Không kết luận MPPI kém hơn ở tốc độ thấp từ hai lượt này. Tất cả đã LAND
và disarm. Các lượt tới đích vẫn có dao động vận tốc và một số deadline miss,
không được diễn giải là pipeline hoàn toàn mượt hay đạt hard real-time.

Thư mục `industrial_speed_20260914_v1` là các lượt lỗi khởi tạo/arm trong
lúc xây harness, không phải dữ liệu navigation. Lượt thăm dò trước sửa decoder
`output/log/yard_pilot_v06_seed7.jsonl` bị dừng thủ công; không gộp vào v2.

Đã chạy bổ sung **1,2 m/s, seeds 7/17** với cùng map, đường và các giá trị
setting: 2/2 tới đích, 35,87–36,28 s mô phỏng; tốc độ XY trung bình
0,955–0,964 m/s, clearance tâm-to-solid nhỏ nhất 2,587 m. Không có brake,
timeout hold hay cycle deadline miss trong hai lượt này; đều LAND/disarm.
RMS gia tốc XY 0,267–0,283 m/s², cao hơn trung bình khoảng 0,255 m/s² của
1,0 m/s ở sweep trước. Đây là khảo sát hai seed, không chứng minh độ ổn định
trên GUI hay phần cứng. Log/CSV/đồ thị bổ sung ở
[`industrial_speed_20260914_v12`](../output/benchmark/industrial_speed_20260914_v12/summary.csv).

## Map và luồng

`worlds/iris_mppi_industrial_yard.sdf` là bãi kho tổng hợp: hai nhà xưởng,
cửa xuất hàng, container xếp ba tầng, pallet, trailer và đường nội bộ.
Không phải digital twin hay benchmark photorealistic. Hình khối kết cấu dùng
cùng kích thước visual/collision; gân container và vạch đường chỉ trang trí.
Không phụ thuộc tải Fuel. Sinh lại bằng:

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/build_industrial_yard.py
```

Luồng: **collision SDF đã biết → A* XY tại z=5 m → rút gọn đường → bo góc
reference → MPPI bám reference và tránh point cloud → MAVLink velocity →
ArduPilot GUIDED → Gazebo**. A* này vẫn dùng lattice/grid 0,5 m; chưa phải
phương án global planner không-grid và chưa dùng bản đồ xây online từ LiDAR.

Start `(0,0,5)`, goal `(32,0,5)` theo world ENU. A* inflate 2,6 m; local safety
radius 1,5 m. Test kiểm tra cả đường đã bo góc, nhưng không bảo đảm UAV thực
tế bám đúng đường đó.

## Chạy toàn bộ ablation headless

Dừng các phiên Gazebo/SITL cũ trước. Script từ chối nếu phát hiện phiên đang
chạy; không tự kill phiên của bạn. Nó chỉ điều khiển SITL localhost, **không
dùng với UAV thật**. Mỗi lượt có world và EEPROM mới, chờ EKF, arm, takeoff,
kiểm tra vị trí xuất phát, chạy MPPI, LAND rồi dừng các process do nó tạo.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --speeds 0.4 0.6 0.8 1.0 1.2 --seeds 7 17 \
  --output output/benchmark/industrial_speed_my_run
```

Tên thư mục output phải mới để không ghi đè log. Ctrl+C dừng thử nghiệm và
thực hiện cleanup/LAND; đợi script kết thúc trước khi mở phiên mới.

Các lượt dùng **cùng một đường A* đã tính trước**, cùng config, cùng seeds,
cùng giới hạn và cost. Script truyền đường đã chốt qua `--global-path` để
sai khác vài centimet sau takeoff không làm A* đổi đường giữa các lượt.
Đây không phải waypoint tự vẽ tay.

Chỉ thay `reference_speed_m_s`: tốc độ tiến dọc reference, không phải tốc độ
đo được hay giới hạn tuyệt đối UAV. `vmax=1.2` giữ cố định và hiện là giới hạn
từng thành phần vx/vy. `goal_slowdown_radius=0` tắt nhánh hút thẳng về đích,
để không làm sai phép so sánh tốc độ. Không áp dụng thay đổi này tự động cho
các profile demo cũ.

File setting: `config/experiments/mppi_industrial_ablation.yaml`.
Giữ H=30, dt=0,1 s, N=350, tần số yêu cầu 10 Hz; mọi cost, noise, bộ lọc và
giới hạn gia tốc giống nhau. Không tune riêng từng tốc độ trong cùng sweep.

## Xem trong Gazebo và RViz

Không mở một Gazebo server mới khi sweep còn chạy. Muốn xem một lượt bằng
GUI, dùng quy trình 5 terminal hiện tại và thay world/cao độ/lệnh MPPI:

### Terminal 1 — Gazebo server

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"
gz sim -v2 -r worlds/iris_mppi_industrial_yard.sdf -s
```

### Terminal 2 — Gazebo GUI

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"
gz sim -v1 -g --gui-config "$PWD/config/gazebo_runway_camera.config"
```

Đây là GUI nối vào server Terminal 1, không khởi động thêm world thứ hai.

### Terminal 3 — SITL + MAVProxy

```bash
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"
cd ~/Projects/ardupilot
MAP_SERVICE=MicrosoftSat python3 Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f JSON -N -w \
  -A "--serial1=tcp:2" \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file="$HOME/Projects/ardupilot_gazebo/config/mppi_velocity.parm"
```

Đợi EKF/pre-arm sẵn sàng, rồi nhập trong MAVProxy:

```text
mode guided
arm throttle
takeoff 5
```

Không force-arm hay tắt arming checks khi bị từ chối; đọc lỗi và đợi sensor
ổn định. UAV cần đang hover gần world `(0,0,5)`, không dùng takeoff 20.

### Terminal 4 — ROS bridge + RViz

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45
./scripts/run_sensor_rviz.sh
```

### Terminal 5 — chọn tốc độ rồi chạy MPPI

Chọn **một** dòng tương ứng lượt muốn xem:

```bash
YARD_SPEED=0.4
```
```bash
YARD_SPEED=0.6
```
```bash
YARD_SPEED=0.8
```
```bash
YARD_SPEED=1.0
```
```bash
YARD_SPEED=1.2
```
```bash
YARD_SPEED=5.0
```
```bash
YARD_SPEED=10.0
```

Các mức 0,4–1,2 dùng profile ablation gốc; 5/10 dùng profile retiming có
`vmax=10.0`. Hai nhóm khác cấu hình, phân tích riêng. Tốc độ ở đây là cruise
ngang XY; cao độ vẫn giữ 5 m, không phải tốc độ bay lên theo trục Z.

Sau đó copy block chung dưới đây vào **cùng Terminal 5**. Đường reference là
kết quả A* đã chốt trong sweep, không đổi đường theo sai số vị trí takeoff.
Giữ seed 7 để so sánh, hoặc thay `YARD_SEED=17` để xem lượt còn lại.

```bash
cd ~/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_mppi_challenge
export ROS_DOMAIN_ID=45
YARD_SEED=7
YARD_RUN_TAG=$(date +%Y%m%d_%H%M%S)
: "${YARD_SPEED:?Chon YARD_SPEED truoc khi chay}"
case "$YARD_SPEED" in
  5|5.0|10|10.0)
    YARD_PROFILE=retimed
    ;;
  0.4|0.6|0.8|1.0|1.2)
    YARD_PROFILE=ablation
    ;;
  *) echo "YARD_SPEED khong hop le: $YARD_SPEED"; return 1 2>/dev/null || exit 1 ;;
esac
YARD_CONFIG="config/experiments/mppi_industrial_${YARD_PROFILE}.yaml"
MAVLINK20=1 /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py \
  --planner mppi --config "$YARD_CONFIG" \
  --mav tcp:127.0.0.1:5762 \
  --goal '32,0,5' \
  --global-path '0,0,5;3.4,1.9,5;14.9,1.9,5;15.4,-2.1,5;27.9,-2.1,5;32,0,5' \
  --reference-speed-m-s "$YARD_SPEED" --seed "$YARD_SEED" \
  --diag-every 10 \
  --diag-jsonl "output/log/yard_gui_${YARD_PROFILE}_v${YARD_SPEED}_seed${YARD_SEED}_${YARD_RUN_TAG}.jsonl" \
  --rviz-traj-topic /mppi/predicted_path \
  --rviz-samples-topic /mppi/sampled_trajectories --rviz-top-k 5 \
  --exit-on-goal
```

RViz: Fixed Frame `odom`, thêm Path `/mppi/predicted_path` và MarkerArray
`/mppi/sampled_trajectories`. Nếu máy nặng khi GUI/RViz chạy cùng lúc, số đo
độ trễ sẽ khác headless; không gộp hai loại lượt vào cùng phép so sánh.

`--exit-on-goal` chỉ xác nhận vào bán kính 0,35 m, **không xác nhận đã dừng
hẳn**. Sau demo dùng `mode land`, đợi DISARMED. Muốn lặp đúng start phải
khởi động lại cả Gazebo và SITL, không chỉ chạy lại MPPI từ cuối tuyến.

**Đổi tốc độ, reset để so sánh:** Terminal 5 Ctrl+C nếu còn chạy → Terminal 3
`mode land`, đợi `DISARMED` → Ctrl+C Terminal 3, 4, 2, 1 → mở lại theo thứ tự
1–4 → `takeoff 5` → chọn tốc độ mới và chạy block Terminal 5. Không chạy năm
planner đồng thời, không dùng `flyto` hay click goal mới trong lượt ablation.
Trong cùng nhóm profile, map/cost/noise/giới hạn giữ nguyên; chỉ thay
`YARD_SPEED`. Khi so sánh 5 với 10 m/s, cả hai cùng dùng profile `retimed`.

## Đọc kết quả

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/analyze_yard_speed_ablation.py \
  output/benchmark/industrial_speed_my_run
```

Đầu ra: `summary.csv`, `summary.json`, `comparison.png`. Mỗi lượt giữ
`planner.jsonl`, `ground_truth.jsonl`, log Gazebo/SITL, sự kiện MAVLink,
`result.json` và câu lệnh thực thi. Manifest ghi đường A*, hash source/config,
commit repo và ArduPilot. Worktree chưa commit được ghi riêng, không xem hash
HEAD là toàn bộ phiên bản code đang thử.

Phân biệt:

- Thời gian bay, vận tốc và gia tốc: từ timestamp **Gazebo**, không lấy quãng
  đường chia thời gian wall-clock khi real-time factor khác 1.
- Compute MPPI và `cycle_to_send_ms`: wall-clock. Compute nhỏ không đủ chứng
  minh toàn pipeline đạt 10 Hz; đọc thêm chu kỳ và deadline miss.
- Clearance: khoảng cách **tâm UAV đến solid SDF**; chưa trừ bán kính rotor,
  không thay thế log contact vật lý. LiDAR có thể downsample/mất điểm nên
  không chỉ dựa vào `nearest` để kết luận không va chạm.
- Smoothness: RMS gia tốc XY đo được và RMS thay đổi lệnh, cùng với đồ thị;
  tốc độ thấp hơn không tự động bảo đảm hết giật hoặc không dừng trước góc.
- Lượt `SETUP_FAILED` chưa bay không tính vào tỷ lệ navigation success.
  `TIMEOUT`, brake hoặc abort trong lúc bay phải giữ lại, không loại để làm
  kết quả đẹp. Hai seeds chỉ là khảo sát nhỏ, chưa chứng minh độ tin cậy.

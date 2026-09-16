# Kết quả protocol A–D — 16/09/2026

Kiểm chứng tiếp theo: [tầng an toàn và bài phanh độc lập](MPPI_MENTOR_SAFETY_PHASE_VI.md).

## Kết luận hiện tại

**Cruise của objective progress đã đạt trên đường thẳng:** hai seed giữ gần
10 m/s liên tục khoảng 22 giây. Không có bằng chứng cần tăng reward để thắng
effort/smoothness trong điều kiện thoáng đã thử.

**Chưa thể xác nhận model phanh đạt yêu cầu:** replay cùng lệnh thực cho sai
số vận tốc vừa phải nhưng sai số dịch chuyển có thể đáng kể so với margin.
Đồng thời có nominal qua collision gate nhưng dự đoán vi phạm guard quãng
dừng. Chưa đủ dữ liệu để kết luận sampling, weighting hay model là nguyên
nhân duy nhất. Không đổi controller, objective hoặc guard trong đợt kiểm chứng.

## A. Cruise: flight mới và J8/J10 cùng state

Chạy Gazebo/SITL headless, world straight 300 m, seeds 7/17,
`mppi_protocol_straight10.yaml`: sao nguyên cấu hình progress-cruise yard,
chỉ đổi known SDF sang đúng world straight. World có mặt nền asphalt và UAV,
không có container chắn đường. Retiming tắt; không bỏ ground geometry khỏi map.

Timeout toàn nhiệm vụ 90 s vì bài dài 300 m; optimizer timeout vẫn 90 ms.
Cả hai tới đích và LAND/disarm, không hold-brake hoặc hold-invalid theo event log.

| Chỉ số | Seed 7 | Seed 17 |
|---|---:|---:|
| Peak XY m/s | 10.003 | 10.003 |
| Liên tục trong 9.5–10.5 m/s, s sim | 21.9 | 22.2 |
| Tốc độ trung bình trong cửa sổ cruise m/s | 9.970 | 9.970 |
| Thời gian nhiệm vụ s sim | 34.068 | 33.864 |
| Compute trung bình ms | 24.03 | 23.90 |
| Cycle deadline miss | 0 | 1 |
| Hold-timeout | 0 | 1 |

Seed 17 còn một hold-timeout; không mô tả toàn bộ chuyến bay là không có
safety intervention. Soft speed cap cho phép overshoot nhỏ; không phải hard
constraint 10 m/s. Cruise criterion ±5% được kiểm chứng, không phải tốc độ
luôn chính xác bằng 10.

### So cost có kiểm soát

Hai raw candidate là `[8,0,0,0]` và `[10,0,0,0]` lặp trong horizon 3 s.
Mỗi cặp dùng cùng measured state, acceleration memory, previous applied
command, geometry, conditioner và model. Tên candidate là raw target, không
phải giả định vận tốc vật lý lập tức bằng target.

Chọn state gần steady: có ít nhất 5 mẫu trong 1 s trước t0, range tốc độ
<0.3 m/s, norm acceleration memory XY <0.3 m/s². Đây là tiêu chí thao tác,
không chứng minh cân bằng động lực học hoàn hảo. Không zero bộ nhớ model.

| Snapshot steady | J8 | J10 | J10 − J8 |
|---|---:|---:|---:|
| Seed 7, cycle 226 | -127400.49 | -149842.19 | -22441.70 |
| Seed 17, cycle 196 | -127522.74 | -149962.49 | -22439.75 |

Cả hai state có vx≈10 m/s. Progress reward thắng phần tăng effort, J10 thấp
hơn J8. Ở seed 7, J10 gồm progress≈-150000, path≈127.81, effort≈30,
input-change≈0; speed-limit gần 0, collision/stop/terminal/reference-velocity=0.
J8 có progress≈-127737.72 và input-change≈189.18 do phải giảm setpoint.
Đây là đúng phép so từ cùng state, nhưng không phải so hai trạng thái steady
khác nhau. Hai snapshot ở khoảng 8 m/s đang tăng tốc được báo riêng trong JSON,
không gán nhãn steady. Trong chúng J10 cũng thấp hơn J8.

Kết luận hẹp: với profile này và world thoáng, không thấy objective ưu tiên
8–9 m/s thay vì 10; không loại trừ ảnh hưởng proposal trong hoàn cảnh khác.

## B. Model validation bằng lệnh thực

Dùng log yard progress-cruise trước đó, không chạy lại chuyến yard để tránh
đổi điều kiện quan sát. Đã đối chiếu hash controller, node và known_geometry
hiện tại với source snapshot của đợt yard: khớp.

8 state: hai seed, gần x=40/45/50/53 m. Có cả state tại cycle bị từ chối,
không chỉ chọn các chu kỳ thành công. Initial state và acceleration memory
lấy từ nominal snapshot; previous applied command giữ nguyên.

Input lấy từ `sent_mavlink_control`, đổi NED→ENU, gồm yaw-rate đổi dấu;
**không conditioner lần hai**. Giữ lệnh ZOH, chia bước tại thời điểm lệnh mới.
Mapping thời điểm gửi wall→simulation dùng cặp timestamp ground truth.
Chưa có timestamp nhận/thực thi ở ArduPilot.

Mỗi state thử delay 0/0.1/0.2 s và bước tích phân tối đa 0.02/0.1 s:
48 cửa sổ, mỗi cửa sổ báo 0.5/1/2 s. Đây là sensitivity sweep, không fit delay
rồi tuyên bố một giá trị là đúng. Không có gaps gửi lớn hơn khoảng 0.16 s trong
vùng kiểm tra; không dùng giả định mỗi cycle đúng 0.1 s.

GT velocity/acceleration lấy đạo hàm position sau resample 0.1 s; có nhiễu và
sai số đạo hàm. Báo velocity error so với odometry riêng. Position displacement
error loại offset vị trí ban đầu; absolute position error vẫn lưu trong JSON.

### Delay=0, bước tối đa 0.02 s, trung bình trên 8 state

| Horizon | Sai số velocity XY TB / max m/s | Sai số dịch chuyển TB m |
|---|---:|---:|
| 0.5 s | 0.348 / 0.795 | 0.133 |
| 1 s | 0.415 / 0.983 | 0.319 |
| 2 s | 0.426 / 0.950 | 0.700 |

Sai số dịch chuyển lớn nhất sau 2 s **1.614 m**. So với odometry, velocity
error trung bình ở 0.5/1/2 s là 0.236/0.417/0.525 m/s. Không gọi đây là chứng
minh model đúng vì sai số trung bình velocity không quá lớn: sai số vị trí
và hướng quan trọng khi sát container.

Delay=0.1 s cho velocity error TB 0.316/0.356/0.437 m/s; displacement error
sau 2 s≈0.633 m. Delay=0.2 s không cải thiện GT đồng đều, nhưng velocity error
so odometry giảm hơn. Điều này đòi hỏi kiểm tra timestamp/state-estimation
latency, không đủ cơ sở gán delay thực bằng 0.2 s.

Với max step=0.1 s, delay=0, velocity error TB sau 2 s≈0.469 m/s, max
displacement error≈1.354 m. Kết quả nhạy với discretization; substep 0.02 s
là tích phân cùng công thức surrogate, không hoàn toàn giống lịch rollout
0.1 s của optimizer. Không chỉ chọn con số tốt hơn để báo cáo.

JSON lưu cả vector position/velocity/acceleration, velocity-direction error
(khi đủ tốc độ), clearance sampled của predicted/actual path theo cùng SDF.
Clearance sampled không phải chứng nhận swept clearance; initial state dùng
odometry còn đối chiếu GT có thể chứa thêm lệch estimator. Chưa tách hoàn toàn
model, delay và state-estimation error. Chưa có tiêu chí error budget được
mentor xác nhận để gắn nhãn model PASS/FAIL.

## C. Decision validation

Rollout raw nominal U bằng model/conditioner, không dùng chuỗi lệnh thực để
thay nominal. Ghi predicted velocity sau 0.5/1/2 s, cost, cloud/SDF gate,
minimum stopping-segment clearance trên mọi bước dự báo, raw/conditioned/sent.

| Seed / cycle | x m | Event | Cloud gate | SDF gate | Min predicted stopping clearance m |
|---|---:|---|---|---|---:|
| 7 / 70 | 40.107 | command | Pass | Pass | **1.357** |
| 7 / 76 | 45.391 | command | Pass | Pass | 2.395 |
| 7 / 82 | 50.194 | hold-invalid | Pass | Fail | 2.622 |
| 7 / 86 | 53.226 | hold-invalid | Pass | Fail | 2.754 |
| 17 / 69 | 39.895 | command | Pass | Pass | 1.636 |
| 17 / 75 | 44.931 | command | Pass | Pass | 2.444 |
| 17 / 81 | 49.843 | command | Pass | Pass | 2.559 |
| 17 / 86 | 53.279 | command | Pass | Pass | 2.485 |

Ngưỡng brake là 1.5 m. Tại seed 7/cycle 70, nominal qua collision gate nhưng
có trạng thái tương lai vi phạm tiêu chí đoạn dừng. Hai kiểm tra này khác nhau:
quỹ đạo tránh va chạm không đồng nghĩa mọi state trên đó đều có đoạn dừng
thẳng an toàn. Stopping soft cost hiện kiểm tra thưa mỗi 5 bước, không là hard
constraint. Chưa xác định riêng việc kiểm tra thưa hay weighting gây lựa chọn đó.

Trong 6 state được gửi command, conditioned command khớp sent command.
Hai state hold-invalid gửi zero đúng gate. Ví dụ seed 17/cycle 86 raw vx≈0.674,
conditioned/sent vx≈4.325 m/s: conditioner thay đổi request đáng kể, nhưng nó
đã có trong rollout; riêng chênh lệch này chưa chứng minh execution bug.

Các cửa sổ tương lai có brake/rejection nên không so nominal-vs-actual rồi
gán toàn bộ sai lệch cho model. Không tạo nominal mới giả cho cycle hold-brake.
Phần C kiểm tra snapshot sẵn có, không phải exact replay RNG/optimizer.

## D. Phân nhánh sau bằng chứng

1. **Không tăng reward cruise lúc này:** phần A đạt; giữ objective progress.
2. **Chưa chọn model “đúng”:** error budget vị trí tới 1.61 m và độ nhạy delay/
   discretization cần được giải quyết trước khi tune mạnh sampling. Cần log
   timestamp/state nội bộ ArduPilot hoặc bài phanh có lệnh định trước để tách
   initialization, estimator delay và response model.
3. **Có bằng chứng về khác biệt tiêu chí gate và braking feasibility:** dùng
   cycle 70 làm case regression; không nới hoặc tắt brake để loại event.
4. **Chưa kết luận safe samples có/không:** log lịch sử chưa chứa toàn bộ
   sample pool, costs và weights của đúng cycle. Nominal/ESS không đủ chứng
   minh weighting sai. Do model chưa qua error budget, chưa tự đổi proposal,
   noise, horizon hay weighting trong đợt này.

Protocol đã thực hiện A, B, C và phân nhánh theo bằng chứng; chưa phải tuyên
bố đã sửa brake. Phần selection đầy đủ còn cần thu thập dữ liệu nếu sau kiểm
chứng model vẫn phải đi theo nhánh đó. Mẫu thử chỉ hai seed, không đại diện
cho mọi tình huống.

## Tái lập và dữ liệu

- [Straight manifest và source snapshot](../output/benchmark/protocol_straight10_20260916_v1/manifest.json).
- [Straight summary](../output/benchmark/protocol_straight10_20260916_v1/summary.json), [cruise check](../output/benchmark/protocol_straight10_20260916_v1/cruise_check.json).
- [Toàn bộ model/decision/candidate cost](../output/benchmark/progress_protocol_analysis_20260916/analysis.json).
- [Summary sai số và source hashes](../output/benchmark/progress_protocol_analysis_20260916/summary.json).
- [Script phân tích](../scripts/run_progress_protocol_analysis.py).

Đã kiểm tra cost component sum, command frame conversion qua đối chiếu
conditioned=sent ở accepted states, rejected sent=zero và sai số hữu hạn.
Đây là kiểm tra dữ liệu/kết quả, không phải một bộ test navigation mới.

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_progress_protocol_analysis.py \
  --straight output/benchmark/protocol_straight10_20260916_v1 \
  --output output/benchmark/progress_protocol_analysis_repeat
```

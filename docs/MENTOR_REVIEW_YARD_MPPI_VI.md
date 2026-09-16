# Báo cáo trao đổi mentor: MPPI 5/10 m/s trong bãi container

**Checkpoint mới nhất:** [Experiment 7A, 1.600 replay và decision branch H1/H2/H3](MPPI_MENTOR_CHECKPOINT_AND_NEXT_PHASE_VI.md).

**Đã chạy protocol A–D:** [kết quả cruise, model và decision validation](MPPI_PROTOCOL_RESULTS_20260916_VI.md).

**Kiểm chứng safety sau góp ý mentor:** [hard stopping guard, 40 pha phanh độc lập và sample pool](MPPI_MENTOR_SAFETY_PHASE_VI.md). Hard guard chặn case regression nhưng hai seed headless vẫn timeout; chưa coi đây là sửa xong bài cua.

**Sửa selection theo feasibility:** [shared predicate, mask trước weighting và kết quả hai seed](MPPI_FEASIBLE_SELECTION_VI.md). Profile 80 sample đã tới đích 2/2; recovery khi không có sample feasible và model phanh live vẫn chưa đạt DoD.

**Cập nhật objective sau trao đổi:** đã thống nhất 10 m/s là cruise mong muốn/
giới hạn, UAV được tự chậm ở cua. Đã triển khai nhánh objective hình học +
progress, bỏ time-indexed position/velocity tracking, và thử nghiệm riêng.
Xem [triển khai, replay và kết quả mới](MPPI_PATH_PROGRESS_VI.md).
Các bảng dưới giữ nguyên kết quả lịch sử trước thay đổi objective.

Ngày tổng hợp: **16/09/2026**. Nguồn: code, tài liệu audit và log Gazebo/ArduPilot SITL trong repository. Đây là báo cáo kết quả hiện có, không phải kết quả một đợt kiểm chứng mới.

## 1. Nội dung có thể trình bày ngay với mentor

> Hệ mô phỏng đã bay và giữ được gần 5/10 m/s trên đường thẳng. Trong bài rẽ giữa container, bật retiming thì hai lượt đã hoàn thành, không kích hoạt brake khẩn cấp. Khi bỏ retiming để MPPI tự quyết định giảm tốc, ban đầu có lỗi reset proposal và thiếu thông tin vật cản trong horizon; sau khi sửa, UAV đã qua cua và tới đích ở hai seed. Tuy nhiên vẫn có một đợt brake trước cua. Các thử nghiệm thêm proposal giảm tốc, cost quãng dừng và tăng horizon giúp giảm rejection hoặc thay đổi lệnh, nhưng chưa loại brake ổn định. Cần xem lại sự phù hợp giữa objective bám reference theo thời gian, mô hình đáp ứng khi phanh và tiêu chí quãng dừng của lớp an toàn.

**Không kết luận “MPPI không chạy được 10 m/s” hoặc “máy tính quá chậm” cho toàn bộ vấn đề.** Có các lượt đạt tốc độ nhưng không qua cua; có các lượt qua cua nhưng còn phanh; chỉ một số cấu hình cost nặng mới vượt ngân sách tính toán.

## 2. Mục tiêu và tiêu chí thành công

Mục tiêu ban đầu là trình diễn UAV bay 5 và 10 m/s trong Gazebo 3D. Sau đó phạm vi nghiên cứu được chốt lại: **retiming tắt, MPPI tự tối ưu lệnh giảm tốc và đổi hướng**, giữ các giới hạn động lực học và kiểm tra an toàn.

Cần phân biệt:

| Tiêu chí | Trạng thái hiện tại |
|---|---|
| Có thể đạt/giữ gần 5 và 10 m/s trên đường thoáng | Đã kiểm chứng đường thẳng 300 m với cấu hình riêng có retiming |
| Có thể đạt gần 10 m/s trước cua trong bài container | Có, nhiều lượt peak khoảng 9.6–9.8 m/s |
| Qua cua và tới đích khi tắt retiming | Có, các cấu hình sau sửa reset đã hoàn thành hai seed |
| Qua cua ít/không có safety hold hoặc emergency brake | Chưa ổn định |
| Giữ gần 10 m/s liên tục ít nhất 5 s trong bài container | Chưa đạt ở các cấu hình được xác nhận gần đây |
| Giữ đúng 10 m/s xuyên cua | Không phải kết quả đã đạt; vẫn cần giảm tốc ở cua |
| 5 m/s với cơ chế recovery/proactive/stopping mới | Chưa có bộ kiểm chứng tương ứng trong các đợt gần đây |

“Reference 10 m/s ngay từ đầu” là yêu cầu cho bộ tối ưu, không phải đặt vận tốc vật lý ban đầu bằng 10 m/s. Các lượt bắt đầu từ hover; ArduPilot và động lực học Gazebo vẫn phải thực hiện quá trình tăng tốc.

## 3. Bài toán mô phỏng và kiến trúc hiện tại

### Thiết lập chung

- World: `worlds/iris_mppi_yard_runup60.sdf`, điểm xuất phát gần `(0,0,5)`.
- Đường tham chiếu thô: `(0,0,5) → (60,0,5) → (60.5,-4,5) → (73,-4,5) → (77.1,-1.9,5)`; có bo hình học reference.
- Cua đầu cách xuất phát 60 m, cao độ yêu cầu 5 m, bán kính tới đích 0.35 m.
- Các đợt so sánh gần đây: reference 10 m/s, seeds 7/17, timeout nhiệm vụ 45 s wall, fresh Gazebo/SITL mỗi lượt.
- SITL dùng `mppi_yard_high_accel.parm`; đọc lại `WP_SPD=10 m/s`, `WP_ACC=3 m/s²`.
- Chu kỳ yêu cầu 10 Hz; `dt=0.1 s`; horizon thường 30 bước = 3 s, 350 samples; thử cuối 40 bước = 4 s, 200 samples.
- Optimizer timeout 90 ms; cả chu kỳ còn có đọc dữ liệu, validation, diagnostics và gửi lệnh.

### Vai trò từng phần

| Thành phần | Vai trò |
|---|---|
| Global path/reference generator | Cung cấp hình học đường và reference theo thời gian |
| Retiming (đang tắt) | Khi bật, điều chỉnh tốc độ reference trước cua theo giới hạn gia tốc; đây là phần trong dự án, không phải ArduPilot |
| MPPI | Rollout các chuỗi lệnh vận tốc, tính cost, cập nhật chuỗi điều khiển |
| Command conditioner | Lọc và giới hạn tốc độ thay đổi setpoint; đã được đưa vào rollout |
| Final trajectory gate | Kiểm tra quỹ đạo cuối sau xử lý lệnh đầu; vi phạm thì gửi zero |
| Brake guard | Xét trạng thái hiện tại và đoạn dừng theo vận tốc; có thể can thiệp trước khi gọi optimizer |
| ArduPilot SITL | Nhận setpoint vận tốc qua MAVLink và điều khiển UAV |
| Gazebo | Mô phỏng chuyển động, sensor và môi trường |

**Tắt retiming không đồng nghĩa dùng MPPI nguyên bản.** Bản hiện tại có prior map SDF, surrogate đáp ứng, conditioner, gate, brake guard và proposal recovery/proactive do dự án bổ sung. Cần mô tả rõ các phần này khi báo cáo phương pháp.

## 4. Tiến trình thí nghiệm và những gì học được

### 4.1. Đường thẳng: xác nhận hệ thống có khả năng đạt tốc độ

Đường 300 m, cấu hình riêng có retiming, hai seed mỗi tốc độ:

| Yêu cầu | Peak seed 7 / 17 | Thời gian liên tục trong ±5% seed 7 / 17 |
|---|---:|---:|
| 5 m/s | 5.144 / 5.176 m/s | 51.4 / 51.4 s |
| 10 m/s | 9.928 / 9.927 m/s | 11.9 / 12.1 s |

4/4 tới đích và LAND/disarm. Có một số deadline miss ở đầu/ngoài pha cruise; không tuyên bố hard real-time. Kết quả này chỉ xác nhận khả năng tốc độ trên đường thoáng, không xác nhận tránh container ở 10 m/s.

### 4.2. Bài cua có retiming: mốc tham khảo

Hai seed tới đích, peak 9.579/9.547 m/s, 0 brake khẩn cấp theo log của đợt đó. Thời gian nhiệm vụ 19.652/19.720 s, clearance tâm nhỏ nhất 2.516/2.488 m. Tốc độ trung bình trong vùng cua chỉ khoảng 1.54/1.56 m/s.

Retiming hỗ trợ bằng cách giảm yêu cầu tốc độ trước cua. Không thể dùng kết quả này làm bằng chứng MPPI tự quyết định toàn bộ lịch giảm tốc. Đây cũng không phải đối chứng chỉ khác một biến với bản hiện tại: code, model và gate đã thay đổi qua nhiều đợt.

### 4.3. Tắt retiming, chỉ tune ban đầu: chưa giải quyết được

Baseline đạt peak khoảng 9.84 m/s nhưng hai seed abort gần container. Năm cấu hình screening thay horizon, samples, noise, lambda, weights, radius hoặc tau đều abort guard hình học ở seed 7. Không có deadline miss trong năm lượt này, nên thiếu tốc độ tính toán không giải thích được các thất bại đó.

Đây là screening thay nhiều biến, không phải ablation độc lập. Abort vùng đệm không đồng nghĩa đã xác nhận va chạm rotor bằng contact sensor.

### 4.4. Audit: sửa diagnostics và thêm gate quỹ đạo cuối

Các vấn đề đã phát hiện/sửa:

1. Bộ nhớ lệnh thực từng ghi đè raw `U[0]`, làm diagnostics/rollout chịu conditioning sai. Đã tách bộ nhớ lệnh thực và nominal U.
2. MPPI cập nhật bằng trung bình nhiễu có trọng số; không có bảo đảm quỹ đạo cuối hợp lệ chỉ vì các mẫu hợp lệ. Đã thêm final gate.
3. Cost collision xét điểm rollout rời rạc, gate xét đoạn liên tục/bảo thủ hơn. Có thể cost collision bằng 0 nhưng gate từ chối.
4. Cloud không chứa toàn bộ vật cản trong horizon. Một log cloud chỉ tới x=57.16 m, container phía trước bắt đầu x=63.1 m; gate cloud pass nhưng đối chiếu SDF offline cho thấy vi phạm margin.

Chưa đủ dữ liệu cloud gốc để phân biệt phần đóng góp của FOV, che khuất và downsampling (giữ tối đa 200 điểm gần). Không coi vùng không có điểm là đã biết an toàn.

### 4.5. Prior map và mô hình gia tốc/phanh

Thêm hình học SDF đã biết vào cost/gate, không giả vờ đây là dữ liệu LiDAR. Bổ sung surrogate có bộ nhớ gia tốc, giới hạn gia tốc và jerk: tau=0.5 s, acceleration XY=3 m/s², jerk XY=4 m/s³; Z vẫn first-order.

Fit offline cho RMSE vận tốc XY sau 1 s giảm từ 2.863/2.911 xuống 0.280/0.274 m/s (seed 7 fit, seed 17 kiểm tra). Tuy nhiên initial acceleration dùng đạo hàm ground truth, khác estimator live; cửa sổ trượt tương quan; chưa tách trễ truyền lệnh/ArduPilot. **Không suy ra sai số live hay quãng phanh đã được bảo đảm.**

Hai lượt sau bổ sung map/model vẫn timeout, dừng x≈53.6/54.4 m, với 369/368 lần rejection. Compute khoảng 20 ms: đây là kẹt tìm/chấp nhận quỹ đạo, không phải optimizer quá chậm.

### 4.6. Sửa reset/proposal: giải quyết vòng lặp kẹt

Sau rejection, code cũ reset trạng thái warm-start, chu kỳ sau lại gán toàn bộ proposal theo reference 10 m/s. Kết quả tối ưu trước đó bị bỏ đi. Probe tại trạng thái đứng xác nhận có chuỗi đi chậm hợp lệ với nominal cost thấp hơn chuỗi bị loại.

Đã sửa: ghi nhận zero thực gửi; chỉ khởi tạo zero một lần khi vào recovery; giữ U để tối ưu tiếp; thêm proposal dừng và đi chậm theo path. Hai seed đã tới đích trong timeout 45 s giữ nguyên. Đây là sửa hành vi reset/sampling, không phải tăng thời gian chờ.

### 4.7. Chủ động giảm tốc và làm mượt

Thử đưa proposal giảm tốc–rẽ vào pool trước rejection, tăng phạt đổi lệnh XY 100→400. Chỉ hai thay đổi này làm rejection tăng lên 46/63. Thêm buffer 0.55 m vào collision cost giúp giảm rejection xuống 11/11; gate vẫn radius 1.5 m. Vẫn có 12 chu kỳ hold-brake mỗi lượt.

Tiếp tục thêm cost quãng dừng:

`L = |v| × delay + |v|² / (2a)`

`cost_stop = w_stop × max(0, margin_stop − clearance_segment)²`

Bản tính cả prior map ở mọi bước quá nặng (~136 ms), phải dừng có LAND. Bản cuối dùng cloud, tính mỗi 5 bước dự báo; margin 2.5 m, weight 1e6, delay 0.25 s. Gate/brake vẫn kiểm tra mỗi chu kỳ. Đây là soft cost, không phải ràng buộc an toàn cứng.

## 5. Bảng kết quả chính sau sửa reset

Mỗi ô ghi **seed 7 / seed 17**. Tất cả các hàng dưới đều tới đích và LAND/disarm hai lượt; retiming tắt. Số hold là số chu kỳ, không phải số đợt phanh độc lập.

| Cấu hình | Peak m/s | Hold-invalid | Hold-brake | Recover-brake | RMS thay đổi lệnh m/s² | Thời gian sim s |
|---|---:|---:|---:|---:|---:|---:|
| Recovery sau rejection | 9.615 / 9.708 | 23 / 20 | 0 / 14 | 0 / 18 | 8.200 / 8.817 | 19.856 / 21.590 |
| Proactive + buffer | 9.645 / 9.680 | 11 / 11 | 12 / 12 | 0 / 0 | 6.333 / 6.141 | 19.312 / 18.054 |
| Stopping cost cloud, H=30/N=350 | 9.599 / 9.496 | 0 / 9 | 12 / 15 | 0 / 0 | 4.678 / 5.675 | 21.182 / 18.836 |
| Stopping cost cloud, H=40/N=200 | 9.096 / 9.319 | 1 / 1 | 15 / 11 | 0 / 0 | 4.296 / 4.393 | 18.700 / 19.890 |

| Cấu hình | Clearance tâm min m | Cycle deadline miss |
|---|---:|---:|
| Recovery | 1.624 / 1.793 | 0 / 0 |
| Proactive + buffer | 2.017 / 2.084 | 3 / 0 |
| Stopping H=30 | 1.946 / 2.268 | 8 / 15 |
| Stopping H=40 | 2.167 / 2.156 | 6 / 7 |

Bản H=40 giảm RMS thay đổi lệnh khoảng 28–32% so với proactive-buffer, nhưng tốc độ đỉnh giảm và brake chưa giảm đồng đều. Không thể gọi là đã sửa xong. RMS lệnh thấp hơn cũng không tự chứng minh quỹ đạo bám path tốt hơn hay chuyển động vật lý mượt hơn; phải xem thêm gia tốc/jerk, sai lệch path và video.

## 6. Brake hiện tại xảy ra như thế nào?

Ví dụ seed 7, stopping cost cloud H=30:

| Vị trí x | Lệnh vx | Vận tốc vx đo | Ý nghĩa |
|---|---:|---:|---|
| 42.60 m | 7.93 m/s | 9.58 m/s | Đã bắt đầu hạ lệnh |
| 53.31 m | 4.78 m/s | 6.67 m/s | UAV còn nhanh hơn setpoint đáng kể |
| 53.93 m | Gửi zero do brake | Tốc độ khoảng 6.36 m/s | Đoạn dừng 8.36 m, clearance 1.36 m < 1.5 m |

Ở seed 17, đợt brake bắt đầu x≈54.19 m, tốc độ≈6.67 m/s, đoạn dừng≈9.07 m, clearance≈1.01 m. Các hold-brake tập trung thành một đợt trước cua. Không nên diễn giải “15 brake cycles” là 15 lần phanh độc lập.

**Điều đã biết:** lệnh đã giảm nhưng trạng thái thực vẫn kích hoạt guard quãng dừng. **Điều chưa biết:** mức đóng góp riêng của sai số model, trễ gửi/thực thi, gia tốc/jerk thực, reference theo thời gian và độ bảo thủ của guard. Chênh lệch vận tốc với setpoint tự nó chưa chứng minh model sai: model cũng có lag. Cần so dự đoán và đo thực theo cùng thời điểm.

## 7. Các vấn đề cần mentor đánh giá

| Vấn đề | Bằng chứng | Câu hỏi còn mở |
|---|---|---|
| Objective bám lịch thời gian | Reference vẫn tiến 10 m/s qua cua; cost path/time tạo áp lực đuổi reference | Nên dùng path progress tối ưu/contouring thay cho bám timestamp cố định không? |
| Cost và gate khác tiêu chí | Cost điểm, gate swept; cost_stop chỉ mỗi 5 bước; vẫn có rejection | Có nên đưa kiểm tra swept/khả năng dừng vào tối ưu hoặc chọn lại mẫu hợp lệ cuối cùng? |
| Khám phá proposal | ESS≈1 trong một số log; reset cũ làm mất tiến bộ | Noise tương quan theo thời gian, nhiều lần cập nhật hoặc proposal khác có phù hợp hơn? |
| Mô hình đáp ứng khi phanh | Zero input từng đi kèm vận tốc tăng thêm trong 0.2 s đầu; fit offline chưa đủ | Cần nhận dạng delay, jerk, acceleration ở pha phanh như thế nào? |
| Brake guard dựa đoạn thẳng | Kiểm tra dừng theo hướng vận tốc hiện tại, không phải quỹ đạo rẽ tối ưu | Có nên dùng backup braking rollout được kiểm chứng thay cho đoạn thẳng? |
| Quan sát không đầy đủ | Cloud từng thiếu container trong horizon | Prior map có được chấp nhận trong phạm vi nghiên cứu? Nếu không, cần occupancy/memory thế nào? |
| Ngân sách thời gian | Cost nặng gây hold-timeout; các bản nhẹ vẫn có cycle miss | Ưu tiên vectorization, giảm logging, giảm mẫu hay solver khác? |

Không đề xuất tắt brake/gate để có video mượt. Nếu thay guard, cần chứng minh kiểm tra thay thế phản ánh quãng dừng và bất định tốt hơn, không chỉ giảm số event.

## 8. Kế hoạch kiểm chứng đề xuất sau khi thống nhất với mentor

**Đã chốt thứ tự A–D:** cruise → model bằng lệnh thực → decision bằng nominal → phân nhánh sửa. [Protocol chi tiết và điều kiện kết luận](MPPI_PATH_PROGRESS_VI.md#quy-trình-kiểm-chứng-đã-thống-nhất-a--b--c--d). Đây là kế hoạch, chưa phải kết quả kiểm chứng.

1. **Nhận dạng pha phanh riêng:** đường thoáng, lệnh 10→8→5→0 và rẽ có kiểm soát; log setpoint gửi/nhận, vận tốc, gia tốc, attitude, target nội bộ ArduPilot nếu có. So rollout tại nhiều mốc trước cua với đo thực ở 0.5/1/2 s, quãng dừng và thời điểm đổi dấu gia tốc.
2. **Replay cùng trạng thái trước brake:** giữ cloud/map, state, U và RNG; so nominal, mẫu tốt nhất, proposal giảm tốc; ghi cost thành phần, clearance swept, stopping clearance, ESS. Phân biệt không có mẫu tốt, mẫu tốt bị cost loại, hay dự đoán tốt nhưng thực thi sai.
3. **Thống nhất bài toán điều khiển:** “tốc độ mong muốn tối đa 10 m/s, được tự chậm ở cua” hay “phải đuổi reference theo lịch 10 m/s”. Hai objective này khác nhau dù đều không dùng retiming ngoài.
4. **Ablation từng thay đổi:** cố định source/world/params/seeds; lần lượt xét proposal, model, cost_stop, horizon, gate. Các lượt trước thay nhiều yếu tố nên chưa đủ kết luận nhân quả riêng.
5. **Xác nhận lại cả 5 và 10 m/s**, nhiều seed hơn rồi chạy GUI/RViz. Báo tốc độ đoạn thẳng, tốc độ cua, hoàn thành, clearance, số đợt và thời lượng hold, sai lệch path/cao độ, RMS/peak slew và deadline.

Các bước trên là đề xuất, chưa được thực hiện đầy đủ. Người dùng đã thấy brake trên GUI; chưa đối chiếu riêng log GUI đó với từng đợt headless nên chưa quy nguyên nhân GUI cho cùng một event.

## 9. Cấu hình hiện hành và giới hạn báo cáo

- Quickstart mặc định vẫn là `mppi_yard_map_response10.yaml` (recovery); có lựa chọn proactive-margin. Hai cấu hình stopping mới chỉ là thử nghiệm.
- Kiểm thử code cuối đợt: 61 passed và 3 subtests. Đây không phải xác nhận navigation trong mọi tình huống.
- Hầu hết các cấu hình được thử hai seed, không có ước lượng tỷ lệ thành công đáng tin cậy.
- Clearance là tâm UAV tới solid, chưa trừ hình học rotor; abort vùng đệm không đồng nghĩa contact thực.
- Reached là vào bán kính đích, chưa chứng minh settled hover. LAND/disarm được ghi riêng.
- `hold-invalid-trajectory` và `hold-brake` đều có thể gửi zero; `recover-brake` có thể gửi lệnh lùi chậm. Vì vậy “0 brake” theo bộ đếm cũ không đồng nghĩa không có phanh/giữ.
- TIMEOUT nhiệm vụ 45 s khác `hold-timeout` do optimizer quá hạn; cycle deadline cũng khác optimizer compute time.
- Snapshot theo đợt cần dùng khi tái lập. Riêng đợt proactive đầu có thay source mặc định buffer=0 trước seed 17, đã ghi chú trong tài liệu; không dùng nó làm ablation sạch.

## 10. Tài liệu và dữ liệu để mở cùng mentor

- [Quickstart hiện hành](RUN_YARD_5_10_MS_QUICKSTART_VI.md).
- [Nhật ký từ đường thẳng tới bài cua, gồm retiming](RUN_INDUSTRIAL_YARD_ABLATION_VI.md).
- [Screening không retiming ban đầu](TUNE_YARD_MPPI_ONLY_10_MS_VI.md).
- [Audit diagnostics, gate và đáp ứng](AUDIT_MPPI_SELECTION_RESPONSE_VI.md).
- [Prior map và fit model](MPPI_KNOWN_MAP_ACCEL_RESPONSE_VI.md).
- [Nguyên nhân reset bị kẹt](MPPI_HOLD_LOOP_DIAGNOSIS_VI.md), [kết quả sửa recovery](MPPI_REJECTION_RECOVERY_VI.md).
- [Proactive proposal và cost buffer](MPPI_PROACTIVE_PROPOSALS_VI.md).
- [Các thử nghiệm stopping cost](MPPI_STOPPING_COST_VI.md).
- [Đồ thị proactive-buffer](../output/benchmark/yard_proactive_margin10_20260915_v1/comparison.png).
- [Đồ thị stopping H=40](../output/benchmark/yard_stopping_long10_20260916_v1/comparison.png).
- [Raw log và snapshot recovery](../output/benchmark/yard_rejection_recovery10_20260915_v1/manifest.json).
- [Summary stopping H=30](../output/benchmark/yard_stopping10_20260916_v3/summary.json), [vị trí từng đợt brake](../output/benchmark/yard_stopping10_20260916_v3/brake_check.json).
- [Summary stopping H=40](../output/benchmark/yard_stopping_long10_20260916_v1/summary.json).

Source chính: [controller](../mppi_ardupilot/mppi_controller.py), [node/gate](../mppi_ardupilot/mppi_local_planner_node.py), [hình học brake](../mppi_ardupilot/braking.py), [prior map](../mppi_ardupilot/known_geometry.py), [harness](../scripts/run_yard_speed_ablation.py).

# Checkpoint mentor: MPPI 10 m/s và kế hoạch kiểm chứng tiếp theo

Ngày chốt dữ liệu: **16/09/2026**.

Tài liệu này tổng hợp trạng thái nghiên cứu của bài bay bãi container với yêu
cầu cruise 10 m/s. Mục tiêu hiện tại là để MPPI tự chọn chuỗi giảm tốc–rẽ–tăng
tốc, không dùng retiming bên ngoài để gán tốc độ theo vị trí. Các kết luận dưới
đây dựa trên code, log Gazebo/ArduPilot SITL và replay offline đã lưu trong repo.

## 1. Kết luận để trao đổi với mentor

Controller hiện đã qua bài headless ở cả seed 7 và 17, tới đích rồi LAND/disarm.
Lỗi chọn quỹ đạo trước đây — có sample an toàn nhưng sample không an toàn lại
nhận gần toàn bộ weight — đã được sửa bằng feasibility mask dùng chung với gate
cuối. Tuy nhiên controller vẫn có những chu kỳ `N_safe=0` và phải gửi zero;
chuyển động vì thế chưa hoàn toàn mượt.

Experiment 7A đã replay **1.600 solve** từ 20 exact planner snapshots:

- 12 snapshot `N_safe=0`, mỗi snapshot chạy `K=80,160,320,640`, 20 RNG/K;
- 8 snapshot đối chứng `N_safe>0`, cùng ma trận trên;
- tại 12 snapshot lỗi, `P_hit=0` ở mọi K, kể cả 640 sample;
- tại 8 snapshot đối chứng, `P_hit=1` ở mọi K.

Kết quả này làm giả thuyết **H1 — chỉ do 80 sample ngẫu nhiên bỏ lỡ nghiệm**
trở nên rất yếu. Nó là bằng chứng mạnh rằng các state lỗi đang **operationally
infeasible với model, horizon, proposal và constraint hiện tại**. Nó chưa phải
chứng minh toán học rằng không tồn tại nghiệm, và chưa phân biệt được:

- **H2 — viability loss:** controller đã đến state quá muộn để tìm chuỗi hợp lệ;
- **H3 — boundary/model uncertainty:** nghiệm nằm sát biên và cách phân loại phụ
  thuộc mạnh vào stopping model, latency hoặc margin.

Do đó phase tiếp theo chưa tune reward, `lambda` hoặc số mẫu online. Cần chạy
kiểm tra độ nhạy model/biên trên chính 20 snapshot, sau đó mới chọn giữa hiệu
chuẩn model và verified recovery.

## 2. Mục tiêu bài toán đã chốt

```text
Đoạn thẳng thoáng: MPPI được khuyến khích tiến nhanh, tối đa 10 m/s
Gần cua/vật cản:  MPPI được quyền tự giảm tốc và đổi hướng
Sau cua:          MPPI được quyền tăng lại
Retiming:         tắt
Global path:      cung cấp hình học, không áp lịch tốc độ theo timestamp
```

Objective hiện dùng khoảng cách tới polyline và reward tiến độ. `vmax=10 m/s`
là giới hạn tốc độ; không có cost bắt UAV bám `v_ref=10 m/s` ở mọi timestep.
Đây là path-following + terminal/path progress của dự án, chưa phải MPCC đầy đủ
vì path progress chưa là một biến tối ưu độc lập.

Trong harness, mỗi mức thử giờ đặt đồng thời `reference_speed_m_s` và `vmax`.
Lỗi cũ chỉ thay reference nhưng giữ `vmax=10` từng khiến lượt ghi nhãn 5 m/s có
thể bay gần 9 m/s; lỗi đó đã được sửa.

## 3. Tiến trình và nguyên nhân đã loại được

### 3.1. Reference và retiming

Retiming của repo gán lịch tốc độ theo vị trí/hình học đường để giảm tốc trước
cua. Khi bật, nó làm bài bay mượt hơn nhưng che mất câu hỏi nghiên cứu “MPPI có
tự tìm được velocity profile hay không”. Vì vậy profile hiện tại giữ retiming
tắt và dùng objective path-progress.

Replay frozen-state trước brake cho thấy time tracking và progress objective có
dự báo giảm tốc gần nhau tại các state được thử. Điều này chỉ cho phép nói rằng
đổi objective không giải quyết các state đó; không loại trừ ảnh hưởng dài hạn
của objective lên warm-start và state distribution trong closed loop.

### 3.2. Reset và proposal sau rejection

Reset cũ có thể xóa warm-start đúng lúc planner cần tiếp tục chuỗi phanh/rẽ.
Logic hiện giữ shifted nominal và có proposal giảm tốc–rẽ có cấu trúc sau khi
bị từ chối. Thay đổi này giúp hệ phục hồi và hoàn thành bài bay, nhưng chưa loại
hết các cụm `N_safe=0`.

### 3.3. Hình học chưa quan sát và mô hình phanh

Planner hiện kết hợp cloud đang quan sát với prior SDF của world để không xem
vùng ngoài FOV là free space. Conditioner và acceleration-memory surrogate
được đưa vào rollout để lệnh dự đoán gần hơn với lệnh thực thi.

Phép thử phanh thẳng Gazebo/SITL gồm 40 pha tại 4/6/8/10 m/s cho thấy công thức
`v*0.25 + v^2/(2*3)` dự đoán thiếu quãng dừng ở cả 40 pha; mức thiếu lớn nhất
tại 10 m/s là **1.302 m**. Surrogate acceleration-memory khởi tạo bằng ground
truth lại bảo thủ ở 40/40 pha offline, nhưng điều đó chưa chứng nhận estimator
live. Vì vậy stopping classification hiện vẫn là một nguồn bất định cần đo.

### 3.4. Chọn quỹ đạo hợp lệ

Pipeline hiện tại:

```text
sample U → conditioner + response model → rollout
         → shared swept/cloud/prior-SDF/stopping predicate
         → sample unsafe nhận weight 0
         → normalize trên feasible set
         → weighted nominal
         → final gate dùng cùng predicate
         → nếu cần, chọn best feasible sample và gate lại
```

Regression lịch sử tại seed 7/cycle 160 có 148/160 sample an toàn nhưng tổng
weight của chúng bằng 0. Sau sửa, sample unsafe không còn thắng khi `N_safe>0`.
Hai flight headless gần nhất không còn trường hợp có sample an toàn nhưng output
cuối bị hard gate từ chối.

## 4. Checkpoint bay hiện tại

Profile: `config/experiments/mppi_yard_progress_feasible80.yaml`.

| Seed | Kết quả | Peak XY | `N_safe=0` | Optimizer timeout |
|---:|---|---:|---:|---:|
| 7 | tới đích, LAND/disarm | 9.19 m/s | 21 | 0 |
| 17 | tới đích, LAND/disarm | 8.98 m/s | 11 | 0 |

Đây là baseline hiệu năng sạch trước khi bật snapshot I/O. Flight dùng để thu
snapshot của Experiment 7A cũng hoàn thành 2/2; peak 9.07/9.15 m/s và có 10/17
chu kỳ `N_safe=0`. Snapshot capture thêm đồng bộ file nên không dùng nó thay
baseline để kết luận deadline/performance.

Profile GUI đã qua một lượt seed 7 cho mỗi tốc độ: peak 4.98 m/s ở request 5 và
8.68 m/s ở request 10, đều tới đích. Không bật RViz trong phép đo. Khi bật cả
bridge camera/depth và RViz, tải render/publish từng làm compute vượt chu kỳ;
không dùng cấu hình năm terminal để đánh giá realtime của controller.

Các giới hạn còn mở:

- chưa giữ ổn định 9.5–10 m/s trong yard;
- vẫn có zero hold khi `N_safe=0`;
- zero setpoint chưa được chứng minh là emergency trajectory hợp lệ;
- stopping predictor live chưa được chứng minh bảo thủ;
- `collision_radius_m=1.5` vẫn là khoảng cách tâm, chưa phải body/rotor envelope
  đã hiệu chuẩn;
- kết quả SITL không phải chứng nhận an toàn bay thật.

## 5. Experiment 7A — phân biệt sampling miss và viability loss

### 5.1. Snapshot và tính tái lập

20 snapshot chứa state, acceleration memory, previous applied command, nominal
trước solve, path/progress, obstacle cloud, prior-map hash, config/runtime,
conditioner state và thông tin rejection recovery. Việc replay dựng lại đúng
state/model/config được lưu, rồi dùng RNG mới có kiểm soát. Đây là offline
reoptimization; không phải exact replay của toàn flight và không mô phỏng các
lần replan tiếp theo.

Tập failure gồm sáu vị trí trên mỗi seed: first occurrence, ngay sau zero hold,
đầu cụm, giữa cụm, gần cua nhất và trước khi recover. Tập control gồm bốn state
có `N_safe>0` trên mỗi seed ở các phần khác nhau của đường.

### 5.2. Kết quả 1.600 solve

| Nhóm | K | Số solve | `P_hit=P(N_safe>0)` | Median `N_safe` | Median solve time |
|---|---:|---:|---:|---:|---:|
| Failure | 80 | 240 | 0.000 | 0 | 52.3 ms |
| Failure | 160 | 240 | 0.000 | 0 | 100.0 ms |
| Failure | 320 | 240 | 0.000 | 0 | 202.0 ms |
| Failure | 640 | 240 | 0.000 | 0 | 411.5 ms |
| Control | 80 | 160 | 1.000 | 80 | 60.0 ms |
| Control | 160 | 160 | 1.000 | 158 | 108.1 ms |
| Control | 320 | 160 | 1.000 | 314.5 | 212.0 ms |
| Control | 640 | 160 | 1.000 | 627 | 423.6 ms |

Không có snapshot failure nào hit ở bất kỳ K/RNG nào. Tất cả snapshot control
đều hit ở tất cả K/RNG. Control không phải luôn 100% feasible ở K lớn, nhưng
luôn có ít nhất một nghiệm và median tỷ lệ feasible vẫn rất cao.

### 5.3. Quyết định từ 7A

| Giả thuyết | Kết quả hiện tại | Quyết định |
|---|---|---|
| H1: 80 random samples bỏ lỡ nghiệm | Bị phản bác mạnh bởi 0/960 hit failure khi K tăng tới 640 | Không tăng K online |
| H2: state đã mất viability dưới mô hình/constraint hiện tại | Phù hợp nhất với dữ liệu | Kiểm tra boundary/model rồi thiết kế recursive feasibility/recovery |
| H3: nghiệm sát biên, classification nhạy với model/margin | Chưa được phân tách | Chạy sensitivity có kiểm soát trên cùng snapshot |

Con số solve time offline không đại diện chính xác latency live, nhưng cho thấy
rõ xu hướng chi phí: K=160 đã khoảng 100 ms median, K=640 khoảng 0.4 s. Vì vậy
tăng K là hướng vừa không cải thiện hit-rate tại failure state, vừa không phù
hợp ngân sách 10 Hz trên máy thử.

## 6. ESS và cost scale

Chỉ tính trên các solve control có feasible samples:

| K | Median ESS_safe | p95 ESS_safe | Median `(J2-J1)/lambda` | p95 `(J2-J1)/lambda` |
|---:|---:|---:|---:|---:|
| 80 | 1.000 | 1.053 | 142.64 | 1840.79 |
| 160 | 1.000 | 1.417 | 83.50 | 1213.49 |
| 320 | 1.000 | 1.072 | 72.18 | 897.66 |
| 640 | 1.000 | 1.056 | 61.00 | 674.25 |

Với `lambda=1`, khoảng cách cost như trên làm `exp(-(J-Jmin)/lambda)` gần bằng
zero cho hầu hết sample. ESS gần 1 vì thế là hệ quả số học có thể giải thích,
không còn chỉ là quan sát từ flight. Tuy nhiên tăng `lambda` không thể tạo nghiệm
ở 12 failure snapshot vì feasible set đang rỗng. Trước khi đổi `lambda`, cần
phân rã `ΔJ` theo path/progress, input, input-change, stopping và collision để
biết component nào tạo scale lớn.

## 7. Đối chiếu phạm vi với MPPI công bố

- Williams et al. (IEEE T-RO 2018) mô tả MPPI dạng stochastic receding horizon
  và cost-weighted update. Repo dùng nguyên lý đó, nhưng không tuyên bố tái hiện
  mọi correction term của bài báo.
- Minařík et al. (IROS 2024) dùng input/input-change cùng full-state reference
  gồm position, attitude, velocity và body rate. Controller hiện tại gửi velocity
  setpoint cho ArduPilot và dùng reduced closed-loop response model; không phải
  full-state UAV MPPI của paper.
- PA-MPPI của Zhai et al. (RA-L 2026) dùng full quadrotor model và báo cáo
  `N=17,500`, `H=15`, `lambda=0.02`, prediction `dt=0.1 s`, control 50 Hz trên
  JAX/Agilicious. Repo hiện dùng N=80, H=30, lambda=1, 10 Hz và prior SDF/cloud;
  đây không phải reproduction của PA-MPPI.
- Feasibility mask, stopping predicate, prior SDF và proactive proposals là các
  bổ sung của project. Ý tưởng dành một phần budget cho proposal có cấu trúc phù
  hợp với guided/informed sampling nói chung, nhưng chưa được gọi là tái hiện
  GMPPI khi chưa audit đúng paper và implementation tương ứng.

Nguồn paper và phạm vi claim đã được ghi trong `reports/pa_mppi_sources.bib` và
`docs/SOURCE_AUDIT.md`.

## 8. Phase 7B — boundary/model sensitivity, chưa đổi controller

### Hypothesis

```text
H3a: N_safe=0 do stopping response/delay đang phân loại quá sát biên.
H3b: N_safe=0 do collision/body margin, không phải stopping response.
H2:  N_safe=0 vẫn giữ nguyên trong dải model/margin hợp lý; state đã mất viability.
```

### Experiment

Dùng đúng 20 snapshot và RNG cố định. Baseline giữ `K=80`; chỉ thay một yếu tố
mỗi lần. Các giá trị giảm margin chỉ dùng để chẩn đoán độ nhạy, không được đưa
thẳng thành cấu hình bay.

| Nhóm sweep | Giá trị đề nghị | Câu hỏi |
|---|---|---|
| Effective delay | 0.15 / **0.25** / 0.35 s | Classification nhạy với latency đến đâu? |
| Deceleration model | 2.0 / **3.0** / 4.0 m/s² | Nghiệm xuất hiện chỉ khi giả định phanh mạnh hơn? |
| Stopping uncertainty | **0.0** / 0.5 / 1.3 m | Sai số đo được có làm vùng infeasible rộng thêm? |
| Center clearance | 1.2 / **1.5** / 1.8 m | Rào cản chủ yếu là collision/body margin? |
| Horizon | **3.0** / 4.0 / 6.0 s | Có thấy chuỗi phanh–rẽ nếu nhìn xa hơn? |

Với mỗi cell, log `P_hit`, `N_safe/K`, loại vi phạm đầu tiên, khoảng thiếu đến
ngưỡng stopping/collision gần nhất và first control của best feasible sample.
Phải giữ state, nominal, cloud/map và RNG giống nhau để các cell so sánh được.

### Decision branch

```text
Nếu P_hit đổi mạnh trong dải model/delay hợp lý
    → H3: hiệu chuẩn response + effective latency + body margin từ dữ liệu,
      đặt uncertainty theo confidence bound rồi chạy lại 7A.

Nếu chỉ giảm safety margin phi thực tế mới tạo nghiệm
    → không dùng config đó; xem như H2 cho deployment hiện tại.

Nếu tăng horizon tạo nghiệm ổn định nhưng compute quá ngân sách
    → giữ budget online và đưa known-safe shifted/recovery trajectories vào pool.

Nếu mọi sweep hợp lý vẫn P_hit≈0
    → H2: triển khai recursive feasibility + verified recovery library.
```

## 9. Phase 7C nếu H2 giữ nguyên — verified recovery

Recovery pool tối thiểu:

```text
shifted previous known-safe trajectory
verified straight braking trajectory
brake + left-turn bias
brake + right-turn bias
path-tangent deceleration
```

Từng candidate phải đi qua đúng shared safety predicate trước khi được thực thi.
Khi MPPI có `N_safe=0`, planner chọn một recovery đã được xác minh thay vì mặc
định coi zero velocity setpoint là emergency trajectory. Muốn giữ recursive
feasibility, known-safe trajectory cần được shift, cập nhật theo state mới và
chỉ giữ lại nếu phần còn lại cộng terminal recovery vẫn qua predicate.

Acceptance criteria cho phase này:

1. Không có output unsafe khi `N_safe>0`.
2. Mỗi cycle `N_safe=0` hoặc có recovery qua predicate, hoặc abort/LAND theo
   policy đã định nghĩa; không gọi zero setpoint là “safe” nếu chưa verify.
3. Hai seed 7/17 hoàn thành mà không có chuỗi zero hold lặp lại.
4. Multi-seed mới báo tỷ lệ thành công; 2/2 hiện tại chỉ là checkpoint.
5. Profile GUI được kiểm tra riêng dưới tải render thực tế.

## 10. Dữ liệu và lệnh tái lập

- [Manifest 20 snapshot](../results/yard_experiment7a_20260916/manifest.json)
- [Tóm tắt 1.600 solve](../results/yard_experiment7a_20260916/summary.json)
- [Toàn bộ solve CSV](../results/yard_experiment7a_20260916/solves.csv)
- [Phân tích ESS bổ sung](../results/yard_experiment7a_20260916/analysis.json)
- [Báo cáo feasibility-selection](MPPI_FEASIBLE_SELECTION_VI.md)
- [Kiểm chứng mô hình phanh và safety](MPPI_MENTOR_SAFETY_PHASE_VI.md)
- [Quickstart GUI/headless hiện tại](RUN_YARD_5_10_MS_QUICKSTART_VI.md)

Thu snapshot:

```bash
cd ~/Projects/ardupilot_gazebo
PY=/opt/miniconda3/envs/ardupilot-rviz/bin/python
$PY scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_progress_feasible80.yaml \
  --params config/experiments/mppi_yard_high_accel.parm --timeout 60 \
  --debug-snapshot-events --debug-control-stride 20 \
  --output output/benchmark/yard_experiment7a_capture_repeat
```

Chọn 20 snapshot và replay:

```bash
$PY scripts/select_experiment7a_snapshots.py \
  output/benchmark/yard_experiment7a_capture_repeat \
  --output output/benchmark/yard_experiment7a_selection_repeat

$PY scripts/replay_experiment7a.py \
  output/benchmark/yard_experiment7a_selection_repeat \
  --samples 80 160 320 640 --realizations 20 \
  --output output/benchmark/yard_experiment7a_replay_repeat
```

Snapshot I/O chỉ bật trong lượt thu dữ liệu. Không bật nó trong benchmark
deadline hoặc demo GUI.

# Objective hình học + tiến độ, không ép bám lịch 10 m/s

**Đã chạy protocol A–D:** [kết quả cruise, model và decision validation](MPPI_PROTOCOL_RESULTS_20260916_VI.md).

## Định nghĩa bài toán đã thống nhất

10 m/s là cruise mong muốn trên đoạn đầu thoáng và giới hạn tốc độ mong muốn.
UAV được tự giảm tốc trước cua, rẽ và tăng tốc lại. Không yêu cầu đi qua mỗi
vị trí tại timestamp do reference 10 m/s đặt ra. Không thêm module gán tốc độ
riêng cho từng cua.

## Thay đổi code

Bật `path_progress_objective: true`:

- Running path cost dùng khoảng cách tới polyline hình học, không dùng
  `reference_positions[t]`. Term bám `reference_velocities[t]` không hoạt động.
- Terminal reward là `-w_progress * (s(p_T)-s(p_0))`, với s là chiều dài dọc
  polyline tại hình chiếu gần nhất. Reward là tiến độ cuối horizon, không
  cộng lại toàn bộ cùng một đoạn tiến độ ở từng timestep.
- Speed cost chỉ phạt `max(0, norm(v_xy)-vmax)^2`; dưới vmax không có penalty
  vì đi chậm. Effort/smoothness vẫn tồn tại, nên không nói mọi tốc độ dưới
  vmax có tổng cost bằng nhau.
- Terminal position term nếu bật hướng về goal, không phải reference thời gian.
- Collision, stopping cost, conditioner, final gate và brake giữ hoạt động.
- Diagnostics có `progress`, `speed_limit`, `reference_velocity`; tổng nominal
  được kiểm thử khớp tổng rollout + terminal cost.

Đây là objective path-following có reward progress, **chưa phải MPCC đầy đủ**
có biến tiến độ tối ưu độc lập, contour/lag error và ràng buộc monotonic progress.
Projection gần nhất có thể nhảy nhánh ở đường tự cắt hoặc hai nhánh gần nhau;
chưa kiểm chứng cho các đường đó.

**Giới hạn 10 m/s hiện là soft cost theo norm XY.** Các giới hạn lệnh sẵn có
vẫn theo từng trục; chưa thêm hard constraint norm XY hay bảo đảm vận tốc
thực không overshoot. Không mô tả soft penalty này như hard constraint.

`reference_speed_m_s=10` vẫn có trong config/harness để tạo proposal và gắn
nhãn thí nghiệm, nhưng không còn là term tracking theo thời gian. Warm-start
cruise chỉ là chuỗi khởi tạo cho optimizer, không phải lệnh bắt buộc hay lịch
giảm tốc theo cua. Retiming vẫn bằng 0.

## Kiểm thử và replay

61 test cũ và 3 test objective mới đã qua. Test mới xác nhận:

- 4/7/10 m/s tại cùng vị trí trên đường không có cost tracking tốc độ/thời gian,
  kể cả khi `w_reference_velocity` cố ý đặt lớn; vượt 10 thì có penalty.
- Progress theo chiều dài polyline, bão hòa ở cuối đường; reward cuối horizon đúng.
- Diagnostics total khớp objective rollout.

Script: `scripts/replay_progress_before_brake.py`, dữ liệu tại
[replay.json](../output/benchmark/progress_replay_20260916/replay.json).

Giữ state/cloud/map và cùng nominal U từ log stopping-v3 tại cycles 70/80,
so objective time-tracking H=40 và progress H=40 với một lần tối ưu, seed 7/17.
Cả hai dự đoán giảm tốc gần như giống nhau: cycle 70, seed 7 khoảng
7.54 m/s sau 1 s và 4.54 sau 2 s. Seed 17 cycle 80 cả hai vẫn không qua gate.
Không thấy bằng chứng đổi objective riêng lẻ giải quyết được trạng thái đó.

Đây là frozen-state reoptimization có RNG khởi tạo mới, không phải exact
replay RNG trong flight. U cũ được nối dài bằng phần tử cuối từ H=30 lên 40.
Không đánh đồng dự báo quỹ đạo hợp lệ với chứng minh phanh thực sẽ an toàn.

## Thử nghiệm headless

Hai cấu hình mới, cùng map/model/brake, timeout 45 s, seeds 7/17:

1. `mppi_yard_progress10.yaml`: H=40, N=200, reward progress=2000,
   warm-start reference tắt.
2. `mppi_yard_progress_cruise10.yaml`: H=30, N=160, reward progress=5000,
   warm-start cruise bật. Đây là screening nhiều biến, không phải ablation
   chứng minh riêng tác dụng từng hyperparameter.

Cấu hình đầu cả hai tới đích và LAND nhưng peak chỉ 6.539/4.927 m/s;
seed 7 có 9 hold-brake, 2 rejection và 12 hold-timeout. Không đạt mục tiêu
cruise 10 m/s dù một lượt đi chậm không có brake. Không chọn làm mặc định.

### Kết quả cấu hình progress-cruise

| Chỉ số | Seed 7 | Seed 17 |
|---|---:|---:|
| Tới đích và LAND/disarm | Có | Có |
| Peak XY m/s | 8.875 | 9.004 |
| Hold-brake | 13 | 3 |
| Hold-invalid | 8 | 1 |
| Hold-timeout | 3 | 0 |
| Thời gian sim s | 19.958 | 17.476 |
| RMS thay đổi lệnh m/s² | 5.819 | 5.508 |
| Clearance tâm nhỏ nhất m | 2.034 | 2.250 |
| Cycle deadline misses | 7 | 0 |

Mọi dòng command đều qua gate. Không đạt ngưỡng >=9.5 m/s, chưa xác nhận
cruise ban đầu gần 10 m/s. Giảm brake ở seed 17 không đủ để khẳng định cải
thiện ổn định; seed 7 còn lỗi quá hạn và brake. Không thay mặc định quickstart
bằng cấu hình này. Không chạy thêm mức 5 m/s hoặc GUI trong đợt này.

[Kết quả progress-cruise](../output/benchmark/yard_progress_cruise10_20260916_v1/summary.json)
· [Đồ thị](../output/benchmark/yard_progress_cruise10_20260916_v1/comparison.png)
· [Events/gate](../output/benchmark/yard_progress_cruise10_20260916_v1/gate_check.json).

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/run_yard_speed_ablation.py \
  --scenario yard-runup60 --speeds 10 --seeds 7 17 \
  --config config/experiments/mppi_yard_progress_cruise10.yaml \
  --params config/experiments/mppi_yard_high_accel.parm \
  --timeout 45 \
  --output "output/benchmark/yard_progress_repeat_$(date +%Y%m%d_%H%M%S)"
```

Bước tiếp theo cần tách kiểm chứng cruise đường thoáng của objective mới,
replay nhiều trạng thái trước cua, và đối chiếu velocity/acceleration rollout
với log thực cùng thời điểm. Không quay lại cost ép bám lịch 10 m/s để chỉ
đẩy peak lên; cũng không coi việc đi chậm toàn đường là đạt mục tiêu cruise.

## Quy trình kiểm chứng đã thống nhất: A → B → C → D

Giữ path-progress objective, conditioner và guard trong giai đoạn chẩn đoán.
Không thay kiến trúc hoặc tune đồng thời các nhóm tham số. Các mục dưới đây
là **kế hoạch kiểm chứng**, không phải kết quả đã chạy.

### A. Cruise validation

Chạy đường thẳng thoáng 150–300 m với objective progress, retiming tắt.
Nếu dùng world straight hiện có, kiểm tra hình học thực của world và giữ các
solid còn lại trong provenance; không gọi là “không obstacle” chỉ vì không
có container chắn đường. Không dùng prior map yard cho world straight.

Tại mỗi state được chọn, so hai candidate hướng tới 8 và 10 m/s, giữ cùng:
state đo, acceleration memory, previous executed command, path/geometry,
conditioner, model, horizon và thời điểm bắt đầu. Báo vận tốc thực sự dự đoán
của từng candidate; tên “10” không có nghĩa candidate đã đạt 10 trong horizon.

Tách các state ở pha tăng tốc khỏi state gần steady cruise. Chỉ gọi steady
khi kiểm tra được biến thiên vận tốc/gia tốc trong cửa sổ trước t0; ghi tiêu
chí và giá trị đo. Không tự đặt acceleration memory bằng zero để tạo steady.

Ghi cost thành phần: path, progress, speed-limit, effort, input-change,
collision, stopping, terminal, total. Trong code, `input_change` là penalty
đổi lệnh; `smoothness`/`w_du` là một term khác, không gộp nhầm. So nominal
objective của candidate; không đánh đồng nó với sample cost có correction
nhiễu trong trọng số MPPI.

Closed-loop báo tốc độ và lệnh theo thời gian, thời gian liên tục trong dải
9.5–10.5 m/s, vị trí bắt đầu cruise, deadline/hold và trạng thái kết thúc.
J8<J10 ở pha tăng tốc chưa chứng minh objective thích cruise 8 m/s.

### B. Model validation: dùng lệnh thực đã gửi

Chọn 4–6 state trước cua, gồm các mốc gần x=40/45/50/53 m nếu log có đủ dữ
liệu, và các state trong pha brake. Lưu t0, measured state, model acceleration
memory, previous executed command và chuỗi lệnh thực gửi sau t0.

**Lệnh đã gửi là lệnh sau conditioner/gate/brake: đưa thẳng vào nhánh model
nhận applied command, không chạy conditioner lần thứ hai.** Chỉ áp dụng lại
conditioner khi input là raw request; khi đó phải tái hiện cả override do
safety và kiểm tra đầu ra khớp lệnh gửi. Phân biệt velocity command với loại
message khác và ghi rõ frame ENU/NED, dấu trục Z.

Căn theo simulation time với mapping từ thời điểm gửi. Giữ từng lệnh theo
zero-order hold giữa các lần gửi; xử lý khoảng thời gian không đều, gaps và
reset state. Send timestamp chưa phải receipt timestamp: nếu thiếu thời điểm
ArduPilot nhận/thực thi, báo bất định đó và thử giả thuyết delay có giới hạn,
không khẳng định đã đồng bộ chính xác tuyệt đối. Không dùng số cycle × dt
thay cho thời gian thực khi có deadline miss.

Tại 0.5/1/2 s, báo sai số position XYZ, velocity XY/vector, acceleration,
hướng vận tốc (chỉ khi tốc độ đủ lớn) và clearance cùng nguồn geometry.
Nếu acceleration đo bằng đạo hàm vận tốc, ghi bộ lọc/cửa sổ và ảnh hưởng
nhiễu. So model với odometry và ground truth riêng để không gộp sai số state
estimation vào dynamics. Loại/đánh dấu cửa sổ thiếu dữ liệu, LAND hoặc đổi mode.

Cùng input mà sai lệch đáng kể là bằng chứng model/delay/state initialization
chưa phù hợp; cần định nghĩa mức sai số chấp nhận theo clearance và quãng
phanh, không chỉ theo một ngưỡng RMSE tùy ý. Nếu fit lại, giữ trial khác để
kiểm tra, không đánh giá chỉ trên các cửa sổ đã dùng fit.

### C. Decision validation: dùng nominal U

Ở cùng các state trước cua, giữ raw U và initial model state đúng cycle.
Rollout với conditioner đúng một lần; ghi velocity tại 0.5/1/2 s, vị trí bắt
đầu giảm tốc, swept clearance, stopping clearance và kết quả final gate.

Đối chiếu chuỗi:
`raw U[0] → conditioned command → gate/brake decision → actual sent command`.
Báo thời điểm và lý do override, không chỉ đếm số hold. Brake có thể chạy
trước optimizer: cycle hold-brake không có nominal mới thì đánh dấu “không
có”, không lấy nominal cũ làm quyết định hiện tại.

Nominal forecast và chuyển động thực dùng chuỗi lệnh khác nhau sau bước đầu;
chênh lệch của chúng không phải phép kiểm chứng model ở mục B. Nominal pass
gate chỉ chứng minh hợp lệ theo model và dữ liệu geometry lúc đó.

Nếu cần phân tích selection, phải ghi đủ sampled actions/states, sample cost,
weights/ESS, RNG hoặc seed + trạng thái RNG, và nominal output. Chỉ có raw U
và ESS không đủ kết luận “không có mẫu an toàn” hay “mẫu tốt bị bỏ qua”.
Sample có cost thấp nhất chưa chắc qua gate; kiểm tra mẫu và nominal bằng
cùng gate/model. Ghi chi phí logging, tránh chính diagnostics làm trễ control.

### D. Điều kiện phân nhánh sửa lỗi

| Bằng chứng | Hướng xử lý tiếp |
|---|---|
| Cùng applied input, model lệch đáng kể sau căn chỉnh | Response/delay model hoặc state initialization; tách sai số estimator |
| Model đạt, không tìm thấy mẫu hợp lệ trong tập đã ghi | Proposal/noise/horizon; không suy ra không tồn tại nghiệm khả thi |
| Có mẫu hợp lệ nhưng nominal kém/invalid | Cost weighting, ESS, phép cập nhật/chọn quỹ đạo; xét correction cost |
| Nominal dự đoán tốt nhưng actual command bị override | Conditioner/gate/brake interaction, kiểm tra nguyên nhân override |
| Cruise đường thoáng không đạt | So candidate cost và mức khám phá trước khi tăng reward |

Báo closed-loop qua nhiều cycle/trial để kiểm tra ảnh hưởng của objective lên
warm-start, phân bố state và thời điểm phanh. Frozen-state replay hiện có chỉ
cho phép kết luận hẹp ở các state đã thử. Chưa xác định nguyên nhân chính
của brake chỉ bằng các replay đó.

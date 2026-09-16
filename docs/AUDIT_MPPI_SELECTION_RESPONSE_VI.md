# Kiểm tra chọn quỹ đạo và mô hình đáp ứng MPPI — 15/09/2026

## Đã sửa diagnostics, thêm kiểm tra quỹ đạo và kiểm chứng đáp ứng

Phần audit ban đầu bên dưới mô tả code trước sửa. Cập nhật này **chưa phải
xác nhận điều khiển 10 m/s đã hoạt động**; retiming vẫn tắt khi kiểm chứng.

### Thay đổi code

- `accept_applied_control()` chỉ cập nhật bộ nhớ lệnh đã áp dụng trong state,
  không ghi đè raw U[0]. MPPI sẽ shift raw U vào chu kỳ kế tiếp. Trajectory
  và cost nominal không bị conditioning lại do mutation này.
- `predict_trajectory(first_applied=...)` có thể rollout với lệnh đầu đã
  conditioning chính xác một lần, phần đuôi vẫn là raw nominal U.
- Thêm `validate_final_trajectory` (mặc định false để giữ cấu hình lịch sử).
  Profile `mppi_yard_validated10.yaml` bật nó. Gate kiểm tra cả đoạn giữa
  các điểm dự đoán với cloud quan sát được, không chỉ các điểm rời rạc.
  Vi phạm, geometry không hữu hạn hoặc cloud rỗng → `hold-invalid-trajectory`,
  gửi zero và log quỹ đạo bị từ chối. Đây không phải chứng nhận dừng kịp.
- Khi bật gate, log thêm raw U, state ban đầu, cloud, dt/tau để replay;
  diagnostics validation và lệnh thực gửi phân biệt rõ với raw control.
  Gate kiểm tra lệnh đầu sau conditioning trước khi feedback; các bước
  tương lai vẫn là dự đoán, sẽ được replan.

### Kiểm tra

49 test core và 4 test mới đạt: giữ nguyên nominal/cost sau feedback,
rollout lệnh đầu chỉ áp dụng một lần, bắt collision giữa hai điểm, từ chối
cloud rỗng/nonfinite, node trả hold khi quỹ đạo cuối bị chặn.
Probe sau sửa cho x bước đầu **.006 m cả trước và sau feedback**, thay vì
.006 → .0018 m trước đây. Test thư viện vẫn chứng minh trung bình hai mẫu
hợp lệ không nhất thiết hợp lệ; gate là phần bổ sung ở wrapper của dự án.

### Mô hình đáp ứng: đã kiểm tra, chưa thay bằng mô hình chưa được xác nhận

Script `scripts/check_velocity_response_model.py` dùng log baseline seed 7,
kiểm tra chéo seed 17, đồng bộ timestamp gửi với ground truth. Sai số một
bước XY vận tốc ở lệnh thông thường: tau=.5 có RMSE .102/.094 m/s;
tau=.8 có .074/.068 m/s. Điều này **không đủ để chọn tau=.8 làm mô hình mới**.
Sau zero, tốc độ ground truth vẫn tăng khoảng .52/.49 m/s trong .2 s đầu.
Mọi mô hình bậc nhất với tau dương và zero input đều dự đoán giảm ngay,
không thể mô tả pha đó chỉ bằng đổi tau. Chưa log target gia tốc/jerk nội bộ
ArduPilot nên chưa tách được các nguồn trễ. Giữ nguyên tau và không tune tiếp.

[Probe sau sửa](../output/benchmark/mppi_implementation_audit_fixed_20260915/probes.json),
[kiểm tra đáp ứng](../output/benchmark/mppi_implementation_audit_fixed_20260915/response_check.json).

### Headless kiểm chứng gate — seed 7

Output `yard_final_validation10_20260915_v1`, bài lấy đà 60 m, reference 10,
retiming tắt, setting baseline cộng gate. Peak **9.874 m/s**, **abort vùng
đệm**, không tới đích; LAND/disarm xác nhận. 79 command, 10 hold-brake,
1 hold-stale ban đầu, **0 hold-invalid-trajectory**; không deadline miss hoặc
timeout-hold. Tất cả command có validation true đối với cloud lúc đó.

Phát hiện mới: tại cycle 80, cloud chỉ tới **x=57.16 m**; container thứ hai
bắt đầu **x=63.1 m**. Gate báo clearance cloud 1.895 m, nhưng đối chiếu offline
nominal với solid SDF (lấy mẫu đoạn cách nhau tối đa .05 m) được **1.082 m**.
Có 14 cycle raw nominal vi phạm margin 1.5 m theo SDF nhưng cloud check vẫn
pass. Offline SDF không được dùng để điều khiển trong lượt này.
Không khẳng định nguyên nhân mất điểm là sensor, occlusion hay downsampling
khi chưa kiểm tra cloud gốc. **Cloud hiện tại thiếu vật cản ở phần horizon
phía trước**, nên việc pass gate cloud không đồng nghĩa đường an toàn.

[Summary](../output/benchmark/yard_final_validation10_20260915_v1/summary.csv),
[đối chiếu SDF](../output/benchmark/yard_final_validation10_20260915_v1/nominal_sdf_crosscheck.json).
Source snapshot đã khớp hash; process thí nghiệm đã dừng.

Phần còn lại trước khi tune: xử lý vùng chưa quan sát/duy trì bản đồ vật cản
hoặc dùng bản đồ đã biết với nguồn gốc rõ ràng; nhận dạng mô hình có trạng
thái gia tốc/jerk. Không thêm retiming để che các hạn chế này.


Đã kiểm tra source local của dự án, thư viện `pytorch_mppi 0.9.1` đang cài,
source ArduPilot local và log thất bại. Chưa sửa controller/tuning, chưa chạy
chuyến bay mới. Hai kiểm chứng tổng hợp trong `scripts/audit_mppi_selection.py`
đều đạt điều kiện assert. Chúng tái hiện hành vi code, không thay thế flight test.

## 1. Quỹ đạo trả về không có chứng nhận hợp lệ

Thư viện `_command()` tính trọng số từ cost các mẫu rồi cộng trung bình nhiễu
vào U. Đây không phải chọn nguyên một mẫu có clearance tốt nhất. Không có
bước rollout và reject collision của U cuối trong hàm đó.

Kiểm chứng có kiểm soát bằng chính hàm cập nhật của thư viện: hai hành động
`(1,1)` và `(1,-1)` có cùng cost, hai đoạn đường đều cách tâm vật cản `(1,0)`
0.707 m, ngoài bán kính 0.4 m. Kết quả trung bình `(1,0)` đi vào tâm vật cản.
Điều này chứng minh cơ chế không bảo toàn tính hợp lệ; không chứng minh đây
là nguyên nhân đã xảy ra trong mọi lượt Gazebo (ESS gần 1 giảm tác động trộn).

Trong dự án, `_collision_indicator()` chỉ kiểm tra khoảng cách các vị trí
rollout tới các điểm cloud. Cost hữu hạn, không có ràng buộc cứng, cũng không
kiểm tra liên tục từng đoạn giữa hai state. Không có điểm cloud thì cost
collision bằng 0; không đồng nghĩa toàn bộ không gian đã được quan sát.
`LocalPlannerNode.step()` kiểm tra brake theo trạng thái/đoạn dừng trước khi
chạy optimizer; sau tối ưu chủ yếu conditioning và timeout rồi trả lệnh.
Không thấy gate xác minh quỹ đạo U cuối tránh vật cản trước khi gửi.

## 2. Lỗi tái dựng diagnostics đã tái hiện được

Thứ tự hiện tại là raw U → conditioning → `accept_applied_control()` ghi đè
U[0] bằng lệnh đã lọc → tính cost và `predict_trajectory()` từ U đó.
Dynamics vẫn coi U[0] là lệnh raw và conditioning thêm lần nữa khi tái dựng.

Probe trạng thái đứng yên, raw vx=10, alpha=.3, giới hạn delta=.3:

- Lệnh gửi đúng là vx=.3 m/s.
- Rollout trước feedback dự đoán x bước đầu=.006 m.
- Rollout log sau feedback dự đoán x bước đầu=.0018 m.

**Đây là lỗi ý nghĩa của trajectory/cost chẩn đoán**, không phải bằng chứng
lệnh MAVLink thực bị lọc hai lần. Vì vậy phải rút lại cách hiểu rằng cost
collision dương trong log chắc chắn là cost của mẫu optimizer gốc được chọn.
Nó là cost quỹ đạo tái dựng sau mutation. Việc thiếu gate hợp lệ vẫn được
xác nhận độc lập qua source.

## 3. Mô hình đáp ứng thiếu trạng thái quan trọng của ArduPilot

MPPI dùng `v_next = v + min(dt/tau,1)*(u_applied-v)`, sau đó cập nhật vị trí.
State giữ previous applied command để mô phỏng conditioner, nhưng không có
trạng thái gia tốc mong muốn/jerk/độ nghiêng của tầng điều khiển ArduPilot.

Source ArduPilot local cho thấy `ModeGuided::velaccel_control_run()` gọi
`AC_PosControl::input_vel_accel_NE_m()`, hàm này dùng `shape_vel_accel_xy()`
với giới hạn gia tốc và jerk. Setpoint zero không làm gia tốc mong muốn và
độ nghiêng đang có đảo dấu ngay lập tức. Đây là khác biệt cấu trúc đã xác
nhận; chưa đủ để định lượng riêng đóng góp của jerk, attitude, lọc state hay
trễ giao tiếp vào từng thất bại.

Log baseline không retiming: sau brake UAV đi thêm 7.47/7.62 m trong giây đầu,
còn khoảng 7.18/7.28 m/s sau 1 s. So sánh minh họa tau=.5 với zero áp dụng
trực tiếp cho khoảng .68/.70 m/s. Mốc state và vận tốc filtered khác ground
truth; không gọi đây là phép nhận dạng tau chính xác hoặc trễ thuần ArduPilot.
Chi tiết và biểu đồ trong [phân tích tracking](../output/benchmark/yard_free10_20260915_v1/tracking_failure.json).

## 4. Hệ quả và thứ tự sửa đề xuất

1. Tách raw nominal snapshot khỏi applied command memory; log đúng trước/sau
   conditioning, cùng U và cloud để replay. Sửa diagnostics trước khi dùng
   nó để đánh giá quỹ đạo cuối hoặc tuning.
2. Kiểm tra tính hợp lệ của nominal cuối, gồm các đoạn giữa state, và phân
   biệt tất cả mẫu không hợp lệ với trung bình làm mất tính hợp lệ. Fallback
   phải được kiểm chứng về động lực học; chỉ gửi zero không bảo đảm dừng kịp.
3. Nhận dạng đáp ứng lệnh tăng/giảm tốc trên đường trống, log cả target vận
   tốc/gia tốc của ArduPilot và ground truth. Bổ sung mô hình có trạng thái
   gia tốc/jerk hoặc mô hình phù hợp số đo; không chỉ tăng tau tùy ý.
4. Sau đó mới chạy lại screening 10 m/s không retiming và nhiều seed.

Chưa có bằng chứng CPU tính không kịp trong các lượt đã thử. Chưa thể quy
một nguyên nhân duy nhất hoặc khẳng định sửa một mục sẽ tự động chạy được 10.

## Tái kiểm chứng

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/audit_mppi_selection.py
```

[Probe JSON và hash source](../output/benchmark/mppi_implementation_audit_20260915/probes.json).
Source tham chiếu:

- [MPPI controller](../mppi_ardupilot/mppi_controller.py): `_dynamics`, `_collision_indicator`, `accept_applied_control`, `predict_trajectory`.
- [Local planner](../mppi_ardupilot/mppi_local_planner_node.py): `LocalPlannerNode.step`.
- Thư viện local: `/opt/miniconda3/envs/ardupilot-rviz/lib/python3.12/site-packages/pytorch_mppi/mppi.py`.
- ArduPilot: `../ardupilot/ArduCopter/mode_guided.cpp` và
  `../ardupilot/libraries/AC_AttitudeControl/AC_PosControl.cpp` (relative repo root).

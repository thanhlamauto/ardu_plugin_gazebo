# Vì sao MPPI giữ trước cua rồi timeout — 15/09/2026

**Cập nhật:** [Đã sửa reset/proposal, 2/2 lượt tới đích](MPPI_REJECTION_RECOVERY_VI.md).
Nội dung và probe bên dưới mô tả hành vi trước sửa.

Phân tích `yard_map_response10_20260915_v1`, hai seed, không chạy flight mới
và không đổi controller. Probe trạng thái cycle 250 giữ đúng state, cloud,
reference progress và known map trong log.

## Nguyên nhân trực tiếp đã tái hiện

Sau `hold-invalid-trajectory`, node gọi `reset_applied_control()`. Hàm này
đặt `_reference_warm_started=False`. Với `reference_warm_start=true`, chu kỳ
sau `command()` lại ghi toàn bộ U bằng reference velocities 10 m/s.
Mọi cải thiện chuỗi U từ lần tối ưu trước bị bỏ đi sau mỗi rejection.
Probe hai lời gọi có reset xen giữa xác nhận initial proposal giống nhau,
raw XY đầu tiên vẫn 10 m/s dù UAV đang đứng gần cua.

Noise XY=.8 m/s, N=350, chỉ một lần cập nhật mỗi chu kỳ và lambda=1. Các log
ở trạng thái đứng có ESS=1. Chuỗi đi chậm khác rất xa tâm proposal 10 m/s;
đây là hạn chế khám phá phù hợp bằng chứng, không phải khẳng định mọi mẫu
đều giống nhau hoặc đo được xác suất tìm đủ đường. Gate chỉ từ chối rồi gửi
zero; hiện không chọn lại một chuỗi hợp lệ thay thế.

## Có phương án ngắn hạn hợp lệ tại chỗ dừng

Kiểm chứng thêm chuỗi zero và chuỗi raw `(1,0,0,0)` lặp trong horizon 3 s,
rollout qua đúng model/conditioner, cùng state, cloud và map. Chuỗi đi chậm
qua cả gate cloud và SDF, tiến thêm khoảng 2.35 m. Nominal objective thấp hơn
chuỗi đang bị từ chối ở cả hai seed. Xem số chính xác trong probe JSON.

Điều này **không chứng minh đã có toàn bộ đường qua cua**, nhưng bác bỏ cách
hiểu rằng mọi chuyển động tiến lên ở chỗ dừng đều không khả thi. Optimizer
không tìm/chọn được phương án ngắn hạn tốt này với proposal/reset hiện tại.
Chi phí so sánh là nominal objective, không bao gồm correction nhiễu trong
trọng số MPPI; đây không phải replay cùng toàn bộ RNG của optimizer.

Reference không retiming vẫn tiến 10 m/s qua đường cong trong horizon, nên
path/time tracking tạo sức ép tiến nhanh. Ngoài ra cost collision chỉ kiểm
tra state rời rạc, còn gate kiểm tra đoạn và margin bảo thủ: có thể xảy ra
collision cost=0 nhưng gate từ chối sát biên. Ví dụ seed 7 cycle 60, SDF
lower bound=1.465 m < 1.5 m dù cost collision=0. Đây giải thích khác biệt
cost/gate tại một số chu kỳ, không phủ nhận gate.

## Hướng sửa tiếp theo

Tách reset bộ nhớ lệnh thực khỏi việc khởi tạo lại proposal. Khi từ chối,
phải ghi nhận zero đã gửi nhưng không lặp vô điều kiện seed reference 10.
Thử các proposal phanh/đi chậm có hệ thống hoặc tối ưu tiếp tại cùng state,
kiểm tra cùng gate; đồng bộ kiểm tra collision giữa cost và validation.
Giữ retiming tắt. Chưa thực hiện các sửa này trong lượt chẩn đoán.

[Probe JSON](../output/benchmark/hold_loop_diagnosis_20260915/probe.json).

```bash
cd ~/Projects/ardupilot_gazebo
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/diagnose_mppi_hold_loop.py
```

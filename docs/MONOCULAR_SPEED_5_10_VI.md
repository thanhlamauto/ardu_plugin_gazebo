# Thử monocular ở mức đặt 5 và 10 m/s

Ba thí nghiệm OpenResearch độc lập, mỗi thí nghiệm có một lượt 5 m/s và một lượt 10 m/s (tổng 6 lượt). Tất cả dùng đúng một camera RGB và odometry Gazebo để lấy pose có thang mét; không dùng LiDAR, depth camera, bản đồ vật cản hoặc SDF trong perception/planner. ArduPilot GUIDED theo lệnh vận tốc, tránh vật cản tích hợp tắt. Vì vẫn dùng odometry lý tưởng, đây **chưa phải định vị chỉ bằng monocular**. `vmax` là mức đặt; vận tốc thực tính lại từ dense ground-truth theo thời gian mô phỏng. Va chạm ở đây là ngưỡng proxy tâm UAV–bề mặt hộp, không phải cảm biến tiếp xúc.

| Thí nghiệm/cảnh | Trạng thái | Vận tốc thực p95 / p99 (m/s) | Khoảng hở nhỏ nhất (m) | Diễn giải |
|---|---|---:|---:|---|
| Pilot, vật cản trước ở x=30 m, mức 5 | Tới đích | 4,23 / 4,78 | 0,874 | Bay an toàn lượt này nhưng chưa duy trì 5 m/s |
| Pilot, hộp hai bên, mức 10 | Tới đích | 6,71 / 7,21 | 1,850 | Chưa đạt 10 m/s |
| Hành lang rộng hơn, mức 5 | Từ chối trước điều khiển | — | 11,63 trong warmup | 0 điểm camera, ngưỡng yêu cầu 12 |
| Hành lang rộng hơn, mức 10 | Từ chối trước điều khiển | — | 11,64 trong warmup | 0 điểm camera, ngưỡng yêu cầu 12 |
| Đường dài có texture gần ban đầu, vật cản trước ở x=50 m, mức 5 | COLLISION_PROXY | 5,03 / 5,04 | **0,327** | Đạt tốc độ nhưng tránh vật cản thất bại |
| Đường dài có texture gần ban đầu, hộp hai bên, mức 10 | Tới đích | 8,48 / 9,69 | 1,833 | Đỉnh 9,80 m/s; chỉ 1,12 s ở ≥9 m/s; chưa thử vật cản chắn đầu |

Đối với lượt 5 m/s thất bại, log planner giữ khoảng 5 m/s tới x≈44,6 m rồi mới phát `hold-brake`; mặt trước hộp ở x=49 m, cách khoảng 4,4 m. Với giảm tốc giả định 1,5 m/s² và trễ 0,25 s, quãng phanh ở 5 m/s là 9,58 m, chưa kể biên va chạm 1,25 m. Điểm camera từ hộp đầu đường vẫn có, nhưng bằng chứng về hộp chắn trước không dẫn tới phanh đủ sớm. Không thể lấy số điểm tổng hoặc hai lượt đạt đích để kết luận tránh vật cản ở 5–10 m/s đã an toàn.

Tầm điểm tam giác hóa trong implementation hiện giới hạn 20 m. Ở 10 m/s, quãng phanh theo cùng giả định là 35,83 m trước biên va chạm. Vì thế **chưa có cơ sở chạy thử tránh vật cản chắn đầu ở 10 m/s** với detector/giới hạn giảm tốc này. Cần chứng minh tầm phát hiện đáng tin cậy dài hơn hoặc đo được giảm tốc thực cao hơn và cập nhật safety guard tương ứng, rồi thử nhiều seed, texture, độ trễ và blackout. Ngưỡng 12 điểm chỉ kiểm tra số điểm hữu hạn, không xác nhận đã thấy mọi mặt vật cản.

Các node và raw artifacts (ổ `Extreme SSD` phải được mount):

Bản audit gọn đã sao chép vào project: [pilot](../results/monocular_research/speed_expansion/highspeed_5_10/pilot_audit.json), [hai lượt từ chối](../results/monocular_research/speed_expansion/highspeed_5_10/wider_rejection_audit.json), [lượt đường dài](../results/monocular_research/speed_expansion/highspeed_5_10/near_texture_audit.json).

- Pilot: experiment `53341f72-393c-46ef-8648-daa8af48742c`, commit `880a472`, run `6a0e9ecd-a33f-4186-9d4c-385ccb1a59a1`. Audit: `/Volumes/Extreme SSD/uav_monocular_research_live/6a0e9ecd-a33f-4186-9d4c-385ccb1a59a1/artifacts/speed_screening/independent_audit.json`.
- Hành lang rộng bị từ chối: experiment `66c96770-cc84-446b-8e8c-792f4a69f509`, commit `f2d33a9`, run `94a59a2e-e481-4e98-a74d-36cba73536f2`. Audit xác nhận cả hai từ chối an toàn trước khi vào planner.
- Đường dài có texture gần: experiment `55b31dcd-7e35-4231-8079-509dffdd4c8f`, commit `9e02275`, run `7aaf6a43-7019-4f59-9911-2181558397bc`. Audit: `/Volumes/Extreme SSD/uav_monocular_research_live/7aaf6a43-7019-4f59-9911-2181558397bc/artifacts/speed_screening/independent_audit.json`.

Auditor kiểm tra source hash, scene/box thật, RGB duy nhất, zero range messages, tham số OA tắt, goal/seed, dense trace, khoảng hở, giới hạn cao độ và hash log nén. Hai cảnh trong main project tương ứng các lượt thành công riêng lẻ: `config/monocular_speed_5.yaml`/`worlds/iris_monocular_highspeed_5.sdf` từ pilot đầu; `config/monocular_speed_10.yaml`/`worlds/iris_monocular_highspeed_10.sdf` từ pilot cuối. Chúng là **cấu hình thử nghiệm**, chưa được chọn làm default, chưa có số liệu độ tin cậy qua nhiều seed. 5 m/s ở pilot đầu cũng chưa duy trì đúng tốc độ đặt.

Ma trận cũ 0,5/1/1,5 m/s không trả lời câu hỏi mới: run 54 lượt bị ngắt ngoài ý muốn sau 44 lượt có result, 10 lượt còn lại không có kết luận; không gộp chúng với sáu lượt trên.

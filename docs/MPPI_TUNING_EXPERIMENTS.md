# Thử nghiệm khó và bài học tune MPPI

## Trạng thái bằng chứng

Các số liệu trong tài liệu này là **VERIFIED OFFLINE** với mô hình point-mass
bậc một của repository. Chúng **không phải kết quả Gazebo, ArduPilot SITL hay
bay thật**. Benchmark không mô phỏng trễ MAVLink, sai số EKF, occlusion LiDAR,
tracking của ArduPilot hoặc động lực học attitude.

Mã chạy: `scripts/benchmark_mppi_tuning.py`.

Bản Gazebo 3D tương ứng và lệnh visualize nằm tại
[`run_mppi_challenge_worlds_vi.md`](run_mppi_challenge_worlds_vi.md).

Sổ tay để tự tạo profile, sửa map và tổ chức thí nghiệm nằm tại
[`MPPI_TUNING_PLAYGROUND_VI.md`](MPPI_TUNING_PLAYGROUND_VI.md).

## Bài test

| Scenario | Mục đích | Hình học chính |
|---|---|---|
| `multi_obstacle_slalom` | Nhiều lần đổi độ cong | Bốn obstacle tròn xen kẽ, năm waypoint |
| `narrow_gate` | Kiểm tra local minimum tại khe hẹp | Khe 5.5 m; với `margin=2 m`, phần nominal-safe chỉ rộng 1.5 m |
| `right_angle_corridor` | Góc cua gần 90 độ | Hành lang chữ L, bốn waypoint |

Obstacle được lấy mẫu thành point cloud tĩnh ở cao độ 20 m, spacing tường
khoảng 0.35 m. Vì vậy clearance báo dưới đây là khoảng cách đến point cloud,
không phải khoảng cách chính xác đến mesh liên tục.

Mọi case dùng cùng mô hình, `dt=0.1 s`, `tau=0.5 s`, `margin=2 m` và cùng logic
conditioner/waypoint/arrival như node thật. Không tune riêng từng seed.

## Cấu hình được so sánh

Baseline dùng `H=30`, `N=350`, `lambda=1`, `noise_xy=0.8`, `vmax=1.8`,
`max_accel_xy=1.2`, `w_goal=1`, `w_terminal=5`, `w_obstacle=300`.

Hai profile cuối cùng dùng để kiểm tra trade-off:

| Profile | H / N | lambda / noise XY | vmax / accel XY | goal / terminal / obstacle |
|---|---:|---:|---:|---:|
| `tight_passage` | 40 / 500 | 3.0 / 0.9 | 1.2 / 0.7 | 2 / 10 / 12 |
| `safe_tight` | 40 / 500 | 3.0 / 0.9 | 1.1 / 0.6 | 2 / 10 / 24 |

Tên profile chỉ mô tả ý định thử nghiệm. `tight_passage` không được coi là an
toàn vì kết quả bên dưới có vi phạm margin.

## Kết quả

Sweep seed 7 ban đầu cho thấy baseline, `smooth_slow`, `long_horizon` và
`high_exploration` đều không hoàn thành cả ba scenario. Chỉ tăng horizon từ 30
lên 40, tăng samples lên 500 hoặc tăng noise không phá được local minimum.

Kết quả cuối với ba seed 7, 19, 31:

| Scenario / profile | Success | Clearance min--max (m) | Path mean (m) | Cmd accel RMS (m/s²) | Compute p95 mean (ms) | ESS mean |
|---|---:|---:|---:|---:|---:|---:|
| Slalom / `tight_passage` | 3/3 | 1.80--1.96 | 40.2 | 0.94 | 14.8 | 1.45 |
| Slalom / `safe_tight` | 1/3 | 2.15--2.34 | 41.8 | 0.89 | 15.0 | 1.48 |
| Khe hẹp / `tight_passage` | 3/3 | 1.90--2.13 | 27.8 | 0.92 | 15.3 | 1.48 |
| Khe hẹp / `safe_tight` | 0/3 | 2.24--2.59 | 22.0 | 0.93 | 15.2 | 1.54 |
| Cua L / `tight_passage` | 3/3 | 2.51--2.86 | 22.9 | 0.90 | 18.7 | 1.50 |
| Cua L / `safe_tight` | 3/3 | 2.42--2.69 | 22.8 | 0.85 | 18.5 | 1.46 |

Với margin 2 m, `tight_passage` vi phạm margin ở slalom trong cả ba seed và ở
một seed của khe hẹp. `safe_tight` giữ margin trong các lần này nhưng kẹt khe
hẹp ở cả ba seed. ESS chỉ khoảng 1.4--1.5 trên 500 rollout: trọng số gần như tập
trung vào một hoặc hai rollout, dấu hiệu cost concentration mạnh.

Dữ liệu đầy đủ:

- `output/benchmark/mppi_tuning_seed7.{json,csv}`
- `output/benchmark/mppi_tuning_final_multiseed.{json,csv}`

## Bài học tune từ kết quả

1. **Kiểm tra tính khả thi hình học trước khi tune.** Bề rộng khe phải xét cả
   hai phía margin, kích thước UAV, sai số point-cloud và tracking. Một khe chỉ
   vừa đủ trên lý thuyết không có robustness cho rollout ngẫu nhiên.
2. **Không bắt đầu bằng `samples` hoặc horizon.** Trong benchmark, tăng cả hai
   làm p95 tăng nhưng không giải quyết local minimum. Trước tiên phải cân bằng
   thang goal/terminal/obstacle.
3. **`w_obstacle` hiện cực kỳ nhạy.** Giảm 300 xuống 12 giúp đi qua, nhưng đổi
   một failure về tiến độ thành failure về clearance. Không dùng profile đó
   trực tiếp cho Gazebo hoặc hardware.
4. **Theo dõi ESS khi tune `lambda` và noise.** ESS xấp xỉ 1 cho biết update bị
   chi phối bởi rất ít mẫu. Tăng noise mà ESS vẫn thấp chỉ tạo mẫu phân tán hơn,
   không tạo quyết định robust hơn.
5. **Đặt horizon theo khoảng dừng và vị trí quyết định.** Prediction distance
   phải thấy được cửa ra/góc cua trước khi UAV không còn đủ khoảng dừng:
   `d_stop >= v^2/(2*a) + v*latency`. Sau đó mới tăng horizon; không tăng chỉ
   vì đường khó.
6. **Tune tốc độ và slew sau khi planner tìm được đường.** Giảm `vmax` và
   `max_accel_xy` làm tracking dễ hơn nhưng không tự phá local minimum.
7. **Waypoint là một phần của bài toán.** Node hiện đổi waypoint theo bán kính
   vị trí, không giảm tốc ở waypoint trung gian. Waypoint dày, đặt trong vùng
   có clearance rõ ràng sẽ ổn hơn một waypoint ở sát góc.
8. **`w_du` hiện không phải jerk cost.** Code đang phạt `u - v`, tức sai khác
   giữa velocity command và velocity state. Muốn phạt thay đổi lệnh giữa hai
   bước phải thêm cost `u_k - u_{k-1}` riêng.
9. **Luôn sweep nhiều seed.** Seed 31 của `safe_tight` hoàn thành slalom nhưng
   seed 7 và 19 không; một video thành công không đủ chứng minh robustness.

## Kết luận kỹ thuật

Không có một bộ weight đã thử nào vừa hoàn thành mọi scenario vừa giữ margin
trong cả ba seed. Tiếp tục giảm `w_obstacle` không phải hướng an toàn. Trước khi
tune live nên cải tiến objective/constraint theo thứ tự:

1. tách hard collision/infeasible rollout khỏi soft clearance preference;
2. chuẩn hóa cost theo horizon và theo scale vật lý;
3. thêm reference corridor/global path hoặc waypoint-transition mượt;
4. thêm cost thực cho `Delta u`;
5. dùng ESS để điều chỉnh `lambda`/sampling và kiểm tra lại đa seed;
6. sau đó mới xác nhận Gazebo với LiDAR, MAVLink và ArduPilot tracking.

## Cách chạy lại

Chạy toàn bộ sweep một seed:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --seeds 7 \
  --output output/benchmark/mppi_tuning_seed7
```

Chạy hai profile cuối với ba seed:

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/benchmark_mppi_tuning.py \
  --profiles tight_passage,safe_tight \
  --seeds 7,19,31 \
  --output output/benchmark/mppi_tuning_final_multiseed
```

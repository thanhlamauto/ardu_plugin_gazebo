# Kiến trúc UAV Navigation

**Trạng thái:** mentor đã duyệt ngày 17/09/2026. M1–M4 đã port A*, safety,
conditioner, MPPI dynamics/rollout/objective/optimizer sang C++ thuần và kiểm
parity bằng golden fixtures. M5 đã nối các module vào `local_navigation_node`,
ROS 2 launch, Gazebo/ArduPilot SITL closed loop, diagnostics và RViz cost view.
M6 đã thay bridge Python bằng C++ `autopilot_adapter_node`; MAVROS chịu trách
nhiệm transport và ENU→NED. Python không còn nằm trong runtime control path.

## 1. Mục tiêu

Tách phần navigation hiện đang nằm trong Python experiment thành một core C++
có thể dùng chung cho Gazebo/SITL và edge device. ROS 2 chỉ đảm nhiệm giao tiếp,
lifecycle, parameters, launch và visualization. Gazebo, MAVLink và ROS không
được xuất hiện trong API của core.

Phạm vi kiến trúc đã chốt:

- định nghĩa module, dependency direction và interface;
- định nghĩa topic, frame, QoS và parameter ownership;
- định nghĩa cách bringup simulation/hardware;
- định nghĩa visualization và test matrix;
- đóng gói để người sau có thể thay adapter mà dùng lại cùng core.

Chưa triển khai sau Milestone 6:

- PA-MPPI;
- hardware qualification;
- full simulation stress matrix và edge-device benchmark;
- tuning thuật toán ngoài cấu hình MPPI hiện đã kiểm chứng.

## 2. Hiện trạng và khoảng trống

Pipeline Python vẫn là oracle/regression baseline. Luồng C++/ROS 2 hiện đã chạy
closed loop trong Gazebo/ArduPilot SITL, nhưng còn các khoảng trống:

- SITL vẫn cần operator arm/takeoff/land, không thuộc navigation launch;
- chưa chạy đủ stress matrix và chưa đo p95/p99 trên edge device mục tiêu;
- hardware sensor/localization/map adapters chưa được chốt.

Mục tiêu của kiến trúc mới là thay các coupling này, không thay đổi kết luận
thuật toán hiện tại.

## 3. Kiến trúc hệ thống

```text
                         ┌──────────────────────┐
LiDAR / prior map ──────►│ Mapping / Cost Grid  │
                         └──────────┬───────────┘
                                    │
RViz goal ──────────────►┌──────────▼───────────┐
                         │ Global Planner (A*)   │
                         └──────────┬───────────┘
                                    │ global path
Odometry ───────────────►┌──────────▼───────────┐
Obstacle representation ►│ Local Planner (MPPI)  │
                         └──────────┬───────────┘
                                    │ timed trajectory + raw control
                         ┌──────────▼───────────┐
                         │ Safety + Conditioner  │
                         └──────────┬───────────┘
                                    │ safe control
                         ┌──────────▼───────────┐
                         │ Autopilot Adapter     │
                         └──────────┬───────────┘
                                    │ safe TwistStamped
                         ┌──────────▼───────────┐
                         │ MAVROS                │
                         └──────────┬───────────┘
                                    │ MAVLink
                                    ▼
                                ArduPilot
```

```text
Simulation adapters                     Hardware adapters

Gazebo PointCloud ──┐                   Real LiDAR ──────────┐
Gazebo Odometry ────┼──► SAME CORE ◄─── VIO/LIO/Odometry ──┤
ArduPilot SITL ◄────┘                   Flight controller ◄──┘
```

Dependency chỉ đi một chiều:

```text
uav_navigation_bringup → uav_navigation_ros → uav_navigation_core
                                              ↑
                              API/header không chứa ROS/Gazebo/MAVLink
```

## 4. Ba package mục tiêu

### `uav_navigation_core`

API C++ thuần, không có ROS, Gazebo hoặc MAVLink runtime/header dependency.
Build hỗ trợ standalone CMake và ament/colcon. Public headers và target không
phụ thuộc ROS, Gazebo hoặc SDF.

Trách nhiệm:

- kiểu dữ liệu SI/ENU và timestamp;
- `IGlobalPlanner`, `ILocalPlanner`, `ISafetyChecker`, `ICommandConditioner`;
- A*, MPPI, response model và stopping/collision predicates;
- deterministic unit tests và benchmark API.

Target deployment artifact sau khi implement:

```text
libuav_navigation_core.so
include/uav_navigation_core/*.hpp
```

Target hiện sinh shared library `libuav_navigation_core` và export CMake package.
Core có `CostGrid2D`, A*, `ICollisionEnvironment`, trajectory safety, velocity
conditioner và toàn bộ MPPI CPU gồm dynamics, rollout, objective, sampling,
proposal và optimizer update. Safety kiểm swept segment và stopping distance
trên dynamic cloud lẫn static geometry. Golden fixtures do Python sinh nhưng
CTest không phụ thuộc Python.

### `uav_navigation_ros`

ROS 2 adapters/nodes. Không chứa thuật toán planning.

Trách nhiệm:

- đổi ROS messages sang core types và ngược lại;
- TF/frame validation;
- parameter validation, lifecycle và diagnostics;
- global-planner node;
- local-navigation node;
- ArduPilot adapter;
- cost/trajectory visualization.

**Quyết định đã duyệt:** MPPI, safety checker và conditioner là ba core objects
nhưng được compose trong cùng `local_navigation_node`. Trajectory có time,
velocity và control không phải serialize qua `nav_msgs/Path`; safety luôn kiểm
đúng object mà MPPI vừa sinh. ROS chỉ publish bản visualization và safe command.

### `uav_navigation_bringup`

Chỉ chứa launch, ROS parameter YAML và RViz config.

- `sim.launch.xml`: Gazebo, SITL, bridge, core ROS nodes và RViz;
- `hardware.launch.xml`: sensor/localization adapters, core ROS nodes và flight
  controller connection;
- `navigation.yaml`: source of truth cho navigation parameters;
- `navigation.rviz`: goal, cost grid, paths và MPPI sample cost.

`sim.launch.xml` chạy Gazebo server/GUI, bridge odometry/TF/LiDAR, C++ global
planner, C++ local navigation và RViz. C++ adapter + MAVROS được tắt mặc định
để operator arm/takeoff SITL trước khi planner gửi setpoint.

## 5. Core C++ contracts

Interface chi tiết nằm tại
[`uav_navigation_core/include/uav_navigation_core/interfaces.hpp`](../uav_navigation_core/include/uav_navigation_core/interfaces.hpp).
Các nguyên tắc phải giữ:

- đơn vị SI; position/velocity/control ở ENU;
- timestamp monotonic, không dùng wall clock để rollout;
- input immutable, output có status và diagnostics;
- không singleton/global mutable state;
- không throw qua control-loop boundary; lỗi runtime trả về status;
- allocation trong hot path phải được đo và giới hạn sau khi backend được chọn.

Flow một chu kỳ local planner:

```text
State + ObstacleMap + GlobalPath
              │
              ▼
ILocalPlanner::compute()
              │ LocalPlan{trajectory, raw_control, diagnostics}
              ▼
ISafetyChecker::evaluate(trajectory)
              │
       safe ───┴── unsafe/no-plan
        │                 │
ICommandConditioner       └──► defined recovery/abort policy
        │
        ▼
safe Control
```

Backend MPPI hiện là C++17 CPU và public interface không phụ thuộc
Eigen/CUDA/Torch, nên có thể thay backend mà không đổi ROS contract.

## 6. ROS graph và topic contract

Frame chuẩn: `odom` cho local navigation; `base_link` cho body. Adapter phải TF
transform hoặc reject message sai frame, không âm thầm coi hai frame giống nhau.

| Topic | Type | Producer → consumer | QoS đề xuất | Ý nghĩa |
|---|---|---|---|---|
| `/localization/odometry` | `nav_msgs/Odometry` | state adapter → local navigation | sensor data, depth 5 | State ENU đã đồng bộ |
| `/perception/obstacles` | `sensor_msgs/PointCloud2` | LiDAR/map adapter → local navigation | best effort, depth 1 | Obstacle cloud mới nhất |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz/mission → global planner | reliable, depth 1 | Goal có frame rõ ràng |
| `/planning/global_costmap` | `nav_msgs/OccupancyGrid` | global planner → RViz | reliable + transient local, depth 1 | 0 free, 1–99 inflated cost, 100 blocked |
| `/planning/global_path` | `nav_msgs/Path` | global planner → local navigation/RViz | reliable + transient local, depth 1 | Geometric path, chưa phải speed schedule |
| `/planning/mppi/predicted_path` | `nav_msgs/Path` | local navigation → RViz | best effort, depth 1 | Nominal rollout visualization |
| `/planning/mppi/cost_samples` | `visualization_msgs/MarkerArray` | local navigation → RViz | best effort, depth 1 | Sample màu theo cost/feasibility |
| `/control/safe_velocity_command` | `geometry_msgs/TwistStamped` | local navigation → autopilot adapter | reliable, depth 1 | ENU velocity + yaw rate đã qua safety |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | mọi node → operator/logger | reliable, depth 10 | deadline, stale input, feasibility, mode |

`nav_msgs/Path` chỉ dùng cho visualization/global geometry. Timed local
trajectory đầy đủ nằm trong process local navigation. Nếu về sau tách safety
thành process khác, phải định nghĩa message riêng có timestamp, velocity,
control và model revision; không dùng `Path` thay thế.

## 7. Parameter ownership

Source of truth mục tiêu:
[`uav_navigation_bringup/config/navigation.yaml`](../uav_navigation_bringup/config/navigation.yaml).

| Owner | Nhóm parameter |
|---|---|
| `global_planner` | algorithm, resolution, bounds, clearance, altitude policy |
| `local_navigation` | rate, horizon, samples, temperature, limits, costs, response model |
| `local_navigation` | stopping/collision predicate và conditioner vì cùng process |
| `autopilot_adapter` | readiness mode, command limits, heartbeat và command timeout |
| MAVROS launch/config | FCU transport URL, system/component ID và ENU→NED conversion |
| `local_navigation` | publish rate, sample count, color/range |
| ArduPilot `.parm` | flight-controller parameters; không copy vào planner YAML |

Mọi parameter safety-critical phải có range validation và được ghi vào run
manifest. Runtime parameter update mặc định bị từ chối cho dynamics, constraint
và frame; chỉ visualization parameters được đổi tự do.

## 8. Simulation và hardware bringup

### Simulation

```text
Gazebo server + optional GUI
ArduPilot SITL
ros_gz_bridge / robot_state_publisher
global_planner_node
local_navigation_node
autopilot_adapter_node + MAVROS
optional RViz
```

### Hardware

```text
LiDAR driver + localization/VIO/LIO
sensor/state adapters
global_planner_node
local_navigation_node
C++ autopilot adapter + MAVROS
optional RViz
```

Core và navigation config schema giữ nguyên. Chỉ launch adapter, topic remap,
device/backend và vehicle calibration khác. Hardware launch không được tự arm;
arming thuộc operator/autopilot safety procedure.

## 9. Failure policy

| Tình huống | Hành vi thiết kế |
|---|---|
| Odometry/obstacle stale | Không gọi planner; phát status và safe hold/abort policy đã cấu hình |
| Goal sai frame/NaN | Reject goal, giữ route hiện tại |
| Global path không tồn tại | Publish `NO_PATH`; không gửi command tiến |
| MPPI timeout | Không dùng output quá deadline; chuyển recovery policy |
| `N_safe=0` | Dùng verified recovery nếu có; nếu không, báo `NO_SAFE_TRAJECTORY` và giao policy cho safety supervisor |
| MAVLink mất heartbeat | Autopilot adapter ngừng stream command và phát fatal diagnostic |
| Node restart | Không tự resume command cho tới khi state/map/path hợp lệ lại |

Zero velocity setpoint hiện chưa được chứng minh là emergency trajectory. Thiết
kế không được gắn nhãn nó là safe recovery trước khi có verification.

## 10. RViz cost view

Global planner xuất `OccupancyGrid` tại altitude lập kế hoạch:

```text
0       free
1–99    inflation / traversal cost
100     blocked
-1      unknown, nếu map backend hỗ trợ unknown
```

RViz hiển thị cost grid và global path để giải thích vì sao A* chọn đường.

MPPI không bị ép thành costmap 2D. Cost nằm trên rollout trajectory, nên publish
`MarkerArray`:

- xanh: feasible, cost thấp;
- vàng: feasible, cost trung bình;
- đỏ: feasible, cost cao;
- xám/đỏ đậm: rejected; namespace cho biết collision/stopping/other;
- nominal path có line width riêng;
- diagnostics hiển thị `N_safe`, ESS, compute time và rejection reason.

Visualization được rate-limit và có thể tắt hoàn toàn. Nó không nằm trong
control-loop critical path.

## 11. Test plan

### Core deterministic tests

- straight/open space;
- góc 90 độ trái/phải;
- narrow passage;
- blocked/unreachable goal;
- obstacle ngoài FOV nhưng có trong prior map;
- stale/NaN/frame mismatch;
- stopping response và delay bounds;
- deterministic seed/replay;
- không có safe sample và recovery policy.

### Closed-loop simulation matrix

| Dimension | Giá trị tối thiểu |
|---|---|
| Speed request | 5, 10 m/s |
| Turn | trái/phải, 45/90 độ |
| Passage | rộng, hẹp, blocked |
| Initial state | hover, đang cruise, lệch path |
| Sensor | nominal, latency, drop, stale |
| Seeds | đủ để báo success rate và tail latency, không chỉ 2 seed |
| Runtime | headless benchmark; GUI; GUI+RViz tách riêng |

### Edge acceptance

- target CPU/GPU/OS/ROS distro được ghi rõ;
- control rate và p95/p99 deadline trên target;
- peak RSS, allocation và thermal throttling;
- restart, mất sensor, mất MAVLink;
- HIL trước flight test.

## 12. Roadmap triển khai

1. Interfaces, topic, frame và failure policy — hoàn thành.
2. Global cost-grid + A* C++ và Python fixtures — hoàn thành.
3. Response model, safety và conditioner C++ — hoàn thành.
4. MPPI C++ CPU và component-level parity — hoàn thành.
5. ROS local navigation, cost visualization và Gazebo/SITL closed loop — hoàn
   thành ở mức integration checkpoint.
6. C++ autopilot adapter + MAVROS — hoàn thành.
7. Full launch regression, stress matrix và deadline statistics — M7.
8. ARM64/x86-64 artifact, HIL và hardware qualification — M8.
9. PA-MPPI — chỉ bắt đầu sau khi kiến trúc deployment ổn định.

Python implementation tiếp tục là oracle/regression reference trong quá trình
port; không xóa trước khi C++ đạt parity.

## 13. Các quyết định deployment còn mở

1. ROS 2 distro và Ubuntu version mục tiêu?
2. Edge device cụ thể, CPU/GPU/RAM và có CUDA hay không?
3. Global map trên phần cứng là prior map, online occupancy hay cả hai?
4. Localization source và frame tree chính thức?
5. MAVROS là backend M6; direct MAVLink chỉ được cân nhắc lại nếu profiling edge yêu cầu.
6. Safety/recovery nằm trong process local navigation hay một supervisor riêng?
7. Có cho phép runtime parameter update đối với nhóm nào?
8. Acceptance rate, latency và hardware test gates cần đạt?

Các mục này phải được chốt trước hardware qualification; chúng không chặn core
C++ và Gazebo/SITL integration hiện tại.

## 14. Tham khảo kiến trúc

- ROS 2 Jazzy launch và `ament_cmake`: tài liệu chính thức tại
  <https://docs.ros.org/en/jazzy/>.
- SUPER của HKU MaRS Lab: tham khảo cách tách planner, map, mission và config;
  không coi project hiện tại là reproduction của SUPER:
  <https://github.com/hku-mars/SUPER>.

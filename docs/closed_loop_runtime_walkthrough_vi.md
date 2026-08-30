# Chạy và quan sát closed loop ArduPilot–Gazebo

Runbook này ghi lại các lệnh đã dùng để chạy Iris, cất cánh lên 2 m và quan sát
trực tiếp hai chiều của UDP physics loop trên macOS.

## Kết quả đã quan sát

Trong lần chạy ngày 2026-08-28:

- Gazebo load thành công `ArduPilotPlugin`, IMU, 8 `LiftDrag` systems và 4
  `ApplyJointForce` systems.
- SITL nhận đủ `timestamp`, IMU, position, quaternion và velocity từ Gazebo.
- Iris cất cánh và hover ở Gazebo `z ≈ +2.19 m`.
- JSON gửi về ArduPilot chứa NED `position[2] ≈ -2.19 m`.
- Một PWM packet khi hover dài 40 byte, có magic `18458`, rate `1200 Hz` và bốn
  motor PWM lần lượt là `1562, 1560, 1559, 1563`.

## 1. Build

Build ArduPilot SITL:

```bash
cd ~/Projects/ardupilot
./waf configure --board sitl
./waf copter
```

Build Gazebo plugin:

```bash
cd ~/Projects/ardupilot_gazebo
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j4
```

## 2. Terminal 1 — chạy Gazebo server

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_walkthrough
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v4 -r "$PWD/worlds/iris_runway.sdf" -s
```

Nếu muốn mở GUI, chạy trong terminal khác với cùng partition:

```bash
export GZ_PARTITION=ardupilot_walkthrough
gz sim -v4 -g
```

## 3. Terminal 2 — chạy ArduPilot và MAVProxy

Máy được dùng trong walkthrough có dependency ArduPilot/MAVProxy trong Python
3.10.12. Đặt Python đó lên đầu `PATH` trước khi gọi `sim_vehicle.py`:

```bash
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"

cd ~/Projects/ardupilot

python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f JSON \
    -N \
    --add-param-file="$HOME/Projects/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
```

`-N` bỏ qua rebuild. Xóa tùy chọn này nếu source ArduPilot vừa thay đổi. Chỉ
thêm `-w` khi muốn xóa EEPROM và load parameter lại từ đầu.

Nếu gặp `ModuleNotFoundError: pexpect`, kiểm tra đúng interpreter:

```bash
python3 -c 'import pexpect, MAVProxy; print("Python dependencies: OK")'
```

### Cách chạy trực tiếp đã dùng trong lần kiểm thử

Nếu không muốn qua `sim_vehicle.py`, mở terminal SITL:

```bash
mkdir -p /tmp/ardupilot-gazebo-walkthrough
cd /tmp/ardupilot-gazebo-walkthrough

~/Projects/ardupilot/build/sitl/bin/arducopter \
    --model JSON \
    --speedup 1 \
    --home 40.072842,-105.230575,0,0 \
    --defaults ~/Projects/ardupilot_gazebo/config/gazebo-iris-gimbal.parm
```

Sau đó mở thêm một terminal để kết nối MAVProxy:

```bash
~/.pyenv/versions/3.10.12/bin/mavproxy.py \
    --master=tcp:127.0.0.1:5760 \
    --console
```

## 4. Cất cánh

Trong MAVProxy, chạy từng lệnh và chờ prompt trả về:

```text
mode guided
arm throttle
takeoff 2
```

Hạ cánh:

```text
mode land
```

Không dùng lệnh trần `land` trong MAVProxy. Lệnh đó gửi
`MAV_CMD_DO_LAND_START` và chỉ hoạt động khi mission có landing sequence.

## 5. Quan sát UDP physics loop

Capture một cặp packet JSON/PWM trên macOS loopback:

```bash
tcpdump -i lo0 -nn -c 2 -s 0 -XX 'udp port 9002'
```

Hai chiều có thể phân biệt bằng port và payload:

```text
SITL source-port → 9002     40-byte binary PWM packet
9002 → SITL source-port     khoảng 480–510 byte JSON state packet
```

Đầu binary packet hover đã quan sát:

```text
1a48 b004 76c0 0000 1a06 1806 1706 1b06
```

Các field là little-endian:

```text
1a48          magic       = 0x481a = 18458
b004          frame_rate  = 0x04b0 = 1200 Hz
76c00000      frame_count = 0x0000c076 = 49270
1a06          pwm[0]      = 0x061a = 1562
1806          pwm[1]      = 0x0618 = 1560
1706          pwm[2]      = 0x0617 = 1559
1b06          pwm[3]      = 0x061b = 1563
```

Trong chiều ngược lại, phần ASCII của packet bắt đầu bằng:

```json
{"timestamp":88.838,"imu":{"gyro":[...],"accel_body":[...]},"position":[...],"quaternion":[...],"velocity":[...]}
```

## 6. Quan sát pose và IMU từ Gazebo Transport

Các terminal quan sát phải dùng cùng partition:

```bash
export GZ_PARTITION=ardupilot_walkthrough
```

Liệt kê topic:

```bash
gz topic -l | grep -E 'imu|pose|joint'
```

Nếu đã cài `ripgrep` bằng `brew install ripgrep`, có thể dùng `rg` thay cho
`grep -E`.

Đọc đúng một pose message và chỉ lấy model Iris:

```bash
gz topic -e -n 1 --json-output \
    -t /world/iris_runway/pose/info \
    | jq '.pose[] | select(.name == "iris_with_gimbal") | {name, position, orientation}'
```

Kết quả hover mẫu:

```json
{
  "name": "iris_with_gimbal",
  "position": {
    "x": 0.0068,
    "y": -0.0015,
    "z": 2.1900
  }
}
```

Đọc một IMU message:

```bash
gz topic -e -n 1 --json-output \
    -t /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu \
    | jq '{angularVelocity, linearAcceleration}'
```

Đo publish rate của IMU:

```bash
gz topic -f \
    -t /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

## 7. Map observation về source code

```text
40-byte UDP packet
  SIM_JSON::output_servos()
  → ArduPilotPlugin::ReceiveServoPacket()

PWM 1562, 1560, 1559, 1563
  ArduPilotPlugin::UpdateMotorCommands()
  → ArduPilotPlugin::ApplyMotorForces()

Gazebo z ≈ +2.19 m
  Gazebo physics / WorldPose
  → ArduPilotPlugin::CreateStateJSON()

JSON position[2] ≈ -2.19 m
  ENU → NED transform
  → ArduPilotPlugin::SendState()
  → SIM_JSON::recv_fdm()
```

Tìm nhanh các hàm:

```bash
cd ~/Projects/ardupilot_gazebo

grep -nE \
    'PreUpdate|ReceiveServoPacket|UpdateMotorCommands|ApplyMotorForces|PostUpdate|CreateStateJSON|SendState' \
    src/ArduPilotPlugin.cc

grep -nE \
    'output_servos|recv_fdm|parse_sensors|JSON::update' \
    ~/Projects/ardupilot/libraries/SITL/SIM_JSON.cpp
```

## 8. Raw output và đường đi qua từng hàm

Phần này dùng dữ liệu capture trong lần chạy ngày 2026-08-28. Iris đang hover
ở khoảng 2.2 m.

### 8.1 Raw Gazebo IMU message

Lệnh:

```bash
export GZ_PARTITION=ardupilot_walkthrough

gz topic -e -n 1 --json-output \
    -t /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

Raw output nhận từ Gazebo Transport:

```json
{"header":{"stamp":{"sec":"67","nsec":882000000},"data":[{"key":"frame_id","value":["imu_link"]},{"key":"seq","value":["67881"]}]},"entityName":"iris_with_gimbal::iris_with_standoffs::imu_link::imu_sensor","orientation":{"x":1.0141056348660269e-05,"y":-0.00015632988019734426,"z":0.0077105977293849357,"w":0.99997026062817662},"orientationCovariance":{"data":[0,0,0,0,0,0,0,0,0]},"angularVelocity":{"x":-0.00088059403271956963,"y":-0.00050360820605934,"z":-2.49387497146798e-05},"angularVelocityCovariance":{"data":[0,0,0,0,0,0,0,0,0]},"linearAcceleration":{"x":0.00070794471700952365,"y":-0.0039944492242087052,"z":-9.8074925511174627},"linearAccelerationCovariance":{"data":[0,0,0,0,0,0,0,0,0]}}
```

Đường đi của message:

```text
Gazebo Imu system
  │ publish gz.msgs.IMU
  ▼
Gazebo Transport topic
  │ Node::Subscribe(..., ImuCb)
  ▼
ArduPilotPluginPrivate::ImuCb(const gz::msgs::IMU &msg)
  │ lock imuMsgMutex
  │ copy msg → dataPtr->imuMsg
  │ set imuMsgValid = true
  ▼
ArduPilotPlugin::CreateStateJSON()
  │ lock mutex và copy latest imuMsg
  │ đọc angular_velocity + linear_acceleration
  ▼
JSON fields imu.gyro và imu.accel_body
```

`ImuCb()` trả về `void`; kết quả của nó là side effect: cập nhật bản IMU mới
nhất trong `dataPtr->imuMsg`. `CreateStateJSON()` đọc bản copy này sau physics
step.

### 8.2 Raw Gazebo WorldPose

Lệnh quan sát:

```bash
gz topic -e -n 1 --json-output \
    -t /world/iris_runway/pose/info \
    | jq -c '.pose[] | select(.name == "iris_with_gimbal") | {name, position, orientation}'
```

Raw output:

```json
{"name":"iris_with_gimbal","position":{"x":0.004394947935510287,"y":0.0017641962959597359,"z":2.1993059944760116},"orientation":{"x":-0.00011319587629428846,"y":0.00011073936277787348,"z":0.70161314069598013,"w":0.71255805077786738}}
```

Topic `/pose/info` chỉ dùng để người vận hành quan sát. `ArduPilotPlugin` không
subscribe topic này. Plugin đọc component trực tiếp từ Gazebo ECS:

```text
Gazebo physics
  │ cập nhật components::WorldPose
  │ cập nhật components::WorldLinearVelocity
  ▼
ArduPilotPlugin::CreateStateJSON(ecm)
  │ ecm.Component<WorldPose>(imuLink)
  │ ecm.Component<WorldLinearVelocity>(imuLink)
  │ ENU/body frame → NED/FRD
  ▼
JSON position + quaternion + velocity
```

Gazebo báo `z = +2.1993 m` vì ENU dùng trục Z hướng lên. UDP JSON phía dưới báo
`position[2] = -2.1994 m` vì ArduPilot dùng NED với trục Z hướng xuống.

### 8.3 Raw UDP JSON: plugin gửi về SITL

Lệnh capture:

```bash
tcpdump -i lo0 -nn -c 2 -s 0 -XX 'udp port 9002'
```

Raw packet đầu tiên:

```text
16:08:31.079403 IP 127.0.0.1.9002 > 127.0.0.1.61774: UDP, length 484
    0x0000:  0200 0000 4500 0200 449b 0000 4011 0000
    0x0010:  7f00 0001 7f00 0001 232a f14e 01ec ffff
    0x0020:  0a7b 2274 696d 6573 7461 6d70 223a 3535
    0x0030:  2e38 3936 2c22 696d 7522 3a7b 2267 7972
    0x0040:  6f22 3a5b 302e 3030 3430 3739 3036 3236
    0x0050:  3232 3534 3730 3638 2c30 2e30 3030 3338
    0x0060:  3935 3831 3537 3031 3831 3331 3539 362c
    0x0070:  2d30 2e30 3030 3032 3938 3531 3034 3534
    0x0080:  3931 3837 3332 325d 2c22 6163 6365 6c5f
    0x0090:  626f 6479 223a 5b30 2e30 3030 3738 3939
    0x00a0:  3732 3432 3636 3635 3239 3635 2c30 2e30
    0x00b0:  3039 3734 3234 3035 3835 3633 3938 3932
    0x00c0:  2c2d 392e 3830 3736 3138 3733 3333 3138
    0x00d0:  3832 365d 7d2c 2270 6f73 6974 696f 6e22
    0x00e0:  3a5b 302e 3030 3231 3334 3439 3830 3539
    0x00f0:  3436 3337 3233 2c2d 302e 3030 3032 3936
    0x0100:  3836 3036 3133 3039 3730 3637 3636 2c2d
    0x0110:  322e 3139 3934 3134 3639 3638 3838 3936
    0x0120:  3039 5d2c 2271 7561 7465 726e 696f 6e22
    0x0130:  3a5b 302e 3939 3939 3730 3633 3131 3933
    0x0140:  3539 3031 2c2d 302e 3030 3031 3138 3739
    0x0150:  3332 3837 3033 3238 3730 3133 2c2d 302e
    0x0160:  3030 3030 3136 3436 3139 3930 3032 3732
    0x0170:  3839 3136 2c30 2e30 3037 3636 3330 3532
    0x0180:  3039 3736 3239 3239 375d 2c22 7665 6c6f
    0x0190:  6369 7479 223a 5b30 2e30 3030 3538 3832
    0x01a0:  3532 3031 3935 3436 3037 3936 2c30 2e30
    0x01b0:  3030 3936 3133 3035 3036 3532 3839 3936
    0x01c0:  3734 2c2d 302e 3030 3134 3438 3436 3330
    0x01d0:  3030 3133 3734 3534 325d 2c22 6e6f 5f74
    0x01e0:  696d 655f 7379 6e63 223a 7472 7565 2c22
    0x01f0:  6e6f 5f6c 6f63 6b73 7465 7022 3a66 616c
    0x0200:  7365 7d0a
```

Payload ASCII được decode thành:

```json
{
  "timestamp": 55.896,
  "imu": {
    "gyro": [0.004079062622547068, 0.00038958157018131596, -0.00002985104549187322],
    "accel_body": [0.0007899724266652965, 0.00974240585639892, -9.807618733318826]
  },
  "position": [0.002134498059463723, -0.00029686061309706766, -2.1994146968889609],
  "quaternion": [0.9999706311935901, -0.00011879328703287013, -0.00001646199002728916, 0.007663052097629297],
  "velocity": [0.0005882520195460796, 0.0009613050652899674, -0.0014484630001374542],
  "no_time_sync": true,
  "no_lockstep": false
}
```

Đường đi qua code:

```text
ArduPilotPlugin::PostUpdate()
  ▼
CreateStateJSON()
  │ return type: void
  │ side effect: ghi dataPtr->json_str
  ▼
SendState()
  │ SocketUDP::sendto(json_str, fcu_address, fcu_port_out)
  ▼
UDP 127.0.0.1:9002 → 127.0.0.1:61774
  ▼
SITL JSON::recv_fdm()
  │ sock.recv() trả số byte nhận được
  ▼
JSON::parse_sensors()
  │ trả uint64_t received_bitmask
  │ bitmask cho biết field nào đã parse thành công
  ▼
accel_body, gyro, velocity_ef, position và attitude của SITL
  ▼
EKF/controller tính actuator output cho frame kế tiếp
```

### 8.4 Raw UDP binary: SITL gửi PWM sang plugin

Raw packet thứ hai trong cùng capture:

```text
16:08:31.079467 IP 127.0.0.1.61774 > 127.0.0.1.9002: UDP, length 40
    0x0000:  0200 0000 4500 0044 3dce 0000 4011 0000
    0x0010:  7f00 0001 7f00 0001 f14e 232a 0030 fe43
    0x0020:  1a48 b004 2ac5 0000 1906 1806 1806 1a06
    0x0030:  0000 0000 0000 0000 4c04 4c04 0807 0000
    0x0040:  0000 0000 0000 0000
```

UDP/IP header kết thúc trước offset `0x0020`. Binary payload 40 byte bắt đầu từ
`1a48`:

```text
Bytes       Field             Decode
1a48        magic             0x481a = 18458
b004        frame_rate        0x04b0 = 1200 Hz
2ac50000    frame_count       0x0000c52a = 50474
1906        pwm[0]            0x0619 = 1561
1806        pwm[1]            0x0618 = 1560
1806        pwm[2]            0x0618 = 1560
1a06        pwm[3]            0x061a = 1562
```

Các integer trong payload này dùng little-endian. Riêng IP và UDP header hiển
thị theo network byte order. Port `61774` là source port tạm thời của SITL;
plugin bind cố định tại port `9002`.

Đường đi qua code:

```text
ArduPilot JSON::update(input)
  ▼
JSON::output_servos(input)
  │ copy input.servos[] → servo_packet_16.pwm[]
  │ SocketAPM::sendto(packet, 40, 127.0.0.1, 9002)
  ▼
UDP 127.0.0.1:61774 → 127.0.0.1:9002
  ▼
ArduPilotPlugin::PreUpdate()
  ▼
ReceiveServoPacket()
  │ getServoPacket() trả ssize_t recvSize = 40
  │ kiểm tra magic + frame_count
  │ trả bool: true nếu đây là frame hợp lệ và mới
  ▼
UpdateMotorCommands(pwm)
  │ return type: void
  │ side effect: ghi controls[i].cmd
  ▼
ApplyMotorForces(dt, ecm)
  │ đọc JointVelocity
  │ PID.Update(current - target, dt)
  │ ghi components::JointForceCmd
  ▼
Gazebo physics + LiftDrag
```

Với motor 0:

```text
PWM       = 1561
raw_cmd   = (1561 - 1100) / (1900 - 1100)
          = 0.57625
target    = 838 × 0.57625
          = 482.8975 rad/s
```

Motor 2 và motor 3 dùng multiplier `-838`, nên target velocity có dấu âm để
mô phỏng chiều quay ngược.

### 8.5 Một simulation frame hoàn chỉnh

```text
JSON::update(input)
  │
  ├─ output_servos() ──40-byte PWM UDP──► ReceiveServoPacket()
  │                                         │
  │                                  UpdateMotorCommands()
  │                                         │
  │                                  ApplyMotorForces()
  │                                         │
  │                                  Gazebo physics
  │                                         │
  │                           IMU callback + WorldPose ECS
  │                                         │
  │                                  CreateStateJSON()
  │                                         │
  └─ recv_fdm() ◄────484-byte JSON UDP──── SendState()
         │
         └─ parse_sensors() → simulated state → controller → next frame
```

## 9. Dừng sạch

Trong MAVProxy, hạ cánh trước:

```text
mode land
```

Sau đó nhấn `Ctrl-C` ở terminal MAVProxy/SITL và cuối cùng ở Gazebo server.
Kiểm tra không còn process mô phỏng:

```bash
pgrep -alf 'gz sim|arducopter|sim_vehicle.py|mavproxy' || true
```

## 10. Chạy sensor suite và RViz

Phần này chạy một lần đồng thời camera RGB, depth camera, LiDAR 3D, IMU, từ
kế, khí áp và NavSat. Mọi terminal phải dùng cùng `GZ_PARTITION`; ROS 2 dùng
cùng `ROS_DOMAIN_ID`.

### Terminal 1: Gazebo server

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_sensor_suite
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -v4 -r "$PWD/worlds/iris_sensor_arena.sdf" -s
```

### Terminal 2: Gazebo GUI

```bash
export GZ_PARTITION=ardupilot_sensor_suite
gz sim -v4 -g
```

### Terminal 3: ArduPilot SITL và MAVProxy

```bash
export PATH="$HOME/.pyenv/versions/3.10.12/bin:$PATH"
cd ~/Projects/ardupilot

python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f JSON \
    -N \
    --add-param-file="$HOME/Projects/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
```

### Terminal 4: bridge ROS 2 và RViz

Script thiết lập bridge cho toàn bộ sensor, static TF, marker trạng thái và mở
RViz với cấu hình có sẵn:

```bash
cd ~/Projects/ardupilot_gazebo

export GZ_PARTITION=ardupilot_sensor_suite
export ROS_DOMAIN_ID=42
./scripts/run_sensor_rviz.sh
```

Tên Conda environment mặc định là `ardupilot-rviz`. Nếu dùng tên khác:

```bash
ARDUPILOT_RVIZ_CONDA_ENV=<ten-moi-truong> ./scripts/run_sensor_rviz.sh
```

### Điều khiển UAV trong MAVProxy

```text
mode guided
arm throttle
takeoff 2
velocity 1 0 0
velocity 0 1 0
mode land
```

Video kết quả thực tế:
[Gazebo + RViz GUIDED flight demo](https://drive.google.com/file/d/1D1ceJg4oJFvSS2aUksKp6LQNnHCnOOei/view?usp=sharing).

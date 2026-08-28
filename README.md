# ArduPilot Gazebo: hướng dẫn SITL và C++ sensor subscriber trên macOS

Repository này chứa ArduPilot Gazebo Plugin cùng hướng dẫn thực hành bằng tiếng
Việt. Nội dung trình bày toàn bộ luồng từ build ArduCopter SITL, kết nối với
Gazebo Harmonic, kiểm tra sensor topic, đến viết một C++ subscriber có callback
và processing thread.

Mục tiêu cuối cùng là hiểu và chạy được pipeline:

```text
ArduPilot SITL ──motor commands──> ArduPilotPlugin ──> Gazebo physics
       ^                                                   │
       └──────────────── state / IMU ──────────────────────┘

Gazebo sensor ──> Gazebo Transport ──> C++ callback
                                             │
                                             v
                                      thread-safe queue
                                             │
                                             v
                                      processing thread
                                             │
                                             v
                                       AI / CV / SLAM
```

> [!NOTE]
> Các lệnh và đường dẫn trong tài liệu giả định ArduPilot nằm tại
> `~/Projects/ardupilot`, còn `ardupilot_gazebo` nằm tại
> `~/Projects/ardupilot_gazebo`.

## 1. Các thành phần trong hệ thống

- **ArduPilot SITL** là flight controller được compile để chạy trực tiếp trên
  CPU của máy Mac thay vì trên Pixhawk.
- **Gazebo server** chạy physics, world và sensor simulation.
- **Gazebo GUI** render môi trường 3D và cho phép tương tác với mô phỏng.
- **ArduPilotPlugin** là bridge: nhận motor output từ ArduPilot và gửi
  state/sensor từ Gazebo về ArduPilot.
- **Gazebo Transport** publish dữ liệu sensor qua topic để chương trình khác có
  thể subscribe.

Gazebo server và GUI là hai process riêng, giao tiếp với nhau qua Gazebo
Transport.

## 2. Build ArduPilot SITL

Cài Command Line Tools và các dependency cần thiết:

```bash
xcode-select --install
brew update
brew install cmake gz-harmonic rapidjson opencv gstreamer
```

Clone ArduPilot cùng các submodule:

```bash
git clone --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git \
    ~/Projects/ardupilot
```

Nếu đã có ArduPilot tại `~/Projects/ardupilot`, bỏ qua bước clone. Sau đó build
SITL từ thư mục gốc của repo:

```bash
cd ~/Projects/ardupilot
./waf configure --board sitl
./waf copter
```

Executable được tạo tại:

```text
build/sitl/bin/arducopter
```

## 3. Kiểm tra SITL độc lập

Chạy ArduCopter bằng physics model có sẵn trong SITL, chưa dùng Gazebo:

```bash
cd ~/Projects/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad
```

Trong MAVProxy, thử một chu trình bay đơn giản:

```text
mode guided
arm throttle
takeoff 5
land
```

Bước này giúp tách lỗi. Nếu SITL độc lập hoạt động nhưng mô phỏng Gazebo không
hoạt động, cần tập trung kiểm tra plugin, cấu hình hoặc kết nối giữa hai hệ
thống thay vì flight controller.

## 4. Cài và kiểm tra Gazebo Harmonic

Cài Gazebo trên macOS bằng Homebrew:

```bash
brew install gz-harmonic
```

Chạy server:

```bash
gz sim -v4 shapes.sdf -s
```

Mở terminal khác và chạy GUI:

```bash
gz sim -v4 -g
```

Nếu GUI hiển thị các vật thể mẫu như cube, sphere và cylinder thì Gazebo
server, GUI và Transport đang hoạt động.

## 5. Build plugin `ardupilot_gazebo`

Clone repository này vào thư mục được dùng xuyên suốt tutorial:

```bash
git clone \
    https://github.com/thanhlamauto/ardu_plugin_gazebo.git \
    ~/Projects/ardupilot_gazebo
```

Build plugin:

```bash
cd ~/Projects/ardupilot_gazebo
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j4
```

Trên macOS, kết quả build gồm `libArduPilotPlugin.dylib`. Plugin thực hiện hai
chiều giao tiếp:

```text
ArduPilot ──motor output──> plugin ──> Gazebo
ArduPilot <──state/sensor── plugin <── Gazebo
```

## 6. Chạy Iris trong Gazebo và nối với ArduPilot

Mỗi terminal trong phần này phải dùng cùng một `GZ_PARTITION`. Biến này tách
network Transport của mô phỏng hiện tại khỏi các Gazebo process khác.

Thiết lập environment:

```bash
export GZ_PARTITION=ardupilot_test
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/Projects/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/Projects/ardupilot_gazebo/models:$HOME/Projects/ardupilot_gazebo/worlds"
```

### Terminal 1: Gazebo server

```bash
export GZ_PARTITION=ardupilot_test
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/Projects/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/Projects/ardupilot_gazebo/models:$HOME/Projects/ardupilot_gazebo/worlds"

gz sim -v4 -r \
    "$HOME/Projects/ardupilot_gazebo/worlds/iris_runway.sdf" \
    -s
```

### Terminal 2: Gazebo GUI

```bash
export GZ_PARTITION=ardupilot_test
gz sim -v4 -g
```

### Terminal 3: ArduPilot SITL

```bash
cd ~/Projects/ardupilot

./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f JSON \
    --add-param-file="$HOME/Projects/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
```

Chỉ thêm `-w` khi cần xóa và nạp lại toàn bộ parameter. Không cần dùng tùy chọn
này trong mỗi lần chạy.

Trong MAVProxy:

```text
mode guided
arm throttle
takeoff 5
land
```

Khi Iris cất cánh trong GUI, closed loop sau đã hoạt động:

```text
ArduPilot ──> motor commands ──> Gazebo physics
ArduPilot <── IMU and state <──── Gazebo sensors
```

## 7. Tìm và kiểm tra sensor topic

Nhớ dùng cùng partition với Gazebo server:

```bash
export GZ_PARTITION=ardupilot_test
gz topic -l
```

Lọc các topic thường dùng:

```bash
gz topic -l | grep -Ei 'camera|image|imu|lidar|scan|point'
```

Tên topic phụ thuộc vào world và model. Với world Iris trong ví dụ, IMU topic
có thể là:

```text
/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

Kiểm tra kiểu message:

```bash
gz topic -i -t \
    /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

Kết quả mong đợi có kiểu:

```text
gz.msgs.IMU
```

Echo dữ liệu thực tế:

```bash
gz topic -e -t \
    /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

Nếu không thấy dữ liệu, kiểm tra theo thứ tự:

1. Gazebo server còn chạy và simulation không bị pause.
2. Terminal hiện tại dùng đúng `GZ_PARTITION`.
3. Topic được copy chính xác từ kết quả `gz topic -l`.
4. Sensor tương ứng có trong model đang chạy.

## 8. Hiểu subscriber và callback

Cốt lõi của subscriber chỉ gồm một node và lời gọi `Subscribe`:

```cpp
gz::transport::Node node;
node.Subscribe(topic, imu_callback);
```

Callback nhận message:

```cpp
void imu_callback(const gz::msgs::IMU &msg)
{
    const auto &accel = msg.linear_acceleration();
    const auto &gyro = msg.angular_velocity();
    // Xử lý dữ liệu ở đây.
}
```

Chương trình không tự gọi `imu_callback`. Khi topic có message mới, Gazebo
Transport gọi hàm đã đăng ký. Một hàm trở thành callback vì nó được đưa cho
framework để framework quyết định thời điểm gọi; callback không phải một loại
hàm đặc biệt trong C++.

Callback có thể log, lưu hoặc chuyển tiếp dữ liệu. Tuy nhiên, callback nên làm
ít việc và return nhanh. Chạy neural network hoặc thuật toán nặng trực tiếp
trong callback có thể tạo latency, làm đầy queue nội bộ hoặc khiến frame bị
drop.

## 9. Subscriber với thread-safe queue

Pattern producer-consumer tách việc nhận message khỏi xử lý nặng:

```text
Gazebo Transport thread             Processing thread
          │                                 │
          │ callback                        │
          v                                 │
     copy message                           │
          │                                 │
          v                                 │
  bounded shared queue ───────────────────> pop
                                            │
                                            v
                                      xử lý dữ liệu
```

Tạo một thư mục thử nghiệm bên ngoài source tree để không đưa build artifact
vào repo:

```bash
mkdir -p /tmp/gz_imu_subscriber
cd /tmp/gz_imu_subscriber
```

Tạo `main.cpp` với nội dung:

```cpp
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace {

constexpr std::size_t MAX_QUEUE_SIZE = 10;

std::atomic<bool> running{true};
std::condition_variable queue_cv;
std::deque<gz::msgs::IMU> imu_queue;
std::mutex queue_mutex;

void imu_callback(const gz::msgs::IMU &msg)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex);

        // Giới hạn queue để latency và memory không tăng vô hạn nếu consumer
        // xử lý chậm hơn tốc độ publish của sensor.
        if (imu_queue.size() >= MAX_QUEUE_SIZE) {
            imu_queue.pop_front();
        }
        imu_queue.push_back(msg);
    }
    queue_cv.notify_one();
}

void process_imu()
{
    while (true) {
        gz::msgs::IMU msg;

        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [] {
                return !running.load() || !imu_queue.empty();
            });

            if (!running.load() && imu_queue.empty()) {
                return;
            }

            msg = std::move(imu_queue.front());
            imu_queue.pop_front();
        }

        const auto &accel = msg.linear_acceleration();
        const auto &gyro = msg.angular_velocity();

        std::cout << "accel [m/s^2]: "
                  << accel.x() << ", "
                  << accel.y() << ", "
                  << accel.z() << " | gyro [rad/s]: "
                  << gyro.x() << ", "
                  << gyro.y() << ", "
                  << gyro.z() << '\n';

        // Đặt AI, CV hoặc SLAM processing tại đây.
    }
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <imu-topic>\n";
        return 1;
    }

    const std::string topic = argv[1];
    gz::transport::Node node;

    if (!node.Subscribe(topic, imu_callback)) {
        std::cerr << "Failed to subscribe to " << topic << '\n';
        return 1;
    }

    std::thread worker(process_imu);

    std::cout << "Subscribed to " << topic << '\n'
              << "Press Enter to stop.\n";
    std::cin.get();

    running.store(false);
    queue_cv.notify_all();
    worker.join();
    return 0;
}
```

Tạo `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(gz_imu_subscriber LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gazebo Harmonic sử dụng gz-transport13 và gz-msgs10.
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gz_imu_subscriber main.cpp)
target_link_libraries(
    gz_imu_subscriber
    PRIVATE
        gz-transport13::core
        gz-msgs10::core
)
```

Build:

```bash
cmake -S . -B build
cmake --build build -j4
```

Chạy subscriber trong terminal có cùng partition:

```bash
export GZ_PARTITION=ardupilot_test

./build/gz_imu_subscriber \
    /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

Thay topic trong ví dụ bằng topic lấy từ `gz topic -l` nếu model của bạn dùng
tên khác.

### Vì sao queue cần giới hạn?

Ví dụ camera publish 30 Hz, tức có frame mới khoảng mỗi 33 ms. Nếu processing
mất 200 ms mỗi frame, producer tạo dữ liệu nhanh hơn consumer. Queue không giới
hạn sẽ liên tục tăng, gây tăng memory và latency. Với dữ liệu real-time, thường
hợp lý hơn khi bỏ frame cũ và ưu tiên dữ liệu mới.

Chính sách drop tùy ứng dụng:

- Queue FIFO nhỏ phù hợp khi vẫn cần xử lý một vài sample liên tiếp.
- Chỉ giữ sample mới nhất phù hợp với hiển thị hoặc điều khiển real-time.
- Không drop phù hợp khi mọi sample đều quan trọng, nhưng consumer phải đủ nhanh
  hoặc cần backpressure/persistence phù hợp.

## 10. Thread giao tiếp với nhau như thế nào?

Các thread trong cùng một process dùng chung address space, nên có thể giao
tiếp qua shared memory. Trong ví dụ trên, `imu_queue` là shared memory được cả
callback thread và processing thread truy cập.

Các primitive có vai trò khác nhau:

- `std::mutex` đảm bảo chỉ một thread thay đổi queue tại một thời điểm, tránh
  race condition.
- `std::condition_variable` cho processing thread ngủ khi queue rỗng và được
  đánh thức khi callback push message mới, thay vì polling liên tục.
- `std::atomic<bool>` cho phép các thread đọc/ghi cờ dừng an toàn.
- `std::thread::join()` chỉ chờ một thread kết thúc. Nó không phải cơ chế truyền
  dữ liệu giữa các thread.

Mutex là cần thiết vì một thao tác tưởng như đơn giản có thể gồm nhiều bước đọc,
sửa và ghi. Nếu hai thread xen kẽ các bước này mà không đồng bộ, kết quả phụ
thuộc timing và tạo ra race condition.

## 11. Tổng kết

Sau tutorial này, pipeline hoàn chỉnh là:

1. ArduPilot SITL chạy flight-controller firmware trên macOS.
2. ArduPilotPlugin nối motor command và simulated state giữa SITL với Gazebo.
3. Gazebo mô phỏng physics và publish dữ liệu sensor qua Transport topic.
4. C++ subscriber đăng ký callback với topic.
5. Transport thread gọi callback khi message đến.
6. Callback copy message vào bounded thread-safe queue rồi return nhanh.
7. Processing thread lấy message để chạy AI, OpenCV, SLAM hoặc logic riêng.

Điểm cốt lõi: queue vẫn là shared memory. `mutex`, `condition_variable` và
`atomic` giúp các thread dùng vùng nhớ chung một cách an toàn và hiệu quả.

## Nguồn dự án và giấy phép

Repository này dựa trên
[ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo). Xem
[`LICENSE.md`](LICENSE.md) để biết thông tin giấy phép.

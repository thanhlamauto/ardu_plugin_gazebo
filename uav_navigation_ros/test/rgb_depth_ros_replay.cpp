#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

namespace {
double Percentile(std::vector<double> values, double p) {
  if (values.empty()) return -1.0;
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1,
                         static_cast<std::size_t>(std::floor((values.size() - 1) * p)))];
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: rgb_depth_ros_replay frame.rgb count\n";
    return 2;
  }
  try {
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) throw std::runtime_error("cannot open RGB frame");
    std::vector<std::uint8_t> rgb{std::istreambuf_iterator<char>(input),
                                  std::istreambuf_iterator<char>()};
    if (rgb.size() != 640 * 360 * 3) throw std::runtime_error("expected RGB 640x360");
    const int frames = std::stoi(argv[2]);
    if (frames < 1 || frames > 1000) throw std::invalid_argument("count must be 1..1000");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rgb_depth_ros_replay");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/sensor_suite/rgb", rclcpp::SensorDataQoS());
    int clouds = 0, diagnostics = 0, ok = 0;
    std::vector<double> inference_ms, processing_ms, age_ms, cpu_one_core_percent;
    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacles_camera", rclcpp::SensorDataQoS().keep_last(100),
        [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++clouds; });
    auto diagnostic_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/perception/monocular_depth/diagnostics", rclcpp::QoS(100),
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          for (const auto &status : message->status) {
            if (status.name != "monocular_depth_cpp") continue;
            ++diagnostics;
            if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) continue;
            ++ok;
            for (const auto &entry : status.values) {
              if (entry.key == "inference_ms") inference_ms.push_back(std::stod(entry.value));
              if (entry.key == "processing_ms") processing_ms.push_back(std::stod(entry.value));
              if (entry.key == "image_age_ms") age_ms.push_back(std::stod(entry.value));
              if (entry.key == "cpu_percent_one_core")
                cpu_one_core_percent.push_back(std::stod(entry.value));
            }
          }
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < discovery_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (publisher->get_subscription_count() == 0)
      throw std::runtime_error("depth node did not subscribe to RGB topic");
    const auto start = std::chrono::steady_clock::now() + 1s;
    std::vector<double> intervals_ms;
    auto previous = start;
    for (int i = 0; i < frames; ++i) {
      const auto target = start + i * 100ms;
      while (std::chrono::steady_clock::now() < target) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
      sensor_msgs::msg::Image message;
      message.header.stamp = node->now();
      message.header.frame_id = "sensor_suite_link";
      message.height = 360;
      message.width = 640;
      message.encoding = "rgb8";
      message.step = 640 * 3;
      message.data = rgb;
      publisher->publish(message);
      const auto now = std::chrono::steady_clock::now();
      if (i > 0) intervals_ms.push_back(std::chrono::duration<double, std::milli>(now - previous).count());
      previous = now;
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    std::cout << "frames_sent=" << frames << " clouds_received=" << clouds
              << " diagnostics=" << diagnostics << " ok=" << ok
              << " publish_interval_p50_ms=" << Percentile(intervals_ms, .5)
              << " publish_interval_p95_ms=" << Percentile(intervals_ms, .95)
              << " inference_p50_ms=" << Percentile(inference_ms, .5)
              << " inference_p95_ms=" << Percentile(inference_ms, .95)
              << " processing_p95_ms=" << Percentile(processing_ms, .95)
              << " image_age_p95_ms=" << Percentile(age_ms, .95)
              << " cpu_one_core_p50_percent=" << Percentile(cpu_one_core_percent, .5)
              << " cpu_one_core_p95_percent=" << Percentile(cpu_one_core_percent, .95)
              << '\n';
    executor.remove_node(node);
    rclcpp::shutdown();
    return ok > 0 && clouds > 0 ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

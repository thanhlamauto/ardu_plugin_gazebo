// Replay a recorded Gazebo RGB/pose snapshot into the C++ depth and MPPI nodes.
// No vehicle adapter is started; output commands are observed but never applied.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace {
constexpr std::size_t kFrameBytes = 640 * 360 * 3;

template <typename T>
std::vector<T> ReadFile(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open " + path);
  std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                          std::istreambuf_iterator<char>()};
  if (bytes.size() % sizeof(T) != 0)
    throw std::runtime_error("invalid binary length: " + path);
  std::vector<T> result(bytes.size() / sizeof(T));
  std::copy(bytes.begin(), bytes.end(), reinterpret_cast<char *>(result.data()));
  return result;
}

double Percentile(std::vector<double> values, double p) {
  if (values.empty()) return -1.0;
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(std::floor((values.size() - 1) * p))];
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "usage: rgb_depth_mppi_shadow samples.rgb poses.f32 scene_index frames\n";
    return 2;
  }
  try {
    const int scene = std::stoi(argv[3]);
    const int frames = std::stoi(argv[4]);
    if (scene < 0 || scene > 1 || frames < 1 || frames > 1000)
      throw std::invalid_argument("scene must be 0/1; frames must be 1..1000");
    const auto rgb = ReadFile<std::uint8_t>(argv[1]);
    const auto poses = ReadFile<float>(argv[2]);
    if (rgb.size() != 2 * kFrameBytes || poses.size() != 6)
      throw std::runtime_error("expected two 640x360 RGB frames and two XYZ poses");
    const std::size_t offset = static_cast<std::size_t>(scene) * kFrameBytes;
    const float *position = poses.data() + scene * 3;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rgb_depth_mppi_shadow");
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/sensor_suite/rgb", rclcpp::SensorDataQoS());
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
        "/localization/odometry", rclcpp::SensorDataQoS());
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>(
        "/planning/global_path", rclcpp::QoS(1).reliable().transient_local());
    tf2_ros::TransformBroadcaster tf_broadcaster(node);

    int clouds = 0, depth_ok = 0, depth_warn = 0, commands = 0;
    int planner_diagnostics = 0, obstacle_points = 0;
    std::map<std::string, int> modes;
    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacles_camera", rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++clouds; });
    auto depth_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/perception/monocular_depth/diagnostics", rclcpp::QoS(100),
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          for (const auto &status : message->status) {
            if (status.name != "monocular_depth_cpp") continue;
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
              ++depth_ok;
            else
              ++depth_warn;
          }
        });
    auto planner_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(100),
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          for (const auto &status : message->status) {
            if (status.name != "local_navigation") continue;
            ++planner_diagnostics;
            for (const auto &value : status.values) {
              if (value.key == "mode") ++modes[value.value];
              if (value.key == "obstacle_points")
                obstacle_points = std::stoi(value.value);
            }
          }
        });
    auto command_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/control/safe_velocity_command", rclcpp::QoS(10),
        [&](geometry_msgs::msg::TwistStamped::ConstSharedPtr) { ++commands; });
    (void)cloud_sub;
    (void)depth_sub;
    (void)planner_sub;
    (void)command_sub;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
    while ((image_pub->get_subscription_count() == 0 ||
            odom_pub->get_subscription_count() == 0 ||
            path_pub->get_subscription_count() == 0) &&
           std::chrono::steady_clock::now() < discovery_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (image_pub->get_subscription_count() == 0 ||
        odom_pub->get_subscription_count() == 0 ||
        path_pub->get_subscription_count() == 0)
      throw std::runtime_error("depth or MPPI subscriber missing");

    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = node->now();
    for (const auto &xyz : std::vector<std::vector<double>>{
             {position[0], position[1], position[2]}, {12.0, 0.0, 3.0}}) {
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      point.pose.position.x = xyz[0];
      point.pose.position.y = xyz[1];
      point.pose.position.z = xyz[2];
      point.pose.orientation.w = 1.0;
      path.poses.push_back(point);
    }
    path_pub->publish(path);

    const auto start = std::chrono::steady_clock::now() + 500ms;
    std::vector<double> intervals_ms;
    auto previous = start;
    for (int i = 0; i < frames; ++i) {
      const auto target = start + i * 100ms;
      while (std::chrono::steady_clock::now() < target) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
      const auto stamp = node->now();
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = stamp;
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = position[0];
      odom.pose.pose.position.y = position[1];
      odom.pose.pose.position.z = position[2];
      odom.pose.pose.orientation.w = 1.0;
      odom_pub->publish(odom);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "odom";
      transform.header.stamp = stamp;
      transform.child_frame_id = "sensor_suite_link";
      transform.transform.translation.x = position[0] + 0.08;
      transform.transform.translation.y = position[1];
      transform.transform.translation.z = position[2] + 0.16;
      transform.transform.rotation.w = 1.0;
      tf_broadcaster.sendTransform(transform);

      sensor_msgs::msg::Image image;
      image.header.frame_id = "sensor_suite_link";
      image.header.stamp = stamp;
      image.width = 640;
      image.height = 360;
      image.encoding = "rgb8";
      image.step = 640 * 3;
      image.data.assign(rgb.begin() + offset, rgb.begin() + offset + kFrameBytes);
      image_pub->publish(image);
      const auto now = std::chrono::steady_clock::now();
      if (i > 0)
        intervals_ms.push_back(
            std::chrono::duration<double, std::milli>(now - previous).count());
      previous = now;
    }
    const auto drain = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < drain) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    std::cout << "scene=" << scene << " frames_sent=" << frames
              << " clouds=" << clouds << " depth_ok=" << depth_ok
              << " depth_warn=" << depth_warn
              << " planner_diagnostics=" << planner_diagnostics
              << " planner_active=" << modes["ACTIVE"]
              << " commands=" << commands
              << " obstacle_points_last=" << obstacle_points
              << " publish_p50_ms=" << Percentile(intervals_ms, .5)
              << " publish_p95_ms=" << Percentile(intervals_ms, .95);
    for (const auto &[mode, count] : modes)
      std::cout << " mode_" << mode << '=' << count;
    std::cout << '\n';
    executor.remove_node(node);
    rclcpp::shutdown();
    return clouds > 0 && depth_ok > 0 && modes["ACTIVE"] > 0 && commands > 0 ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

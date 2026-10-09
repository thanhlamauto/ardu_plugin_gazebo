#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "uav_navigation_core/metric_depth.hpp"
#include "uav_navigation_ros/depth_anything_onnx.hpp"
#ifdef UAV_DEPTH_TENSORRT
#include "uav_navigation_ros/depth_anything_tensorrt.hpp"
#endif

using namespace std::chrono_literals;

namespace uav_navigation_ros {
namespace {

double ProcessCpuSeconds() {
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0)
    throw std::runtime_error("getrusage failed");
  const auto seconds = [](const timeval &value) {
    return static_cast<double>(value.tv_sec) +
           static_cast<double>(value.tv_usec) * 1e-6;
  };
  return seconds(usage.ru_utime) + seconds(usage.ru_stime);
}

diagnostic_msgs::msg::KeyValue KeyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue field;
  field.key = std::move(key);
  field.value = std::move(value);
  return field;
}

} // namespace

class MonocularDepthNode final : public rclcpp::Node {
public:
  MonocularDepthNode() : Node("monocular_depth_cpp") {
    const auto model_path = declare_parameter<std::string>("model_path", "");
    const auto backend = declare_parameter<std::string>("backend", "cpu");
    const auto image_topic = declare_parameter<std::string>(
        "image_topic", "/sensor_suite/rgb");
    const auto cloud_topic = declare_parameter<std::string>(
        "cloud_topic", "/perception/obstacles_camera");
    const auto diagnostics_topic = declare_parameter<std::string>(
        "diagnostics_topic", "/perception/monocular_depth/diagnostics");
    max_image_age_s_ = declare_parameter<double>("max_image_age_s", 0.25);
    max_inference_ms_ = declare_parameter<double>("max_inference_ms", 100.0);
    max_processing_ms_ = declare_parameter<double>("max_processing_ms", 100.0);
    const auto opencv_threads = declare_parameter<int>("opencv_threads", 1);
    point_stride_ = declare_parameter<int>("point_stride", 8);
    if (model_path.empty() || !std::filesystem::is_regular_file(model_path))
      throw std::invalid_argument("model_path must name an ONNX model or TensorRT engine");
    if (max_image_age_s_ <= 0 || max_inference_ms_ <= 0 ||
        max_processing_ms_ <= 0 ||
        opencv_threads < 1 || point_stride_ < 1)
      throw std::invalid_argument("invalid depth runtime limits");
    cv::setNumThreads(opencv_threads);
    if (backend == "tensorrt") {
#ifdef UAV_DEPTH_TENSORRT
      trt_predictor_ = std::make_unique<DepthAnythingTensorRt>(model_path);
#else
      throw std::runtime_error("TensorRT backend was not built into this image");
#endif
    } else {
      predictor_ = std::make_unique<DepthAnythingOnnx>(model_path, backend);
    }
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS());
    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic, rclcpp::QoS(10));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
          OnImage(*image);
        });
    cpu_start_ = ProcessCpuSeconds();
    cpu_wall_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
                "C++ Depth Anything ready: %s, input %s, deadline %.1f ms",
                backend.c_str(), image_topic.c_str(), max_inference_ms_);
  }

private:
  void PublishDiagnostic(std::uint8_t level, const std::string &message,
                         double inference_ms, double image_age_ms,
                         std::size_t points, double processing_ms = 0.0) {
    const auto wall_now = std::chrono::steady_clock::now();
    const double wall_s =
        std::chrono::duration<double>(wall_now - cpu_wall_start_).count();
    if (wall_s >= 1.0) {
      const double cpu_now = ProcessCpuSeconds();
      cpu_percent_one_core_ = 100.0 * (cpu_now - cpu_start_) / wall_s;
      cpu_start_ = cpu_now;
      cpu_wall_start_ = wall_now;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "monocular_depth_cpp";
    status.hardware_id = "camera_rgb";
    status.message = message;
    status.values.push_back(KeyValue("inference_ms", std::to_string(inference_ms)));
    status.values.push_back(KeyValue("processing_ms", std::to_string(processing_ms)));
    status.values.push_back(KeyValue("image_age_ms", std::to_string(image_age_ms)));
    status.values.push_back(KeyValue("points", std::to_string(points)));
    status.values.push_back(KeyValue("cpu_percent_one_core",
                                     std::to_string(cpu_percent_one_core_)));
    status.values.push_back(KeyValue("cpu_logical_cores",
                                     std::to_string(std::thread::hardware_concurrency())));
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  void OnImage(const sensor_msgs::msg::Image &image) {
    const auto start = std::chrono::steady_clock::now();
    const bool bgr = image.encoding == "bgr8";
    if ((!bgr && image.encoding != "rgb8") || image.width != 640 ||
        image.height != 360 || image.header.frame_id.empty() ||
        image.step < image.width * 3 ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        "invalid RGB image", 0.0, 0.0, 0);
      return;
    }
    const auto stamp = rclcpp::Time(image.header.stamp);
    const double initial_age_s = (now() - stamp).seconds();
    if (initial_age_s < -0.05 || initial_age_s > max_image_age_s_) {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                        "image timestamp stale or ahead of ROS clock", 0.0,
                        initial_age_s * 1000, 0);
      return;
    }
    try {
      const auto depth =
#ifdef UAV_DEPTH_TENSORRT
          trt_predictor_ ? trt_predictor_->Predict(image.data.data(), image.width,
                                                   image.height, image.step, bgr) :
#endif
                           predictor_->Predict(image.data.data(), image.width,
                                               image.height, image.step, bgr);
      const double inference_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start)
              .count();
      const double age_ms = (now() - stamp).seconds() * 1000.0;
      if (inference_ms > max_inference_ms_ || age_ms > max_image_age_s_ * 1000) {
        PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "depth missed freshness/deadline gate", inference_ms,
                          age_ms, 0, inference_ms);
        return;
      }
      const auto points = uav_navigation_core::MetricDepthToPoints(
          depth.data(), image.width, image.height, point_stride_);
      if (points.empty()) {
        PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "no valid occupied depth points", inference_ms,
                          age_ms, 0, inference_ms);
        return;
      }
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = image.header;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
      for (const auto &point : points) {
        *x = static_cast<float>(point.x);
        *y = static_cast<float>(point.y);
        *z = static_cast<float>(point.z);
        ++x;
        ++y;
        ++z;
      }
      const double processing_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start)
              .count();
      const double publish_age_ms = (now() - stamp).seconds() * 1000.0;
      if (processing_ms > max_processing_ms_ || publish_age_ms < -50.0 ||
          publish_age_ms > max_image_age_s_ * 1000.0) {
        PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "cloud missed freshness/processing gate", inference_ms,
                          publish_age_ms, 0, processing_ms);
        return;
      }
      cloud_publisher_->publish(cloud);
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                        "occupied depth cloud published", inference_ms,
                        publish_age_ms, points.size(), processing_ms);
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "monocular depth failed: %s", error.what());
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        error.what(), 0.0, 0.0, 0);
    }
  }

  double max_image_age_s_{0.25};
  double max_inference_ms_{100.0};
  double max_processing_ms_{100.0};
  int point_stride_{8};
  double cpu_percent_one_core_{0.0};
  double cpu_start_{0.0};
  std::chrono::steady_clock::time_point cpu_wall_start_{};
  std::unique_ptr<DepthAnythingOnnx> predictor_;
#ifdef UAV_DEPTH_TENSORRT
  std::unique_ptr<DepthAnythingTensorRt> trt_predictor_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
};

} // namespace uav_navigation_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uav_navigation_ros::MonocularDepthNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("monocular_depth_cpp"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

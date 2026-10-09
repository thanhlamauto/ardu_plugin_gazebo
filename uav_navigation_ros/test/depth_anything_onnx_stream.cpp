#include "uav_navigation_ros/depth_anything_onnx.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>
#include <vector>

#include "opencv2/core.hpp"

namespace {
double CpuSeconds() {
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0)
    throw std::runtime_error("getrusage failed");
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec * 1e-6;
}
} // namespace

// Demo protocol: read packed 640x360 RGB frames from stdin until EOF. For each
// frame, write two native-endian doubles (wall_ms, CPU % of one core), followed
// by 640x360 float32 metric depth values. The model stays loaded across frames.
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: depth_anything_onnx_stream model.onnx\n";
    return 2;
  }
  try {
    using Model = uav_navigation_ros::DepthAnythingOnnx;
    constexpr std::size_t kPixels =
        static_cast<std::size_t>(Model::kCameraWidth) * Model::kCameraHeight;
    std::vector<std::uint8_t> rgb(kPixels * 3);
    cv::setNumThreads(1);
    Model model(argv[1], "cpu");
    while (true) {
      std::cin.read(reinterpret_cast<char *>(rgb.data()),
                    static_cast<std::streamsize>(rgb.size()));
      if (std::cin.gcount() == 0 && std::cin.eof())
        break;
      if (static_cast<std::size_t>(std::cin.gcount()) != rgb.size())
        throw std::runtime_error("incomplete RGB frame on stdin");
      const auto wall_start = std::chrono::steady_clock::now();
      const double cpu_start = CpuSeconds();
      const auto depth = model.Predict(rgb.data(), Model::kCameraWidth,
                                       Model::kCameraHeight,
                                       Model::kCameraWidth * 3, false);
      if (depth.size() != kPixels)
        throw std::runtime_error("unexpected model output dimensions");
      const double wall_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wall_start).count();
      const double metrics[2] = {wall_s * 1000.0,
                                 100.0 * (CpuSeconds() - cpu_start) / wall_s};
      std::cout.write(reinterpret_cast<const char *>(metrics), sizeof(metrics));
      std::cout.write(reinterpret_cast<const char *>(depth.data()),
                      static_cast<std::streamsize>(depth.size() * sizeof(float)));
      std::cout.flush();
      if (!std::cout)
        throw std::runtime_error("failed to write depth frame");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}

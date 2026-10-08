#include "uav_navigation_ros/depth_anything_onnx.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
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

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: depth_anything_onnx_probe model.onnx input.rgb "
                 "output.float32 [timed_repeats]\n";
    return 2;
  }
  try {
    std::ifstream input(argv[2], std::ios::binary);
    if (!input)
      throw std::runtime_error("cannot open RGB input");
    const std::vector<std::uint8_t> rgb{std::istreambuf_iterator<char>(input),
                                        std::istreambuf_iterator<char>()};
    using Model = uav_navigation_ros::DepthAnythingOnnx;
    if (rgb.size() != Model::kCameraWidth * Model::kCameraHeight * 3)
      throw std::runtime_error("expected packed 640x360 RGB input");
    cv::setNumThreads(1);
    Model model(argv[1], "cpu");
    auto predict = [&] {
      return model.Predict(rgb.data(), Model::kCameraWidth,
                           Model::kCameraHeight, Model::kCameraWidth * 3,
                           false);
    };
    auto depth = predict(); // warmup, excluded from timing
    const int repeats = argc == 5 ? std::stoi(argv[4]) : 1;
    if (repeats < 1 || repeats > 1000)
      throw std::invalid_argument("timed_repeats must be 1..1000");
    std::vector<double> timings;
    timings.reserve(static_cast<std::size_t>(repeats));
    const double cpu_start = CpuSeconds();
    const auto wall_start = std::chrono::steady_clock::now();
    for (int i = 0; i < repeats; ++i) {
      const auto frame_start = std::chrono::steady_clock::now();
      depth = predict();
      timings.push_back(std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - frame_start)
                            .count());
    }
    const double wall_s = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - wall_start)
                              .count();
    const double cpu_percent = 100.0 * (CpuSeconds() - cpu_start) / wall_s;
    std::sort(timings.begin(), timings.end());
    std::ofstream output(argv[3], std::ios::binary);
    if (!output)
      throw std::runtime_error("cannot open depth output");
    output.write(reinterpret_cast<const char *>(depth.data()),
                 static_cast<std::streamsize>(depth.size() * sizeof(float)));
    std::cout << "depth " << Model::kCameraWidth << 'x' << Model::kCameraHeight
              << ", frames " << repeats << ", p50_ms "
              << timings[static_cast<std::size_t>(repeats / 2)] << ", p95_ms "
              << timings[static_cast<std::size_t>(std::ceil(.95 * repeats) - 1)]
              << ", fps " << repeats / wall_s << ", cpu_one_core_percent "
              << cpu_percent << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}

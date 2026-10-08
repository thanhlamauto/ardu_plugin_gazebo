#include "uav_navigation_core/metric_depth.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uav_navigation_core {
namespace {

constexpr std::int64_t kCoefficientScale = 1LL << 22;
struct Coefficients {
  int first{0};
  std::vector<std::int32_t> weights;
};

double Cubic(double value) {
  const double x = std::abs(value);
  if (x < 1.0)
    return ((1.5 * x - 2.5) * x * x) + 1.0;
  if (x < 2.0)
    return ((-0.5 * x + 2.5) * x - 4.0) * x + 2.0;
  return 0.0;
}

std::vector<Coefficients> MakeCoefficients(int input_size, int output_size) {
  const double scale = static_cast<double>(input_size) / output_size;
  const double filter_scale = std::max(1.0, scale);
  const double support = 2.0 * filter_scale;
  std::vector<Coefficients> all(static_cast<std::size_t>(output_size));
  for (int target = 0; target < output_size; ++target) {
    const double center = (target + 0.5) * scale;
    const int first = std::max(0, static_cast<int>(center - support + 0.5));
    const int end = std::min(input_size,
                             static_cast<int>(center + support + 0.5));
    if (first >= end)
      throw std::runtime_error("empty bicubic resize support");
    auto &coeff = all[static_cast<std::size_t>(target)];
    coeff.first = first;
    std::vector<double> raw;
    raw.reserve(static_cast<std::size_t>(end - first));
    double sum = 0.0;
    for (int source = first; source < end; ++source) {
      const double weight = Cubic((source - center + 0.5) / filter_scale);
      raw.push_back(weight);
      sum += weight;
    }
    if (sum == 0.0)
      throw std::runtime_error("zero bicubic resize weight");
    for (const double weight : raw)
      coeff.weights.push_back(static_cast<std::int32_t>(
          std::round(weight / sum * kCoefficientScale)));
  }
  return all;
}

std::uint8_t ClipRounded(std::int64_t sum) {
  return static_cast<std::uint8_t>(
      std::clamp<std::int64_t>(sum / kCoefficientScale, 0, 255));
}

} // namespace

std::vector<float> PrepareDepthAnythingInput(const std::uint8_t *data,
                                             int width, int height,
                                             std::size_t row_step,
                                             bool bgr_input, int output_width,
                                             int output_height) {
  if (data == nullptr || width <= 0 || height <= 0 || output_width <= 0 ||
      output_height <= 0 || width > 8192 || height > 8192 ||
      output_width > 2048 || output_height > 2048 ||
      static_cast<std::size_t>(width) >
          std::numeric_limits<std::size_t>::max() / 3 ||
      row_step < static_cast<std::size_t>(width) * 3 ||
      static_cast<std::size_t>(height) >
          std::numeric_limits<std::size_t>::max() / row_step)
    throw std::invalid_argument("invalid RGB image shape or row stride");

  const auto horizontal = MakeCoefficients(width, output_width);
  const auto vertical = MakeCoefficients(height, output_height);
  const std::size_t middle_size =
      static_cast<std::size_t>(height) * output_width * 3;
  const std::size_t result_size =
      static_cast<std::size_t>(output_height) * output_width * 3;
  if (middle_size / 3 / static_cast<std::size_t>(output_width) !=
          static_cast<std::size_t>(height) ||
      result_size / 3 / static_cast<std::size_t>(output_width) !=
          static_cast<std::size_t>(output_height))
    throw std::invalid_argument("RGB resize allocation overflow");
  std::vector<std::uint8_t> middle(middle_size);
  for (int y = 0; y < height; ++y) {
    const auto *row = data + static_cast<std::size_t>(y) * row_step;
    for (int x = 0; x < output_width; ++x) {
      const auto &coeff = horizontal[static_cast<std::size_t>(x)];
      for (int channel = 0; channel < 3; ++channel) {
        std::int64_t sum = kCoefficientScale / 2;
        const int source_channel = bgr_input ? 2 - channel : channel;
        for (std::size_t tap = 0; tap < coeff.weights.size(); ++tap)
          sum += static_cast<std::int64_t>(
                     row[(static_cast<std::size_t>(coeff.first) + tap) * 3 +
                         source_channel]) *
                 coeff.weights[tap];
        middle[(static_cast<std::size_t>(y) * output_width + x) * 3 +
               channel] = ClipRounded(sum);
      }
    }
  }

  const std::size_t plane_size =
      static_cast<std::size_t>(output_width) * output_height;
  std::vector<float> tensor(plane_size * 3);
  constexpr std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  constexpr std::array<float, 3> stddev{0.229F, 0.224F, 0.225F};
  for (int y = 0; y < output_height; ++y) {
    const auto &coeff = vertical[static_cast<std::size_t>(y)];
    for (int x = 0; x < output_width; ++x) {
      for (int channel = 0; channel < 3; ++channel) {
        std::int64_t sum = kCoefficientScale / 2;
        for (std::size_t tap = 0; tap < coeff.weights.size(); ++tap)
          sum += static_cast<std::int64_t>(
                     middle[((static_cast<std::size_t>(coeff.first) + tap) *
                                  output_width +
                              x) *
                                 3 +
                             channel]) *
                 coeff.weights[tap];
        const auto pixel = ClipRounded(sum);
        tensor[static_cast<std::size_t>(channel) * plane_size +
               static_cast<std::size_t>(y) * output_width + x] =
            (static_cast<float>(pixel) / 255.0F - mean[channel]) /
            stddev[channel];
      }
    }
  }
  return tensor;
}

std::vector<Vec3> MetricDepthToPoints(const float *depth, int width, int height,
                                      int stride, double horizontal_fov_rad,
                                      double forward_offset_m,
                                      double min_depth_m, double max_depth_m) {
  if (depth == nullptr || width <= 0 || height <= 0 || stride <= 0 ||
      !std::isfinite(horizontal_fov_rad) || horizontal_fov_rad <= 0.0 ||
      horizontal_fov_rad >= 3.14159265358979323846 ||
      !std::isfinite(forward_offset_m) || !std::isfinite(min_depth_m) ||
      !std::isfinite(max_depth_m) || min_depth_m <= 0.0 ||
      max_depth_m <= min_depth_m)
    throw std::invalid_argument("invalid depth geometry configuration");
  const double focal = width / (2.0 * std::tan(horizontal_fov_rad / 2.0));
  std::vector<Vec3> points;
  points.reserve(static_cast<std::size_t>((height + stride - 1) / stride) *
                 ((width + stride - 1) / stride));
  for (int v = 0; v < height; v += stride)
    for (int u = 0; u < width; u += stride) {
      const double z = depth[static_cast<std::size_t>(v) * width + u];
      if (!std::isfinite(z) || z < min_depth_m || z > max_depth_m)
        continue;
      points.push_back({z + forward_offset_m,
                        -((u - (width - 1) / 2.0) * z / focal),
                        -((v - (height - 1) / 2.0) * z / focal)});
    }
  return points;
}

} // namespace uav_navigation_core

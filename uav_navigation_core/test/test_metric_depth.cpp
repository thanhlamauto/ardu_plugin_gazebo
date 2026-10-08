#include "uav_navigation_core/metric_depth.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace core = uav_navigation_core;
namespace {
int failures = 0;

void Check(bool ok, const char *label) {
  if (!ok) {
    std::cerr << "FAIL: " << label << '\n';
    ++failures;
  }
}

template <typename Function> void CheckInvalid(Function function) {
  try {
    function();
    Check(false, "expected invalid_argument");
  } catch (const std::invalid_argument &) {
  }
}

void CameraInput() {
  // Row stride includes two padding bytes; the padding cannot affect resize.
  const std::uint8_t rgb[]{255, 0, 0, 0, 255, 0, 99, 99,
                           255, 0, 0, 0, 255, 0, 99, 99};
  const auto input = core::PrepareDepthAnythingInput(rgb, 2, 2, 8, false, 4, 4);
  Check(input.size() == 48, "NCHW input shape");
  Check(std::isfinite(input[0]), "normalized pixel finite");
  // BGR input with reversed source channels must produce the same tensor.
  const std::uint8_t bgr[]{0, 0, 255, 0, 255, 0, 99, 99,
                           0, 0, 255, 0, 255, 0, 99, 99};
  const auto swapped =
      core::PrepareDepthAnythingInput(bgr, 2, 2, 8, true, 4, 4);
  for (std::size_t index = 0; index < input.size(); ++index)
    Check(input[index] == swapped[index], "BGR conversion parity");
  CheckInvalid([&] {
    core::PrepareDepthAnythingInput(rgb, 2, 2, 5, false, 4, 4);
  });
}

void DepthGeometry() {
  float depth[16];
  for (auto &value : depth)
    value = 2.0F;
  depth[0] = std::numeric_limits<float>::quiet_NaN();
  depth[1] = 0.1F;
  const auto points = core::MetricDepthToPoints(depth, 4, 4, 2);
  Check(points.size() == 3, "invalid depth samples omitted");
  for (const auto &point : points)
    Check(std::abs(point.x - 2.09) < 1e-9, "camera offset in FLU x");
  Check(points[0].y < 0.0, "image right maps to FLU right negative y");
  Check(points[1].z < 0.0, "image down maps to FLU down negative z");
  CheckInvalid([&] { core::MetricDepthToPoints(depth, 4, 4, 0); });
}
} // namespace

int main() {
  CameraInput();
  DepthGeometry();
  return failures == 0 ? 0 : 1;
}

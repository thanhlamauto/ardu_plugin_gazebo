#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "opencv2/dnn.hpp"

namespace uav_navigation_ros {

// Fixed-shape export of Depth Anything V2 Metric Outdoor Small. The graph is
// prepared offline; no Python or PyTorch is loaded by this C++ runtime.
class DepthAnythingOnnx {
public:
  DepthAnythingOnnx(const std::string &model_path,
                    const std::string &backend);

  std::vector<float> Predict(const std::uint8_t *image, int width, int height,
                             std::size_t row_step, bool bgr_input);

  static constexpr int kCameraWidth = 640;
  static constexpr int kCameraHeight = 360;
  static constexpr int kInputWidth = 518;
  static constexpr int kInputHeight = 294;

private:
  cv::dnn::Net network_;
};

} // namespace uav_navigation_ros

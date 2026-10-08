#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

// Reproduce the saved Depth Anything DPTImageProcessor: RGB uint8, Pillow-style
// antialiased bicubic resize, [0,1] rescale, ImageNet normalization, NCHW.
// Input rows may be padded. BGR is accepted for camera drivers that use bgr8.
std::vector<float> PrepareDepthAnythingInput(const std::uint8_t *data,
                                             int width, int height,
                                             std::size_t row_step,
                                             bool bgr_input,
                                             int output_width = 518,
                                             int output_height = 294);

// Convert dense axial depth to sparse occupied points in sensor_suite_link
// (forward/left/up). Unknown pixels are not represented as free space.
std::vector<Vec3> MetricDepthToPoints(const float *depth, int width, int height,
                                      int stride = 8,
                                      double horizontal_fov_rad = 1.3962634,
                                      double forward_offset_m = 0.09,
                                      double min_depth_m = 0.2,
                                      double max_depth_m = 25.0);

} // namespace uav_navigation_core

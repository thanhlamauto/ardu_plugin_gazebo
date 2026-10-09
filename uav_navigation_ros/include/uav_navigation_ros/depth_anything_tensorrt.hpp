#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

namespace uav_navigation_ros {

class DepthAnythingTensorRt {
public:
  explicit DepthAnythingTensorRt(const std::string &engine_path);
  ~DepthAnythingTensorRt();
  DepthAnythingTensorRt(const DepthAnythingTensorRt &) = delete;
  DepthAnythingTensorRt &operator=(const DepthAnythingTensorRt &) = delete;

  std::vector<float> Predict(const std::uint8_t *image, int width, int height,
                             std::size_t row_step, bool bgr_input);

private:
  struct Logger final : nvinfer1::ILogger {
    void log(Severity severity, const char *message) noexcept override;
  } logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string input_name_;
  std::string output_name_;
  void *input_buffer_{nullptr};
  void *output_buffer_{nullptr};
  cudaStream_t stream_{nullptr};
  std::vector<float> small_;
};

} // namespace uav_navigation_ros

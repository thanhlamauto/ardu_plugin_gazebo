#include "uav_navigation_ros/depth_anything_onnx.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "uav_navigation_core/metric_depth.hpp"

namespace uav_navigation_ros {

DepthAnythingOnnx::DepthAnythingOnnx(const std::string &model_path,
                                     const std::string &backend)
    : network_(cv::dnn::readNetFromONNX(model_path)) {
  if (network_.empty())
    throw std::runtime_error("Depth Anything ONNX model is empty");
  if (backend == "cpu") {
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  } else if (backend == "cuda") {
    const auto targets = cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA);
    if (std::find(targets.begin(), targets.end(), cv::dnn::DNN_TARGET_CUDA) ==
        targets.end())
      throw std::runtime_error("OpenCV DNN CUDA target is unavailable");
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
  } else if (backend == "cuda_fp16") {
    const auto targets = cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA);
    if (std::find(targets.begin(), targets.end(),
                  cv::dnn::DNN_TARGET_CUDA_FP16) == targets.end())
      throw std::runtime_error("OpenCV DNN CUDA FP16 target is unavailable");
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
  } else {
    throw std::invalid_argument("depth backend must be cpu, cuda or cuda_fp16");
  }
}

std::vector<float> DepthAnythingOnnx::Predict(const std::uint8_t *image,
                                              int width, int height,
                                              std::size_t row_step,
                                              bool bgr_input) {
  if (width != kCameraWidth || height != kCameraHeight)
    throw std::invalid_argument("ONNX model requires 640x360 RGB images");
  const auto tensor = uav_navigation_core::PrepareDepthAnythingInput(
      image, width, height, row_step, bgr_input, kInputWidth, kInputHeight);
  const int shape[]{1, 3, kInputHeight, kInputWidth};
  cv::Mat blob(4, shape, CV_32F);
  std::copy(tensor.begin(), tensor.end(), blob.ptr<float>());
  network_.setInput(blob);
  const cv::Mat predicted = network_.forward();
  if (predicted.type() != CV_32F ||
      predicted.total() != static_cast<std::size_t>(kInputWidth * kInputHeight) ||
      !predicted.isContinuous())
    throw std::runtime_error("unexpected Depth Anything ONNX output");
  cv::Mat small(kInputHeight, kInputWidth, CV_32F,
                const_cast<float *>(predicted.ptr<float>()));
  cv::Mat full;
  // Matches torch.nn.functional.interpolate(..., mode='bicubic',
  // align_corners=False) in the existing Python predictor.
  cv::resize(small, full, cv::Size(width, height), 0.0, 0.0,
             cv::INTER_CUBIC);
  return {full.ptr<float>(), full.ptr<float>() + full.total()};
}

} // namespace uav_navigation_ros

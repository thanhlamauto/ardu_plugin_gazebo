#include "uav_navigation_ros/depth_anything_tensorrt.hpp"

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "uav_navigation_core/metric_depth.hpp"

namespace uav_navigation_ros {
namespace {
constexpr int kWidth = 640, kHeight = 360;
constexpr int kInputWidth = 518, kInputHeight = 294;

void CheckCuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
}

std::size_t Elements(const nvinfer1::Dims &dims) {
  std::size_t elements = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("TensorRT engine requires fixed I/O shapes");
    elements *= static_cast<std::size_t>(dims.d[i]);
  }
  return elements;
}
} // namespace

void DepthAnythingTensorRt::Logger::log(Severity severity,
                                        const char *message) noexcept {
  if (severity <= Severity::kWARNING) std::cerr << "TensorRT: " << message << '\n';
}

DepthAnythingTensorRt::DepthAnythingTensorRt(const std::string &engine_path)
    : small_(kInputWidth * kInputHeight) {
  std::ifstream input(engine_path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open TensorRT engine: " + engine_path);
  const std::vector<char> plan{std::istreambuf_iterator<char>(input),
                               std::istreambuf_iterator<char>()};
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) throw std::runtime_error("createInferRuntime failed");
  engine_.reset(runtime_->deserializeCudaEngine(plan.data(), plan.size()));
  if (!engine_) throw std::runtime_error("deserializeCudaEngine failed");
  context_.reset(engine_->createExecutionContext());
  if (!context_) throw std::runtime_error("createExecutionContext failed");
  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char *name = engine_->getIOTensorName(i);
    if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT)
      throw std::runtime_error("TensorRT engine must have FP32 I/O");
    switch (engine_->getTensorIOMode(name)) {
    case nvinfer1::TensorIOMode::kINPUT:
      if (!input_name_.empty()) throw std::runtime_error("expected one engine input");
      input_name_ = name;
      break;
    case nvinfer1::TensorIOMode::kOUTPUT:
      if (!output_name_.empty()) throw std::runtime_error("expected one engine output");
      output_name_ = name;
      break;
    default: throw std::runtime_error("invalid TensorRT tensor mode");
    }
  }
  if (input_name_.empty() || output_name_.empty() ||
      Elements(context_->getTensorShape(input_name_.c_str())) !=
          3 * kInputWidth * kInputHeight ||
      Elements(context_->getTensorShape(output_name_.c_str())) !=
          kInputWidth * kInputHeight)
    throw std::runtime_error("unexpected Depth Anything TensorRT engine shape");
  CheckCuda(cudaMalloc(&input_buffer_, 3 * kInputWidth * kInputHeight * sizeof(float)), "cudaMalloc input");
  try {
    CheckCuda(cudaMalloc(&output_buffer_, small_.size() * sizeof(float)), "cudaMalloc output");
    CheckCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
    if (!context_->setTensorAddress(input_name_.c_str(), input_buffer_) ||
        !context_->setTensorAddress(output_name_.c_str(), output_buffer_))
      throw std::runtime_error("setTensorAddress failed");
  } catch (...) {
    if (stream_) cudaStreamDestroy(stream_);
    if (output_buffer_) cudaFree(output_buffer_);
    cudaFree(input_buffer_);
    input_buffer_ = nullptr;
    output_buffer_ = nullptr;
    stream_ = nullptr;
    throw;
  }
}

DepthAnythingTensorRt::~DepthAnythingTensorRt() {
  if (stream_) cudaStreamDestroy(stream_);
  if (output_buffer_) cudaFree(output_buffer_);
  if (input_buffer_) cudaFree(input_buffer_);
}

std::vector<float> DepthAnythingTensorRt::Predict(const std::uint8_t *image,
                                                    int width, int height,
                                                    std::size_t row_step,
                                                    bool bgr_input) {
  if (width != kWidth || height != kHeight)
    throw std::invalid_argument("TensorRT depth requires 640x360 RGB images");
  const auto tensor = uav_navigation_core::PrepareDepthAnythingInput(
      image, width, height, row_step, bgr_input, kInputWidth, kInputHeight);
  CheckCuda(cudaMemcpyAsync(input_buffer_, tensor.data(), tensor.size() * sizeof(float),
                            cudaMemcpyHostToDevice, stream_), "H2D");
  if (!context_->enqueueV3(stream_)) throw std::runtime_error("enqueueV3 failed");
  CheckCuda(cudaMemcpyAsync(small_.data(), output_buffer_, small_.size() * sizeof(float),
                            cudaMemcpyDeviceToHost, stream_), "D2H");
  CheckCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
  cv::Mat source(kInputHeight, kInputWidth, CV_32F, small_.data());
  cv::Mat full;
  cv::resize(source, full, cv::Size(width, height), 0.0, 0.0, cv::INTER_CUBIC);
  return {full.ptr<float>(), full.ptr<float>() + full.total()};
}

} // namespace uav_navigation_ros

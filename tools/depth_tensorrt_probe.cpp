// Standalone C++ RGB-to-depth probe for a fixed-shape TensorRT engine.
// Build/run in the JetPack container with scripts/probe_cpp_depth_tensorrt_orin.sh.
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <vector>

#include "uav_navigation_core/metric_depth.hpp"

namespace {
constexpr int kWidth = 640, kHeight = 360;
constexpr int kInputWidth = 518, kInputHeight = 294;

float Cubic(float x) {
  x = std::abs(x);
  if (x <= 1.0F) return (1.25F * x - 2.25F) * x * x + 1.0F;
  if (x < 2.0F) return ((-0.75F * x + 3.75F) * x - 6.0F) * x + 3.0F;
  return 0.0F;
}

struct AxisTap {
  std::array<int, 4> index{};
  std::array<float, 4> weight{};
};

std::vector<AxisTap> MakeAxis(int source_size, int target_size) {
  std::vector<AxisTap> axis(target_size);
  for (int target = 0; target < target_size; ++target) {
    const float source = (target + 0.5F) * source_size / target_size - 0.5F;
    const int base = static_cast<int>(std::floor(source));
    for (int tap = 0; tap < 4; ++tap) {
      const int pixel = base + tap - 1;
      axis[target].index[tap] = std::clamp(pixel, 0, source_size - 1);
      axis[target].weight[tap] = Cubic(source - pixel);
    }
  }
  return axis;
}

void ResizeDepth(const float *small, float *full) {
  static const auto horizontal = MakeAxis(kInputWidth, kWidth);
  static const auto vertical = MakeAxis(kInputHeight, kHeight);
  std::vector<float> middle(kInputHeight * kWidth);
  for (int y = 0; y < kInputHeight; ++y)
    for (int x = 0; x < kWidth; ++x) {
      float value = 0.0F;
      for (int tap = 0; tap < 4; ++tap)
        value += small[y * kInputWidth + horizontal[x].index[tap]] *
                 horizontal[x].weight[tap];
      middle[y * kWidth + x] = value;
    }
  for (int y = 0; y < kHeight; ++y)
    for (int x = 0; x < kWidth; ++x) {
      float value = 0.0F;
      for (int tap = 0; tap < 4; ++tap)
        value += middle[vertical[y].index[tap] * kWidth + x] *
                 vertical[y].weight[tap];
      full[y * kWidth + x] = value;
    }
}

struct Logger final : nvinfer1::ILogger {
  void log(Severity severity, const char *message) noexcept override {
    if (severity <= Severity::kWARNING) std::cerr << "TensorRT: " << message << '\n';
  }
};

void CheckCuda(cudaError_t result, const char *what) {
  if (result != cudaSuccess)
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(result));
}

struct DeviceBuffer {
  void *ptr = nullptr;
  explicit DeviceBuffer(std::size_t bytes) { CheckCuda(cudaMalloc(&ptr, bytes), "cudaMalloc"); }
  ~DeviceBuffer() { if (ptr) cudaFree(ptr); }
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer &operator=(const DeviceBuffer &) = delete;
};

struct Stream {
  cudaStream_t stream = nullptr;
  Stream() { CheckCuda(cudaStreamCreate(&stream), "cudaStreamCreate"); }
  ~Stream() { if (stream) cudaStreamDestroy(stream); }
};

std::vector<char> ReadAll(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open " + path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::size_t Elements(const nvinfer1::Dims &dims) {
  std::size_t n = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("dynamic/invalid tensor shape");
    n *= static_cast<std::size_t>(dims.d[i]);
  }
  return n;
}

double CpuSeconds() {
  rusage r{};
  if (getrusage(RUSAGE_SELF, &r)) throw std::runtime_error("getrusage failed");
  return r.ru_utime.tv_sec + r.ru_utime.tv_usec * 1e-6 +
         r.ru_stime.tv_sec + r.ru_stime.tv_usec * 1e-6;
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: depth_tensorrt_probe engine.plan input.rgb output.f32 [repeats|--sequence]\n";
    return 2;
  }
  try {
    const bool sequence = argc == 5 && std::string(argv[4]) == "--sequence";
    const auto rgb = ReadAll(argv[2]);
    constexpr std::size_t kFrameBytes = kWidth * kHeight * 3;
    if (rgb.empty() || rgb.size() % kFrameBytes != 0 ||
        (!sequence && rgb.size() != kFrameBytes))
      throw std::runtime_error("expected packed RGB 640x360 input");
    const int repeats = sequence ? static_cast<int>(rgb.size() / kFrameBytes)
                                 : argc == 5 ? std::stoi(argv[4]) : 20;
    if (repeats < 1 || repeats > 1000) throw std::invalid_argument("frames/repeats must be 1..1000");
    const auto plan = ReadAll(argv[1]);
    Logger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));
    if (!runtime) throw std::runtime_error("createInferRuntime failed");
    std::unique_ptr<nvinfer1::ICudaEngine> engine(runtime->deserializeCudaEngine(plan.data(), plan.size()));
    if (!engine) throw std::runtime_error("deserializeCudaEngine failed");
    std::unique_ptr<nvinfer1::IExecutionContext> context(engine->createExecutionContext());
    if (!context) throw std::runtime_error("createExecutionContext failed");
    const char *input_name = nullptr, *output_name = nullptr;
    for (int i = 0; i < engine->getNbIOTensors(); ++i) {
      const char *name = engine->getIOTensorName(i);
      if (engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT)
        throw std::runtime_error("expected FP32 I/O tensors");
      const auto mode = engine->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        if (input_name) throw std::runtime_error("expected one input");
        input_name = name;
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
        if (output_name) throw std::runtime_error("expected one output");
        output_name = name;
      }
    }
    if (!input_name || !output_name ||
        Elements(context->getTensorShape(input_name)) != 3 * kInputWidth * kInputHeight ||
        Elements(context->getTensorShape(output_name)) != kInputWidth * kInputHeight)
      throw std::runtime_error("unexpected engine I/O shape");
    DeviceBuffer gpu_input(3 * kInputWidth * kInputHeight * sizeof(float));
    DeviceBuffer gpu_output(kInputWidth * kInputHeight * sizeof(float));
    if (!context->setTensorAddress(input_name, gpu_input.ptr) ||
        !context->setTensorAddress(output_name, gpu_output.ptr))
      throw std::runtime_error("setTensorAddress failed");
    Stream stream;
    std::vector<float> small(kInputWidth * kInputHeight);
    std::vector<float> full(kWidth * kHeight);
    auto predict = [&](const std::uint8_t *frame) {
      const auto tensor = uav_navigation_core::PrepareDepthAnythingInput(
          frame, kWidth, kHeight,
          kWidth * 3, false, kInputWidth, kInputHeight);
      CheckCuda(cudaMemcpyAsync(gpu_input.ptr, tensor.data(), tensor.size() * sizeof(float),
                                cudaMemcpyHostToDevice, stream.stream), "H2D");
      if (!context->enqueueV3(stream.stream)) throw std::runtime_error("enqueueV3 failed");
      CheckCuda(cudaMemcpyAsync(small.data(), gpu_output.ptr, small.size() * sizeof(float),
                                cudaMemcpyDeviceToHost, stream.stream), "D2H");
      CheckCuda(cudaStreamSynchronize(stream.stream), "cudaStreamSynchronize");
      ResizeDepth(small.data(), full.data());
    };
    const auto *frames = reinterpret_cast<const std::uint8_t *>(rgb.data());
    predict(frames); // warmup, excluded
    std::vector<double> times;
    times.reserve(repeats);
    std::ofstream output(argv[3], std::ios::binary);
    if (!output) throw std::runtime_error("cannot write output");
    float depth_min = std::numeric_limits<float>::infinity();
    float depth_max = -std::numeric_limits<float>::infinity();
    const double cpu_before = CpuSeconds();
    const auto wall_before = std::chrono::steady_clock::now();
    for (int i = 0; i < repeats; ++i) {
      const auto start = std::chrono::steady_clock::now();
      predict(frames + (sequence ? static_cast<std::size_t>(i) * kFrameBytes : 0));
      times.push_back(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start).count());
      if (sequence) {
        output.write(reinterpret_cast<const char *>(full.data()), full.size() * sizeof(float));
        if (!output) throw std::runtime_error("cannot write sequence depth");
      }
      const auto [low, high] = std::minmax_element(full.begin(), full.end());
      depth_min = std::min(depth_min, *low);
      depth_max = std::max(depth_max, *high);
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_before).count();
    std::sort(times.begin(), times.end());
    if (!sequence) output.write(reinterpret_cast<const char *>(full.data()), full.size() * sizeof(float));
    if (!output) throw std::runtime_error("cannot write depth");
    std::cout << "RGB 640x360 -> metric depth 640x360, frames " << repeats
              << ", mode " << (sequence ? "sequence" : "repeat")
              << ", p50_ms " << times[repeats / 2]
              << ", p95_ms " << times[static_cast<std::size_t>(std::ceil(.95 * repeats) - 1)]
              << ", fps " << repeats / elapsed
              << ", cpu_one_core_percent " << 100.0 * (CpuSeconds() - cpu_before) / elapsed
              << ", depth_min_m " << depth_min << ", depth_max_m " << depth_max << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

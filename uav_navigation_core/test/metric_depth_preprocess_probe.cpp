#include "uav_navigation_core/metric_depth.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "usage: metric_depth_preprocess_probe input.rgb width height "
                 "output_width output.float32\n";
    return 2;
  }
  try {
    const int width = std::stoi(argv[2]);
    const int height = std::stoi(argv[3]);
    const int output_width = std::stoi(argv[4]);
    const int output_height = static_cast<int>(
        std::lround(static_cast<double>(height) * output_width / width / 14)) *
                              14;
    std::ifstream input(argv[1], std::ios::binary);
    if (!input)
      throw std::runtime_error("cannot open RGB input");
    const std::vector<std::uint8_t> rgb{std::istreambuf_iterator<char>(input),
                                        std::istreambuf_iterator<char>()};
    if (rgb.size() != static_cast<std::size_t>(width) * height * 3)
      throw std::runtime_error("RGB file size does not match width and height");
    const auto tensor = uav_navigation_core::PrepareDepthAnythingInput(
        rgb.data(), width, height, static_cast<std::size_t>(width) * 3,
        false, output_width, output_height);
    std::ofstream output(argv[5], std::ios::binary);
    if (!output)
      throw std::runtime_error("cannot open output");
    output.write(reinterpret_cast<const char *>(tensor.data()),
                 static_cast<std::streamsize>(tensor.size() * sizeof(float)));
    std::cout << "NCHW 1x3x" << output_height << 'x' << output_width << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}

#include <cstdlib>
#include <iostream>
#include <string>

#include "uav_navigation_core/cost_grid.hpp"
#include "uav_navigation_ros/sdf_static_map_loader.hpp"

namespace core = uav_navigation_core;
namespace ros_adapter = uav_navigation_ros;

int main() {
  int failures = 0;
  const auto check = [&failures](bool condition, const std::string& message) {
    if (!condition) {
      ++failures;
      std::cerr << "FAIL: " << message << '\n';
    }
  };
  try {
    ros_adapter::StaticMapConfig config;
    config.resolution_m = 0.5;
    config.clearance_m = 1.8;
    ros_adapter::SdfStaticMapLoader loader(config);
    loader.Load(TEST_WORLD);
    check(!loader.obstacles().empty(), "yard SDF contains static collisions");
    bool has_box = false;
    for (const auto& obstacle : loader.obstacles()) {
      has_box = has_box || obstacle.kind == ros_adapter::StaticObstacle2D::Kind::kBox;
    }
    check(has_box, "box geometry is parsed");
    const auto grid = loader.BuildCostGrid(5.0, {{0.0, 0.0, 5.0}, {100.0, 0.0, 5.0}});
    check(core::IsValid(grid), "rasterized yard grid is valid");
    const auto start = core::WorldToCell(grid, {0.0, 0.0, 5.0});
    check(start.has_value() && !core::IsBlocked(grid, *start),
          "run-up start remains free at flight altitude");
    std::size_t blocked = 0;
    for (const auto value : grid.costs) blocked += value == 100 ? 1U : 0U;
    check(blocked > 0, "active obstacles produce occupied cells");
    const auto ground_grid = loader.BuildCostGrid(-0.15);
    const auto ground_origin = core::WorldToCell(ground_grid, {0.0, 0.0, -0.15});
    check(ground_origin.has_value() && core::IsBlocked(ground_grid, *ground_origin),
          "ground collision is active at ground altitude");
    ros_adapter::SdfStaticMapLoader cylinder_loader(config);
    cylinder_loader.Load(TEST_CYLINDER_WORLD);
    bool has_cylinder = false;
    for (const auto& obstacle : cylinder_loader.obstacles()) {
      has_cylinder = has_cylinder ||
                     obstacle.kind == ros_adapter::StaticObstacle2D::Kind::kCylinder;
    }
    check(has_cylinder, "cylinder geometry is parsed");
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (failures != 0) return EXIT_FAILURE;
  std::cout << "All SDF static-map checks passed\n";
  return EXIT_SUCCESS;
}

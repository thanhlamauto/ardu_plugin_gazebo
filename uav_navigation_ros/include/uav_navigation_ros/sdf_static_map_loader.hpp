#pragma once

#include <string>
#include <vector>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_ros {

struct StaticObstacle2D {
  enum class Kind { kBox, kCylinder };
  std::string name;
  Kind kind{Kind::kBox};
  double center_x_m{0.0};
  double center_y_m{0.0};
  double z_min_m{0.0};
  double z_max_m{0.0};
  double yaw_rad{0.0};
  double half_size_x_m{0.0};
  double half_size_y_m{0.0};
  double radius_m{0.0};
};

struct StaticMapConfig {
  std::string frame_id{"odom"};
  double resolution_m{0.5};
  double clearance_m{1.8};
  double vertical_clearance_m{0.3};
  double bounds_padding_m{4.0};
  std::size_t max_grid_cells{5000000};
};

class SdfStaticMapLoader {
 public:
  explicit SdfStaticMapLoader(StaticMapConfig config = {});
  void Load(const std::string& sdf_path);
  uav_navigation_core::CostGrid2D BuildCostGrid(
      double altitude_m,
      const std::vector<uav_navigation_core::Vec3>& required_points = {}) const;
  const std::vector<StaticObstacle2D>& obstacles() const { return obstacles_; }

 private:
  StaticMapConfig config_;
  std::vector<StaticObstacle2D> obstacles_;
};

}  // namespace uav_navigation_ros

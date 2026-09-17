#include "uav_navigation_ros/sdf_static_map_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include "uav_navigation_core/cost_grid.hpp"

namespace uav_navigation_ros {
namespace {

struct Pose6 {
  double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};
};

std::string Text(const tinyxml2::XMLElement* element) {
  return element != nullptr && element->GetText() != nullptr ? element->GetText() : "";
}

Pose6 ParsePose(const tinyxml2::XMLElement* element) {
  if (element == nullptr || Text(element).empty()) return {};
  Pose6 pose;
  std::istringstream input(Text(element));
  if (!(input >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
      !std::isfinite(pose.roll) || !std::isfinite(pose.pitch) || !std::isfinite(pose.yaw)) {
    throw std::runtime_error("SDF pose must contain six finite values");
  }
  double trailing = 0.0;
  if (input >> trailing) throw std::runtime_error("SDF pose contains extra values");
  return pose;
}

Pose6 ComposePlanar(const Pose6& parent, const Pose6& child) {
  const double c = std::cos(parent.yaw), s = std::sin(parent.yaw);
  return {parent.x + c * child.x - s * child.y,
          parent.y + s * child.x + c * child.y,
          parent.z + child.z, parent.roll + child.roll,
          parent.pitch + child.pitch, parent.yaw + child.yaw};
}

std::array<double, 3> ParsePositiveTriple(const tinyxml2::XMLElement* element,
                                          const std::string& context) {
  std::array<double, 3> value{};
  std::istringstream input(Text(element));
  if (!(input >> value[0] >> value[1] >> value[2]) || value[0] <= 0.0 ||
      value[1] <= 0.0 || value[2] <= 0.0 || !std::isfinite(value[0]) ||
      !std::isfinite(value[1]) || !std::isfinite(value[2])) {
    throw std::runtime_error("invalid box size at " + context);
  }
  return value;
}

double ParsePositiveScalar(const tinyxml2::XMLElement* element,
                           const std::string& context) {
  double value = std::numeric_limits<double>::quiet_NaN();
  std::istringstream input(Text(element));
  if (!(input >> value) || value <= 0.0 || !std::isfinite(value)) {
    throw std::runtime_error("invalid positive scalar at " + context);
  }
  return value;
}

bool ActiveAt(const StaticObstacle2D& obstacle, double altitude,
              double vertical_clearance) {
  return altitude >= obstacle.z_min_m - vertical_clearance &&
         altitude <= obstacle.z_max_m + vertical_clearance;
}

bool ContainsInflated(const StaticObstacle2D& obstacle, double x, double y,
                      double clearance) {
  const double dx = x - obstacle.center_x_m;
  const double dy = y - obstacle.center_y_m;
  if (obstacle.kind == StaticObstacle2D::Kind::kCylinder) {
    const double radius = obstacle.radius_m + clearance;
    return dx * dx + dy * dy <= radius * radius;
  }
  const double c = std::cos(obstacle.yaw_rad), s = std::sin(obstacle.yaw_rad);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  return std::abs(local_x) <= obstacle.half_size_x_m + clearance &&
         std::abs(local_y) <= obstacle.half_size_y_m + clearance;
}

std::array<double, 2> Extent(const StaticObstacle2D& obstacle, double clearance) {
  if (obstacle.kind == StaticObstacle2D::Kind::kCylinder) {
    return {obstacle.radius_m + clearance, obstacle.radius_m + clearance};
  }
  const double hx = obstacle.half_size_x_m + clearance;
  const double hy = obstacle.half_size_y_m + clearance;
  const double c = std::abs(std::cos(obstacle.yaw_rad));
  const double s = std::abs(std::sin(obstacle.yaw_rad));
  return {c * hx + s * hy, s * hx + c * hy};
}

std::string AttributeOr(const tinyxml2::XMLElement* element, const char* name,
                        const std::string& fallback) {
  const char* value = element->Attribute(name);
  return value == nullptr ? fallback : value;
}

}  // namespace

SdfStaticMapLoader::SdfStaticMapLoader(StaticMapConfig config) : config_(config) {
  if (config_.resolution_m <= 0.0 || config_.clearance_m < 0.0 ||
      config_.vertical_clearance_m < 0.0 || config_.bounds_padding_m <= 0.0 ||
      config_.max_grid_cells == 0) {
    throw std::invalid_argument("invalid static-map configuration");
  }
}

void SdfStaticMapLoader::Load(const std::string& sdf_path) {
  tinyxml2::XMLDocument document;
  const auto error = document.LoadFile(sdf_path.c_str());
  if (error != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot load SDF world: " + sdf_path + ": " +
                             document.ErrorStr());
  }
  const auto* sdf = document.FirstChildElement("sdf");
  const auto* world = sdf == nullptr ? nullptr : sdf->FirstChildElement("world");
  if (world == nullptr) throw std::runtime_error("SDF has no <world>: " + sdf_path);

  for (auto* include = world->FirstChildElement("include"); include != nullptr;
       include = include->NextSiblingElement("include")) {
    const std::string uri = Text(include->FirstChildElement("uri"));
    if (uri.find("iris_with_") == std::string::npos) {
      throw std::runtime_error("unresolved static SDF <include>: " + uri);
    }
  }

  std::vector<StaticObstacle2D> parsed;
  for (auto* model = world->FirstChildElement("model"); model != nullptr;
       model = model->NextSiblingElement("model")) {
    const std::string is_static = Text(model->FirstChildElement("static"));
    if (is_static != "true" && is_static != "1") continue;
    const auto model_pose = ParsePose(model->FirstChildElement("pose"));
    std::size_t link_index = 0;
    for (auto* link = model->FirstChildElement("link"); link != nullptr;
         link = link->NextSiblingElement("link"), ++link_index) {
      const auto link_pose = ComposePlanar(model_pose, ParsePose(link->FirstChildElement("pose")));
      std::size_t collision_index = 0;
      for (auto* collision = link->FirstChildElement("collision"); collision != nullptr;
           collision = collision->NextSiblingElement("collision"), ++collision_index) {
        const auto pose = ComposePlanar(link_pose, ParsePose(collision->FirstChildElement("pose")));
        const std::string name = AttributeOr(model, "name", "model") + "/" +
            AttributeOr(link, "name", std::to_string(link_index)) + "/" +
            AttributeOr(collision, "name", std::to_string(collision_index));
        if (std::abs(pose.roll) > 1e-9 || std::abs(pose.pitch) > 1e-9) {
          throw std::runtime_error("2.5D map does not support collision roll/pitch at " + name);
        }
        const auto* geometry = collision->FirstChildElement("geometry");
        if (geometry == nullptr) continue;
        StaticObstacle2D obstacle;
        obstacle.name = name;
        obstacle.center_x_m = pose.x;
        obstacle.center_y_m = pose.y;
        obstacle.yaw_rad = pose.yaw;
        if (const auto* box = geometry->FirstChildElement("box")) {
          const auto size = ParsePositiveTriple(box->FirstChildElement("size"), name);
          obstacle.kind = StaticObstacle2D::Kind::kBox;
          obstacle.half_size_x_m = 0.5 * size[0];
          obstacle.half_size_y_m = 0.5 * size[1];
          obstacle.z_min_m = pose.z - 0.5 * size[2];
          obstacle.z_max_m = pose.z + 0.5 * size[2];
        } else if (const auto* cylinder = geometry->FirstChildElement("cylinder")) {
          obstacle.kind = StaticObstacle2D::Kind::kCylinder;
          obstacle.radius_m = ParsePositiveScalar(cylinder->FirstChildElement("radius"), name);
          const double length = ParsePositiveScalar(cylinder->FirstChildElement("length"), name);
          obstacle.z_min_m = pose.z - 0.5 * length;
          obstacle.z_max_m = pose.z + 0.5 * length;
        } else {
          throw std::runtime_error("unsupported SDF geometry at " + name);
        }
        parsed.push_back(obstacle);
      }
    }
  }
  obstacles_ = std::move(parsed);
}

uav_navigation_core::CostGrid2D SdfStaticMapLoader::BuildCostGrid(
    double altitude_m, const std::vector<uav_navigation_core::Vec3>& required_points) const {
  if (!std::isfinite(altitude_m)) throw std::invalid_argument("altitude must be finite");
  std::vector<const StaticObstacle2D*> active;
  // A cell is considered occupied when any part of it may intersect the
  // requested inflated footprint. The half diagonal is a conservative bound
  // for arbitrary obstacle orientation inside a square grid cell.
  const double raster_clearance = config_.clearance_m +
      std::sqrt(0.5) * config_.resolution_m;
  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
  for (const auto& point : required_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("required map point must be finite");
    }
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  for (const auto& obstacle : obstacles_) {
    if (!ActiveAt(obstacle, altitude_m, config_.vertical_clearance_m)) continue;
    active.push_back(&obstacle);
    const auto extent = Extent(obstacle, raster_clearance);
    min_x = std::min(min_x, obstacle.center_x_m - extent[0]);
    max_x = std::max(max_x, obstacle.center_x_m + extent[0]);
    min_y = std::min(min_y, obstacle.center_y_m - extent[1]);
    max_y = std::max(max_y, obstacle.center_y_m + extent[1]);
  }
  min_x = std::floor((min_x - config_.bounds_padding_m) / config_.resolution_m) * config_.resolution_m;
  min_y = std::floor((min_y - config_.bounds_padding_m) / config_.resolution_m) * config_.resolution_m;
  max_x = std::ceil((max_x + config_.bounds_padding_m) / config_.resolution_m) * config_.resolution_m;
  max_y = std::ceil((max_y + config_.bounds_padding_m) / config_.resolution_m) * config_.resolution_m;
  const auto width = static_cast<std::uint32_t>(std::ceil((max_x - min_x) / config_.resolution_m));
  const auto height = static_cast<std::uint32_t>(std::ceil((max_y - min_y) / config_.resolution_m));
  const auto cell_count = static_cast<std::size_t>(width) * height;
  if (width == 0 || height == 0 || cell_count > config_.max_grid_cells) {
    throw std::runtime_error("SDF cost grid exceeds max_grid_cells");
  }
  uav_navigation_core::CostGrid2D grid;
  grid.frame_id = config_.frame_id;
  grid.resolution_m = config_.resolution_m;
  grid.origin_enu_m = {min_x, min_y, altitude_m};
  grid.width = width;
  grid.height = height;
  grid.costs.assign(cell_count, 0);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto center = uav_navigation_core::CellCenter(
          grid, {static_cast<int>(x), static_cast<int>(y)}, altitude_m);
      for (const auto* obstacle : active) {
        if (ContainsInflated(*obstacle, center.x, center.y, raster_clearance)) {
          grid.costs[static_cast<std::size_t>(y) * width + x] = 100;
          break;
        }
      }
    }
  }
  return grid;
}

}  // namespace uav_navigation_ros

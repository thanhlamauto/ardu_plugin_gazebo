#include "uav_navigation_core/cost_grid.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav_navigation_core {

bool IsValid(const CostGrid2D& grid) {
  if (!std::isfinite(grid.resolution_m) || grid.resolution_m <= 0.0 ||
      grid.width == 0 || grid.height == 0) {
    return false;
  }
  const auto cells = static_cast<std::uint64_t>(grid.width) * grid.height;
  return cells <= std::numeric_limits<std::size_t>::max() &&
         grid.costs.size() == static_cast<std::size_t>(cells);
}

bool Contains(const CostGrid2D& grid, GridCell cell) {
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < static_cast<int>(grid.width) &&
         cell.y < static_cast<int>(grid.height);
}

std::size_t FlatIndex(const CostGrid2D& grid, GridCell cell) {
  if (!Contains(grid, cell)) {
    throw std::out_of_range("grid cell is outside CostGrid2D");
  }
  return static_cast<std::size_t>(cell.y) * grid.width +
         static_cast<std::size_t>(cell.x);
}

std::optional<GridCell> WorldToCell(const CostGrid2D& grid,
                                    const Vec3& point_enu_m) {
  if (!IsValid(grid) || !std::isfinite(point_enu_m.x) ||
      !std::isfinite(point_enu_m.y)) {
    return std::nullopt;
  }
  GridCell cell{
      static_cast<int>(std::floor((point_enu_m.x - grid.origin_enu_m.x) /
                                  grid.resolution_m)),
      static_cast<int>(std::floor((point_enu_m.y - grid.origin_enu_m.y) /
                                  grid.resolution_m)),
  };
  return Contains(grid, cell) ? std::optional<GridCell>(cell) : std::nullopt;
}

Vec3 CellCenter(const CostGrid2D& grid, GridCell cell, double altitude_m) {
  if (!Contains(grid, cell)) {
    throw std::out_of_range("grid cell is outside CostGrid2D");
  }
  return Vec3{
      grid.origin_enu_m.x + (static_cast<double>(cell.x) + 0.5) * grid.resolution_m,
      grid.origin_enu_m.y + (static_cast<double>(cell.y) + 0.5) * grid.resolution_m,
      altitude_m,
  };
}

std::int8_t CostAt(const CostGrid2D& grid, GridCell cell) {
  return grid.costs.at(FlatIndex(grid, cell));
}

bool IsBlocked(const CostGrid2D& grid, GridCell cell, std::int8_t threshold) {
  const auto cost = CostAt(grid, cell);
  return cost < 0 || cost >= threshold;
}

}  // namespace uav_navigation_core

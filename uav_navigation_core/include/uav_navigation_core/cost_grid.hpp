#pragma once

#include <cstddef>
#include <optional>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

struct GridCell {
  int x{0};
  int y{0};

  bool operator==(const GridCell& other) const {
    return x == other.x && y == other.y;
  }
};

bool IsValid(const CostGrid2D& grid);
bool Contains(const CostGrid2D& grid, GridCell cell);
std::size_t FlatIndex(const CostGrid2D& grid, GridCell cell);
std::optional<GridCell> WorldToCell(const CostGrid2D& grid, const Vec3& point_enu_m);
Vec3 CellCenter(const CostGrid2D& grid, GridCell cell, double altitude_m);
std::int8_t CostAt(const CostGrid2D& grid, GridCell cell);
bool IsBlocked(const CostGrid2D& grid, GridCell cell, std::int8_t threshold = 100);

}  // namespace uav_navigation_core

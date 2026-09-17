#include "uav_navigation_core/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "uav_navigation_core/cost_grid.hpp"

namespace uav_navigation_core {
namespace {

double Distance(const Vec3& a, const Vec3& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double Heuristic(GridCell a, GridCell b, double resolution, bool diagonal) {
  const auto dx = static_cast<double>(std::abs(a.x - b.x));
  const auto dy = static_cast<double>(std::abs(a.y - b.y));
  if (!diagonal) return resolution * (dx + dy);
  const double minimum = std::min(dx, dy);
  return resolution * ((dx + dy) + (std::sqrt(2.0) - 2.0) * minimum);
}

struct QueueEntry {
  double f{0.0};
  double g{0.0};
  std::uint64_t serial{0};
  GridCell cell{};
};

struct GreaterEntry {
  bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
    if (lhs.f != rhs.f) return lhs.f > rhs.f;
    if (lhs.g != rhs.g) return lhs.g > rhs.g;
    return lhs.serial > rhs.serial;
  }
};

bool SamePoint(const Vec3& a, const Vec3& b) {
  constexpr double kEpsilon = 1e-12;
  return std::abs(a.x - b.x) < kEpsilon &&
         std::abs(a.y - b.y) < kEpsilon &&
         std::abs(a.z - b.z) < kEpsilon;
}

}  // namespace

AStarPlanner::AStarPlanner(AStarConfig config) : config_(config) {}

GlobalPlan AStarPlanner::Plan(const State& start, const Goal& goal,
                              const CostGrid2D& map) {
  GlobalPlan result;
  result.cost_grid = map;
  try {
    if (!IsValid(map) || !std::isfinite(start.position_enu_m.x) ||
        !std::isfinite(start.position_enu_m.y) ||
        !std::isfinite(start.position_enu_m.z) ||
        !std::isfinite(goal.position_enu_m.x) ||
        !std::isfinite(goal.position_enu_m.y) ||
        !std::isfinite(goal.position_enu_m.z) ||
        config_.max_expansions == 0 || config_.blocked_cost <= 0 ||
        config_.traversal_cost_weight < 0.0) {
      result.status = StatusCode::kInvalidInput;
      result.diagnostics.detail = "invalid grid, state, goal, or A* config";
      return result;
    }
    if (!goal.frame_id.empty() && goal.frame_id != map.frame_id) {
      result.status = StatusCode::kInvalidInput;
      result.diagnostics.detail = "goal and cost-grid frames differ";
      return result;
    }
    const auto start_cell = WorldToCell(map, start.position_enu_m);
    const auto goal_cell = WorldToCell(map, goal.position_enu_m);
    if (!start_cell || !goal_cell) {
      result.status = StatusCode::kNoPath;
      result.diagnostics.detail = "start or goal lies outside the cost grid";
      return result;
    }
    if (IsBlocked(map, *start_cell, config_.blocked_cost)) {
      result.status = StatusCode::kNoPath;
      result.diagnostics.detail = "start cell is blocked";
      return result;
    }
    if (IsBlocked(map, *goal_cell, config_.blocked_cost)) {
      result.status = StatusCode::kNoPath;
      result.diagnostics.detail = "goal cell is blocked";
      return result;
    }
    if (start_cell->x == goal_cell->x && start_cell->y == goal_cell->y) {
      result.path.points_enu_m.push_back(start.position_enu_m);
      if (!SamePoint(start.position_enu_m, goal.position_enu_m)) {
        result.path.points_enu_m.push_back(goal.position_enu_m);
      }
      result.status = StatusCode::kOk;
      result.diagnostics.detail = "expanded=0, start and goal share a cell";
      return result;
    }

    const auto cell_count = map.costs.size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(cell_count, infinity);
    std::vector<std::int64_t> came_from(cell_count, -1);
    std::vector<bool> closed(cell_count, false);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, GreaterEntry> open;
    const auto start_index = FlatIndex(map, *start_cell);
    const auto goal_index = FlatIndex(map, *goal_cell);
    g_score[start_index] = 0.0;
    std::uint64_t serial = 0;
    open.push(QueueEntry{Heuristic(*start_cell, *goal_cell, map.resolution_m,
                                   config_.allow_diagonal),
                         0.0, serial++, *start_cell});

    constexpr std::array<GridCell, 8> kMoves{{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    }};
    std::uint32_t expanded = 0;
    bool found = false;
    while (!open.empty()) {
      const auto current_entry = open.top();
      open.pop();
      const auto current_index = FlatIndex(map, current_entry.cell);
      if (closed[current_index] || current_entry.g > g_score[current_index]) continue;
      closed[current_index] = true;
      ++expanded;
      if (expanded > config_.max_expansions) {
        result.status = StatusCode::kNoPath;
        result.diagnostics.detail = "A* exceeded max_expansions";
        result.diagnostics.expanded_nodes = expanded;
        return result;
      }
      if (current_index == goal_index) {
        found = true;
        break;
      }

      const std::size_t move_count = config_.allow_diagonal ? kMoves.size() : 4U;
      for (std::size_t move_index = 0; move_index < move_count; ++move_index) {
        const auto move = kMoves[move_index];
        const GridCell neighbour{current_entry.cell.x + move.x,
                                 current_entry.cell.y + move.y};
        if (!Contains(map, neighbour) ||
            IsBlocked(map, neighbour, config_.blocked_cost)) continue;
        const bool diagonal = move.x != 0 && move.y != 0;
        if (diagonal && config_.prevent_corner_cutting) {
          const GridCell side_x{current_entry.cell.x + move.x, current_entry.cell.y};
          const GridCell side_y{current_entry.cell.x, current_entry.cell.y + move.y};
          if (IsBlocked(map, side_x, config_.blocked_cost) ||
              IsBlocked(map, side_y, config_.blocked_cost)) continue;
        }
        const auto neighbour_index = FlatIndex(map, neighbour);
        if (closed[neighbour_index]) continue;
        const auto raw_cost = CostAt(map, neighbour);
        const double normalized_cost = std::max(0, static_cast<int>(raw_cost)) / 100.0;
        const double step_length = map.resolution_m *
            (diagonal ? std::sqrt(2.0) : 1.0);
        const double tentative = g_score[current_index] + step_length *
            (1.0 + config_.traversal_cost_weight * normalized_cost);
        if (tentative >= g_score[neighbour_index]) continue;
        g_score[neighbour_index] = tentative;
        came_from[neighbour_index] = static_cast<std::int64_t>(current_index);
        const double f = tentative + Heuristic(neighbour, *goal_cell,
                                                map.resolution_m,
                                                config_.allow_diagonal);
        open.push(QueueEntry{f, tentative, serial++, neighbour});
      }
    }

    result.diagnostics.expanded_nodes = expanded;
    if (!found) {
      result.status = StatusCode::kNoPath;
      result.diagnostics.detail = "A* found no path";
      return result;
    }

    std::vector<std::size_t> reversed_indices;
    for (std::int64_t cursor = static_cast<std::int64_t>(goal_index);
         cursor >= 0; cursor = came_from[static_cast<std::size_t>(cursor)]) {
      reversed_indices.push_back(static_cast<std::size_t>(cursor));
      if (static_cast<std::size_t>(cursor) == start_index) break;
    }
    if (reversed_indices.empty() || reversed_indices.back() != start_index) {
      result.status = StatusCode::kInternalError;
      result.diagnostics.detail = "A* predecessor chain is incomplete";
      return result;
    }
    std::reverse(reversed_indices.begin(), reversed_indices.end());

    auto& points = result.path.points_enu_m;
    points.reserve(reversed_indices.size() + 2);
    points.push_back(start.position_enu_m);
    for (const auto index : reversed_indices) {
      const GridCell cell{static_cast<int>(index % map.width),
                          static_cast<int>(index / map.width)};
      const auto center = CellCenter(map, cell, goal.position_enu_m.z);
      if (!SamePoint(points.back(), center)) points.push_back(center);
    }
    if (!SamePoint(points.back(), goal.position_enu_m)) {
      points.push_back(goal.position_enu_m);
    }
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      length += Distance(points[i - 1], points[i]);
    }
    std::ostringstream detail;
    detail << "expanded=" << expanded << ", length_m=" << length;
    result.status = StatusCode::kOk;
    result.diagnostics.detail = detail.str();
    return result;
  } catch (const std::exception& error) {
    result.status = StatusCode::kInternalError;
    result.diagnostics.detail = error.what();
    result.path.points_enu_m.clear();
    return result;
  }
}

}  // namespace uav_navigation_core

#include "uav_navigation_core/astar_planner.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "uav_navigation_core/cost_grid.hpp"

namespace core = uav_navigation_core;

namespace {
int failures = 0;

void Check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

core::CostGrid2D Grid(std::uint32_t width = 20, std::uint32_t height = 20,
                      double resolution = 1.0) {
  core::CostGrid2D grid;
  grid.frame_id = "odom";
  grid.resolution_m = resolution;
  grid.width = width;
  grid.height = height;
  grid.costs.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

void Block(core::CostGrid2D& grid, int x, int y) {
  grid.costs[core::FlatIndex(grid, {x, y})] = 100;
}

core::GlobalPlan Plan(const core::CostGrid2D& grid, core::Vec3 start,
                      core::Vec3 goal, core::AStarConfig config = {}) {
  core::State state;
  state.position_enu_m = start;
  core::Goal target;
  target.position_enu_m = goal;
  target.frame_id = grid.frame_id;
  return core::AStarPlanner(config).Plan(state, target, grid);
}

bool PathIsCellSafe(const core::GlobalPlan& plan, const core::CostGrid2D& grid) {
  for (const auto& point : plan.path.points_enu_m) {
    const auto cell = core::WorldToCell(grid, point);
    if (!cell || core::IsBlocked(grid, *cell)) return false;
  }
  return true;
}

double PathLength(const core::GlobalPlan& plan) {
  double length = 0.0;
  for (std::size_t i = 1; i < plan.path.points_enu_m.size(); ++i) {
    const auto& a = plan.path.points_enu_m[i - 1];
    const auto& b = plan.path.points_enu_m[i];
    length += std::hypot(a.x - b.x, a.y - b.y);
  }
  return length;
}

void StraightPath() {
  const auto grid = Grid();
  const auto plan = Plan(grid, {1.2, 1.2, 5.0}, {15.2, 1.2, 5.0});
  Check(plan.status == core::StatusCode::kOk, "straight path succeeds");
  Check(PathIsCellSafe(plan, grid), "straight path stays in free cells");
  Check(std::abs(plan.path.points_enu_m.front().x - 1.2) < 1e-12,
        "straight path preserves start endpoint");
  Check(std::abs(plan.path.points_enu_m.back().x - 15.2) < 1e-12,
        "straight path preserves goal endpoint");
}

void SameCellPath() {
  const auto grid = Grid();
  const auto plan = Plan(grid, {1.1, 1.1, 5.0}, {1.8, 1.8, 5.0});
  Check(plan.status == core::StatusCode::kOk, "same-cell path succeeds");
  Check(plan.path.points_enu_m.size() == 2,
        "same-cell path contains only exact endpoints");
}

void ObstacleDetour() {
  auto grid = Grid();
  for (int y = 0; y < 15; ++y) Block(grid, 8, y);
  const auto plan = Plan(grid, {2.2, 2.2, 5.0}, {15.2, 2.2, 5.0});
  Check(plan.status == core::StatusCode::kOk, "wall detour succeeds through gap");
  Check(PathIsCellSafe(plan, grid), "wall detour avoids occupied cells");
  Check(PathLength(plan) > 13.0, "wall detour is longer than direct path");
}

void RasterizedRotatedBox() {
  auto grid = Grid(30, 30, 0.5);
  const double cx = 7.5, cy = 7.5, yaw = 0.65, hx = 2.5, hy = 0.8;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int y = 0; y < static_cast<int>(grid.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.width); ++x) {
      const auto p = core::CellCenter(grid, {x, y}, 5.0);
      const double dx = p.x - cx, dy = p.y - cy;
      const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
      if (std::abs(lx) <= hx && std::abs(ly) <= hy) Block(grid, x, y);
    }
  }
  const auto plan = Plan(grid, {1.0, 7.5, 5.0}, {14.0, 7.5, 5.0});
  Check(plan.status == core::StatusCode::kOk, "rotated-box detour succeeds");
  Check(PathIsCellSafe(plan, grid), "rotated-box path avoids rasterized box");
}

void RasterizedCylinder() {
  auto grid = Grid(30, 20, 0.5);
  for (int y = 0; y < static_cast<int>(grid.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.width); ++x) {
      const auto p = core::CellCenter(grid, {x, y}, 5.0);
      if (std::hypot(p.x - 7.5, p.y - 5.0) <= 1.5) Block(grid, x, y);
    }
  }
  const auto plan = Plan(grid, {1.0, 5.0, 5.0}, {14.0, 5.0, 5.0});
  Check(plan.status == core::StatusCode::kOk, "cylinder detour succeeds");
  Check(PathIsCellSafe(plan, grid), "cylinder path avoids rasterized disk");
}

void NarrowPassage() {
  auto grid = Grid(20, 20);
  for (int x = 4; x < 16; ++x) {
    if (x != 10) {
      Block(grid, x, 9);
      Block(grid, x, 11);
    }
  }
  const auto plan = Plan(grid, {2.2, 10.2, 5.0}, {17.2, 10.2, 5.0});
  Check(plan.status == core::StatusCode::kOk, "narrow free corridor succeeds");
  Check(PathIsCellSafe(plan, grid), "narrow corridor path is valid");
}

void BlockedEndpoints() {
  auto grid = Grid();
  Block(grid, 1, 1);
  auto plan = Plan(grid, {1.2, 1.2, 5.0}, {10.2, 10.2, 5.0});
  Check(plan.status == core::StatusCode::kNoPath,
        "blocked start returns NO_PATH");
  Check(plan.diagnostics.detail == "start cell is blocked",
        "blocked start has explicit reason");
  grid.costs.assign(grid.costs.size(), 0);
  Block(grid, 10, 10);
  plan = Plan(grid, {1.2, 1.2, 5.0}, {10.2, 10.2, 5.0});
  Check(plan.status == core::StatusCode::kNoPath,
        "blocked goal returns NO_PATH");
  Check(plan.diagnostics.detail == "goal cell is blocked",
        "blocked goal has explicit reason");
}

void UnreachableGoal() {
  auto grid = Grid();
  for (int y = 0; y < 20; ++y) Block(grid, 8, y);
  const auto plan = Plan(grid, {2.2, 2.2, 5.0}, {15.2, 2.2, 5.0});
  Check(plan.status == core::StatusCode::kNoPath, "solid wall returns NO_PATH");
}

void DiagonalPolicy() {
  const auto grid = Grid();
  core::AStarConfig diagonal;
  diagonal.allow_diagonal = true;
  core::AStarConfig cardinal;
  cardinal.allow_diagonal = false;
  const auto diagonal_plan = Plan(grid, {1.2, 1.2, 5.0}, {10.2, 10.2, 5.0}, diagonal);
  const auto cardinal_plan = Plan(grid, {1.2, 1.2, 5.0}, {10.2, 10.2, 5.0}, cardinal);
  Check(diagonal_plan.status == core::StatusCode::kOk &&
            cardinal_plan.status == core::StatusCode::kOk,
        "both diagonal policies find a path");
  Check(PathLength(diagonal_plan) < PathLength(cardinal_plan),
        "diagonal path is shorter than cardinal path");
}

void NoCornerCutting() {
  auto grid = Grid(3, 3);
  Block(grid, 1, 0);
  Block(grid, 0, 1);
  const auto plan = Plan(grid, {0.2, 0.2, 5.0}, {2.2, 2.2, 5.0});
  Check(plan.status == core::StatusCode::kNoPath,
        "diagonal cannot cut between blocked cardinal neighbours");
}

void MaxExpansions() {
  const auto grid = Grid(50, 50);
  core::AStarConfig config;
  config.max_expansions = 1;
  const auto plan = Plan(grid, {0.2, 0.2, 5.0}, {49.2, 49.2, 5.0}, config);
  Check(plan.status == core::StatusCode::kNoPath,
        "max_expansions returns NO_PATH");
  Check(plan.diagnostics.detail == "A* exceeded max_expansions",
        "max_expansions reason is explicit");
}

void CostAvoidance() {
  auto grid = Grid(20, 7);
  for (int x = 4; x < 16; ++x) {
    grid.costs[core::FlatIndex(grid, {x, 3})] = 99;
  }
  core::AStarConfig config;
  config.traversal_cost_weight = 10.0;
  const auto plan = Plan(grid, {1.2, 3.2, 5.0}, {18.2, 3.2, 5.0}, config);
  Check(plan.status == core::StatusCode::kOk, "inflated-cost detour succeeds");
  bool used_high_cost = false;
  for (const auto& point : plan.path.points_enu_m) {
    const auto cell = core::WorldToCell(grid, point);
    used_high_cost = used_high_cost || (cell && core::CostAt(grid, *cell) == 99);
  }
  Check(!used_high_cost, "planner avoids high-cost cells when a cheap detour exists");
}
}  // namespace

int main() {
  StraightPath();
  SameCellPath();
  ObstacleDetour();
  RasterizedRotatedBox();
  RasterizedCylinder();
  NarrowPassage();
  BlockedEndpoints();
  UnreachableGoal();
  DiagonalPolicy();
  NoCornerCutting();
  MaxExpansions();
  CostAvoidance();
  if (failures != 0) {
    std::cerr << failures << " A* checks failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All A* checks passed\n";
  return EXIT_SUCCESS;
}

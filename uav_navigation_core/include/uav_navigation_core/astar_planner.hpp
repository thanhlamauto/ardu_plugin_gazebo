#pragma once

#include <cstdint>

#include "uav_navigation_core/interfaces.hpp"

namespace uav_navigation_core {

struct AStarConfig {
  bool allow_diagonal{true};
  bool prevent_corner_cutting{true};
  std::uint32_t max_expansions{250000};
  std::int8_t blocked_cost{100};
  double traversal_cost_weight{1.0};
};

class AStarPlanner final : public IGlobalPlanner {
 public:
  explicit AStarPlanner(AStarConfig config = {});

  GlobalPlan Plan(const State& start, const Goal& goal,
                  const CostGrid2D& map) override;

  const AStarConfig& config() const { return config_; }

 private:
  AStarConfig config_;
};

}  // namespace uav_navigation_core

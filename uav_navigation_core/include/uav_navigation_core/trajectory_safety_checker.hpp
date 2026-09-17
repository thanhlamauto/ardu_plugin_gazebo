#pragma once

#include <memory>

#include "uav_navigation_core/collision_environment.hpp"
#include "uav_navigation_core/interfaces.hpp"

namespace uav_navigation_core {

struct TrajectorySafetyConfig {
  double collision_radius_m{1.5};
  double braking_acceleration_m_s2{3.0};
  double stopping_delay_s{0.25};
  double stopping_clearance_m{1.5};
  double stopping_uncertainty_m{0.0};
  double map_sample_spacing_m{0.1};
  double cloud_map_tolerance_m{0.1};
};

class TrajectorySafetyChecker final : public ISafetyChecker {
public:
  explicit TrajectorySafetyChecker(
      TrajectorySafetyConfig config = {},
      std::shared_ptr<const ICollisionEnvironment> environment = nullptr);

  SafetyResult Evaluate(const State &initial_state,
                        const Trajectory &trajectory,
                        const ObstacleMap &obstacles) const override;

  const TrajectorySafetyConfig &config() const { return config_; }

private:
  TrajectorySafetyConfig config_;
  std::shared_ptr<const ICollisionEnvironment> environment_;
};

} // namespace uav_navigation_core

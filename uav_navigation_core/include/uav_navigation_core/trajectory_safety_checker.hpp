#pragma once

#include <memory>
#include <string>
#include <vector>

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

// Exact spatial index over a filtered obstacle cloud. Defined in the
// implementation file; the prepared snapshot holds it by shared ownership.
class CloudIndex;

// Trajectory-independent safety preprocessing for one immutable obstacle
// snapshot. Preparing once and evaluating many candidate trajectories produces
// exactly the same result as calling the obstacle-taking Evaluate overload for
// each trajectory, while avoiding repeated cloud/environment filtering and
// repeated linear scans of the cloud.
struct PreparedCollisionEnvironment {
  // When false, every trajectory evaluates to Invalid(invalid_reason), which
  // mirrors the fail-closed behavior of the original single-call implementation.
  bool valid{true};
  std::string invalid_reason{"clear_trajectory"};
  bool cloud_present{false};
  double map_expansion_m{0.0};
  std::vector<Vec3> cloud;
  // Exact nearest-point-to-segment acceleration. Returns bit-identical distances
  // to a linear scan, only faster.
  std::shared_ptr<const CloudIndex> index;
};

class TrajectorySafetyChecker final : public ISafetyChecker {
public:
  explicit TrajectorySafetyChecker(
      TrajectorySafetyConfig config = {},
      std::shared_ptr<const ICollisionEnvironment> environment = nullptr);

  // Filters the obstacle cloud against the known geometry and detects invalid
  // inputs once. Must be followed by Evaluate with the same checker instance.
  PreparedCollisionEnvironment Prepare(const ObstacleMap &obstacles) const;

  // Trajectory-dependent evaluation using a prepared snapshot. The safety
  // predicate, first-failure classification and clearance reporting are
  // identical to the obstacle-taking overload.
  SafetyResult Evaluate(const State &initial_state, const Trajectory &trajectory,
                        const PreparedCollisionEnvironment &prepared) const;

  SafetyResult Evaluate(const State &initial_state,
                        const Trajectory &trajectory,
                        const ObstacleMap &obstacles) const override;

  const TrajectorySafetyConfig &config() const { return config_; }

private:
  TrajectorySafetyConfig config_;
  std::shared_ptr<const ICollisionEnvironment> environment_;
};

} // namespace uav_navigation_core

#pragma once

#include <array>
#include <memory>
#include <vector>

#include "uav_navigation_core/collision_environment.hpp"
#include "uav_navigation_core/mppi/path_reference.hpp"
#include "uav_navigation_core/mppi/rollout.hpp"

namespace uav_navigation_core::mppi {

enum class CostProfile { kProject, kPaper };

struct MppiCostConfig {
  CostProfile profile{CostProfile::kProject};
  double margin_m{4.0};
  double w_goal{1.0};
  double w_terminal{5.0};
  double w_obstacle{300.0};
  double w_collision{1.0e6};
  double collision_radius_m{0.5};
  double collision_cost_buffer_m{0.0};
  double w_effort{0.05};
  double w_stopping{0.0};
  double stopping_margin_m{2.0};
  double stopping_delay_s{0.25};
  double braking_acceleration_m_s2{1.5};
  double w_smoothness{0.2};
  double w_yaw{0.2};
  double w_path{0.0};
  double path_scale_m{1.0};
  double w_reference_velocity{0.0};
  bool path_progress_objective{false};
  double w_progress{0.0};
  double w_speed_limit{0.0};
  double vmax_m_s{2.0};
  std::array<double, 4> paper_r_u{{0.01, 0.05, 0.05, 0.10}};
  std::array<double, 4> paper_r_delta_u{{0.05, 0.10, 0.10, 0.30}};
};

struct CostContext {
  Vec3 goal_enu_m{};
  std::vector<Vec3> obstacles_enu_m{};
  std::shared_ptr<const ICollisionEnvironment> known_geometry{};
  std::shared_ptr<const PathReference> reference_path{};
  std::vector<Vec3> reference_positions{};
  std::vector<Vec3> reference_velocities{};
  MppiState initial_state{};
};

struct CostBreakdown {
  double goal{0.0};
  double obstacle{0.0};
  double collision{0.0};
  double stopping{0.0};
  double effort{0.0};
  double smoothness{0.0};
  double yaw{0.0};
  double path{0.0};
  double reference_velocity{0.0};
  double speed_limit{0.0};
  double terminal{0.0};
  double progress{0.0};
  double input_change{0.0};

  double Total() const;
};

class CostEvaluator {
public:
  explicit CostEvaluator(MppiCostConfig config = {});

  CostBreakdown EvaluateTrajectory(const MppiTrajectory &trajectory,
                                   const ControlSequence &requested_actions,
                                   const CostContext &context) const;
  const MppiCostConfig &config() const { return config_; }

private:
  MppiCostConfig config_;
};

} // namespace uav_navigation_core::mppi

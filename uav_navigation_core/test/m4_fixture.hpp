#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mppi_fixture.hpp"
#include "uav_navigation_core/mppi/cost_evaluator.hpp"
#include "uav_navigation_core/mppi/optimizer.hpp"

namespace m4_test {
namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;

inline std::vector<core::Vec3> Vec3Array(const std::vector<double> &flat) {
  std::vector<core::Vec3> result;
  for (std::size_t i = 0; i < flat.size(); i += 3)
    result.push_back({flat.at(i), flat.at(i + 1), flat.at(i + 2)});
  return result;
}

inline mppi::ControlSequence ControlArray(const std::vector<double> &flat) {
  mppi::ControlSequence result;
  for (std::size_t i = 0; i < flat.size(); i += 4)
    result.push_back(mppi_test::ControlAt(flat, i));
  return result;
}

struct Fixture {
  std::string json;
  mppi::MppiDynamicsConfig dynamics;
  mppi::MppiCostConfig cost;
  mppi::CostContext context;
  mppi::MppiState initial;
  mppi::ControlSequence controls;
  double dt_s{0.0};
};

inline Fixture Load(const std::string &name) {
  Fixture result;
  result.json = ReadFixture("mppi_m4/" + name + ".json");
  const auto &json = result.json;
  result.dt_s = Number(json, "dt");
  result.dynamics.tau_s = Number(json, "tau");
  result.dynamics.command_alpha = Number(json, "command_alpha");
  result.dynamics.max_accel_xy_m_s2 = Number(json, "max_accel_xy");
  result.dynamics.max_accel_z_m_s2 = Number(json, "max_accel_z");
  result.dynamics.max_yaw_accel_rad_s2 = Number(json, "max_yaw_accel");
  result.dynamics.vmax_m_s = Number(json, "vmax");
  result.dynamics.vzmax_m_s = Number(json, "vzmax");
  result.dynamics.yaw_rate_max_rad_s = Number(json, "yaw_rate_max");
  result.dynamics.response_accel_model = Boolean(json, "response_accel_model");
  result.dynamics.response_accel_xy_m_s2 = Number(json, "response_accel_xy");
  result.dynamics.response_jerk_xy_m_s3 = Number(json, "response_jerk_xy");

  result.cost.profile = String(json, "cost_profile") == "paper"
                            ? mppi::CostProfile::kPaper
                            : mppi::CostProfile::kProject;
  result.cost.margin_m = Number(json, "margin");
  result.cost.w_goal = Number(json, "w_goal");
  result.cost.w_terminal = Number(json, "w_terminal");
  result.cost.w_obstacle = Number(json, "w_obstacle");
  result.cost.w_collision = Number(json, "w_collision");
  result.cost.collision_radius_m = Number(json, "collision_radius_m");
  result.cost.collision_cost_buffer_m = Number(json, "collision_cost_buffer_m");
  result.cost.w_effort = Number(json, "w_u");
  result.cost.w_stopping = Number(json, "w_stopping");
  result.cost.stopping_margin_m = Number(json, "stopping_margin_m");
  result.cost.stopping_delay_s = Number(json, "stopping_delay_s");
  result.cost.braking_acceleration_m_s2 = Number(json, "max_accel_xy");
  result.cost.w_smoothness = Number(json, "w_du");
  result.cost.w_yaw = Number(json, "w_yaw");
  result.cost.w_path = Number(json, "w_path");
  result.cost.path_scale_m = Number(json, "path_scale_m");
  result.cost.w_reference_velocity = Number(json, "w_reference_velocity");
  result.cost.path_progress_objective =
      Boolean(json, "path_progress_objective");
  result.cost.w_progress = Number(json, "w_progress");
  result.cost.w_speed_limit = Number(json, "w_speed_limit");
  result.cost.vmax_m_s = Number(json, "vmax");
  const auto paper_u = Numbers(json, "paper_r_u");
  const auto paper_du = Numbers(json, "paper_r_delta_u");
  for (std::size_t i = 0; i < 4; ++i) {
    result.cost.paper_r_u[i] = paper_u.at(i);
    result.cost.paper_r_delta_u[i] = paper_du.at(i);
  }

  const auto initial = Numbers(json, "initial_state_flat");
  result.initial = mppi_test::StateAt(initial, 0, initial.size(), 1000000000);
  result.controls = ControlArray(Numbers(json, "controls_flat"));
  const auto goal = Numbers(json, "goal_flat");
  result.context.goal_enu_m = {goal.at(0), goal.at(1), goal.at(2)};
  result.context.obstacles_enu_m = Vec3Array(Numbers(json, "obstacles_flat"));
  const auto path = Vec3Array(Numbers(json, "path_flat"));
  if (!path.empty())
    result.context.reference_path = std::make_shared<mppi::PathReference>(path);
  result.context.reference_positions =
      Vec3Array(Numbers(json, "reference_positions_flat"));
  result.context.reference_velocities =
      Vec3Array(Numbers(json, "reference_velocities_flat"));
  result.context.initial_state = result.initial;
  return result;
}

inline std::vector<double> Flatten(const mppi::ControlSequence &sequence) {
  std::vector<double> result;
  for (const auto &control : sequence) {
    const auto values =
        mppi_test::Flatten(mppi::MppiState{0, {}, {}, 0.0, control, {}}, false);
    result.insert(result.end(), values.begin() + 7, values.end());
  }
  return result;
}

inline std::vector<double> Terms(const mppi::CostBreakdown &value) {
  return {value.goal,        value.obstacle, value.collision,
          value.stopping,    value.effort,   value.smoothness,
          value.yaw,         value.path,     value.reference_velocity,
          value.speed_limit, value.terminal, value.progress,
          value.input_change};
}

} // namespace m4_test

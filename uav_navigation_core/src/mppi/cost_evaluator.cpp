#include "uav_navigation_core/mppi/cost_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace uav_navigation_core::mppi {
namespace {
Vec3 Sub(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 Scale(const Vec3 &a, double scale) {
  return {a.x * scale, a.y * scale, a.z * scale};
}
double Dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
double Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }
double Distance(const Vec3 &a, const Vec3 &b) { return Norm(Sub(a, b)); }
double WrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
bool Finite(double value) { return std::isfinite(value); }
bool Finite(const Vec3 &value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}
bool Finite(const Control &value) {
  return Finite(value.velocity_enu_m_s) && Finite(value.yaw_rate_enu_rad_s);
}
bool Finite(const MppiState &value) {
  return Finite(value.position_enu_m) && Finite(value.velocity_enu_m_s) &&
         Finite(value.yaw_enu_rad) && Finite(value.applied_control) &&
         Finite(value.acceleration_memory_enu_m_s2);
}
std::array<double, 4> Components(const Control &control) {
  return {control.velocity_enu_m_s.x, control.velocity_enu_m_s.y,
          control.velocity_enu_m_s.z, control.yaw_rate_enu_rad_s};
}
double NearestPoint(const Vec3 &point, const std::vector<Vec3> &obstacles) {
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles)
    nearest = std::min(nearest, Distance(point, obstacle));
  return nearest;
}
double Softplus(double value, double beta) {
  const double scaled = beta * value;
  if (scaled > 0.0)
    return (scaled + std::log1p(std::exp(-scaled))) / beta;
  return std::log1p(std::exp(scaled)) / beta;
}
double StoppingClearance(const Vec3 &position, const Vec3 &velocity,
                         const std::vector<Vec3> &obstacles, double delay_s,
                         double acceleration_m_s2) {
  const double speed = Norm(velocity);
  const double length =
      speed * delay_s + speed * speed / (2.0 * acceleration_m_s2);
  const Vec3 direction = Scale(velocity, 1.0 / std::max(speed, 1e-9));
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles) {
    const Vec3 offset = Sub(obstacle, position);
    const double along =
        std::min(std::max(Dot(offset, direction), 0.0), length);
    nearest = std::min(nearest, Norm(Sub(offset, Scale(direction, along))));
  }
  return nearest;
}
void ValidateContext(const CostContext &context) {
  if (!Finite(context.goal_enu_m) || !Finite(context.initial_state))
    throw std::invalid_argument(
        "MPPI cost goal and initial state must be finite");
  for (const auto &point : context.obstacles_enu_m)
    if (!Finite(point))
      throw std::invalid_argument("MPPI cost obstacles must be finite");
  for (const auto &point : context.reference_positions)
    if (!Finite(point))
      throw std::invalid_argument("MPPI position references must be finite");
  for (const auto &velocity : context.reference_velocities)
    if (!Finite(velocity))
      throw std::invalid_argument("MPPI velocity references must be finite");
}
} // namespace

double CostBreakdown::Total() const {
  return goal + obstacle + collision + stopping + effort + smoothness + yaw +
         path + reference_velocity + speed_limit + terminal + progress +
         input_change;
}

CostEvaluator::CostEvaluator(MppiCostConfig config)
    : config_(std::move(config)) {
  const double nonnegative[] = {
      config_.margin_m,
      config_.w_goal,
      config_.w_terminal,
      config_.w_obstacle,
      config_.w_collision,
      config_.collision_radius_m,
      config_.collision_cost_buffer_m,
      config_.w_effort,
      config_.w_stopping,
      config_.stopping_margin_m,
      config_.stopping_delay_s,
      config_.w_smoothness,
      config_.w_yaw,
      config_.w_path,
      config_.w_reference_velocity,
      config_.w_progress,
      config_.w_speed_limit,
  };
  for (double value : nonnegative)
    if (!Finite(value) || value < 0.0)
      throw std::invalid_argument(
          "MPPI cost weights must be finite and nonnegative");
  if (!Finite(config_.path_scale_m) || config_.path_scale_m <= 0.0 ||
      !Finite(config_.vmax_m_s) || config_.vmax_m_s <= 0.0 ||
      !Finite(config_.braking_acceleration_m_s2) ||
      config_.braking_acceleration_m_s2 <= 0.0)
    throw std::invalid_argument("MPPI cost scales must be finite and positive");
  for (double value : config_.paper_r_u)
    if (!Finite(value) || value < 0.0)
      throw std::invalid_argument("MPPI paper effort weights are invalid");
  for (double value : config_.paper_r_delta_u)
    if (!Finite(value) || value < 0.0)
      throw std::invalid_argument("MPPI paper delta weights are invalid");
}

CostBreakdown
CostEvaluator::EvaluateTrajectory(const MppiTrajectory &trajectory,
                                  const ControlSequence &requested_actions,
                                  const CostContext &context) const {
  ValidateContext(context);
  if (trajectory.points.size() != requested_actions.size() + 1 ||
      trajectory.points.empty())
    throw std::invalid_argument("MPPI cost requires N actions and N+1 states");
  const bool timed_reference = !config_.path_progress_objective &&
                               config_.profile == CostProfile::kPaper &&
                               !context.reference_positions.empty();
  if (timed_reference && context.reference_velocities.empty())
    throw std::invalid_argument("MPPI time reference needs velocity samples");

  CostBreakdown result;
  std::vector<Control> feasible_actions;
  feasible_actions.reserve(requested_actions.size());
  for (std::size_t t = 0; t < requested_actions.size(); ++t) {
    const auto &state = trajectory.points[t + 1].state;
    const auto &requested = requested_actions[t];
    if (!Finite(state.position_enu_m) || !Finite(state.velocity_enu_m_s) ||
        !Finite(state.yaw_enu_rad) || !Finite(state.applied_control) ||
        !Finite(requested))
      throw std::invalid_argument("MPPI cost trajectory must be finite");
    const Vec3 &position = state.position_enu_m;
    const Vec3 &velocity = state.velocity_enu_m_s;
    const Control &feasible = state.applied_control;
    feasible_actions.push_back(feasible);

    result.goal += config_.w_goal * Distance(position, context.goal_enu_m);
    if (config_.profile == CostProfile::kPaper) {
      double nearest = NearestPoint(position, context.obstacles_enu_m);
      if (context.known_geometry)
        nearest =
            std::min(nearest, context.known_geometry->Clearance(position));
      result.collision +=
          config_.w_collision *
          static_cast<double>(nearest <= config_.collision_radius_m +
                                             config_.collision_cost_buffer_m);
    } else if (!context.obstacles_enu_m.empty()) {
      const double nearest = NearestPoint(position, context.obstacles_enu_m);
      result.obstacle +=
          config_.w_obstacle *
          (Softplus(config_.margin_m - nearest, 2.0) - std::log(2.0) / 2.0);
    }

    if (config_.w_stopping > 0.0 && t % 5 == 0) {
      const double nearest = StoppingClearance(
          position, velocity, context.obstacles_enu_m, config_.stopping_delay_s,
          config_.braking_acceleration_m_s2);
      const double violation =
          std::max(config_.stopping_margin_m - nearest, 0.0);
      result.stopping += config_.w_stopping * violation * violation;
    }

    const auto effort_action = Components(
        config_.profile == CostProfile::kPaper ? feasible : requested);
    if (config_.profile == CostProfile::kPaper) {
      for (std::size_t i = 0; i < 4; ++i)
        result.effort +=
            effort_action[i] * effort_action[i] * config_.paper_r_u[i];
    } else {
      result.effort += config_.w_effort * Dot(requested.velocity_enu_m_s,
                                              requested.velocity_enu_m_s);
    }

    const Vec3 tracking_error = Sub(requested.velocity_enu_m_s, velocity);
    result.smoothness +=
        config_.w_smoothness * Dot(tracking_error, tracking_error);
    const double desired_yaw =
        std::atan2(requested.velocity_enu_m_s.y, requested.velocity_enu_m_s.x);
    const double yaw_error = WrapAngle(desired_yaw - state.yaw_enu_rad);
    const double requested_xy =
        std::hypot(requested.velocity_enu_m_s.x, requested.velocity_enu_m_s.y);
    result.yaw += config_.w_yaw * requested_xy * yaw_error * yaw_error;

    if (timed_reference) {
      const auto position_index =
          std::min(t, context.reference_positions.size() - 1);
      const auto velocity_index =
          std::min(t, context.reference_velocities.size() - 1);
      const Vec3 path_error =
          Sub(position, context.reference_positions[position_index]);
      const Vec3 velocity_error =
          Sub(velocity, context.reference_velocities[velocity_index]);
      result.path += config_.w_path * Dot(path_error, path_error) /
                     (config_.path_scale_m * config_.path_scale_m);
      result.reference_velocity +=
          config_.w_reference_velocity * Dot(velocity_error, velocity_error);
    } else if (context.reference_path) {
      const double distance =
          context.reference_path->Distance(position) / config_.path_scale_m;
      result.path += config_.w_path * distance * distance;
    }
    if (config_.path_progress_objective) {
      const double speed_xy = std::hypot(velocity.x, velocity.y);
      const double excess = std::max(speed_xy - config_.vmax_m_s, 0.0);
      result.speed_limit += config_.w_speed_limit * excess * excess;
    }
  }

  const Vec3 &terminal_position = trajectory.points.back().state.position_enu_m;
  const Vec3 terminal_reference =
      timed_reference ? context.reference_positions.back() : context.goal_enu_m;
  const double terminal_distance =
      Distance(terminal_position, terminal_reference);
  result.terminal = config_.w_terminal * terminal_distance * terminal_distance;
  if (config_.path_progress_objective && context.reference_path) {
    result.progress = -config_.w_progress *
                      (context.reference_path->Progress(terminal_position) -
                       context.reference_path->Progress(
                           context.initial_state.position_enu_m));
  }
  if (config_.profile == CostProfile::kPaper) {
    auto previous = Components(context.initial_state.applied_control);
    for (const auto &action : feasible_actions) {
      const auto current = Components(action);
      for (std::size_t i = 0; i < 4; ++i) {
        const double delta = current[i] - previous[i];
        result.input_change += delta * delta * config_.paper_r_delta_u[i];
      }
      previous = current;
    }
  }
  return result;
}

} // namespace uav_navigation_core::mppi

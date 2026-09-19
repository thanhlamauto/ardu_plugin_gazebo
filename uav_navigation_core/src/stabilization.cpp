#include "uav_navigation_core/stabilization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uav_navigation_core {
namespace {
double WrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
double TangentHeading(const mppi::PathReference &path, double progress_m) {
  const double length = path.length_m();
  const double lower = std::clamp(progress_m - 0.5, 0.0, length);
  const double upper = std::clamp(progress_m + 0.5, 0.0, length);
  if (upper - lower < 1e-6)
    return 0.0;
  const Vec3 a = path.Sample(lower);
  const Vec3 b = path.Sample(upper);
  return std::atan2(b.y - a.y, b.x - a.x);
}
double DistanceToNextTurn(const mppi::PathReference &path, double progress_m,
                          double turn_angle_rad) {
  const double length = path.length_m();
  const double step = 0.5;
  const double base = TangentHeading(path, progress_m);
  for (double s = progress_m + step; s <= length; s += step)
    if (std::abs(WrapAngle(TangentHeading(path, s) - base)) > turn_angle_rad)
      return s - progress_m;
  return std::numeric_limits<double>::infinity();
}
} // namespace

double ComputeReferenceSpeedCap(const SpeedShapingConfig &config,
                                const mppi::PathReference &path,
                                const Vec3 &position_enu_m,
                                const Vec3 &velocity_enu_m_s) {
  if (!config.enabled)
    return std::numeric_limits<double>::infinity();
  const double progress = path.Progress(position_enu_m);
  const double length = path.length_m();
  const double speed =
      std::hypot(velocity_enu_m_s.x, velocity_enu_m_s.y);
  double cap = config.nominal_speed_m_s;

  const double lookahead =
      std::min(config.lookahead_m, std::max(0.0, length - progress));
  if (lookahead > 1e-3) {
    const double heading_now = TangentHeading(path, progress);
    const double heading_ahead = TangentHeading(path, progress + lookahead);
    const double heading_change = std::abs(WrapAngle(heading_ahead - heading_now));
    if (heading_change > 1e-6) {
      const double radius = lookahead / heading_change;
      cap = std::min(cap, std::sqrt(config.lateral_accel_m_s2 * radius));
    }
  }

  const double distance =
      DistanceToNextTurn(path, progress, config.turn_angle_rad);
  if (std::isfinite(distance) && speed > config.turn_speed_m_s) {
    const double required =
        (speed * speed - config.turn_speed_m_s * config.turn_speed_m_s) /
            (2.0 * config.braking_accel_m_s2) +
        config.reaction_delay_s * speed;
    if (distance <= required)
      cap = std::min(cap, config.turn_speed_m_s);
  }
  return std::clamp(cap, config.min_speed_m_s, config.nominal_speed_m_s);
}

bool StoppingDominant(const mppi::RejectionCounts &counts) {
  if (counts.total == 0 || counts.reject_stopping_distance == 0)
    return false;
  return counts.reject_stopping_distance >= counts.reject_swept_collision &&
         counts.reject_stopping_distance >= counts.reject_static_collision &&
         counts.reject_stopping_distance >= counts.reject_dynamic_collision &&
         counts.reject_stopping_distance >= counts.reject_non_finite &&
         counts.reject_stopping_distance >= counts.reject_other;
}

mppi::ControlSequence BuildBrakingSequence(const Vec3 &velocity_enu_m_s,
                                           double deceleration_m_s2,
                                           std::size_t horizon, double dt_s) {
  const double speed =
      std::hypot(velocity_enu_m_s.x, velocity_enu_m_s.y);
  if (speed < 1e-3 || horizon == 0 || deceleration_m_s2 <= 0.0 || dt_s <= 0.0)
    return {};
  const double direction_x = velocity_enu_m_s.x / speed;
  const double direction_y = velocity_enu_m_s.y / speed;
  mppi::ControlSequence sequence(horizon);
  double remaining = speed;
  for (std::size_t t = 0; t < horizon; ++t) {
    remaining = std::max(0.0, remaining - deceleration_m_s2 * dt_s);
    sequence[t].velocity_enu_m_s = {direction_x * remaining,
                                    direction_y * remaining, 0.0};
    sequence[t].yaw_rate_enu_rad_s = 0.0;
  }
  return sequence;
}

} // namespace uav_navigation_core

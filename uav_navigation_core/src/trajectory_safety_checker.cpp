#include "uav_navigation_core/trajectory_safety_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace uav_navigation_core {
namespace {
constexpr double kTiny = 1e-12;
bool Finite(double v) { return std::isfinite(v); }
bool Finite(const Vec3 &v) { return Finite(v.x) && Finite(v.y) && Finite(v.z); }
bool Finite(const State &s) {
  return Finite(s.position_enu_m) && Finite(s.velocity_enu_m_s) &&
         Finite(s.acceleration_enu_m_s2) && Finite(s.yaw_enu_rad);
}
bool Finite(const Control &u) {
  return Finite(u.velocity_enu_m_s) && Finite(u.yaw_rate_enu_rad_s);
}
Vec3 Sub(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 Add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 Scale(const Vec3 &a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double Dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
double Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }
double PointSegmentDistance(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
  const Vec3 ab = Sub(b, a);
  const double denom = std::max(Dot(ab, ab), kTiny);
  const double t = std::clamp(Dot(Sub(p, a), ab) / denom, 0.0, 1.0);
  return Norm(Sub(p, Add(a, Scale(ab, t))));
}
double CloudSegmentClearance(const Vec3 &a, const Vec3 &b,
                             const std::vector<Vec3> &cloud) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto &p : cloud)
    result = std::min(result, PointSegmentDistance(p, a, b));
  return result;
}
double MapClearanceLowerBound(const ICollisionEnvironment &env, const Vec3 &a,
                              const Vec3 &b, double spacing) {
  const Vec3 ab = Sub(b, a);
  const double length = Norm(ab);
  const auto count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / spacing)));
  double result = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i <= count; ++i) {
    result = std::min(
        result,
        env.Clearance(Add(a, Scale(ab, static_cast<double>(i) / count))));
  }
  return result - length / (2.0 * count);
}
SafetyResult Invalid(const char *reason) {
  return {false, -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(), reason};
}
} // namespace

TrajectorySafetyChecker::TrajectorySafetyChecker(
    TrajectorySafetyConfig config,
    std::shared_ptr<const ICollisionEnvironment> environment)
    : config_(std::move(config)), environment_(std::move(environment)) {
  const double values[] = {
      config_.collision_radius_m,     config_.braking_acceleration_m_s2,
      config_.stopping_delay_s,       config_.stopping_clearance_m,
      config_.stopping_uncertainty_m, config_.map_sample_spacing_m,
      config_.cloud_map_tolerance_m};
  for (double value : values)
    if (!Finite(value))
      throw std::invalid_argument("non-finite safety parameter");
  if (config_.collision_radius_m <= 0.0 ||
      config_.braking_acceleration_m_s2 <= 0.0 ||
      config_.stopping_delay_s < 0.0 || config_.stopping_clearance_m < 0.0 ||
      config_.stopping_uncertainty_m < 0.0 ||
      config_.map_sample_spacing_m <= 0.0 ||
      config_.cloud_map_tolerance_m < 0.0)
    throw std::invalid_argument("invalid safety parameter");
}

SafetyResult
TrajectorySafetyChecker::Evaluate(const State &initial_state,
                                  const Trajectory &trajectory,
                                  const ObstacleMap &obstacles) const {
  if (!Finite(initial_state) || trajectory.points.size() < 2)
    return Invalid("invalid_trajectory");
  double previous_time = -std::numeric_limits<double>::infinity();
  for (const auto &point : trajectory.points) {
    if (!Finite(point.time_from_start_s) ||
        point.time_from_start_s < previous_time || !Finite(point.state) ||
        !Finite(point.control))
      return Invalid("invalid_trajectory");
    previous_time = point.time_from_start_s;
  }
  for (const auto &point : obstacles.points_enu_m)
    if (!Finite(point))
      return Invalid("invalid_obstacle_map");

  try {
    const bool cloud_present = !obstacles.points_enu_m.empty();
    double map_expansion = 0.0;
    std::vector<Vec3> cloud;
    cloud.reserve(obstacles.points_enu_m.size());
    for (const auto &point : obstacles.points_enu_m) {
      if (environment_) {
        const double residual = environment_->Clearance(point);
        if (!Finite(residual) || residual < 0.0)
          return Invalid("invalid_collision_environment");
        if (residual <= config_.cloud_map_tolerance_m) {
          map_expansion = std::max(map_expansion, residual);
          continue;
        }
      }
      cloud.push_back(point);
    }

    double cloud_collision = cloud_present
                                 ? std::numeric_limits<double>::infinity()
                                 : -std::numeric_limits<double>::infinity();
    double map_collision = std::numeric_limits<double>::infinity();
    bool map_safe = true;
    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto &a = trajectory.points[i - 1].state.position_enu_m;
      const auto &b = trajectory.points[i].state.position_enu_m;
      cloud_collision =
          std::min(cloud_collision, CloudSegmentClearance(a, b, cloud));
      if (environment_) {
        map_safe =
            map_safe && environment_->SegmentSafe(
                            a, b, config_.collision_radius_m + map_expansion);
        map_collision =
            std::min(map_collision,
                     MapClearanceLowerBound(*environment_, a, b,
                                            config_.map_sample_spacing_m));
      }
    }

    const double required_stop =
        config_.stopping_clearance_m + config_.stopping_uncertainty_m;
    double cloud_stop = std::numeric_limits<double>::infinity();
    double map_stop = std::numeric_limits<double>::infinity();
    bool stop_map_safe = true;
    for (const auto &point : trajectory.points) {
      const Vec3 &p = point.state.position_enu_m;
      const Vec3 &velocity = point.state.velocity_enu_m_s;
      const double speed = Norm(velocity);
      const double stop_length =
          speed * config_.stopping_delay_s +
          speed * speed / (2.0 * config_.braking_acceleration_m_s2);
      const Vec3 stop_end =
          speed > 1e-9 ? Add(p, Scale(velocity, stop_length / speed)) : p;
      cloud_stop =
          std::min(cloud_stop, CloudSegmentClearance(p, stop_end, cloud));
      if (environment_) {
        stop_map_safe =
            stop_map_safe && environment_->SegmentSafe(
                                 p, stop_end, required_stop + map_expansion);
        map_stop = std::min(
            map_stop, MapClearanceLowerBound(*environment_, p, stop_end,
                                             config_.map_sample_spacing_m));
      }
    }
    const bool cloud_safe =
        cloud_present && cloud_collision > config_.collision_radius_m;
    const bool stopping_safe = cloud_stop > required_stop && stop_map_safe;
    SafetyResult result;
    result.minimum_collision_clearance_m =
        std::min(cloud_collision, map_collision);
    result.minimum_stopping_clearance_m = std::min(cloud_stop, map_stop);
    result.safe = cloud_safe && map_safe && stopping_safe;
    result.reason = result.safe   ? "clear_trajectory"
                    : !cloud_safe ? "swept_collision"
                    : !map_safe   ? "known_map_collision"
                                  : "predicted_stopping_clearance";
    return result;
  } catch (...) {
    return Invalid("invalid_collision_environment");
  }
}
} // namespace uav_navigation_core

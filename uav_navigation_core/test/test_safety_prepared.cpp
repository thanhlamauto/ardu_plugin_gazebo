#include "uav_navigation_core/trajectory_safety_checker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace core = uav_navigation_core;
namespace {
int failures = 0;
void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}
bool FiniteD(double v) { return std::isfinite(v); }
bool FiniteV(const core::Vec3 &v) {
  return FiniteD(v.x) && FiniteD(v.y) && FiniteD(v.z);
}
bool FiniteState(const core::State &s) {
  return FiniteV(s.position_enu_m) && FiniteV(s.velocity_enu_m_s) &&
         FiniteV(s.acceleration_enu_m_s2) && FiniteD(s.yaw_enu_rad);
}
bool FiniteControl(const core::Control &u) {
  return FiniteV(u.velocity_enu_m_s) && FiniteD(u.yaw_rate_enu_rad_s);
}
double Dot(core::Vec3 a, core::Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
core::Vec3 Sub(core::Vec3 a, core::Vec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
core::Vec3 Add(core::Vec3 a, core::Vec3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
core::Vec3 Scale(core::Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double Norm(core::Vec3 a) { return std::sqrt(Dot(a, a)); }
double Distance(core::Vec3 a, core::Vec3 b) { return Norm(Sub(a, b)); }
double PointSegmentDistance(core::Vec3 p, core::Vec3 a, core::Vec3 b) {
  const auto ab = Sub(b, a);
  const double den = std::max(Dot(ab, ab), 1e-12);
  const double t = std::clamp(Dot(Sub(p, a), ab) / den, 0.0, 1.0);
  return Distance(p, Add(a, Scale(ab, t)));
}
double CloudSegmentClearance(core::Vec3 a, core::Vec3 b,
                             const std::vector<core::Vec3> &cloud) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto &p : cloud)
    result = std::min(result, PointSegmentDistance(p, a, b));
  return result;
}
double MapClearanceLowerBound(const core::ICollisionEnvironment &env,
                              core::Vec3 a, core::Vec3 b, double spacing) {
  const auto ab = Sub(b, a);
  const double length = Norm(ab);
  const auto count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / spacing)));
  double result = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i <= count; ++i)
    result = std::min(
        result, env.Clearance(Add(a, Scale(ab, static_cast<double>(i) / count))));
  return result - length / (2.0 * count);
}

// Faithful reimplementation of the pre-M7.4 TrajectorySafetyChecker::Evaluate
// (linear cloud scan with per-trajectory preparation) used only as the
// benchmark baseline. It must agree with the checker on every input.
core::SafetyResult LegacyEvaluate(const core::State &initial_state,
                                  const core::Trajectory &trajectory,
                                  const core::ObstacleMap &obstacles,
                                  const core::TrajectorySafetyConfig &config,
                                  const core::ICollisionEnvironment *env) {
  const auto invalid = [](const char *reason) {
    return core::SafetyResult{false, -std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity(),
                              reason};
  };
  if (!FiniteState(initial_state) || trajectory.points.size() < 2)
    return invalid("invalid_trajectory");
  double previous_time = -std::numeric_limits<double>::infinity();
  for (const auto &point : trajectory.points) {
    if (!FiniteD(point.time_from_start_s) ||
        point.time_from_start_s < previous_time || !FiniteState(point.state) ||
        !FiniteControl(point.control))
      return invalid("invalid_trajectory");
    previous_time = point.time_from_start_s;
  }
  for (const auto &point : obstacles.points_enu_m)
    if (!FiniteV(point))
      return invalid("invalid_obstacle_map");
  const bool cloud_present =
      obstacles.observation_valid || !obstacles.points_enu_m.empty();
  double map_expansion = 0.0;
  std::vector<core::Vec3> cloud;
  cloud.reserve(obstacles.points_enu_m.size());
  for (const auto &point : obstacles.points_enu_m) {
    if (env) {
      const double residual = env->Clearance(point);
      if (!FiniteD(residual) || residual < 0.0)
        return invalid("invalid_collision_environment");
      if (residual <= config.cloud_map_tolerance_m) {
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
    if (env) {
      map_safe = map_safe && env->SegmentSafe(
                                 a, b, config.collision_radius_m + map_expansion);
      map_collision = std::min(
          map_collision, MapClearanceLowerBound(*env, a, b,
                                                config.map_sample_spacing_m));
    }
  }
  const double required_stop =
      config.stopping_clearance_m + config.stopping_uncertainty_m;
  double cloud_stop = std::numeric_limits<double>::infinity();
  double map_stop = std::numeric_limits<double>::infinity();
  bool stop_map_safe = true;
  for (const auto &point : trajectory.points) {
    const core::Vec3 &p = point.state.position_enu_m;
    const core::Vec3 &velocity = point.state.velocity_enu_m_s;
    const double speed = Norm(velocity);
    const double stop_length =
        speed * config.stopping_delay_s +
        speed * speed / (2.0 * config.braking_acceleration_m_s2);
    const core::Vec3 stop_end =
        speed > 1e-9 ? Add(p, Scale(velocity, stop_length / speed)) : p;
    cloud_stop = std::min(cloud_stop, CloudSegmentClearance(p, stop_end, cloud));
    if (env) {
      stop_map_safe =
          stop_map_safe && env->SegmentSafe(p, stop_end,
                                            required_stop + map_expansion);
      map_stop = std::min(
          map_stop, MapClearanceLowerBound(*env, p, stop_end,
                                           config.map_sample_spacing_m));
    }
  }
  const bool cloud_safe =
      cloud_present && cloud_collision > config.collision_radius_m;
  const bool stopping_safe = cloud_stop > required_stop && stop_map_safe;
  core::SafetyResult result;
  result.minimum_collision_clearance_m = std::min(cloud_collision, map_collision);
  result.minimum_stopping_clearance_m = std::min(cloud_stop, map_stop);
  result.safe = cloud_safe && map_safe && stopping_safe;
  result.reason = result.safe   ? "clear_trajectory"
                  : !cloud_safe ? "swept_collision"
                  : !map_safe   ? "known_map_collision"
                                : "predicted_stopping_clearance";
  return result;
}

struct Sphere {
  core::Vec3 center;
  double radius;
};
class SphereEnvironment final : public core::ICollisionEnvironment {
public:
  explicit SphereEnvironment(std::vector<Sphere> spheres)
      : spheres_(std::move(spheres)) {}
  double Clearance(const core::Vec3 &p) const override {
    double value = std::numeric_limits<double>::infinity();
    for (const auto &s : spheres_)
      value = std::min(value, std::max(Distance(p, s.center) - s.radius, 0.0));
    return value;
  }
  bool SegmentSafe(const core::Vec3 &a, const core::Vec3 &b,
                   double clearance) const override {
    for (const auto &s : spheres_)
      if (PointSegmentDistance(s.center, a, b) - s.radius <= clearance)
        return false;
    return true;
  }

private:
  std::vector<Sphere> spheres_;
};

core::ObstacleMap MakeCloud(int kind, std::size_t count) {
  core::ObstacleMap obstacles;
  obstacles.observation_valid = true;
  obstacles.points_enu_m.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double t = static_cast<double>(i);
    if (kind == 0) {
      obstacles.points_enu_m.push_back({20.0, -30.0 + 0.01 * t, 3.0});
    } else if (kind == 1) {
      if (i % 2 == 0)
        obstacles.points_enu_m.push_back({6.0 + 0.01 * t, 6.0, 3.0});
      else
        obstacles.points_enu_m.push_back({12.0, 1.0 + 0.01 * t, 3.0});
    } else {
      const double y = (i % 2 == 0) ? (5.0 + 0.02 * t) : (-5.0 - 0.02 * t);
      obstacles.points_enu_m.push_back({10.0 + 0.005 * (t - count / 2.0), y, 3.0});
    }
  }
  return obstacles;
}

std::vector<core::Trajectory> MakeTrajectories(std::size_t samples,
                                               std::size_t steps) {
  std::vector<core::Trajectory> batch;
  batch.reserve(samples);
  for (std::size_t s = 0; s < samples; ++s) {
    core::Trajectory trajectory;
    trajectory.points.reserve(steps + 1);
    const double heading = 0.15 * (static_cast<double>(s) / samples - 0.5);
    const double speed = 4.0 + 2.0 * (static_cast<double>(s) / samples);
    for (std::size_t t = 0; t <= steps; ++t) {
      core::TrajectoryPoint point;
      point.time_from_start_s = 0.1 * static_cast<double>(t);
      const double x = speed * 0.1 * static_cast<double>(t);
      point.state.position_enu_m = {x, x * std::tan(heading), 5.0};
      point.state.velocity_enu_m_s = {speed, speed * std::tan(heading), 0.0};
      point.control.velocity_enu_m_s = point.state.velocity_enu_m_s;
      trajectory.points.push_back(point);
    }
    batch.push_back(std::move(trajectory));
  }
  return batch;
}

std::vector<Sphere> MakeSpheres(int kind, std::size_t count) {
  std::vector<Sphere> spheres;
  spheres.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double t = static_cast<double>(i);
    if (kind == 0)
      spheres.push_back({{20.0, -30.0 + 0.5 * t, 3.0}, 0.4});
    else if (kind == 1)
      spheres.push_back({{6.0 + 0.1 * t, 6.0, 3.0}, 0.5});
    else
      spheres.push_back({{10.0, 5.0 + 0.1 * t, 3.0}, 0.5});
  }
  return spheres;
}

void ParityAndBenchmark(int kind, const std::string &label,
                        std::size_t cloud_points, std::size_t samples,
                        std::size_t steps) {
  core::TrajectorySafetyConfig config;
  config.collision_radius_m = 1.5;
  config.braking_acceleration_m_s2 = 3.0;
  config.stopping_delay_s = 0.25;
  config.stopping_clearance_m = 1.5;
  config.cloud_map_tolerance_m = 0.1;
  auto environment = std::make_shared<SphereEnvironment>(MakeSpheres(kind, 80));
  core::TrajectorySafetyChecker checker(config, environment);
  const auto obstacles = MakeCloud(kind, cloud_points);
  const auto trajectories = MakeTrajectories(samples, steps);
  const core::State initial = trajectories.front().points.front().state;

  // Pre-M7.4 algorithm: linear cloud scan, preparation repeated per trajectory.
  std::vector<core::SafetyResult> legacy;
  legacy.reserve(trajectories.size());
  double legacy_ms = std::numeric_limits<double>::infinity();
  for (int repeat = 0; repeat < 3; ++repeat) {
    legacy.clear();
    const auto started = std::chrono::steady_clock::now();
    for (const auto &trajectory : trajectories)
      legacy.push_back(LegacyEvaluate(initial, trajectory, obstacles, config,
                                      environment.get()));
    legacy_ms = std::min(
        legacy_ms,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started)
            .count());
  }

  // M7.4: prepare once (filter + exact spatial index), evaluate every trajectory.
  const auto prepared = checker.Prepare(obstacles);
  std::vector<core::SafetyResult> upgraded;
  upgraded.reserve(trajectories.size());
  double prepared_ms = std::numeric_limits<double>::infinity();
  for (int repeat = 0; repeat < 3; ++repeat) {
    upgraded.clear();
    const auto started = std::chrono::steady_clock::now();
    for (const auto &trajectory : trajectories)
      upgraded.push_back(checker.Evaluate(initial, trajectory, prepared));
    prepared_ms = std::min(
        prepared_ms,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started)
            .count());
  }
  double prepare_ms = std::numeric_limits<double>::infinity();
  for (int repeat = 0; repeat < 3; ++repeat) {
    const auto started = std::chrono::steady_clock::now();
    const auto candidate = checker.Prepare(obstacles);
    prepare_ms = std::min(
        prepare_ms,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started)
            .count());
    Check(candidate.cloud.size() == prepared.cloud.size(),
          label + " repeated preparation is stable");
  }
  const double prepared_total_ms = prepared_ms + prepare_ms;

  for (std::size_t i = 0; i < trajectories.size(); ++i) {
    Check(legacy[i].safe == upgraded[i].safe,
          label + " safe parity at sample " + std::to_string(i));
    Check(legacy[i].reason == upgraded[i].reason,
          label + " reason parity at sample " + std::to_string(i));
    Check(legacy[i].minimum_collision_clearance_m ==
              upgraded[i].minimum_collision_clearance_m,
          label + " collision clearance parity at sample " + std::to_string(i));
    Check(legacy[i].minimum_stopping_clearance_m ==
              upgraded[i].minimum_stopping_clearance_m,
          label + " stopping clearance parity at sample " + std::to_string(i));
  }
  const auto legacy_safe =
      std::count_if(legacy.begin(), legacy.end(),
                    [](const core::SafetyResult &r) { return r.safe; });
  const auto upgraded_safe =
      std::count_if(upgraded.begin(), upgraded.end(),
                    [](const core::SafetyResult &r) { return r.safe; });
  Check(legacy_safe == upgraded_safe, label + " safe sample count parity");

  std::cout << label << ": old=" << legacy_ms << " ms  new="
            << prepared_total_ms << " ms (prepare " << prepare_ms
            << " + eval " << prepared_ms << ")  speedup="
            << (prepared_total_ms > 0.0 ? legacy_ms / prepared_total_ms : 0.0)
            << "x  safe_samples=" << upgraded_safe << "/" << upgraded.size()
            << '\n';
  // Timing is reported for the release benchmark; only a gross regression is a
  // hard failure because debug-build timings are dominated by KD-tree overhead.
  Check(prepared_total_ms <= legacy_ms * 2.5,
        label + " new evaluation did not regress grossly");
}

void InvalidInputParity() {
  core::TrajectorySafetyConfig config;
  core::TrajectorySafetyChecker checker(config);
  core::ObstacleMap obstacles;
  obstacles.points_enu_m.push_back({0.0, 0.0, 0.0});
  auto trajectory = MakeTrajectories(1, 3).front();
  auto prepared = checker.Prepare(obstacles);
  const auto legacy = checker.Evaluate(trajectory.points.front().state,
                                       trajectory, obstacles);
  const auto upgraded = checker.Evaluate(trajectory.points.front().state,
                                         trajectory, prepared);
  Check(legacy.safe == upgraded.safe && legacy.reason == upgraded.reason,
        "valid obstacle parity");

  auto invalid_cloud = obstacles;
  invalid_cloud.points_enu_m[0].z = std::numeric_limits<double>::infinity();
  const auto invalid_prepared = checker.Prepare(invalid_cloud);
  Check(!invalid_prepared.valid &&
            invalid_prepared.invalid_reason == "invalid_obstacle_map",
        "invalid obstacle is detected once at prepare time");
  Check(checker.Evaluate(trajectory.points.front().state, trajectory,
                         invalid_prepared)
                .reason == "invalid_obstacle_map",
        "prepared invalid obstacle fails closed");

  auto one_point = trajectory;
  one_point.points.resize(1);
  Check(checker.Evaluate(one_point.points.front().state, one_point,
                         invalid_prepared)
                .reason == "invalid_trajectory",
        "invalid trajectory keeps precedence in the prepared path");
}
} // namespace

int main() {
  ParityAndBenchmark(0, "open_space", 400, 80, 30);
  ParityAndBenchmark(1, "s03_turn", 800, 80, 30);
  ParityAndBenchmark(2, "s05_gate", 800, 80, 30);
  InvalidInputParity();
  return failures == 0 ? 0 : 1;
}

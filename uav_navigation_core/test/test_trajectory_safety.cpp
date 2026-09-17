#include "uav_navigation_core/trajectory_safety_checker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "json_fixture.hpp"

namespace core = uav_navigation_core;
namespace {
int failures = 0;
void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}
double Dot(core::Vec3 a, core::Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
core::Vec3 Sub(core::Vec3 a, core::Vec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
double Distance(core::Vec3 a, core::Vec3 b) {
  auto d = Sub(a, b);
  return std::sqrt(Dot(d, d));
}
double SegmentDistance(core::Vec3 p, core::Vec3 a, core::Vec3 b) {
  auto ab = Sub(b, a);
  double den = std::max(Dot(ab, ab), 1e-12);
  double t = std::clamp(Dot(Sub(p, a), ab) / den, 0.0, 1.0);
  return Distance(p, {a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z});
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
      if (SegmentDistance(s.center, a, b) - s.radius <= clearance)
        return false;
    return true;
  }

private:
  std::vector<Sphere> spheres_;
};
struct Case {
  core::TrajectorySafetyChecker checker;
  core::State initial;
  core::Trajectory trajectory;
  core::ObstacleMap obstacles;
};
Case Load(const std::string &file) {
  auto json = ReadFixture(file);
  core::TrajectorySafetyConfig config;
  config.collision_radius_m = Number(json, "collision_radius");
  config.braking_acceleration_m_s2 = Number(json, "acceleration");
  config.stopping_delay_s = Number(json, "delay");
  config.stopping_clearance_m = Number(json, "stopping_clearance");
  config.stopping_uncertainty_m = Number(json, "uncertainty");
  config.map_sample_spacing_m = Number(json, "sample_spacing");
  config.cloud_map_tolerance_m = Number(json, "cloud_map_tolerance");
  std::vector<Sphere> spheres;
  auto sphere_values = Numbers(json, "spheres_flat");
  for (std::size_t i = 0; i < sphere_values.size(); i += 4)
    spheres.push_back(
        {{sphere_values[i], sphere_values[i + 1], sphere_values[i + 2]},
         sphere_values[i + 3]});
  std::shared_ptr<const core::ICollisionEnvironment> environment;
  if (!spheres.empty())
    environment = std::make_shared<SphereEnvironment>(spheres);
  Case value{core::TrajectorySafetyChecker(config, environment), {}, {}, {}};
  auto states = Numbers(json, "states_flat");
  for (std::size_t i = 0; i < states.size(); i += 6) {
    core::TrajectoryPoint p;
    p.time_from_start_s = static_cast<double>(i / 6) * .1;
    p.state.position_enu_m = {states[i], states[i + 1], states[i + 2]};
    p.state.velocity_enu_m_s = {states[i + 3], states[i + 4], states[i + 5]};
    value.trajectory.points.push_back(p);
  }
  value.initial = value.trajectory.points.front().state;
  auto cloud = Numbers(json, "cloud_flat");
  for (std::size_t i = 0; i < cloud.size(); i += 3)
    value.obstacles.points_enu_m.push_back(
        {cloud[i], cloud[i + 1], cloud[i + 2]});
  return value;
}
void Golden(const std::string &file) {
  auto json = ReadFixture(file);
  auto value = Load(file);
  auto result =
      value.checker.Evaluate(value.initial, value.trajectory, value.obstacles);
  Check(result.safe == Boolean(json, "expected_safe"),
        file + " classification parity");
  Check(result.reason == String(json, "expected_reason"),
        file + " reason parity");
  if (Boolean(json, "expected_collision_finite"))
    Check(std::abs(result.minimum_collision_clearance_m -
                   Number(json, "expected_collision_clearance")) < 1e-9,
          file + " collision clearance parity");
  else
    Check(!std::isfinite(result.minimum_collision_clearance_m),
          file + " non-finite collision clearance parity");
  if (Boolean(json, "expected_stopping_finite"))
    Check(std::abs(result.minimum_stopping_clearance_m -
                   Number(json, "expected_stopping_clearance")) < 1e-9,
          file + " stopping clearance parity");
}
void EdgeCases() {
  auto base = Load("safety_clear.json");
  auto one = base.trajectory;
  one.points.resize(1);
  Check(base.checker.Evaluate(base.initial, one, base.obstacles).reason ==
            "invalid_trajectory",
        "one-point trajectory invalid");
  auto nonfinite = base.trajectory;
  nonfinite.points[0].state.position_enu_m.x =
      std::numeric_limits<double>::quiet_NaN();
  Check(base.checker.Evaluate(base.initial, nonfinite, base.obstacles).reason ==
            "invalid_trajectory",
        "non-finite trajectory invalid");
  auto invalid_cloud = base.obstacles;
  invalid_cloud.points_enu_m[0].z = std::numeric_limits<double>::infinity();
  Check(base.checker.Evaluate(base.initial, base.trajectory, invalid_cloud)
                .reason == "invalid_obstacle_map",
        "non-finite obstacle cloud invalid");
  auto zero = base.trajectory;
  for (auto &p : zero.points)
    p.state.velocity_enu_m_s = {};
  Check(base.checker.Evaluate(base.initial, zero, base.obstacles).safe,
        "zero velocity is valid and safe");
  auto low = Load("safety_stopping.json");
  for (auto &p : low.trajectory.points)
    p.state.velocity_enu_m_s = {1, 0, 0};
  Check(low.checker.Evaluate(low.initial, low.trajectory, low.obstacles).safe,
        "low speed can stop before obstacle");
  auto long_segment = Load("safety_collision.json");
  long_segment.trajectory.points[1].state.position_enu_m.x = 100;
  long_segment.obstacles.points_enu_m[0].x = 50;
  Check(SegmentDistance(
            long_segment.obstacles.points_enu_m[0],
            long_segment.trajectory.points[0].state.position_enu_m,
            long_segment.trajectory.points[0].state.position_enu_m) > 1.0 &&
            SegmentDistance(
                long_segment.obstacles.points_enu_m[0],
                long_segment.trajectory.points[1].state.position_enu_m,
                long_segment.trajectory.points[1].state.position_enu_m) > 1.0,
        "long-segment endpoints are individually clear");
  Check(!long_segment.checker
             .Evaluate(long_segment.initial, long_segment.trajectory,
                       long_segment.obstacles)
             .safe,
        "endpoint-clear long segment cannot tunnel through obstacle");

  bool threw = false;
  core::TrajectorySafetyConfig invalid_config;
  invalid_config.braking_acceleration_m_s2 = 0.0;
  try {
    core::TrajectorySafetyChecker invalid(invalid_config);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "invalid safety configuration is rejected");
}
} // namespace
int main() {
  for (const auto &name :
       {"safety_clear.json", "safety_collision.json", "safety_stopping.json",
        "safety_static_collision.json", "safety_overlap.json",
        "safety_empty_cloud.json"})
    Golden(name);
  EdgeCases();
  return failures == 0 ? 0 : 1;
}

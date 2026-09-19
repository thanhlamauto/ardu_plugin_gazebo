#include "uav_navigation_core/stabilization.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "uav_navigation_core/mppi/cost_evaluator.hpp"
#include "uav_navigation_core/mppi/path_reference.hpp"

namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;
namespace {
int failures = 0;
void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

core::SpeedShapingConfig ShapingConfig() {
  core::SpeedShapingConfig config;
  config.enabled = true;
  config.nominal_speed_m_s = 10.0;
  config.turn_speed_m_s = 3.0;
  config.min_speed_m_s = 0.5;
  config.lateral_accel_m_s2 = 2.0;
  config.braking_accel_m_s2 = 3.0;
  config.reaction_delay_s = 0.25;
  config.turn_angle_rad = 0.35;
  config.lookahead_m = 2.0;
  return config;
}

mppi::PathReference StraightPath() {
  return mppi::PathReference({{0.0, 0.0, 5.0}, {25.0, 0.0, 5.0},
                              {50.0, 0.0, 5.0}});
}

mppi::PathReference CornerPath() {
  return mppi::PathReference({{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0},
                              {9.0, 0.0, 5.0}, {10.0, 0.0, 5.0},
                              {10.0, 5.0, 5.0}, {10.0, 10.0, 5.0}});
}

void SpeedShaping() {
  const auto config = ShapingConfig();
  auto disabled = config;
  disabled.enabled = false;
  const auto straight = StraightPath();
  const auto corner = CornerPath();

  Check(!std::isfinite(core::ComputeReferenceSpeedCap(
              disabled, corner, {9.0, 0.0, 5.0}, {8.0, 0.0, 0.0})),
        "disabled shaping returns an unbounded cap");

  const double straight_cap = core::ComputeReferenceSpeedCap(
      config, straight, {10.0, 0.0, 5.0}, {5.0, 0.0, 0.0});
  Check(std::abs(straight_cap - config.nominal_speed_m_s) < 1e-9,
        "straight open path is not speed limited");

  const double corner_cap = core::ComputeReferenceSpeedCap(
      config, corner, {9.0, 0.0, 5.0}, {8.0, 0.0, 0.0});
  Check(corner_cap <= config.turn_speed_m_s + 1e-9,
        "sharp turn approach reduces the cap to the turn speed");
  Check(corner_cap >= config.min_speed_m_s - 1e-9,
        "speed cap never drops below the configured minimum");

  const double far_cap = core::ComputeReferenceSpeedCap(
      config, corner, {7.0, 0.0, 5.0}, {2.0, 0.0, 0.0});
  Check(far_cap > config.turn_speed_m_s,
        "slow approach far from the turn stays above the turn speed");

  // Cap is monotonic non-increasing as the turn gets closer.
  double previous = config.nominal_speed_m_s + 1.0;
  for (double x = 0.0; x <= 9.0; x += 1.0) {
    const double cap = core::ComputeReferenceSpeedCap(
        config, corner, {x, 0.0, 5.0}, {8.0, 0.0, 0.0});
    Check(cap <= previous + 1e-9, "shaping cap is non-increasing toward the turn");
    Check(cap <= config.nominal_speed_m_s + 1e-9,
          "shaping never exceeds the nominal speed");
    previous = cap;
  }
}

void BrakingSequence() {
  const core::Vec3 velocity{3.0, 4.0, 0.0};  // 5 m/s
  const auto sequence = core::BuildBrakingSequence(velocity, 3.0, 30, 0.1);
  Check(sequence.size() == 30, "braking sequence has the requested horizon");
  double previous = std::hypot(velocity.x, velocity.y);
  for (const auto &control : sequence) {
    const double speed = std::hypot(control.velocity_enu_m_s.x,
                                    control.velocity_enu_m_s.y);
    Check(speed <= previous + 1e-9, "braking command speed is non-increasing");
    Check(speed <= std::hypot(velocity.x, velocity.y) + 1e-9,
          "braking never exceeds the measured speed");
    Check(control.yaw_rate_enu_rad_s == 0.0, "braking does not command yaw rate");
    if (speed > 1e-9) {
      const double direction = std::atan2(control.velocity_enu_m_s.y,
                                          control.velocity_enu_m_s.x);
      Check(std::abs(direction - std::atan2(velocity.y, velocity.x)) < 1e-9,
            "braking preserves the velocity direction");
    }
    previous = speed;
  }
  Check(sequence.back().velocity_enu_m_s.x == 0.0 &&
            sequence.back().velocity_enu_m_s.y == 0.0,
        "long-enough braking ramps to zero speed");
  Check(core::BuildBrakingSequence({0.0, 0.0, 0.0}, 3.0, 30, 0.1).empty(),
        "stopped vehicle has no braking sequence");
  Check(core::BuildBrakingSequence(velocity, 0.0, 30, 0.1).empty(),
        "non-positive deceleration has no braking sequence");
}

void StoppingDominance() {
  mppi::RejectionCounts counts;
  counts.total = 80;
  counts.reject_stopping_distance = 60;
  counts.reject_swept_collision = 20;
  Check(core::StoppingDominant(counts),
        "stopping dominance detected when stopping is the largest share");

  counts.reject_swept_collision = 70;
  Check(!core::StoppingDominant(counts),
        "stopping dominance rejected when swept collision dominates");

  mppi::RejectionCounts none;
  none.total = 80;
  none.reject_swept_collision = 80;
  Check(!core::StoppingDominant(none),
        "no stopping rejects is not stopping dominant");

  mppi::RejectionCounts tie;
  tie.total = 80;
  tie.reject_stopping_distance = 40;
  tie.reject_swept_collision = 40;
  Check(core::StoppingDominant(tie),
        "ties count as stopping dominant (deterministic first-failure policy)");
}

void RecoveryTrackerTransitions() {
  core::StoppingRecoveryTracker tracker;
  tracker.Configure(true);
  Check(!tracker.active(), "recovery starts inactive");
  tracker.RecordOtherCycle();
  Check(tracker.exits() == 0, "inactive other cycle is not an exit");

  tracker.RecordRecoveryCycle();
  tracker.RecordRecoveryCycle();
  Check(tracker.active(), "recovery becomes active");
  Check(tracker.entries() == 1, "recovery entry counted once");
  Check(tracker.cycles() == 2, "recovery cycles counted");
  Check(tracker.episode_cycles() == 2, "episode duration counted");

  tracker.RecordOtherCycle();
  Check(!tracker.active(), "recovery ends when another cycle occurs");
  Check(tracker.exits() == 1, "recovery exit counted");
  Check(tracker.episode_cycles() == 0, "episode duration resets after exit");

  tracker.RecordRecoveryCycle();
  Check(tracker.entries() == 2 && tracker.exits() == 1,
        "second episode counted independently");

  core::StoppingRecoveryTracker disabled;
  disabled.Configure(false);
  disabled.RecordRecoveryCycle();
  Check(!disabled.active() && disabled.entries() == 0,
        "disabled tracker never records recovery");
}

void SpeedLimitContext() {
  mppi::MppiCostConfig config;
  config.path_progress_objective = true;
  config.w_speed_limit = 10000.0;
  config.vmax_m_s = 10.0;
  config.w_path = 0.0;
  config.w_progress = 0.0;
  config.w_effort = 0.0;
  config.w_smoothness = 0.0;
  config.w_yaw = 0.0;
  config.w_terminal = 0.0;
  const mppi::CostEvaluator evaluator(config);
  mppi::MppiTrajectory trajectory;
  for (std::size_t t = 0; t <= 2; ++t) {
    mppi::MppiTrajectoryPoint point;
    point.time_from_start_s = 0.1 * static_cast<double>(t);
    point.state.velocity_enu_m_s = {8.0, 0.0, 0.0};
    point.state.applied_control.velocity_enu_m_s = {8.0, 0.0, 0.0};
    point.requested_control.velocity_enu_m_s = {8.0, 0.0, 0.0};
    trajectory.points.push_back(point);
  }
  const mppi::ControlSequence actions(2, {{8.0, 0.0, 0.0}, 0.0});
  mppi::CostContext context;
  const auto unlimited = evaluator.EvaluateTrajectory(trajectory, actions, context);
  mppi::CostContext limited;
  limited.reference_speed_limit_m_s = 2.0;
  const auto capped = evaluator.EvaluateTrajectory(trajectory, actions, limited);
  Check(unlimited.speed_limit == 0.0,
        "high speed is free when the cap is the configured vmax");
  Check(capped.speed_limit > 0.0,
        "lowering the reference cap penalizes excess speed");
  Check(capped.Total() > unlimited.Total(),
        "reference cap changes the cost used for selection");

  mppi::CostContext invalid;
  invalid.reference_speed_limit_m_s = -1.0;
  bool threw = false;
  try {
    evaluator.EvaluateTrajectory(trajectory, actions, invalid);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "non-positive reference speed cap is rejected");
}
} // namespace

int main() {
  SpeedShaping();
  BrakingSequence();
  StoppingDominance();
  RecoveryTrackerTransitions();
  SpeedLimitContext();
  return failures == 0 ? 0 : 1;
}

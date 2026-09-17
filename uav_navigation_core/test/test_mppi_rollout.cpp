#include "uav_navigation_core/mppi/rollout.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mppi_fixture.hpp"

namespace mppi = uav_navigation_core::mppi;
namespace {
int failures = 0;

void Check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

template <typename Function>
void CheckInvalid(Function function, const std::string &message) {
  bool threw = false;
  try {
    function();
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, message);
}

void CheckGolden(const std::string &name) {
  const auto fixture = mppi_test::Load(name);
  const mppi::MultirotorMotionModel model(fixture.config);
  const auto actual =
      mppi::Rollout(model, fixture.initial, fixture.controls, fixture.dt_s);
  Check(actual.points.size() == fixture.controls.size() + 1,
        name + " has N+1 rollout points");
  Check(actual.points.size() == fixture.expected_times.size(),
        name + " point count matches Python");
  for (std::size_t point = 0; point < actual.points.size(); ++point) {
    const auto flat = mppi_test::Flatten(actual.points[point].state,
                                         fixture.response_accel_model);
    for (std::size_t component = 0; component < fixture.state_width;
         ++component) {
      const double expected =
          fixture.expected_states[point * fixture.state_width + component];
      Check(std::abs(flat[component] - expected) < 1e-9,
            name + " Python parity at point " + std::to_string(point) +
                ", component " + std::to_string(component));
    }
    Check(std::abs(actual.points[point].time_from_start_s -
                   fixture.expected_times[point]) < 1e-12,
          name + " rollout time parity");
    Check(actual.points[point].state.stamp_ns ==
              static_cast<uav_navigation_core::TimeNs>(
                  fixture.expected_stamps[point]),
          name + " rollout timestamp parity");
  }
}

void GoldenRollouts() {
  const std::vector<std::string> names{
      "hover",          "forward",  "lateral",       "climb",
      "descend",        "yaw",      "mixed_control", "long_horizon",
      "control_bounds", "large_dt", "small_dt",      "single_step"};
  for (const auto &name : names)
    CheckGolden(name);
}

void EdgeCasesAndBatch() {
  const auto fixture = mppi_test::Load("mixed_control");
  const mppi::MultirotorMotionModel model(fixture.config);
  const auto empty = mppi::Rollout(model, fixture.initial, {}, fixture.dt_s);
  Check(empty.points.size() == 1,
        "zero-horizon rollout contains only its initial state");
  Check(empty.points.front().state.stamp_ns == fixture.initial.stamp_ns,
        "zero-horizon rollout preserves the initial state");

  mppi::ControlBatch batch{fixture.controls, {}, {fixture.controls.front()}};
  const auto trajectories =
      mppi::RolloutBatch(model, fixture.initial, batch, fixture.dt_s);
  Check(trajectories.size() == 3, "batch returns one trajectory per sequence");
  Check(trajectories[0].points.size() == fixture.controls.size() + 1,
        "batch preserves full sequence length");
  Check(trajectories[1].points.size() == 1,
        "batch supports an empty control sequence");
  Check(trajectories[2].points.size() == 2,
        "batch supports a single-step sequence");
  Check(mppi::RolloutBatch(model, fixture.initial, {}, fixture.dt_s).empty(),
        "empty batch returns no trajectories");
  CheckInvalid([&] { mppi::RolloutBatch(model, fixture.initial, {}, 0.0); },
               "empty batch still rejects zero dt");

  CheckInvalid([&] { mppi::Rollout(model, fixture.initial, {}, 0.0); },
               "rollout rejects zero dt even with zero horizon");
  auto invalid = fixture.initial;
  invalid.velocity_enu_m_s.z = std::numeric_limits<double>::quiet_NaN();
  CheckInvalid([&] { mppi::Rollout(model, invalid, {}, fixture.dt_s); },
               "zero-horizon rollout rejects non-finite initial state");
}
} // namespace

int main() {
  GoldenRollouts();
  EdgeCasesAndBatch();
  return failures == 0 ? 0 : 1;
}

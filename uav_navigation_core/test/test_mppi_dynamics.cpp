#include "uav_navigation_core/mppi/dynamics.hpp"

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

template <typename Exception, typename Function>
void CheckThrows(Function function, const std::string &message) {
  bool threw = false;
  try {
    function();
  } catch (const Exception &) {
    threw = true;
  }
  Check(threw, message);
}

void GoldenFirstSteps() {
  const std::vector<std::string> names{
      "hover",          "forward",  "lateral",       "climb",
      "descend",        "yaw",      "mixed_control", "long_horizon",
      "control_bounds", "large_dt", "small_dt",      "single_step"};
  for (const auto &name : names) {
    const auto fixture = mppi_test::Load(name);
    const mppi::MultirotorMotionModel model(fixture.config);
    const auto actual =
        model.Step(fixture.initial, fixture.controls.front(), fixture.dt_s);
    const auto flat = mppi_test::Flatten(actual, fixture.response_accel_model);
    for (std::size_t index = 0; index < fixture.state_width; ++index) {
      const double expected =
          fixture.expected_states[fixture.state_width + index];
      Check(std::abs(flat[index] - expected) < 1e-10,
            name + " first-step Python parity at component " +
                std::to_string(index));
    }
    Check(actual.stamp_ns == static_cast<uav_navigation_core::TimeNs>(
                                 fixture.expected_stamps.at(1)),
          name + " timestamp parity");
  }
}

void ValidationAndSemantics() {
  mppi::MppiDynamicsConfig invalid;
  invalid.tau_s = 0.0;
  CheckThrows<std::invalid_argument>(
      [&] { mppi::MultirotorMotionModel model(invalid); },
      "zero response time is rejected");
  invalid = {};
  invalid.command_alpha = 1.01;
  CheckThrows<std::invalid_argument>(
      [&] { mppi::MultirotorMotionModel model(invalid); },
      "command alpha above one is rejected");

  const mppi::MultirotorMotionModel model;
  mppi::MppiState state;
  uav_navigation_core::Control control;
  CheckThrows<std::invalid_argument>([&] { model.Step(state, control, 0.0); },
                                     "zero dt is rejected");
  CheckThrows<std::invalid_argument>(
      [&] {
        model.Step(state, control, std::numeric_limits<double>::infinity());
      },
      "non-finite dt is rejected");
  state.position_enu_m.x = std::numeric_limits<double>::quiet_NaN();
  CheckThrows<std::invalid_argument>([&] { model.Step(state, control, 0.1); },
                                     "non-finite state is rejected");
  state = {};
  control.velocity_enu_m_s.y = std::numeric_limits<double>::infinity();
  CheckThrows<std::invalid_argument>([&] { model.Step(state, control, 0.1); },
                                     "non-finite control is rejected");
  control = {};
  state.stamp_ns = std::numeric_limits<uav_navigation_core::TimeNs>::max();
  CheckThrows<std::overflow_error>([&] { model.Step(state, control, 0.1); },
                                   "timestamp overflow is rejected");

  state = {};
  state.yaw_enu_rad = 3.2;
  control.yaw_rate_enu_rad_s = 0.6;
  const auto next = model.Step(state, control, 1.0);
  Check(next.yaw_enu_rad > 3.2,
        "yaw integration preserves Python unwrapped-angle semantics");
}
} // namespace

int main() {
  GoldenFirstSteps();
  ValidationAndSemantics();
  return failures == 0 ? 0 : 1;
}

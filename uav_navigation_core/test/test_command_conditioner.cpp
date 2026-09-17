#include "uav_navigation_core/command_conditioner.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

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
core::Control Make(const std::vector<double> &v, std::size_t i) {
  return {{v[i], v[i + 1], v[i + 2]}, v[i + 3]};
}
void GoldenSequence() {
  const auto json = ReadFixture("conditioner_sequence.json");
  core::VelocityCommandConditionerConfig config;
  config.dt_s = Number(json, "dt");
  config.alpha = Number(json, "alpha");
  config.max_accel_xy_m_s2 = Number(json, "max_accel_xy");
  config.max_accel_z_m_s2 = Number(json, "max_accel_z");
  config.max_yaw_accel_rad_s2 = Number(json, "max_yaw_accel");
  const auto minimum = Numbers(json, "u_min"), maximum = Numbers(json, "u_max");
  config.minimum = {{minimum[0], minimum[1], minimum[2], minimum[3]}};
  config.maximum = {{maximum[0], maximum[1], maximum[2], maximum[3]}};
  core::VelocityCommandConditioner conditioner(config);
  core::State measured;
  const auto velocity = Numbers(json, "measured_velocity");
  measured.velocity_enu_m_s = {velocity[0], velocity[1], velocity[2]};
  const auto requests = Numbers(json, "requests_flat"),
             expected = Numbers(json, "expected_flat");
  for (std::size_t i = 0; i < requests.size(); i += 4) {
    const auto actual = conditioner.Apply(measured, Make(requests, i));
    const auto values = std::vector<double>{
        actual.velocity_enu_m_s.x, actual.velocity_enu_m_s.y,
        actual.velocity_enu_m_s.z, actual.yaw_rate_enu_rad_s};
    for (std::size_t j = 0; j < 4; ++j)
      Check(std::abs(values[j] - expected[i + j]) < 1e-12,
            "Python golden sequence parity");
  }
  const auto before_reset = *conditioner.previous();
  conditioner.Reset();
  Check(!conditioner.previous(), "Reset clears command memory");
  const auto after_reset = conditioner.Apply(measured, Make(requests, 0));
  Check(std::abs(after_reset.velocity_enu_m_s.x - expected[0]) < 1e-12,
        "Reset reinitializes from measured velocity");
  Check(std::abs(before_reset.velocity_enu_m_s.x -
                 after_reset.velocity_enu_m_s.x) > 1e-6,
        "Reset changes sequence state");
}
void Validation() {
  core::VelocityCommandConditioner conditioner;
  core::State state;
  core::Control bad;
  bad.velocity_enu_m_s.x = std::numeric_limits<double>::quiet_NaN();
  bool threw = false;
  try {
    conditioner.Apply(state, bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "non-finite requested command is rejected");
  state.velocity_enu_m_s.x = std::numeric_limits<double>::infinity();
  threw = false;
  try {
    conditioner.Apply(state, {});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "non-finite initial measured velocity is rejected");
  core::VelocityCommandConditionerConfig invalid;
  invalid.dt_s = 0;
  threw = false;
  try {
    core::VelocityCommandConditioner value(invalid);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  Check(threw, "invalid configuration is rejected");

  core::VelocityCommandConditionerConfig bounded;
  bounded.alpha = 1.0;
  bounded.max_accel_xy_m_s2 = 1000;
  bounded.max_accel_z_m_s2 = 1000;
  bounded.max_yaw_accel_rad_s2 = 1000;
  bounded.minimum = {{-2, -2, -.5, -.4}};
  bounded.maximum = {{2, 2, .5, .4}};
  core::VelocityCommandConditioner bounded_conditioner(bounded);
  const auto limited = bounded_conditioner.Apply({}, {{10, -10, 10}, 10});
  Check(limited.velocity_enu_m_s.x == 2 && limited.velocity_enu_m_s.y == -2 &&
            limited.velocity_enu_m_s.z == .5 &&
            limited.yaw_rate_enu_rad_s == .4,
        "configured command bounds clamp every component");
}
} // namespace
int main() {
  GoldenSequence();
  Validation();
  return failures == 0 ? 0 : 1;
}

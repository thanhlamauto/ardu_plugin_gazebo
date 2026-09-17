#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "json_fixture.hpp"
#include "uav_navigation_core/mppi/mppi_config.hpp"
#include "uav_navigation_core/mppi/mppi_types.hpp"

namespace mppi_test {
namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;

struct Fixture {
  double dt_s{0.0};
  bool response_accel_model{false};
  std::size_t state_width{0};
  mppi::MppiDynamicsConfig config{};
  mppi::MppiState initial{};
  mppi::ControlSequence controls{};
  std::vector<double> expected_states{};
  std::vector<double> expected_times{};
  std::vector<double> expected_stamps{};
};

inline core::Control ControlAt(const std::vector<double> &values,
                               std::size_t offset) {
  return {{values.at(offset), values.at(offset + 1), values.at(offset + 2)},
          values.at(offset + 3)};
}

inline mppi::MppiState StateAt(const std::vector<double> &values,
                               std::size_t offset, std::size_t width,
                               core::TimeNs stamp_ns) {
  mppi::MppiState state;
  state.stamp_ns = stamp_ns;
  state.position_enu_m = {values.at(offset), values.at(offset + 1),
                          values.at(offset + 2)};
  state.velocity_enu_m_s = {values.at(offset + 3), values.at(offset + 4),
                            values.at(offset + 5)};
  state.yaw_enu_rad = values.at(offset + 6);
  state.applied_control = ControlAt(values, offset + 7);
  if (width == 14) {
    state.acceleration_memory_enu_m_s2 = {
        values.at(offset + 11), values.at(offset + 12), values.at(offset + 13)};
  }
  return state;
}

inline Fixture Load(const std::string &name) {
  const std::string json = ReadFixture("mppi/" + name + ".json");
  Fixture result;
  result.dt_s = Number(json, "dt");
  result.response_accel_model = Boolean(json, "response_accel_model");
  result.state_width = static_cast<std::size_t>(Number(json, "state_width"));
  result.config.tau_s = Number(json, "tau");
  result.config.command_alpha = Number(json, "command_alpha");
  result.config.max_accel_xy_m_s2 = Number(json, "max_accel_xy");
  result.config.max_accel_z_m_s2 = Number(json, "max_accel_z");
  result.config.max_yaw_accel_rad_s2 = Number(json, "max_yaw_accel");
  result.config.vmax_m_s = Number(json, "vmax");
  result.config.vzmax_m_s = Number(json, "vzmax");
  result.config.yaw_rate_max_rad_s = Number(json, "yaw_rate_max");
  result.config.response_accel_model = result.response_accel_model;
  result.config.response_accel_xy_m_s2 = Number(json, "response_accel_xy");
  result.config.response_jerk_xy_m_s3 = Number(json, "response_jerk_xy");
  const auto initial_values = Numbers(json, "initial_state_flat");
  const auto initial_stamp =
      static_cast<core::TimeNs>(Number(json, "initial_stamp_ns"));
  result.initial =
      StateAt(initial_values, 0, result.state_width, initial_stamp);
  const auto controls = Numbers(json, "controls_flat");
  for (std::size_t offset = 0; offset < controls.size(); offset += 4) {
    result.controls.push_back(ControlAt(controls, offset));
  }
  result.expected_states = Numbers(json, "expected_states_flat");
  result.expected_times = Numbers(json, "expected_times_flat");
  result.expected_stamps = Numbers(json, "expected_stamps_flat");
  return result;
}

inline std::vector<double> Flatten(const mppi::MppiState &state,
                                   bool include_acceleration) {
  std::vector<double> result{
      state.position_enu_m.x,
      state.position_enu_m.y,
      state.position_enu_m.z,
      state.velocity_enu_m_s.x,
      state.velocity_enu_m_s.y,
      state.velocity_enu_m_s.z,
      state.yaw_enu_rad,
      state.applied_control.velocity_enu_m_s.x,
      state.applied_control.velocity_enu_m_s.y,
      state.applied_control.velocity_enu_m_s.z,
      state.applied_control.yaw_rate_enu_rad_s,
  };
  if (include_acceleration) {
    result.push_back(state.acceleration_memory_enu_m_s2.x);
    result.push_back(state.acceleration_memory_enu_m_s2.y);
    result.push_back(state.acceleration_memory_enu_m_s2.z);
  }
  return result;
}

} // namespace mppi_test

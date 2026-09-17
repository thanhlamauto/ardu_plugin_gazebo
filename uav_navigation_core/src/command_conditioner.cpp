#include "uav_navigation_core/command_conditioner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace uav_navigation_core {
namespace {
bool Finite(double v) { return std::isfinite(v); }
bool Finite(const Vec3 &v) { return Finite(v.x) && Finite(v.y) && Finite(v.z); }
bool Finite(const Control &u) {
  return Finite(u.velocity_enu_m_s) && Finite(u.yaw_rate_enu_rad_s);
}
std::array<double, 4> Array(const Control &u) {
  return {u.velocity_enu_m_s.x, u.velocity_enu_m_s.y, u.velocity_enu_m_s.z,
          u.yaw_rate_enu_rad_s};
}
Control ControlFrom(const std::array<double, 4> &a) {
  return {{a[0], a[1], a[2]}, a[3]};
}
void Clamp(std::array<double, 4> &value,
           const VelocityCommandConditionerConfig &config) {
  if (!config.minimum)
    return;
  for (std::size_t i = 0; i < value.size(); ++i) {
    value[i] = std::clamp(value[i], (*config.minimum)[i], (*config.maximum)[i]);
  }
}
} // namespace

VelocityCommandConditioner::VelocityCommandConditioner(
    VelocityCommandConditionerConfig config)
    : config_(std::move(config)) {
  if (!Finite(config_.dt_s) || config_.dt_s <= 0.0 || !Finite(config_.alpha) ||
      config_.alpha <= 0.0 || config_.alpha > 1.0 ||
      !Finite(config_.max_accel_xy_m_s2) || config_.max_accel_xy_m_s2 <= 0.0 ||
      !Finite(config_.max_accel_z_m_s2) || config_.max_accel_z_m_s2 <= 0.0 ||
      !Finite(config_.max_yaw_accel_rad_s2) ||
      config_.max_yaw_accel_rad_s2 <= 0.0) {
    throw std::invalid_argument(
        "invalid velocity command conditioner parameters");
  }
  if (config_.minimum.has_value() != config_.maximum.has_value()) {
    throw std::invalid_argument(
        "minimum and maximum bounds must be provided together");
  }
  if (config_.minimum) {
    for (std::size_t i = 0; i < 4; ++i) {
      if (!Finite((*config_.minimum)[i]) || !Finite((*config_.maximum)[i]) ||
          (*config_.minimum)[i] >= (*config_.maximum)[i]) {
        throw std::invalid_argument("invalid velocity command bounds");
      }
    }
  }
}

Control VelocityCommandConditioner::Apply(const State &measured_state,
                                          const Control &requested_control) {
  if (!Finite(requested_control)) {
    throw std::invalid_argument("velocity command must be finite");
  }
  if (!previous_) {
    if (!Finite(measured_state.velocity_enu_m_s)) {
      throw std::invalid_argument("measured velocity must be finite");
    }
    std::array<double, 4> initial{measured_state.velocity_enu_m_s.x,
                                  measured_state.velocity_enu_m_s.y,
                                  measured_state.velocity_enu_m_s.z, 0.0};
    Clamp(initial, config_);
    previous_ = ControlFrom(initial);
  }
  auto prior = Array(*previous_);
  const auto raw = Array(requested_control);
  std::array<double, 4> delta{};
  for (std::size_t i = 0; i < 4; ++i)
    delta[i] = config_.alpha * (raw[i] - prior[i]);
  const double xy_norm = std::hypot(delta[0], delta[1]);
  const double xy_limit = config_.max_accel_xy_m_s2 * config_.dt_s;
  if (xy_norm > xy_limit) {
    delta[0] *= xy_limit / xy_norm;
    delta[1] *= xy_limit / xy_norm;
  }
  delta[2] = std::clamp(delta[2], -config_.max_accel_z_m_s2 * config_.dt_s,
                        config_.max_accel_z_m_s2 * config_.dt_s);
  delta[3] = std::clamp(delta[3], -config_.max_yaw_accel_rad_s2 * config_.dt_s,
                        config_.max_yaw_accel_rad_s2 * config_.dt_s);
  for (std::size_t i = 0; i < 4; ++i)
    prior[i] += delta[i];
  Clamp(prior, config_);
  previous_ = ControlFrom(prior);
  return *previous_;
}

void VelocityCommandConditioner::Reset() { previous_.reset(); }
} // namespace uav_navigation_core

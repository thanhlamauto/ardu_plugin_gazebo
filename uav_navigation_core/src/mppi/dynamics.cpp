#include "uav_navigation_core/mppi/dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace uav_navigation_core::mppi {
namespace {

bool Finite(double value) { return std::isfinite(value); }
bool Finite(const Vec3 &value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}
bool Finite(const Control &value) {
  return Finite(value.velocity_enu_m_s) && Finite(value.yaw_rate_enu_rad_s);
}
bool Finite(const MppiState &value) {
  return Finite(value.position_enu_m) && Finite(value.velocity_enu_m_s) &&
         Finite(value.yaw_enu_rad) && Finite(value.applied_control) &&
         Finite(value.acceleration_memory_enu_m_s2);
}

double NormXY(const Vec3 &value) { return std::hypot(value.x, value.y); }

void LimitXY(Vec3 &value, double bound) {
  const double norm = NormXY(value);
  if (norm > bound) {
    value.x *= bound / norm;
    value.y *= bound / norm;
  }
}

Control Condition(const MppiDynamicsConfig &config, const Control &previous,
                  const Control &requested, double dt_s) {
  Control delta;
  delta.velocity_enu_m_s = {
      config.command_alpha *
          (requested.velocity_enu_m_s.x - previous.velocity_enu_m_s.x),
      config.command_alpha *
          (requested.velocity_enu_m_s.y - previous.velocity_enu_m_s.y),
      config.command_alpha *
          (requested.velocity_enu_m_s.z - previous.velocity_enu_m_s.z)};
  delta.yaw_rate_enu_rad_s =
      config.command_alpha *
      (requested.yaw_rate_enu_rad_s - previous.yaw_rate_enu_rad_s);

  LimitXY(delta.velocity_enu_m_s, config.max_accel_xy_m_s2 * dt_s);
  delta.velocity_enu_m_s.z =
      std::clamp(delta.velocity_enu_m_s.z, -config.max_accel_z_m_s2 * dt_s,
                 config.max_accel_z_m_s2 * dt_s);
  delta.yaw_rate_enu_rad_s =
      std::clamp(delta.yaw_rate_enu_rad_s, -config.max_yaw_accel_rad_s2 * dt_s,
                 config.max_yaw_accel_rad_s2 * dt_s);

  Control applied;
  applied.velocity_enu_m_s = {
      previous.velocity_enu_m_s.x + delta.velocity_enu_m_s.x,
      previous.velocity_enu_m_s.y + delta.velocity_enu_m_s.y,
      previous.velocity_enu_m_s.z + delta.velocity_enu_m_s.z};
  applied.yaw_rate_enu_rad_s =
      previous.yaw_rate_enu_rad_s + delta.yaw_rate_enu_rad_s;
  applied.velocity_enu_m_s.x =
      std::clamp(applied.velocity_enu_m_s.x, -config.vmax_m_s, config.vmax_m_s);
  applied.velocity_enu_m_s.y =
      std::clamp(applied.velocity_enu_m_s.y, -config.vmax_m_s, config.vmax_m_s);
  applied.velocity_enu_m_s.z = std::clamp(applied.velocity_enu_m_s.z,
                                          -config.vzmax_m_s, config.vzmax_m_s);
  applied.yaw_rate_enu_rad_s =
      std::clamp(applied.yaw_rate_enu_rad_s, -config.yaw_rate_max_rad_s,
                 config.yaw_rate_max_rad_s);
  return applied;
}

} // namespace

MultirotorMotionModel::MultirotorMotionModel(MppiDynamicsConfig config)
    : config_(std::move(config)) {
  const double positive[] = {
      config_.tau_s,
      config_.max_accel_xy_m_s2,
      config_.max_accel_z_m_s2,
      config_.max_yaw_accel_rad_s2,
      config_.vmax_m_s,
      config_.vzmax_m_s,
      config_.yaw_rate_max_rad_s,
      config_.response_accel_xy_m_s2,
      config_.response_jerk_xy_m_s3,
  };
  for (double value : positive) {
    if (!Finite(value) || value <= 0.0) {
      throw std::invalid_argument(
          "MPPI dynamics limits must be finite and positive");
    }
  }
  if (!Finite(config_.command_alpha) || config_.command_alpha <= 0.0 ||
      config_.command_alpha > 1.0) {
    throw std::invalid_argument("MPPI command alpha must be in (0, 1]");
  }
}

MppiState MultirotorMotionModel::Step(const MppiState &state,
                                      const Control &requested_control,
                                      double dt_s) const {
  if (!Finite(state) || !Finite(requested_control) || !Finite(dt_s) ||
      dt_s <= 0.0) {
    throw std::invalid_argument(
        "MPPI dynamics input must be finite and dt > 0");
  }
  const long double delta_ns_ld =
      std::round(static_cast<long double>(dt_s) * 1.0e9L);
  if (delta_ns_ld <= 0.0L ||
      delta_ns_ld >
          static_cast<long double>(std::numeric_limits<TimeNs>::max())) {
    throw std::invalid_argument("MPPI dt cannot be represented as nanoseconds");
  }
  const auto delta_ns = static_cast<TimeNs>(delta_ns_ld);
  if (state.stamp_ns > std::numeric_limits<TimeNs>::max() - delta_ns) {
    throw std::overflow_error("MPPI state timestamp overflow");
  }

  const Control applied =
      Condition(config_, state.applied_control, requested_control, dt_s);
  Vec3 velocity_next;
  Vec3 acceleration{};
  if (config_.response_accel_model) {
    Vec3 desired{
        (applied.velocity_enu_m_s.x - state.velocity_enu_m_s.x) / config_.tau_s,
        (applied.velocity_enu_m_s.y - state.velocity_enu_m_s.y) / config_.tau_s,
        (applied.velocity_enu_m_s.z - state.velocity_enu_m_s.z) / config_.tau_s,
    };
    LimitXY(desired, config_.response_accel_xy_m_s2);
    Vec3 change{
        desired.x - state.acceleration_memory_enu_m_s2.x,
        desired.y - state.acceleration_memory_enu_m_s2.y,
        desired.z - state.acceleration_memory_enu_m_s2.z,
    };
    LimitXY(change, config_.response_jerk_xy_m_s3 * dt_s);
    acceleration = {
        state.acceleration_memory_enu_m_s2.x + change.x,
        state.acceleration_memory_enu_m_s2.y + change.y,
        desired.z,
    };
    velocity_next = {
        state.velocity_enu_m_s.x + acceleration.x * dt_s,
        state.velocity_enu_m_s.y + acceleration.y * dt_s,
        state.velocity_enu_m_s.z + acceleration.z * dt_s,
    };
  } else {
    const double alpha = std::min(dt_s / config_.tau_s, 1.0);
    velocity_next = {
        state.velocity_enu_m_s.x +
            alpha * (applied.velocity_enu_m_s.x - state.velocity_enu_m_s.x),
        state.velocity_enu_m_s.y +
            alpha * (applied.velocity_enu_m_s.y - state.velocity_enu_m_s.y),
        state.velocity_enu_m_s.z +
            alpha * (applied.velocity_enu_m_s.z - state.velocity_enu_m_s.z),
    };
  }

  MppiState result;
  result.stamp_ns = state.stamp_ns + delta_ns;
  result.velocity_enu_m_s = velocity_next;
  result.position_enu_m = {
      state.position_enu_m.x + velocity_next.x * dt_s,
      state.position_enu_m.y + velocity_next.y * dt_s,
      state.position_enu_m.z + velocity_next.z * dt_s,
  };
  result.yaw_enu_rad = state.yaw_enu_rad + applied.yaw_rate_enu_rad_s * dt_s;
  result.applied_control = applied;
  result.acceleration_memory_enu_m_s2 = acceleration;
  return result;
}

} // namespace uav_navigation_core::mppi

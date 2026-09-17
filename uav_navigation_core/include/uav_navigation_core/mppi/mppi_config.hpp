#pragma once

namespace uav_navigation_core::mppi {

struct MppiDynamicsConfig {
  double tau_s{0.5};
  double command_alpha{0.45};
  double max_accel_xy_m_s2{1.5};
  double max_accel_z_m_s2{0.8};
  double max_yaw_accel_rad_s2{1.2};
  double vmax_m_s{2.0};
  double vzmax_m_s{1.0};
  double yaw_rate_max_rad_s{0.6};
  bool response_accel_model{false};
  double response_accel_xy_m_s2{3.0};
  double response_jerk_xy_m_s3{5.0};
};

} // namespace uav_navigation_core::mppi

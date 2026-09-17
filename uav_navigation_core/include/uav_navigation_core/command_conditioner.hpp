#pragma once

#include <array>
#include <optional>

#include "uav_navigation_core/interfaces.hpp"

namespace uav_navigation_core {

struct VelocityCommandConditionerConfig {
  double dt_s{0.1};
  double alpha{0.45};
  double max_accel_xy_m_s2{1.5};
  double max_accel_z_m_s2{0.8};
  double max_yaw_accel_rad_s2{1.2};
  std::optional<std::array<double, 4>> minimum{};
  std::optional<std::array<double, 4>> maximum{};
};

class VelocityCommandConditioner final : public ICommandConditioner {
public:
  explicit VelocityCommandConditioner(
      VelocityCommandConditionerConfig config = {});

  Control Apply(const State &measured_state,
                const Control &requested_control) override;
  void Reset() override;

  const std::optional<Control> &previous() const { return previous_; }
  const VelocityCommandConditionerConfig &config() const { return config_; }

private:
  VelocityCommandConditionerConfig config_;
  std::optional<Control> previous_;
};

} // namespace uav_navigation_core

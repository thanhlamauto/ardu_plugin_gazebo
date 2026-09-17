#pragma once

#include "uav_navigation_core/mppi/mppi_config.hpp"
#include "uav_navigation_core/mppi/mppi_types.hpp"

namespace uav_navigation_core::mppi {

class IMotionModel {
public:
  virtual ~IMotionModel() = default;
  virtual MppiState Step(const MppiState &state,
                         const Control &requested_control,
                         double dt_s) const = 0;
};

// Velocity-level point-mass surrogate used by the validated Python MPPI.
// It includes the same command conditioner and optional XY acceleration memory.
class MultirotorMotionModel final : public IMotionModel {
public:
  explicit MultirotorMotionModel(MppiDynamicsConfig config = {});

  MppiState Step(const MppiState &state, const Control &requested_control,
                 double dt_s) const override;

  const MppiDynamicsConfig &config() const { return config_; }

private:
  MppiDynamicsConfig config_;
};

} // namespace uav_navigation_core::mppi

#pragma once

#include <vector>

#include "uav_navigation_core/mppi/dynamics.hpp"

namespace uav_navigation_core::mppi {

MppiTrajectory Rollout(const IMotionModel &model,
                       const MppiState &initial_state,
                       const ControlSequence &controls, double dt_s);

std::vector<MppiTrajectory> RolloutBatch(const IMotionModel &model,
                                         const MppiState &initial_state,
                                         const ControlBatch &control_sequences,
                                         double dt_s);

} // namespace uav_navigation_core::mppi

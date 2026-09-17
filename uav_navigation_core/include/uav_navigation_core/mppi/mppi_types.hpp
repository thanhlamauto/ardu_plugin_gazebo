#pragma once

#include <vector>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core::mppi {

// Rollout state mirrors the Python velocity-level MPPI state. The applied
// control and optional acceleration memory are model state, not optimizer cost.
struct MppiState {
  TimeNs stamp_ns{0};
  Vec3 position_enu_m{};
  Vec3 velocity_enu_m_s{};
  double yaw_enu_rad{0.0};
  Control applied_control{};
  Vec3 acceleration_memory_enu_m_s2{};
};

struct MppiTrajectoryPoint {
  double time_from_start_s{0.0};
  MppiState state{};
  // Request that produced this point. The initial point has a zero request.
  Control requested_control{};
};

struct MppiTrajectory {
  // Includes the initial state at t=0, so N controls produce N+1 points.
  std::vector<MppiTrajectoryPoint> points;
};

using ControlSequence = std::vector<Control>;
using ControlBatch = std::vector<ControlSequence>;

} // namespace uav_navigation_core::mppi

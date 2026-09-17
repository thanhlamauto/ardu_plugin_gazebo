#include "uav_navigation_core/mppi/rollout.hpp"

#include <cmath>
#include <stdexcept>

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

void ValidateInput(const MppiState &initial_state, double dt_s) {
  if (!Finite(initial_state) || !Finite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument(
        "MPPI rollout state and dt must be finite and dt positive");
  }
}

} // namespace

MppiTrajectory Rollout(const IMotionModel &model,
                       const MppiState &initial_state,
                       const ControlSequence &controls, double dt_s) {
  ValidateInput(initial_state, dt_s);
  MppiTrajectory result;
  result.points.reserve(controls.size() + 1);
  result.points.push_back({0.0, initial_state, {}});
  MppiState state = initial_state;
  for (std::size_t index = 0; index < controls.size(); ++index) {
    state = model.Step(state, controls[index], dt_s);
    result.points.push_back(
        {(static_cast<double>(index) + 1.0) * dt_s, state, controls[index]});
  }
  return result;
}

std::vector<MppiTrajectory> RolloutBatch(const IMotionModel &model,
                                         const MppiState &initial_state,
                                         const ControlBatch &control_sequences,
                                         double dt_s) {
  ValidateInput(initial_state, dt_s);
  std::vector<MppiTrajectory> result;
  result.reserve(control_sequences.size());
  for (const auto &controls : control_sequences) {
    result.push_back(Rollout(model, initial_state, controls, dt_s));
  }
  return result;
}

} // namespace uav_navigation_core::mppi

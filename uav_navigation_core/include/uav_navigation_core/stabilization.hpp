#pragma once

#include <cstdint>
#include <vector>

#include "uav_navigation_core/mppi/optimizer.hpp"
#include "uav_navigation_core/mppi/path_reference.hpp"
#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

// M7.4 Candidate 1 parameters. All thresholds are conservative bounds: the
// shaping never commands above nominal_speed_m_s and never below
// min_speed_m_s.
struct SpeedShapingConfig {
  bool enabled{false};
  double nominal_speed_m_s{10.0};
  double turn_speed_m_s{3.0};
  double min_speed_m_s{0.5};
  double lateral_accel_m_s2{2.0};
  double braking_accel_m_s2{3.0};
  double reaction_delay_s{0.25};
  double turn_angle_rad{0.2617993877991494};
  double lookahead_m{2.0};
};

// Conservative speed cap derived only from planned heading change, distance to
// the next major turn and configured braking capability. Returns +infinity when
// disabled, which reproduces the pre-M7.4 behavior. Does not modify the path.
double ComputeReferenceSpeedCap(const SpeedShapingConfig &config,
                                const mppi::PathReference &path,
                                const Vec3 &position_enu_m,
                                const Vec3 &velocity_enu_m_s);

// True when the stopping-clearance predicate accounts for the largest share of
// rejected samples (and at least one sample was rejected by it).
bool StoppingDominant(const mppi::RejectionCounts &counts);

// Bounded deceleration along the current horizontal velocity direction. The
// command speed is monotonically non-increasing and never exceeds the measured
// speed; yaw rate is zero. Returns an empty sequence when the vehicle is
// effectively stopped.
mppi::ControlSequence BuildBrakingSequence(const Vec3 &velocity_enu_m_s,
                                           double deceleration_m_s2,
                                           std::size_t horizon, double dt_s);

// Diagnostic bookkeeping for the STOPPING_RECOVERY state. Entry/exit and
// duration accounting only; it never commands anything.
class StoppingRecoveryTracker {
public:
  void Configure(bool enabled) { enabled_ = enabled; }
  bool enabled() const { return enabled_; }
  bool active() const { return active_; }

  // Record one cycle that published a verified stopping-recovery command.
  void RecordRecoveryCycle() {
    if (!enabled_)
      return;
    if (!active_) {
      active_ = true;
      ++entries_;
      episode_cycles_ = 0;
    }
    ++cycles_;
    ++episode_cycles_;
  }

  // Record one cycle that did not use stopping recovery. Ends an active episode.
  void RecordOtherCycle() {
    if (!active_)
      return;
    active_ = false;
    ++exits_;
    episode_cycles_ = 0;
  }

  std::uint64_t entries() const { return entries_; }
  std::uint64_t exits() const { return exits_; }
  std::uint64_t cycles() const { return cycles_; }
  std::uint64_t episode_cycles() const { return episode_cycles_; }

private:
  bool enabled_{false};
  bool active_{false};
  std::uint64_t entries_{0};
  std::uint64_t exits_{0};
  std::uint64_t cycles_{0};
  std::uint64_t episode_cycles_{0};
};

} // namespace uav_navigation_core

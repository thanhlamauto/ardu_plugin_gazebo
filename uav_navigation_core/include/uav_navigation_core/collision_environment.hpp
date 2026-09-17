#pragma once

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

// Geometry contract shared by simulation and future hardware map adapters.
// Clearance is the non-negative Euclidean distance to the nearest occupied
// solid. SegmentSafe must continuously test the entire radius-swept segment.
class ICollisionEnvironment {
public:
  virtual ~ICollisionEnvironment() = default;

  virtual double Clearance(const Vec3 &point_enu_m) const = 0;
  virtual bool SegmentSafe(const Vec3 &start_enu_m, const Vec3 &end_enu_m,
                           double clearance_m) const = 0;
};

} // namespace uav_navigation_core

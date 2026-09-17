#pragma once

#include <vector>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core::mppi {

class PathReference {
public:
  explicit PathReference(std::vector<Vec3> points);

  double Distance(const Vec3 &point) const;
  double Progress(const Vec3 &point) const;
  Vec3 Sample(double progress_m) const;
  double length_m() const { return cumulative_lengths_.back(); }
  const std::vector<Vec3> &points() const { return points_; }

private:
  std::vector<Vec3> points_;
  std::vector<double> segment_lengths_;
  std::vector<double> cumulative_lengths_;
};

} // namespace uav_navigation_core::mppi

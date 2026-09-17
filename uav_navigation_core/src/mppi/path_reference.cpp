#include "uav_navigation_core/mppi/path_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace uav_navigation_core::mppi {
namespace {
Vec3 Sub(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 Add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 Scale(const Vec3 &a, double scale) {
  return {a.x * scale, a.y * scale, a.z * scale};
}
double Dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
double Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }
bool Finite(const Vec3 &a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
} // namespace

PathReference::PathReference(std::vector<Vec3> points)
    : points_(std::move(points)) {
  if (points_.empty())
    throw std::invalid_argument("MPPI reference path cannot be empty");
  for (const auto &point : points_)
    if (!Finite(point))
      throw std::invalid_argument("MPPI reference path must be finite");
  cumulative_lengths_.push_back(0.0);
  for (std::size_t i = 1; i < points_.size(); ++i) {
    segment_lengths_.push_back(Norm(Sub(points_[i], points_[i - 1])));
    cumulative_lengths_.push_back(cumulative_lengths_.back() +
                                  segment_lengths_.back());
  }
  if (points_.size() > 1 && cumulative_lengths_.back() <= 1e-9)
    throw std::invalid_argument("MPPI reference path has no nonzero segment");
}

double PathReference::Distance(const Vec3 &point) const {
  if (!Finite(point))
    throw std::invalid_argument("MPPI path query must be finite");
  if (points_.size() == 1)
    return Norm(Sub(point, points_.front()));
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < segment_lengths_.size(); ++i) {
    const Vec3 ab = Sub(points_[i + 1], points_[i]);
    const Vec3 relative = Sub(point, points_[i]);
    const double denominator = std::max(Dot(ab, ab), 1e-12);
    const double fraction =
        std::clamp(Dot(relative, ab) / denominator, 0.0, 1.0);
    const Vec3 residual = Sub(relative, Scale(ab, fraction));
    best_squared = std::min(best_squared, Dot(residual, residual));
  }
  return std::sqrt(std::max(best_squared, 0.0));
}

double PathReference::Progress(const Vec3 &point) const {
  if (!Finite(point))
    throw std::invalid_argument("MPPI path query must be finite");
  if (points_.size() < 2)
    return 0.0;
  double best_squared = std::numeric_limits<double>::infinity();
  double best_progress = 0.0;
  for (std::size_t i = 0; i < segment_lengths_.size(); ++i) {
    const Vec3 ab = Sub(points_[i + 1], points_[i]);
    const Vec3 relative = Sub(point, points_[i]);
    const double denominator = std::max(Dot(ab, ab), 1e-12);
    const double fraction =
        std::clamp(Dot(relative, ab) / denominator, 0.0, 1.0);
    const double squared = Dot(Sub(relative, Scale(ab, fraction)),
                               Sub(relative, Scale(ab, fraction)));
    if (squared < best_squared) {
      best_squared = squared;
      best_progress = cumulative_lengths_[i] + fraction * segment_lengths_[i];
    }
  }
  return best_progress;
}

Vec3 PathReference::Sample(double progress_m) const {
  if (!std::isfinite(progress_m))
    throw std::invalid_argument("MPPI path progress must be finite");
  if (points_.size() == 1)
    return points_.front();
  const double progress = std::clamp(progress_m, 0.0, length_m());
  auto upper = std::upper_bound(cumulative_lengths_.begin(),
                                cumulative_lengths_.end(), progress);
  std::size_t index =
      upper == cumulative_lengths_.begin()
          ? 0
          : static_cast<std::size_t>(upper - cumulative_lengths_.begin() - 1);
  index = std::min(index, segment_lengths_.size() - 1);
  const double fraction =
      segment_lengths_[index] > 1e-12
          ? (progress - cumulative_lengths_[index]) / segment_lengths_[index]
          : 0.0;
  return Add(points_[index],
             Scale(Sub(points_[index + 1], points_[index]), fraction));
}

} // namespace uav_navigation_core::mppi

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace uav_navigation_core {

using TimeNs = std::int64_t;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct State {
  TimeNs stamp_ns{0};
  Vec3 position_enu_m{};
  Vec3 velocity_enu_m_s{};
  Vec3 acceleration_enu_m_s2{};
  double yaw_enu_rad{0.0};
};

struct Goal {
  Vec3 position_enu_m{};
  std::string frame_id{"odom"};
};

struct Control {
  Vec3 velocity_enu_m_s{};
  double yaw_rate_enu_rad_s{0.0};
};

struct Path {
  std::vector<Vec3> points_enu_m;
};

struct TrajectoryPoint {
  double time_from_start_s{0.0};
  State state{};
  Control control{};
};

struct Trajectory {
  std::vector<TrajectoryPoint> points;
};

struct CostGrid2D {
  std::string frame_id{"odom"};
  double resolution_m{0.0};
  Vec3 origin_enu_m{};
  std::uint32_t width{0};
  std::uint32_t height{0};
  // Row-major: -1 unknown, 0 free, 1..99 inflated cost, 100 blocked.
  std::vector<std::int8_t> costs;
};

struct ObstacleMap {
  TimeNs stamp_ns{0};
  std::vector<Vec3> points_enu_m;
  std::optional<CostGrid2D> prior_cost_grid;
};

enum class StatusCode {
  kOk,
  kInvalidInput,
  kStaleInput,
  kNoPath,
  kNoSafeTrajectory,
  kDeadlineMiss,
  kInternalError,
};

struct PlannerDiagnostics {
  double compute_time_ms{0.0};
  std::uint32_t expanded_nodes{0};
  std::uint32_t sampled_trajectories{0};
  std::uint32_t feasible_trajectories{0};
  double effective_sample_size{0.0};
  std::string detail;
};

struct GlobalPlan {
  StatusCode status{StatusCode::kInternalError};
  Path path{};
  CostGrid2D cost_grid{};
  PlannerDiagnostics diagnostics{};
};

struct LocalPlan {
  StatusCode status{StatusCode::kInternalError};
  Trajectory trajectory{};
  Control raw_control{};
  PlannerDiagnostics diagnostics{};
};

struct SafetyResult {
  bool safe{false};
  double minimum_collision_clearance_m{0.0};
  double minimum_stopping_clearance_m{0.0};
  std::string reason;
};

}  // namespace uav_navigation_core

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "uav_navigation_core/command_conditioner.hpp"
#include "uav_navigation_core/mppi/cost_evaluator.hpp"
#include "uav_navigation_core/mppi/dynamics.hpp"
#include "uav_navigation_core/mppi/optimizer.hpp"
#include "uav_navigation_core/mppi/path_reference.hpp"
#include "uav_navigation_core/mppi/rollout.hpp"
#include "uav_navigation_core/trajectory_safety_checker.hpp"

namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;
using namespace std::chrono_literals;

namespace uav_navigation_ros {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Diagnostic-only threshold: a path-heading change above 15 degrees marks the
// start of a "major turn" for the distance-to-next-turn metric. It does not
// alter path geometry or controller behavior.
constexpr double kTurnAngle = 0.2617993877991494;

enum class Mode {
  kWaitingForState,
  kWaitingForPath,
  kWaitingForObstacles,
  kActive,
  kHoldStale,
  kNoSafeTrajectory,
  kPlannerTimeout,
  kGoalReached,
  kInvalidInput,
};

const char *ModeName(Mode mode) {
  switch (mode) {
  case Mode::kWaitingForState:
    return "WAITING_FOR_STATE";
  case Mode::kWaitingForPath:
    return "WAITING_FOR_PATH";
  case Mode::kWaitingForObstacles:
    return "WAITING_FOR_OBSTACLES";
  case Mode::kActive:
    return "ACTIVE";
  case Mode::kHoldStale:
    return "HOLD_STALE";
  case Mode::kNoSafeTrajectory:
    return "NO_SAFE_TRAJECTORY";
  case Mode::kPlannerTimeout:
    return "PLANNER_TIMEOUT";
  case Mode::kGoalReached:
    return "GOAL_REACHED";
  case Mode::kInvalidInput:
    return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

const char *ProposalSourceName(mppi::ProposalSource source) {
  switch (source) {
  case mppi::ProposalSource::kGaussian:
    return "gaussian";
  case mppi::ProposalSource::kReferenceProposal:
    return "reference";
  case mppi::ProposalSource::kBrakingProposal:
    return "braking";
  case mppi::ProposalSource::kRecoveryProposal:
    return "recovery";
  case mppi::ProposalSource::kSpecificAction:
    return "specific_action";
  case mppi::ProposalSource::kCount:
    return "unknown";
  }
  return "unknown";
}

bool Finite(double value) { return std::isfinite(value); }
bool Finite(const core::Vec3 &value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}
double Norm(const core::Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}
double Distance(const core::Vec3 &a, const core::Vec3 &b) {
  return Norm({a.x - b.x, a.y - b.y, a.z - b.z});
}
double Yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
core::Vec3 TransformPoint(const core::Vec3 &point,
                          const geometry_msgs::msg::Transform &transform) {
  const auto &q = transform.rotation;
  const core::Vec3 u{q.x, q.y, q.z};
  const double dot_uv = u.x * point.x + u.y * point.y + u.z * point.z;
  const double dot_uu = u.x * u.x + u.y * u.y + u.z * u.z;
  const core::Vec3 cross{u.y * point.z - u.z * point.y,
                         u.z * point.x - u.x * point.z,
                         u.x * point.y - u.y * point.x};
  return {2.0 * dot_uv * u.x + (q.w * q.w - dot_uu) * point.x +
              2.0 * q.w * cross.x + transform.translation.x,
          2.0 * dot_uv * u.y + (q.w * q.w - dot_uu) * point.y +
              2.0 * q.w * cross.y + transform.translation.y,
          2.0 * dot_uv * u.z + (q.w * q.w - dot_uu) * point.z +
              2.0 * q.w * cross.z + transform.translation.z};
}
core::State ToCoreState(const mppi::MppiState &state) {
  core::State output;
  output.stamp_ns = state.stamp_ns;
  output.position_enu_m = state.position_enu_m;
  output.velocity_enu_m_s = state.velocity_enu_m_s;
  output.acceleration_enu_m_s2 = state.acceleration_memory_enu_m_s2;
  output.yaw_enu_rad = state.yaw_enu_rad;
  return output;
}
core::Trajectory ToCoreTrajectory(const mppi::MppiTrajectory &trajectory) {
  core::Trajectory output;
  output.points.reserve(trajectory.points.size());
  for (const auto &point : trajectory.points) {
    core::TrajectoryPoint converted;
    converted.time_from_start_s = point.time_from_start_s;
    converted.state = ToCoreState(point.state);
    converted.control = point.requested_control;
    output.points.push_back(converted);
  }
  return output;
}
diagnostic_msgs::msg::KeyValue KeyValue(const std::string &key,
                                        const std::string &value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}
} // namespace

class LocalNavigationNode final : public rclcpp::Node {
public:
  LocalNavigationNode()
      : Node("local_navigation"), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
    obstacle_timeout_s_ = declare_parameter<double>("obstacle_timeout_s", 1.0);
    max_compute_time_ms_ =
        declare_parameter<double>("max_compute_time_ms", 80.0);
    const double validation_deadline_override = declare_parameter<double>(
        "validation.max_compute_time_ms_override", -1.0);
    if (validation_deadline_override > 0.0)
      max_compute_time_ms_ = validation_deadline_override;
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.5);
    goal_speed_tolerance_m_s_ =
        declare_parameter<double>("goal_speed_tolerance_m_s", 0.5);
    const auto obstacle_max_points =
        declare_parameter<int64_t>("obstacle_max_points", 800);
    visualization_rate_hz_ =
        declare_parameter<double>("visualization.publish_rate_hz", 5.0);
    visualization_enabled_ =
        declare_parameter<bool>("visualization.enabled", true);
    const auto visualization_max_samples =
        declare_parameter<int64_t>("visualization.max_mppi_samples", 80);

    const double dt = declare_parameter<double>("mppi.dt_s", 0.1);
    const auto horizon = declare_parameter<int64_t>("mppi.horizon_steps", 30);
    const auto samples = declare_parameter<int64_t>("mppi.samples", 80);
    const double lambda = declare_parameter<double>("mppi.lambda", 1.0);
    const auto seed = declare_parameter<int64_t>("mppi.seed", 7);
    const double vmax =
        declare_parameter<double>("mppi.max_speed_xy_m_s", 10.0);
    const double vzmax = declare_parameter<double>("mppi.max_speed_z_m_s", 0.6);
    const double yaw_rate_max =
        declare_parameter<double>("mppi.max_yaw_rate_rad_s", 0.6);
    const double accel_xy =
        declare_parameter<double>("mppi.max_accel_xy_m_s2", 3.0);
    const double accel_z =
        declare_parameter<double>("mppi.max_accel_z_m_s2", 0.4);
    const double yaw_accel =
        declare_parameter<double>("mppi.max_yaw_accel_rad_s2", 0.6);
    const double command_alpha =
        declare_parameter<double>("conditioner.alpha", 0.30);
    const double noise_xy = declare_parameter<double>("mppi.noise_xy", 0.8);
    const double noise_z = declare_parameter<double>("mppi.noise_z", 0.3);
    const double noise_yaw = declare_parameter<double>("mppi.noise_yaw", 0.3);
    proactive_proposals_ =
        declare_parameter<bool>("mppi.proactive_proposals", true);

    if (planning_frame_.empty() || body_frame_.empty() ||
        control_rate_hz_ <= 0 || state_timeout_s_ <= 0 ||
        obstacle_timeout_s_ <= 0 || max_compute_time_ms_ <= 0 || horizon <= 0 ||
        samples <= 0 || obstacle_max_points <= 0 ||
        visualization_rate_hz_ <= 0 || visualization_max_samples < 0)
      throw std::invalid_argument(
          "invalid local-navigation timing or size parameter");
    dt_s_ = dt;
    horizon_ = static_cast<std::size_t>(horizon);
    samples_ = static_cast<std::size_t>(samples);
    obstacle_max_points_ = static_cast<std::size_t>(obstacle_max_points);
    visualization_max_samples_ =
        static_cast<std::size_t>(visualization_max_samples);

    mppi::MppiDynamicsConfig dynamics;
    dynamics.tau_s = declare_parameter<double>("response.tau_s", 0.5);
    dynamics.command_alpha = command_alpha;
    dynamics.max_accel_xy_m_s2 = accel_xy;
    dynamics.max_accel_z_m_s2 = accel_z;
    dynamics.max_yaw_accel_rad_s2 = yaw_accel;
    dynamics.vmax_m_s = vmax;
    dynamics.vzmax_m_s = vzmax;
    dynamics.yaw_rate_max_rad_s = yaw_rate_max;
    dynamics.response_accel_model =
        declare_parameter<bool>("response.enabled", true);
    dynamics.response_accel_xy_m_s2 =
        declare_parameter<double>("response.accel_xy_m_s2", accel_xy);
    dynamics.response_jerk_xy_m_s3 =
        declare_parameter<double>("response.jerk_xy_m_s3", 4.0);
    motion_model_ = std::make_unique<mppi::MultirotorMotionModel>(dynamics);

    mppi::MppiCostConfig cost;
    const auto profile =
        declare_parameter<std::string>("mppi.cost_profile", "project");
    if (profile == "paper")
      cost.profile = mppi::CostProfile::kPaper;
    else if (profile != "project")
      throw std::invalid_argument("mppi.cost_profile must be project or paper");
    cost.path_progress_objective =
        declare_parameter<bool>("mppi.path_progress_objective", true);
    cost.w_goal = declare_parameter<double>("mppi.goal_weight", 0.0);
    cost.w_terminal = declare_parameter<double>("mppi.terminal_weight", 0.0);
    cost.w_obstacle = declare_parameter<double>("mppi.obstacle_weight", 300.0);
    cost.margin_m = declare_parameter<double>("mppi.obstacle_margin_m", 4.0);
    cost.w_collision =
        declare_parameter<double>("mppi.collision_weight", 1.0e6);
    cost.collision_radius_m =
        declare_parameter<double>("safety.collision_radius_m", 1.5);
    const double validation_collision_radius_override =
        declare_parameter<double>("validation.collision_radius_m_override",
                                  -1.0);
    if (validation_collision_radius_override > 0.0)
      cost.collision_radius_m = validation_collision_radius_override;
    cost.collision_cost_buffer_m =
        declare_parameter<double>("mppi.collision_cost_buffer_m", 0.0);
    cost.w_effort = declare_parameter<double>("mppi.effort_weight", 0.05);
    cost.w_smoothness =
        declare_parameter<double>("mppi.smoothness_weight", 0.2);
    cost.w_yaw = declare_parameter<double>("mppi.yaw_weight", 0.2);
    cost.w_path = declare_parameter<double>("mppi.path_weight", 400.0);
    cost.path_scale_m = declare_parameter<double>("mppi.path_scale_m", 1.0);
    cost.w_progress = declare_parameter<double>("mppi.progress_weight", 5000.0);
    cost.w_speed_limit =
        declare_parameter<double>("mppi.speed_limit_weight", 10000.0);
    cost.vmax_m_s = vmax;
    cost.w_stopping = declare_parameter<double>("mppi.stopping_weight", 0.0);
    cost.stopping_margin_m =
        declare_parameter<double>("mppi.stopping_margin_m", 1.5);
    cost.stopping_delay_s =
        declare_parameter<double>("safety.stopping_delay_s", 0.25);
    cost.braking_acceleration_m_s2 = accel_xy;
    cost_evaluator_ = std::make_unique<mppi::CostEvaluator>(cost);

    mppi::MppiOptimizerConfig optimizer;
    optimizer.lambda = lambda;
    optimizer.noise_sigma = {{noise_xy, noise_xy, noise_z, noise_yaw}};
    optimizer.minimum = {{-vmax, -vmax, -vzmax, -yaw_rate_max}};
    optimizer.maximum = {{vmax, vmax, vzmax, yaw_rate_max}};
    optimizer_ = std::make_unique<mppi::MppiOptimizer>(optimizer);
    noise_sampler_ = std::make_unique<mppi::GaussianNoiseSampler>(
        optimizer.noise_sigma, seed);

    core::TrajectorySafetyConfig safety;
    safety.collision_radius_m = cost.collision_radius_m;
    safety.braking_acceleration_m_s2 = accel_xy;
    safety.stopping_delay_s = cost.stopping_delay_s;
    safety.stopping_clearance_m =
        declare_parameter<double>("safety.stopping_clearance_m", 1.5);
    safety.stopping_uncertainty_m =
        declare_parameter<double>("safety.stopping_uncertainty_m", 0.0);
    safety_checker_ = std::make_unique<core::TrajectorySafetyChecker>(safety);
    stopping_delay_s_ = safety.stopping_delay_s;
    braking_accel_m_s2_ = accel_xy;

    core::VelocityCommandConditionerConfig conditioner;
    conditioner.dt_s = dt;
    conditioner.alpha = command_alpha;
    conditioner.max_accel_xy_m_s2 = accel_xy;
    conditioner.max_accel_z_m_s2 = accel_z;
    conditioner.max_yaw_accel_rad_s2 = yaw_accel;
    conditioner.minimum = optimizer.minimum;
    conditioner.maximum = optimizer.maximum;
    conditioner_ =
        std::make_unique<core::VelocityCommandConditioner>(conditioner);
    nominal_.resize(horizon_);

    const auto odometry_topic = declare_parameter<std::string>(
        "odometry_topic", "/localization/odometry");
    const auto path_topic = declare_parameter<std::string>(
        "global_path_topic", "/planning/global_path");
    const auto obstacles_topic = declare_parameter<std::string>(
        "obstacles_topic", "/perception/obstacles");
    const auto output_topic = declare_parameter<std::string>(
        "output_command_topic", "/control/safe_velocity_command");
    const auto raw_output_topic = declare_parameter<std::string>(
        "raw_command_topic", "/control/raw_velocity_command");
    const auto predicted_topic = declare_parameter<std::string>(
        "predicted_path_topic", "/planning/mppi/predicted_path");
    const auto samples_topic = declare_parameter<std::string>(
        "cost_samples_topic", "/planning/mppi/cost_samples");
    const auto diagnostics_topic =
        declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    command_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 1);
    raw_command_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(raw_output_topic, 1);
    predicted_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(predicted_topic, 1);
    sample_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        samples_topic, 1);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic, 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          OnOdometry(*message);
        });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::Path::ConstSharedPtr message) {
          OnPath(*message);
        });
    obstacle_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          OnObstacles(*message);
        });

    control_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_hz_),
                          [this] { ControlCycle(); });
    if (visualization_enabled_) {
      visualization_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / visualization_rate_hz_),
          [this] { PublishVisualization(); });
    }
    RCLCPP_INFO(get_logger(),
                "local navigation ready: %zu samples x %zu steps, %.1f Hz, "
                "deadline %.1f ms",
                samples_, horizon_, control_rate_hz_, max_compute_time_ms_);
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  // M7.3 diagnostic-only bookkeeping for one control cycle. None of these
  // values feed back into control or safety decisions.
  struct CycleDiagnostics {
    double scheduling_delay_ms{0.0};
    double wall_cycle_ms{0.0};
    double input_snapshot_ms{0.0};
    double path_reference_ms{0.0};
    double sampling_ms{0.0};
    double optimizer_update_ms{0.0};
    double publish_ms{0.0};
    double speed_total_m_s{0.0};
    double speed_xy_m_s{0.0};
    double speed_z_m_s{0.0};
    double yaw_rad{0.0};
    double yaw_rate_rad_s{0.0};
    double path_tangent_heading_rad{0.0};
    double heading_error_rad{0.0};
    double path_curvature_per_m{0.0};
    double distance_to_next_turn_m{
        std::numeric_limits<double>::infinity()};
    double stopping_distance_m{0.0};
    double path_progress_m{0.0};
    double minimum_collision_clearance_m{
        std::numeric_limits<double>::infinity()};
    double minimum_stopping_clearance_m{
        std::numeric_limits<double>::infinity()};
    std::array<double, 4> warm_start_u{};
    std::array<double, 4> selected_u{};
    int selected_source{static_cast<int>(mppi::ProposalSource::kGaussian)};
    std::uint64_t local_active_path_id{0};
    std::uint64_t previous_path_id{0};
    std::int64_t active_path_stamp_ns{0};
    std::uint64_t solves_on_active_path{0};
    bool first_solve_on_active_path{false};
    bool first_safe_command_on_active_path{false};
  };

  void OnOdometry(const nav_msgs::msg::Odometry &message) {
    const std::string frame = message.header.frame_id.empty()
                                  ? planning_frame_
                                  : message.header.frame_id;
    if (frame != planning_frame_) {
      odometry_error_ = "odometry frame " + frame + " != " + planning_frame_;
      return;
    }
    const auto &p = message.pose.pose.position;
    const auto &v = message.twist.twist.linear;
    const auto &q = message.pose.pose.orientation;
    if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z) || !Finite(v.x) ||
        !Finite(v.y) || !Finite(v.z) || !Finite(q.x) || !Finite(q.y) ||
        !Finite(q.z) || !Finite(q.w)) {
      odometry_error_ = "non-finite odometry";
      return;
    }
    const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    core::Vec3 acceleration{};
    if (have_state_ && stamp > state_.stamp_ns) {
      const double elapsed =
          static_cast<double>(stamp - state_.stamp_ns) * 1e-9;
      if (elapsed <= 0.5) {
        acceleration = {(v.x - state_.velocity_enu_m_s.x) / elapsed,
                        (v.y - state_.velocity_enu_m_s.y) / elapsed,
                        (v.z - state_.velocity_enu_m_s.z) / elapsed};
      }
    }
    state_.stamp_ns = stamp;
    state_.position_enu_m = {p.x, p.y, p.z};
    state_.velocity_enu_m_s = {v.x, v.y, v.z};
    state_.acceleration_memory_enu_m_s2 = acceleration;
    state_.yaw_enu_rad = Yaw(q);
    last_state_received_ = std::chrono::steady_clock::now();
    have_state_ = true;
    odometry_error_.clear();
  }

  void OnPath(const nav_msgs::msg::Path &message) {
    const std::string frame = message.header.frame_id.empty()
                                  ? planning_frame_
                                  : message.header.frame_id;
    if (frame != planning_frame_) {
      path_error_ = "path frame " + frame + " != " + planning_frame_;
      return;
    }
    std::vector<core::Vec3> points;
    points.reserve(message.poses.size());
    for (const auto &pose : message.poses) {
      const auto &p = pose.pose.position;
      if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z)) {
        path_error_ = "non-finite global path";
        return;
      }
      points.push_back({p.x, p.y, p.z});
    }
    if (points.size() < 2) {
      reference_path_.reset();
      ResetController();
      path_error_ = "global path has fewer than two points";
      return;
    }
    try {
      reference_path_ =
          std::make_shared<mppi::PathReference>(std::move(points));
      goal_ = reference_path_->points().back();
      // Diagnostic path versioning: a new accepted path gets a monotonic local
      // id and its originating message stamp so the analyzer can join it to
      // the global planner goal_id/global_path_id. This does not change which
      // path the controller uses.
      previous_path_id_ = local_active_path_id_;
      ++local_active_path_id_;
      active_path_stamp_ns_ =
          rclcpp::Time(message.header.stamp).nanoseconds();
      path_received_monotonic_ = std::chrono::steady_clock::now();
      solves_on_active_path_ = 0;
      have_first_solve_on_path_ = false;
      have_first_safe_on_path_ = false;
      // The global path is a replacement plan, so the MPPI warm start and the
      // command conditioner are reset rather than carried across the handoff.
      ResetController();
      path_error_.clear();
      RCLCPP_INFO(get_logger(),
                  "accepted global path id=%llu previous_id=%llu stamp_ns=%lld",
                  static_cast<unsigned long long>(local_active_path_id_),
                  static_cast<unsigned long long>(previous_path_id_),
                  static_cast<long long>(active_path_stamp_ns_));
    } catch (const std::exception &error) {
      reference_path_.reset();
      path_error_ = error.what();
    }
  }

  void OnObstacles(const sensor_msgs::msg::PointCloud2 &message) {
    geometry_msgs::msg::Transform transform;
    const std::string source = message.header.frame_id.empty()
                                   ? planning_frame_
                                   : message.header.frame_id;
    if (source != planning_frame_) {
      try {
        transform = tf_buffer_
                        .lookupTransform(planning_frame_, source,
                                         message.header.stamp, 50ms)
                        .transform;
      } catch (const std::exception &error) {
        obstacle_error_ =
            std::string("obstacle TF unavailable: ") + error.what();
        return;
      }
    } else {
      transform.rotation.w = 1.0;
    }
    std::vector<core::Vec3> points;
    try {
      const std::size_t total =
          static_cast<std::size_t>(message.width) * message.height;
      const std::size_t stride = std::max<std::size_t>(
          1, (total + obstacle_max_points_ - 1) / obstacle_max_points_);
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      std::size_t index = 0;
      for (; x != x.end(); ++x, ++y, ++z, ++index) {
        if (index % stride != 0)
          continue;
        const core::Vec3 point{*x, *y, *z};
        if (Finite(point))
          points.push_back(TransformPoint(point, transform));
      }
    } catch (const std::exception &error) {
      obstacle_error_ = std::string("invalid obstacle cloud: ") + error.what();
      return;
    }
    obstacles_.stamp_ns = rclcpp::Time(message.header.stamp).nanoseconds();
    obstacles_.observation_valid = true;
    obstacles_.points_enu_m = std::move(points);
    last_obstacles_received_ = std::chrono::steady_clock::now();
    have_obstacles_ = true;
    obstacle_error_.clear();
  }

  void ResetController() {
    std::fill(nominal_.begin(), nominal_.end(), core::Control{});
    conditioner_->Reset();
    last_result_.reset();
    selected_trajectory_.reset();
  }

  void Hold(Mode mode, const std::string &reason) {
    mode_ = mode;
    reason_ = reason;
    ResetController();
    PublishDiagnostic(0.0, 0.0, 0.0, 0.0, 0, 0, 0.0,
                      std::numeric_limits<double>::infinity());
  }

  void ControlCycle() {
    const auto cycle_started = std::chrono::steady_clock::now();
    const auto ms_since = [](SteadyTime start) {
      return std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - start)
          .count();
    };
    ++cycle_counter_;
    cycle_diag_ = CycleDiagnostics{};
    last_rejection_ = mppi::RejectionCounts{};
    last_proposal_stats_ = {};
    const double period_ms = 1000.0 / control_rate_hz_;
    if (have_previous_cycle_) {
      cycle_diag_.wall_cycle_ms = std::chrono::duration<double, std::milli>(
                                      cycle_started - previous_cycle_start_)
                                      .count();
      cycle_diag_.scheduling_delay_ms = cycle_diag_.wall_cycle_ms - period_ms;
      if (std::abs(cycle_diag_.scheduling_delay_ms) > 3.0 * period_ms)
        cycle_diag_.scheduling_delay_ms = 0.0;
    }
    previous_cycle_start_ = cycle_started;
    have_previous_cycle_ = true;

    const auto age = [&](SteadyTime stamp) {
      return std::chrono::duration<double>(cycle_started - stamp).count();
    };
    const std::string input_error =
        !odometry_error_.empty()
            ? odometry_error_
            : (!path_error_.empty() ? path_error_ : obstacle_error_);
    if (!input_error.empty()) {
      Hold(Mode::kInvalidInput, input_error);
      return;
    }
    if (!have_state_) {
      Hold(Mode::kWaitingForState, "no odometry received");
      return;
    }
    if (!reference_path_) {
      Hold(Mode::kWaitingForPath, "no global path received");
      return;
    }
    if (!have_obstacles_) {
      Hold(Mode::kWaitingForObstacles, "no obstacle observation received");
      return;
    }
    if (age(last_state_received_) > state_timeout_s_ ||
        age(last_obstacles_received_) > obstacle_timeout_s_) {
      Hold(Mode::kHoldStale, "odometry or obstacle observation is stale");
      return;
    }
    if (Distance(state_.position_enu_m, goal_) <= goal_tolerance_m_ &&
        Norm(state_.velocity_enu_m_s) <= goal_speed_tolerance_m_s_) {
      Hold(Mode::kGoalReached, "terminal position and speed reached");
      return;
    }

    const auto input_snapshot_started = std::chrono::steady_clock::now();
    if (const auto &previous = conditioner_->previous())
      state_.applied_control = *previous;
    else
      state_.applied_control = {state_.velocity_enu_m_s, 0.0};
    mppi::CostContext context;
    context.goal_enu_m = goal_;
    context.obstacles_enu_m = obstacles_.points_enu_m;
    context.reference_path = reference_path_;
    context.initial_state = state_;
    const core::Control warm_start = nominal_.front();
    FillMotionDiagnostics(warm_start);
    cycle_diag_.input_snapshot_ms = ms_since(input_snapshot_started);

    const auto sampling_started = std::chrono::steady_clock::now();
    const auto noise = noise_sampler_->Sample(samples_, horizon_);
    cycle_diag_.sampling_ms = ms_since(sampling_started);

    mppi::ControlBatch proposals;
    std::vector<mppi::ProposalSource> proposal_sources;
    const auto reference_started = std::chrono::steady_clock::now();
    if (proactive_proposals_) {
      mppi::RecoveryProposalConfig proposal_config;
      proposal_config.horizon = horizon_;
      proposal_config.max_samples = samples_;
      proposal_config.dt_s = dt_s_;
      proposal_config.reference_speed_m_s = optimizer_->config().maximum[0];
      proposal_config.minimum = optimizer_->config().minimum;
      proposal_config.maximum = optimizer_->config().maximum;
      proposal_config.proactive = true;
      auto recovered = mppi::BuildRecoveryProposalsWithSources(
          reference_path_.get(),
          reference_path_->Progress(state_.position_enu_m),
          std::hypot(state_.velocity_enu_m_s.x, state_.velocity_enu_m_s.y),
          proposal_config);
      proposals = std::move(recovered.proposals);
      proposal_sources = std::move(recovered.sources);
    }
    cycle_diag_.path_reference_ms = ms_since(reference_started);

    const auto optimizer_started = std::chrono::steady_clock::now();
    mppi::MppiOptimizationResult result;
    try {
      result = optimizer_->OptimizeInjected(
          *motion_model_, *cost_evaluator_, context, state_, nominal_, {},
          noise, dt_s_, proposals, safety_checker_.get(), &obstacles_,
          proposal_sources.empty() ? nullptr : &proposal_sources);
    } catch (const std::exception &error) {
      Hold(Mode::kInvalidInput,
           std::string("planner exception: ") + error.what());
      return;
    }
    const double optimizer_total_ms = ms_since(optimizer_started);
    cycle_diag_.optimizer_update_ms =
        std::max(0.0, optimizer_total_ms - result.rollout_time_ms -
                          result.cost_time_ms - result.safety_time_ms);
    last_rejection_ = result.rejection_counts;
    last_proposal_stats_ = result.proposal_source_stats;
    const double planner_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_started)
            .count();
    if (planner_ms > max_compute_time_ms_) {
      Hold(Mode::kPlannerTimeout, "MPPI output exceeded control deadline");
      return;
    }

    mppi::ControlSequence selected_actions = result.updated_nominal;
    mppi::MppiTrajectory selected =
        mppi::Rollout(*motion_model_, state_, selected_actions, dt_s_);
    auto safety_started = std::chrono::steady_clock::now();
    auto final_safety = safety_checker_->Evaluate(
        ToCoreState(state_), ToCoreTrajectory(selected), obstacles_);
    double final_safety_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - safety_started)
            .count();
    cycle_diag_.minimum_collision_clearance_m =
        final_safety.minimum_collision_clearance_m;
    cycle_diag_.minimum_stopping_clearance_m =
        final_safety.minimum_stopping_clearance_m;
    if (!final_safety.safe) {
      std::optional<std::size_t> best;
      for (std::size_t i = 0; i < result.safe.size(); ++i)
        if (result.safe[i] &&
            (!best || result.total_costs[i] < result.total_costs[*best]))
          best = i;
      if (!best) {
        last_result_ = std::move(result);
        mode_ = Mode::kNoSafeTrajectory;
        reason_ = final_safety.reason;
        std::fill(nominal_.begin(), nominal_.end(), core::Control{});
        conditioner_->Reset();
        selected_trajectory_.reset();
        PublishDiagnostic(planner_ms, last_result_->rollout_time_ms,
                          last_result_->cost_time_ms,
                          last_result_->safety_time_ms + final_safety_ms,
                          last_result_->weights.size(), 0, 0.0,
                          std::min(final_safety.minimum_collision_clearance_m,
                                   final_safety.minimum_stopping_clearance_m));
        return;
      }
      selected_actions = result.perturbed_actions[*best];
      selected = result.trajectories[*best];
      cycle_diag_.selected_source = static_cast<int>(
          result.sample_sources.empty() ? mppi::ProposalSource::kGaussian
                                        : result.sample_sources[*best]);
      final_safety = safety_checker_->Evaluate(
          ToCoreState(state_), ToCoreTrajectory(selected), obstacles_);
      cycle_diag_.minimum_collision_clearance_m =
          final_safety.minimum_collision_clearance_m;
      cycle_diag_.minimum_stopping_clearance_m =
          final_safety.minimum_stopping_clearance_m;
      if (!final_safety.safe) {
        Hold(Mode::kNoSafeTrajectory, "feasible sample failed final predicate");
        return;
      }
    }
    FillSelectedControl(selected_actions.front());

    const auto publish_started = std::chrono::steady_clock::now();
    geometry_msgs::msg::TwistStamped raw_command;
    raw_command.header.stamp = now();
    raw_command.header.frame_id = planning_frame_;
    raw_command.twist.linear.x = selected_actions.front().velocity_enu_m_s.x;
    raw_command.twist.linear.y = selected_actions.front().velocity_enu_m_s.y;
    raw_command.twist.linear.z = selected_actions.front().velocity_enu_m_s.z;
    raw_command.twist.angular.z = selected_actions.front().yaw_rate_enu_rad_s;
    raw_command_publisher_->publish(raw_command);

    const auto conditioner_started = std::chrono::steady_clock::now();
    const auto safe_control =
        conditioner_->Apply(ToCoreState(state_), selected_actions.front());
    const double conditioner_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - conditioner_started)
            .count();
    const double total_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_started)
            .count();
    if (total_ms > max_compute_time_ms_) {
      Hold(Mode::kPlannerTimeout, "safe command became late before publish");
      return;
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    command.twist.linear.x = safe_control.velocity_enu_m_s.x;
    command.twist.linear.y = safe_control.velocity_enu_m_s.y;
    command.twist.linear.z = safe_control.velocity_enu_m_s.z;
    command.twist.angular.z = safe_control.yaw_rate_enu_rad_s;
    command_publisher_->publish(command);
    cycle_diag_.publish_ms = ms_since(publish_started);

    nominal_ = std::move(selected_actions);
    mode_ = Mode::kActive;
    reason_ = "safe command published";
    selected_trajectory_ = std::move(selected);
    const auto safe_count = static_cast<std::size_t>(
        std::count(result.safe.begin(), result.safe.end(), true));
    double squared_weights = 0.0;
    for (double weight : result.weights)
      squared_weights += weight * weight;
    const double ess = squared_weights > 0 ? 1.0 / squared_weights : 0.0;
    last_result_ = std::move(result);
    double best_feasible_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < last_result_->total_costs.size(); ++i)
      if (last_result_->safe[i])
        best_feasible_cost =
            std::min(best_feasible_cost, last_result_->total_costs[i]);
    ++solves_on_active_path_;
    if (!have_first_solve_on_path_) {
      have_first_solve_on_path_ = true;
      cycle_diag_.first_solve_on_active_path = true;
    }
    if (!have_first_safe_on_path_) {
      have_first_safe_on_path_ = true;
      cycle_diag_.first_safe_command_on_active_path = true;
    }
    PublishDiagnostic(total_ms, last_result_->rollout_time_ms,
                      last_result_->cost_time_ms,
                      last_result_->safety_time_ms + final_safety_ms,
                      last_result_->weights.size(), safe_count, ess,
                      std::min(final_safety.minimum_collision_clearance_m,
                               final_safety.minimum_stopping_clearance_m),
                      conditioner_ms, best_feasible_cost);
  }

  static double WrapAngle(double angle) {
    while (angle > kPi)
      angle -= 2.0 * kPi;
    while (angle < -kPi)
      angle += 2.0 * kPi;
    return angle;
  }

  double PathTangentHeading(const mppi::PathReference &path,
                            double progress_m) const {
    const double length = path.length_m();
    const double lower = std::clamp(progress_m - 0.5, 0.0, length);
    const double upper = std::clamp(progress_m + 0.5, 0.0, length);
    if (upper - lower < 1e-6)
      return state_.yaw_enu_rad;
    const core::Vec3 a = path.Sample(lower);
    const core::Vec3 b = path.Sample(upper);
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  double DistanceToNextTurn(const mppi::PathReference &path,
                            double progress_m) const {
    const double length = path.length_m();
    const double step = 0.5;
    const double base = PathTangentHeading(path, progress_m);
    for (double s = progress_m + step; s <= length; s += step)
      if (std::abs(WrapAngle(PathTangentHeading(path, s) - base)) > kTurnAngle)
        return s - progress_m;
    return std::numeric_limits<double>::infinity();
  }

  void FillMotionDiagnostics(const core::Control &warm_start) {
    if (!reference_path_)
      return;
    const double progress = reference_path_->Progress(state_.position_enu_m);
    const double tangent = PathTangentHeading(*reference_path_, progress);
    const double speed_total = Norm(state_.velocity_enu_m_s);
    cycle_diag_.speed_total_m_s = speed_total;
    cycle_diag_.speed_xy_m_s =
        std::hypot(state_.velocity_enu_m_s.x, state_.velocity_enu_m_s.y);
    cycle_diag_.speed_z_m_s = state_.velocity_enu_m_s.z;
    cycle_diag_.yaw_rad = state_.yaw_enu_rad;
    cycle_diag_.yaw_rate_rad_s = state_.applied_control.yaw_rate_enu_rad_s;
    cycle_diag_.path_progress_m = progress;
    cycle_diag_.path_tangent_heading_rad = tangent;
    cycle_diag_.heading_error_rad = WrapAngle(state_.yaw_enu_rad - tangent);
    const double lookahead = std::min(2.0, reference_path_->length_m() - progress);
    if (lookahead > 1e-3) {
      const double ahead =
          PathTangentHeading(*reference_path_, progress + lookahead);
      cycle_diag_.path_curvature_per_m =
          std::abs(WrapAngle(ahead - tangent)) / lookahead;
    }
    cycle_diag_.distance_to_next_turn_m =
        DistanceToNextTurn(*reference_path_, progress);
    cycle_diag_.stopping_distance_m =
        speed_total * stopping_delay_s_ +
        speed_total * speed_total / (2.0 * braking_accel_m_s2_);
    const auto warm = ComponentsOf(warm_start);
    for (std::size_t i = 0; i < 4; ++i)
      cycle_diag_.warm_start_u[i] = warm[i];
    cycle_diag_.local_active_path_id = local_active_path_id_;
    cycle_diag_.active_path_stamp_ns = active_path_stamp_ns_;
    cycle_diag_.previous_path_id = previous_path_id_;
    cycle_diag_.solves_on_active_path = solves_on_active_path_;
  }

  void FillSelectedControl(const core::Control &selected) {
    const auto values = ComponentsOf(selected);
    for (std::size_t i = 0; i < 4; ++i)
      cycle_diag_.selected_u[i] = values[i];
  }

  static std::array<double, 4> ComponentsOf(const core::Control &control) {
    return {control.velocity_enu_m_s.x, control.velocity_enu_m_s.y,
            control.velocity_enu_m_s.z, control.yaw_rate_enu_rad_s};
  }

  void PublishDiagnostic(double total_ms, double rollout_ms, double cost_ms,
                         double safety_ms, std::size_t samples,
                         std::size_t safe_samples, double ess, double clearance,
                         double conditioner_ms = 0.0,
                         double best_feasible_cost =
                             std::numeric_limits<double>::infinity()) {
    const auto diagnostics_started = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "local_navigation";
    status.hardware_id = "uav_navigation_core";
    status.level = mode_ == Mode::kActive || mode_ == Mode::kGoalReached
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : (mode_ == Mode::kWaitingForState ||
                                  mode_ == Mode::kWaitingForPath ||
                                  mode_ == Mode::kWaitingForObstacles
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    const double diagnostics_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() -
                                      diagnostics_started)
                                      .count();
    const auto source_stats = [&](mppi::ProposalSource source) {
      return last_proposal_stats_[static_cast<std::size_t>(source)];
    };
    const auto gaussian = source_stats(mppi::ProposalSource::kGaussian);
    const auto reference = source_stats(mppi::ProposalSource::kReferenceProposal);
    const auto braking = source_stats(mppi::ProposalSource::kBrakingProposal);
    const auto recovery = source_stats(mppi::ProposalSource::kRecoveryProposal);
    const auto specific = source_stats(mppi::ProposalSource::kSpecificAction);
    status.message = std::string(ModeName(mode_)) + ": " + reason_;
    status.values = {
        KeyValue("mode", ModeName(mode_)),
        KeyValue("reason", reason_),
        KeyValue("t_total_ms", std::to_string(total_ms)),
        KeyValue("t_rollout_ms", std::to_string(rollout_ms)),
        KeyValue("t_cost_ms", std::to_string(cost_ms)),
        KeyValue("t_safety_ms", std::to_string(safety_ms)),
        KeyValue("t_conditioner_ms", std::to_string(conditioner_ms)),
        KeyValue("t_scheduling_delay_ms",
                 std::to_string(cycle_diag_.scheduling_delay_ms)),
        KeyValue("t_wall_cycle_ms", std::to_string(cycle_diag_.wall_cycle_ms)),
        KeyValue("t_input_snapshot_ms",
                 std::to_string(cycle_diag_.input_snapshot_ms)),
        KeyValue("t_path_reference_ms",
                 std::to_string(cycle_diag_.path_reference_ms)),
        KeyValue("t_sampling_ms", std::to_string(cycle_diag_.sampling_ms)),
        KeyValue("t_optimizer_update_ms",
                 std::to_string(cycle_diag_.optimizer_update_ms)),
        KeyValue("t_publish_ms", std::to_string(cycle_diag_.publish_ms)),
        KeyValue("t_diagnostics_ms", std::to_string(diagnostics_ms)),
        KeyValue("samples", std::to_string(samples)),
        KeyValue("safe_samples", std::to_string(safe_samples)),
        KeyValue("ess", std::to_string(ess)),
        KeyValue("minimum_clearance_m", std::to_string(clearance)),
        KeyValue("best_feasible_cost", std::to_string(best_feasible_cost)),
        KeyValue("cross_track_error_m",
                 reference_path_
                     ? std::to_string(reference_path_->Distance(
                           state_.position_enu_m))
                     : "0"),
        KeyValue("deadline_miss",
                 mode_ == Mode::kPlannerTimeout ? "true" : "false"),
        KeyValue("path_progress_m",
                 reference_path_ ? std::to_string(reference_path_->Progress(
                                       state_.position_enu_m))
                                 : "0"),
        KeyValue("speed_total_m_s", std::to_string(cycle_diag_.speed_total_m_s)),
        KeyValue("speed_xy_m_s", std::to_string(cycle_diag_.speed_xy_m_s)),
        KeyValue("speed_z_m_s", std::to_string(cycle_diag_.speed_z_m_s)),
        KeyValue("yaw_rad", std::to_string(cycle_diag_.yaw_rad)),
        KeyValue("yaw_rate_rad_s", std::to_string(cycle_diag_.yaw_rate_rad_s)),
        KeyValue("path_tangent_heading_rad",
                 std::to_string(cycle_diag_.path_tangent_heading_rad)),
        KeyValue("heading_error_rad",
                 std::to_string(cycle_diag_.heading_error_rad)),
        KeyValue("path_curvature_per_m",
                 std::to_string(cycle_diag_.path_curvature_per_m)),
        KeyValue("distance_to_next_turn_m",
                 std::to_string(cycle_diag_.distance_to_next_turn_m)),
        KeyValue("stopping_distance_m",
                 std::to_string(cycle_diag_.stopping_distance_m)),
        KeyValue("minimum_collision_clearance_m",
                 std::to_string(cycle_diag_.minimum_collision_clearance_m)),
        KeyValue("minimum_stopping_clearance_m",
                 std::to_string(cycle_diag_.minimum_stopping_clearance_m)),
        KeyValue("warm_start_u_x", std::to_string(cycle_diag_.warm_start_u[0])),
        KeyValue("warm_start_u_y", std::to_string(cycle_diag_.warm_start_u[1])),
        KeyValue("warm_start_u_z", std::to_string(cycle_diag_.warm_start_u[2])),
        KeyValue("warm_start_u_yaw",
                 std::to_string(cycle_diag_.warm_start_u[3])),
        KeyValue("selected_u_x", std::to_string(cycle_diag_.selected_u[0])),
        KeyValue("selected_u_y", std::to_string(cycle_diag_.selected_u[1])),
        KeyValue("selected_u_z", std::to_string(cycle_diag_.selected_u[2])),
        KeyValue("selected_u_yaw", std::to_string(cycle_diag_.selected_u[3])),
        KeyValue("selected_source",
                 ProposalSourceName(static_cast<mppi::ProposalSource>(
                     cycle_diag_.selected_source))),
        KeyValue("N_total", std::to_string(last_rejection_.total)),
        KeyValue("N_safe", std::to_string(last_rejection_.safe)),
        KeyValue("reject_non_finite",
                 std::to_string(last_rejection_.reject_non_finite)),
        KeyValue("reject_swept_collision",
                 std::to_string(last_rejection_.reject_swept_collision)),
        KeyValue("reject_static_collision",
                 std::to_string(last_rejection_.reject_static_collision)),
        KeyValue("reject_dynamic_collision",
                 std::to_string(last_rejection_.reject_dynamic_collision)),
        KeyValue("reject_stopping_distance",
                 std::to_string(last_rejection_.reject_stopping_distance)),
        KeyValue("reject_other",
                 std::to_string(last_rejection_.reject_other)),
        KeyValue("gaussian_generated", std::to_string(gaussian.generated)),
        KeyValue("gaussian_safe", std::to_string(gaussian.safe)),
        KeyValue("gaussian_best_cost", std::to_string(gaussian.best_cost)),
        KeyValue("reference_generated", std::to_string(reference.generated)),
        KeyValue("reference_safe", std::to_string(reference.safe)),
        KeyValue("reference_best_cost", std::to_string(reference.best_cost)),
        KeyValue("braking_generated", std::to_string(braking.generated)),
        KeyValue("braking_safe", std::to_string(braking.safe)),
        KeyValue("braking_best_cost", std::to_string(braking.best_cost)),
        KeyValue("recovery_generated", std::to_string(recovery.generated)),
        KeyValue("recovery_safe", std::to_string(recovery.safe)),
        KeyValue("recovery_best_cost", std::to_string(recovery.best_cost)),
        KeyValue("specific_generated", std::to_string(specific.generated)),
        KeyValue("specific_safe", std::to_string(specific.safe)),
        KeyValue("specific_best_cost", std::to_string(specific.best_cost)),
        KeyValue("local_active_path_id",
                 std::to_string(cycle_diag_.local_active_path_id)),
        KeyValue("previous_path_id",
                 std::to_string(cycle_diag_.previous_path_id)),
        KeyValue("active_path_stamp_ns",
                 std::to_string(cycle_diag_.active_path_stamp_ns)),
        KeyValue("solves_on_active_path",
                 std::to_string(cycle_diag_.solves_on_active_path)),
        KeyValue("first_solve_on_active_path",
                 cycle_diag_.first_solve_on_active_path ? "true" : "false"),
        KeyValue("first_safe_command_on_active_path",
                 cycle_diag_.first_safe_command_on_active_path ? "true"
                                                              : "false")};
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  void PublishVisualization() {
    const auto stamp = now();
    if (selected_trajectory_) {
      nav_msgs::msg::Path path;
      path.header.stamp = stamp;
      path.header.frame_id = planning_frame_;
      for (const auto &point : selected_trajectory_->points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.state.position_enu_m.x;
        pose.pose.position.y = point.state.position_enu_m.y;
        pose.pose.position.z = point.state.position_enu_m.z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(std::move(pose));
      }
      predicted_path_publisher_->publish(path);
    }
    if (!last_result_)
      return;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < last_result_->total_costs.size(); ++i)
      if (last_result_->safe[i]) {
        minimum = std::min(minimum, last_result_->total_costs[i]);
        maximum = std::max(maximum, last_result_->total_costs[i]);
      }
    const auto count = std::min<std::size_t>(last_result_->trajectories.size(),
                                             visualization_max_samples_);
    for (std::size_t i = 0; i < count; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = planning_frame_;
      marker.ns = last_result_->safe[i] ? "mppi_feasible" : "mppi_rejected";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.025;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 0.75;
      if (!last_result_->safe[i]) {
        marker.color.r = 0.45;
        marker.color.g = 0.02;
        marker.color.b = 0.02;
      } else {
        const double ratio =
            maximum > minimum
                ? (last_result_->total_costs[i] - minimum) / (maximum - minimum)
                : 0.0;
        marker.color.r = static_cast<float>(ratio);
        marker.color.g = static_cast<float>(1.0 - 0.5 * ratio);
        marker.color.b = 0.0;
      }
      for (const auto &point : last_result_->trajectories[i].points) {
        geometry_msgs::msg::Point position;
        position.x = point.state.position_enu_m.x;
        position.y = point.state.position_enu_m.y;
        position.z = point.state.position_enu_m.z;
        marker.points.push_back(position);
      }
      array.markers.push_back(std::move(marker));
    }
    sample_publisher_->publish(array);
  }

  std::string planning_frame_;
  std::string body_frame_;
  double control_rate_hz_{10.0};
  double dt_s_{0.1};
  double state_timeout_s_{1.0};
  double obstacle_timeout_s_{1.0};
  double max_compute_time_ms_{80.0};
  double goal_tolerance_m_{0.5};
  double goal_speed_tolerance_m_s_{0.5};
  double visualization_rate_hz_{5.0};
  std::size_t horizon_{30};
  std::size_t samples_{80};
  std::size_t obstacle_max_points_{800};
  std::size_t visualization_max_samples_{80};
  bool proactive_proposals_{true};
  bool visualization_enabled_{true};
  bool have_state_{false};
  bool have_obstacles_{false};
  Mode mode_{Mode::kWaitingForState};
  std::string reason_{"startup"};
  std::string odometry_error_;
  std::string path_error_;
  std::string obstacle_error_;
  SteadyTime last_state_received_{};
  SteadyTime last_obstacles_received_{};
  core::Vec3 goal_{};
  mppi::MppiState state_{};
  core::ObstacleMap obstacles_{};
  mppi::ControlSequence nominal_{};
  std::shared_ptr<const mppi::PathReference> reference_path_{};
  std::unique_ptr<mppi::MultirotorMotionModel> motion_model_;
  std::unique_ptr<mppi::CostEvaluator> cost_evaluator_;
  std::unique_ptr<mppi::MppiOptimizer> optimizer_;
  std::unique_ptr<mppi::GaussianNoiseSampler> noise_sampler_;
  std::unique_ptr<core::TrajectorySafetyChecker> safety_checker_;
  std::unique_ptr<core::VelocityCommandConditioner> conditioner_;
  std::optional<mppi::MppiOptimizationResult> last_result_;
  std::optional<mppi::MppiTrajectory> selected_trajectory_;
  CycleDiagnostics cycle_diag_{};
  mppi::RejectionCounts last_rejection_{};
  std::array<mppi::ProposalSourceStats,
             static_cast<std::size_t>(mppi::ProposalSource::kCount)>
      last_proposal_stats_{};
  double stopping_delay_s_{0.25};
  double braking_accel_m_s2_{3.0};
  SteadyTime previous_cycle_start_{};
  bool have_previous_cycle_{false};
  std::uint64_t cycle_counter_{0};
  std::uint64_t local_active_path_id_{0};
  std::uint64_t previous_path_id_{0};
  std::int64_t active_path_stamp_ns_{0};
  SteadyTime path_received_monotonic_{};
  std::uint64_t solves_on_active_path_{0};
  bool have_first_solve_on_path_{false};
  bool have_first_safe_on_path_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      raw_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      sample_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

} // namespace uav_navigation_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uav_navigation_ros::LocalNavigationNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("local_navigation"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

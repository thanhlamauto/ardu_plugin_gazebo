#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "uav_navigation_core/astar_planner.hpp"
#include "uav_navigation_ros/sdf_static_map_loader.hpp"

namespace core = uav_navigation_core;

namespace uav_navigation_ros {
namespace {

nav_msgs::msg::OccupancyGrid ToRosGrid(const core::CostGrid2D& grid,
                                       const rclcpp::Time& stamp) {
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = grid.frame_id;
  message.info.resolution = static_cast<float>(grid.resolution_m);
  message.info.width = grid.width;
  message.info.height = grid.height;
  message.info.origin.position.x = grid.origin_enu_m.x;
  message.info.origin.position.y = grid.origin_enu_m.y;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.w = 1.0;
  message.data.assign(grid.costs.begin(), grid.costs.end());
  return message;
}

nav_msgs::msg::Path ToRosPath(const core::Path& path, const std::string& frame,
                              const rclcpp::Time& stamp) {
  nav_msgs::msg::Path message;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  message.poses.reserve(path.points_enu_m.size());
  for (const auto& point : path.points_enu_m) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    message.poses.push_back(pose);
  }
  return message;
}

std::string StatusName(core::StatusCode status) {
  switch (status) {
    case core::StatusCode::kOk: return "OK";
    case core::StatusCode::kInvalidInput: return "INVALID_INPUT";
    case core::StatusCode::kStaleInput: return "STALE_INPUT";
    case core::StatusCode::kNoPath: return "NO_PATH";
    case core::StatusCode::kNoSafeTrajectory: return "NO_SAFE_TRAJECTORY";
    case core::StatusCode::kDeadlineMiss: return "DEADLINE_MISS";
    case core::StatusCode::kInternalError: return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

}  // namespace

class GlobalPlannerNode final : public rclcpp::Node {
 public:
  GlobalPlannerNode() : Node("global_planner") {
    const auto algorithm = declare_parameter<std::string>("algorithm", "astar");
    if (algorithm != "astar") throw std::invalid_argument("only algorithm=astar is implemented");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    planning_altitude_m_ = declare_parameter<double>("planning_altitude_m", 5.0);
    const auto world_file = declare_parameter<std::string>("world_file", "");
    const auto resolution = declare_parameter<double>("resolution_m", 0.5);
    const auto clearance = declare_parameter<double>("clearance_m", 1.8);
    const auto vertical_clearance = declare_parameter<double>("vertical_clearance_m", 0.3);
    const auto bounds_padding = declare_parameter<double>("bounds_padding_m", 4.0);
    const auto allow_diagonal = declare_parameter<bool>("allow_diagonal", true);
    const auto prevent_corner_cutting =
        declare_parameter<bool>("prevent_corner_cutting", true);
    const auto max_expansions = declare_parameter<int64_t>("max_expansions", 250000);
    const auto max_grid_cells = declare_parameter<int64_t>("max_grid_cells", 5000000);
    const auto traversal_cost_weight =
        declare_parameter<double>("traversal_cost_weight", 1.0);
    const auto odometry_topic =
        declare_parameter<std::string>("odometry_topic", "/localization/odometry");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto costmap_topic =
        declare_parameter<std::string>("costmap_topic", "/planning/global_costmap");
    const auto path_topic =
        declare_parameter<std::string>("path_topic", "/planning/global_path");
    const auto diagnostics_topic = declare_parameter<std::string>(
        "diagnostics_topic", "/planning/global_planner/diagnostics");
    if (world_file.empty()) throw std::invalid_argument("world_file parameter is required");
    if (max_expansions <= 0 ||
        max_expansions > static_cast<int64_t>(std::numeric_limits<std::uint32_t>::max()) ||
        max_grid_cells <= 0) {
      throw std::invalid_argument("max_expansions and max_grid_cells must be positive");
    }

    StaticMapConfig map_config;
    map_config.frame_id = planning_frame_;
    map_config.resolution_m = resolution;
    map_config.clearance_m = clearance;
    map_config.vertical_clearance_m = vertical_clearance;
    map_config.bounds_padding_m = bounds_padding;
    map_config.max_grid_cells = static_cast<std::size_t>(max_grid_cells);
    map_loader_ = std::make_unique<SdfStaticMapLoader>(map_config);
    map_loader_->Load(world_file);

    core::AStarConfig planner_config;
    planner_config.allow_diagonal = allow_diagonal;
    planner_config.prevent_corner_cutting = prevent_corner_cutting;
    planner_config.max_expansions = static_cast<std::uint32_t>(max_expansions);
    planner_config.traversal_cost_weight = traversal_cost_weight;
    planner_ = std::make_unique<core::AStarPlanner>(planner_config);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, latched_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic, latched_qos);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { OnOdometry(*message); });
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { OnGoal(*message); });

    cost_grid_ = map_loader_->BuildCostGrid(planning_altitude_m_);
    costmap_publisher_->publish(ToRosGrid(cost_grid_, now()));
    RCLCPP_INFO(get_logger(),
                "Loaded %zu SDF obstacles; map=%ux%u at %.2f m; waiting for odometry and goals",
                map_loader_->obstacles().size(), cost_grid_.width, cost_grid_.height,
                cost_grid_.resolution_m);
  }

 private:
  void OnOdometry(const nav_msgs::msg::Odometry& message) {
    if (!message.header.frame_id.empty() && message.header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring odometry in frame '%s'; expected '%s'",
                           message.header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    const auto& p = message.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    state_.stamp_ns = rclcpp::Time(message.header.stamp).nanoseconds();
    state_.position_enu_m = {p.x, p.y, planning_altitude_m_};
    const auto& v = message.twist.twist.linear;
    state_.velocity_enu_m_s = {v.x, v.y, v.z};
    have_odometry_ = true;
  }

  void OnGoal(const geometry_msgs::msg::PoseStamped& message) {
    const auto frame = message.header.frame_id.empty() ? planning_frame_ : message.header.frame_id;
    if (frame != planning_frame_) {
      PublishDiagnostic(core::StatusCode::kInvalidInput,
                        "goal frame '" + frame + "' differs from " + planning_frame_, 0, 0.0);
      RCLCPP_ERROR(get_logger(), "Rejected goal in frame '%s'", frame.c_str());
      return;
    }
    if (!have_odometry_) {
      PublishDiagnostic(core::StatusCode::kStaleInput, "no odometry received", 0, 0.0);
      RCLCPP_WARN(get_logger(), "Rejected goal: no odometry received yet");
      return;
    }
    const auto& p = message.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      PublishDiagnostic(core::StatusCode::kInvalidInput, "goal position is not finite", 0, 0.0);
      return;
    }
    core::Goal goal;
    goal.frame_id = planning_frame_;
    goal.position_enu_m = {p.x, p.y, planning_altitude_m_};
    try {
      cost_grid_ = map_loader_->BuildCostGrid(
          planning_altitude_m_, {state_.position_enu_m, goal.position_enu_m});
      const auto start_time = std::chrono::steady_clock::now();
      auto result = planner_->Plan(state_, goal, cost_grid_);
      const auto elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start_time).count();
      result.diagnostics.compute_time_ms = elapsed;
      const auto stamp = now();
      costmap_publisher_->publish(ToRosGrid(cost_grid_, stamp));
      path_publisher_->publish(ToRosPath(result.path, planning_frame_, stamp));
      PublishDiagnostic(result.status, result.diagnostics.detail,
                        result.diagnostics.expanded_nodes, elapsed);
      if (result.status == core::StatusCode::kOk) {
        RCLCPP_INFO(get_logger(), "A* OK: %zu points, %u expanded, %.2f ms",
                    result.path.points_enu_m.size(), result.diagnostics.expanded_nodes, elapsed);
      } else {
        RCLCPP_WARN(get_logger(), "A* %s: %s", StatusName(result.status).c_str(),
                    result.diagnostics.detail.c_str());
      }
    } catch (const std::exception& error) {
      path_publisher_->publish(ToRosPath({}, planning_frame_, now()));
      PublishDiagnostic(core::StatusCode::kInternalError, error.what(), 0, 0.0);
      RCLCPP_ERROR(get_logger(), "Global planning failed: %s", error.what());
    }
  }

  void PublishDiagnostic(core::StatusCode status, const std::string& detail,
                         std::uint32_t expanded, double compute_time_ms) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = "global_planner";
    item.hardware_id = "sdf_static_map";
    item.level = status == core::StatusCode::kOk
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : (status == core::StatusCode::kNoPath
                            ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                            : diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    item.message = StatusName(status) + ": " + detail;
    diagnostic_msgs::msg::KeyValue expanded_value;
    expanded_value.key = "expanded_nodes";
    expanded_value.value = std::to_string(expanded);
    diagnostic_msgs::msg::KeyValue time_value;
    time_value.key = "compute_time_ms";
    time_value.value = std::to_string(compute_time_ms);
    item.values = {expanded_value, time_value};
    array.status.push_back(item);
    diagnostics_publisher_->publish(array);
  }

  std::string planning_frame_;
  double planning_altitude_m_{5.0};
  bool have_odometry_{false};
  core::State state_;
  core::CostGrid2D cost_grid_;
  std::unique_ptr<SdfStaticMapLoader> map_loader_;
  std::unique_ptr<core::AStarPlanner> planner_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
};

}  // namespace uav_navigation_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uav_navigation_ros::GlobalPlannerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("global_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

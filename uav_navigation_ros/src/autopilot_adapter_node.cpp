#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "uav_navigation_ros/autopilot_adapter.hpp"

namespace uav_navigation_ros {
namespace {

diagnostic_msgs::msg::KeyValue KeyValue(const std::string& key,
                                        const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

class MavrosAutopilotBackend final : public IAutopilotBackend {
 public:
  MavrosAutopilotBackend(rclcpp::Node& node, const std::string& setpoint_topic,
                         const std::string& state_topic,
                         const std::string& output_frame)
      : node_(node), output_frame_(output_frame) {
    setpoint_publisher_ =
        node_.create_publisher<geometry_msgs::msg::TwistStamped>(
            setpoint_topic, rclcpp::SensorDataQoS());
    state_subscription_ = node_.create_subscription<mavros_msgs::msg::State>(
        state_topic, rclcpp::QoS(10).transient_local(),
        [this](mavros_msgs::msg::State::ConstSharedPtr message) {
          status_.connected = message->connected;
          status_.armed = message->armed;
          status_.mode = message->mode;
          have_status_ = true;
          last_status_received_ = std::chrono::steady_clock::now();
        });
  }

  bool SendVelocityCommand(const VelocityCommand& command) override {
    if (setpoint_publisher_->get_subscription_count() == 0) return false;
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = node_.now();
    message.header.frame_id = output_frame_;
    message.twist.linear.x = command.east_m_s;
    message.twist.linear.y = command.north_m_s;
    message.twist.linear.z = command.up_m_s;
    message.twist.angular.z = command.yaw_rate_enu_rad_s;
    setpoint_publisher_->publish(message);
    return true;
  }

  AutopilotStatus GetStatus() const override { return status_; }
  bool HasStatus() const override { return have_status_; }
  double StatusAgeSeconds() const override {
    if (!have_status_) return 0.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         last_status_received_)
        .count();
  }

 private:
  rclcpp::Node& node_;
  std::string output_frame_;
  AutopilotStatus status_;
  bool have_status_{false};
  std::chrono::steady_clock::time_point last_status_received_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      setpoint_publisher_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
};

}  // namespace

class AutopilotAdapterNode final : public rclcpp::Node {
 public:
  AutopilotAdapterNode() : Node("autopilot_adapter") {
    AutopilotAdapterConfig config;
    config.input_frame = declare_parameter<std::string>("input_frame", "odom");
    config.required_mode =
        declare_parameter<std::string>("required_mode", "GUIDED");
    config.command_timeout_s =
        declare_parameter<double>("command_timeout_s", 0.25);
    config.heartbeat_timeout_s =
        declare_parameter<double>("heartbeat_timeout_s", 2.0);
    config.setpoint_rate_hz =
        declare_parameter<double>("setpoint_rate_hz", 10.0);
    config.max_speed_xy_m_s =
        declare_parameter<double>("max_speed_xy_m_s", 10.0);
    config.max_speed_z_m_s =
        declare_parameter<double>("max_speed_z_m_s", 0.6);
    config.max_yaw_rate_rad_s =
        declare_parameter<double>("max_yaw_rate_rad_s", 0.6);
    const bool arm_on_start = declare_parameter<bool>("arm_on_start", false);
    if (arm_on_start) {
      throw std::invalid_argument(
          "arm_on_start=true is unsupported; M6 never arms the vehicle");
    }
    config_ = config;
    translator_ = std::make_unique<CommandTranslator>(config_);
    monitor_ = std::make_unique<ConnectionMonitor>(config_);

    const auto input_topic = declare_parameter<std::string>(
        "input_topic", "/control/safe_velocity_command");
    const auto mavros_setpoint_topic = declare_parameter<std::string>(
        "mavros_setpoint_topic", "/mavros/setpoint_velocity/cmd_vel");
    const auto mavros_state_topic =
        declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    const auto diagnostics_topic =
        declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    backend_ = std::make_unique<MavrosAutopilotBackend>(
        *this, mavros_setpoint_topic, mavros_state_topic, config_.input_frame);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic, 10);
    command_subscription_ =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            input_topic, rclcpp::QoS(1).reliable(),
            [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
              OnCommand(*message);
            });
    stream_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / config_.setpoint_rate_hz),
        [this] { StreamCycle(); });
    RCLCPP_INFO(get_logger(),
                "C++ MAVROS adapter ready: %.1f Hz, command timeout %.0f ms, "
                "required mode %s; automatic arming disabled",
                config_.setpoint_rate_hz, config_.command_timeout_s * 1000.0,
                config_.required_mode.c_str());
  }

 private:
  void OnCommand(const geometry_msgs::msg::TwistStamped& message) {
    std::string rejection;
    auto translated = translator_->Validate(
        rclcpp::Time(message.header.stamp).nanoseconds(),
        message.header.frame_id, message.twist.linear.x, message.twist.linear.y,
        message.twist.linear.z, message.twist.angular.z, &rejection);
    if (!translated) {
      command_fault_ = std::move(rejection);
      last_command_.reset();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "safe command rejected: %s",
                            command_fault_.c_str());
      return;
    }
    command_fault_.clear();
    last_command_ = *translated;
    last_command_received_ = std::chrono::steady_clock::now();
  }

  void StreamCycle() {
    const bool have_command = last_command_.has_value();
    const double command_age_s =
        have_command
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             last_command_received_)
                  .count()
            : 0.0;
    const auto decision = monitor_->Evaluate(
        backend_->GetStatus(), backend_->HasStatus(),
        backend_->StatusAgeSeconds(), have_command, command_age_s,
        command_fault_);
    state_ = decision.state;
    reason_ = decision.reason;
    last_send_success_ = false;
    if (decision.send_command && last_command_) {
      last_send_success_ = backend_->SendVelocityCommand(*last_command_);
      if (!last_send_success_) {
        state_ = AdapterState::kFault;
        reason_ = "MAVROS backend rejected setpoint publication";
      }
    }
    PublishDiagnostic(command_age_s);
  }

  void PublishDiagnostic(double command_age_s) {
    const auto status_value = backend_->GetStatus();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "autopilot_adapter";
    status.hardware_id = "mavros_fcu";
    status.level = state_ == AdapterState::kStreaming ||
                           state_ == AdapterState::kReady
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                   : state_ == AdapterState::kConnectedNotReady
                       ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                       : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = std::string(AdapterStateName(state_)) + ": " + reason_;
    status.values = {
        KeyValue("adapter_state", AdapterStateName(state_)),
        KeyValue("fault_reason", reason_),
        KeyValue("connected", status_value.connected ? "true" : "false"),
        KeyValue("armed", status_value.armed ? "true" : "false"),
        KeyValue("mode", status_value.mode),
        KeyValue("command_age_ms",
                 last_command_ ? std::to_string(command_age_s * 1000.0)
                               : "-1"),
        KeyValue("heartbeat_age_ms",
                 backend_->HasStatus()
                     ? std::to_string(backend_->StatusAgeSeconds() * 1000.0)
                     : "-1"),
        KeyValue("setpoint_rate_hz",
                 std::to_string(config_.setpoint_rate_hz)),
        KeyValue("last_send_success",
                 last_send_success_ ? "true" : "false")};
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  AutopilotAdapterConfig config_;
  AdapterState state_{AdapterState::kDisconnected};
  std::string reason_{"startup"};
  std::string command_fault_;
  bool last_send_success_{false};
  std::optional<VelocityCommand> last_command_;
  std::chrono::steady_clock::time_point last_command_received_{};
  std::unique_ptr<CommandTranslator> translator_;
  std::unique_ptr<ConnectionMonitor> monitor_;
  std::unique_ptr<IAutopilotBackend> backend_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      command_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
};

}  // namespace uav_navigation_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uav_navigation_ros::AutopilotAdapterNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("autopilot_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include "uav_navigation_ros/autopilot_adapter.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace uav_navigation_ros {
namespace {

bool Finite(double value) { return std::isfinite(value); }

void Reject(std::string* output, const std::string& reason) {
  if (output != nullptr) *output = reason;
}

void ValidateConfig(const AutopilotAdapterConfig& config) {
  if (config.input_frame.empty() || config.required_mode.empty() ||
      !Finite(config.command_timeout_s) || config.command_timeout_s <= 0.0 ||
      !Finite(config.heartbeat_timeout_s) ||
      config.heartbeat_timeout_s <= 0.0 ||
      !Finite(config.setpoint_rate_hz) || config.setpoint_rate_hz <= 0.0 ||
      !Finite(config.max_speed_xy_m_s) || config.max_speed_xy_m_s <= 0.0 ||
      !Finite(config.max_speed_z_m_s) || config.max_speed_z_m_s <= 0.0 ||
      !Finite(config.max_yaw_rate_rad_s) ||
      config.max_yaw_rate_rad_s <= 0.0) {
    throw std::invalid_argument("invalid autopilot adapter configuration");
  }
}

}  // namespace

const char* AdapterStateName(AdapterState state) {
  switch (state) {
    case AdapterState::kDisconnected:
      return "DISCONNECTED";
    case AdapterState::kConnectedNotReady:
      return "CONNECTED_NOT_READY";
    case AdapterState::kReady:
      return "READY";
    case AdapterState::kStreaming:
      return "STREAMING";
    case AdapterState::kFault:
      return "FAULT";
    case AdapterState::kStaleCommand:
      return "STALE_COMMAND";
  }
  return "UNKNOWN";
}

CommandTranslator::CommandTranslator(AutopilotAdapterConfig config)
    : config_(std::move(config)) {
  ValidateConfig(config_);
}

std::optional<VelocityCommand> CommandTranslator::Validate(
    std::int64_t stamp_ns, const std::string& frame_id, double linear_x,
    double linear_y, double linear_z, double angular_z,
    std::string* rejection_reason) const {
  if (frame_id.empty()) {
    Reject(rejection_reason, "command frame is empty");
    return std::nullopt;
  }
  if (frame_id != config_.input_frame) {
    Reject(rejection_reason, "command frame mismatch");
    return std::nullopt;
  }
  if (!Finite(linear_x) || !Finite(linear_y) || !Finite(linear_z) ||
      !Finite(angular_z)) {
    Reject(rejection_reason, "non-finite command");
    return std::nullopt;
  }
  if (std::hypot(linear_x, linear_y) > config_.max_speed_xy_m_s + 1e-9 ||
      std::abs(linear_z) > config_.max_speed_z_m_s + 1e-9 ||
      std::abs(angular_z) > config_.max_yaw_rate_rad_s + 1e-9) {
    Reject(rejection_reason, "command outside configured limits");
    return std::nullopt;
  }
  if (rejection_reason != nullptr) rejection_reason->clear();
  // MAVROS setpoint_velocity owns ENU -> NED conversion. Preserve ROS ENU here.
  return VelocityCommand{stamp_ns, linear_x, linear_y, linear_z, angular_z};
}

ConnectionMonitor::ConnectionMonitor(AutopilotAdapterConfig config)
    : config_(std::move(config)) {
  ValidateConfig(config_);
}

AdapterDecision ConnectionMonitor::Evaluate(
    const AutopilotStatus& status, bool have_status, double status_age_s,
    bool have_command, double command_age_s,
    const std::string& command_fault) const {
  if (!have_status) {
    return {AdapterState::kDisconnected, "no MAVROS FCU state received", false};
  }
  if (!Finite(status_age_s) || status_age_s > config_.heartbeat_timeout_s) {
    return {AdapterState::kDisconnected, "MAVROS FCU state is stale", false};
  }
  if (!status.connected) {
    return {AdapterState::kDisconnected, "MAVROS reports FCU disconnected",
            false};
  }
  if (!command_fault.empty()) {
    return {AdapterState::kFault, command_fault, false};
  }
  if (!status.armed) {
    return {AdapterState::kConnectedNotReady, "FCU is disarmed", false};
  }
  if (status.mode != config_.required_mode) {
    return {AdapterState::kConnectedNotReady,
            "FCU mode is " + status.mode + "; expected " +
                config_.required_mode,
            false};
  }
  if (!have_command) {
    return {AdapterState::kReady, "waiting for first safe command", false};
  }
  if (!Finite(command_age_s) || command_age_s > config_.command_timeout_s) {
    return {AdapterState::kStaleCommand, "safe command is stale", false};
  }
  return {AdapterState::kStreaming, "fresh safe command forwarded", true};
}

}  // namespace uav_navigation_ros

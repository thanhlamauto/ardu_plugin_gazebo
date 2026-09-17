#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace uav_navigation_ros {

struct VelocityCommand {
  std::int64_t stamp_ns{0};
  double east_m_s{0.0};
  double north_m_s{0.0};
  double up_m_s{0.0};
  double yaw_rate_enu_rad_s{0.0};
};

struct AutopilotStatus {
  bool connected{false};
  bool armed{false};
  std::string mode;
};

enum class AdapterState {
  kDisconnected,
  kConnectedNotReady,
  kReady,
  kStreaming,
  kFault,
  kStaleCommand,
};

const char* AdapterStateName(AdapterState state);

struct AutopilotAdapterConfig {
  std::string input_frame{"odom"};
  std::string required_mode{"GUIDED"};
  double command_timeout_s{0.25};
  double heartbeat_timeout_s{2.0};
  double setpoint_rate_hz{10.0};
  double max_speed_xy_m_s{10.0};
  double max_speed_z_m_s{0.6};
  double max_yaw_rate_rad_s{0.6};
};

class CommandTranslator {
 public:
  explicit CommandTranslator(AutopilotAdapterConfig config);

  std::optional<VelocityCommand> Validate(
      std::int64_t stamp_ns, const std::string& frame_id, double linear_x,
      double linear_y, double linear_z, double angular_z,
      std::string* rejection_reason = nullptr) const;

 private:
  AutopilotAdapterConfig config_;
};

struct AdapterDecision {
  AdapterState state{AdapterState::kDisconnected};
  std::string reason;
  bool send_command{false};
};

class ConnectionMonitor {
 public:
  explicit ConnectionMonitor(AutopilotAdapterConfig config);

  AdapterDecision Evaluate(const AutopilotStatus& status, bool have_status,
                           double status_age_s, bool have_command,
                           double command_age_s,
                           const std::string& command_fault) const;

 private:
  AutopilotAdapterConfig config_;
};

class IAutopilotBackend {
 public:
  virtual ~IAutopilotBackend() = default;
  virtual bool SendVelocityCommand(const VelocityCommand& command) = 0;
  virtual AutopilotStatus GetStatus() const = 0;
  virtual bool HasStatus() const = 0;
  virtual double StatusAgeSeconds() const = 0;
};

}  // namespace uav_navigation_ros

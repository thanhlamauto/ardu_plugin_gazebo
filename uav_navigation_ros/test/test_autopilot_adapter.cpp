#include "uav_navigation_ros/autopilot_adapter.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace nav = uav_navigation_ros;
namespace {

int failures = 0;

void Check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

void TranslatorTests() {
  nav::AutopilotAdapterConfig config;
  nav::CommandTranslator translator(config);
  std::string reason;

  const auto east = translator.Validate(1, "odom", 2.0, 0.0, 0.0, 0.0, &reason);
  Check(east && east->east_m_s == 2.0 && east->north_m_s == 0.0,
        "+X remains ENU east for MAVROS conversion");
  const auto north = translator.Validate(2, "odom", 0.0, 2.0, 0.0, 0.0);
  Check(north && north->north_m_s == 2.0,
        "+Y remains ENU north for MAVROS conversion");
  const auto up = translator.Validate(3, "odom", 0.0, 0.0, 0.5, 0.0);
  Check(up && up->up_m_s == 0.5,
        "+Z remains ENU up for MAVROS conversion");
  const auto yaw = translator.Validate(4, "odom", 0.0, 0.0, 0.0, 0.4);
  Check(yaw && yaw->yaw_rate_enu_rad_s == 0.4,
        "positive ROS yaw rate is preserved for MAVROS conversion");

  Check(!translator.Validate(0, "map", 0.0, 0.0, 0.0, 0.0, &reason) &&
            reason == "command frame mismatch",
        "wrong frame rejected");
  Check(!translator.Validate(0, "", 0.0, 0.0, 0.0, 0.0, &reason) &&
            reason == "command frame is empty",
        "missing frame rejected");
  Check(!translator.Validate(0, "odom",
                             std::numeric_limits<double>::quiet_NaN(), 0.0,
                             0.0, 0.0, &reason) &&
            reason == "non-finite command",
        "NaN rejected");
  Check(!translator.Validate(0, "odom", 8.0, 8.0, 0.0, 0.0, &reason) &&
            reason == "command outside configured limits",
        "XY vector outside limit rejected");
  Check(!translator.Validate(0, "odom", 0.0, 0.0, 0.7, 0.0, &reason),
        "vertical command outside limit rejected");
  Check(!translator.Validate(0, "odom", 0.0, 0.0, 0.0, 0.7, &reason),
        "yaw-rate command outside limit rejected");
}

void MonitorTests() {
  nav::AutopilotAdapterConfig config;
  nav::ConnectionMonitor monitor(config);
  nav::AutopilotStatus status{true, true, "GUIDED"};

  Check(monitor.Evaluate(status, false, 0.0, false, 0.0, {}).state ==
            nav::AdapterState::kDisconnected,
        "missing FCU state is disconnected");
  Check(monitor.Evaluate(status, true, 3.0, false, 0.0, {}).state ==
            nav::AdapterState::kDisconnected,
        "stale heartbeat is disconnected");
  status.connected = false;
  Check(monitor.Evaluate(status, true, 0.0, false, 0.0, {}).state ==
            nav::AdapterState::kDisconnected,
        "FCU disconnect reported");
  status.connected = true;
  status.armed = false;
  Check(monitor.Evaluate(status, true, 0.0, false, 0.0, {}).state ==
            nav::AdapterState::kConnectedNotReady,
        "disarmed FCU is not ready");
  status.armed = true;
  status.mode = "LOITER";
  Check(monitor.Evaluate(status, true, 0.0, true, 0.0, {}).state ==
            nav::AdapterState::kConnectedNotReady,
        "wrong mode is not ready");
  status.mode = "GUIDED";
  Check(monitor.Evaluate(status, true, 0.0, false, 0.0, {}).state ==
            nav::AdapterState::kReady,
        "armed GUIDED FCU waits without inventing a command");
  Check(monitor.Evaluate(status, true, 0.0, true, 0.3, {}).state ==
            nav::AdapterState::kStaleCommand,
        "stale command is not resent");
  const auto streaming = monitor.Evaluate(status, true, 0.0, true, 0.1, {});
  Check(streaming.state == nav::AdapterState::kStreaming &&
            streaming.send_command,
        "fresh command streams only when FCU is ready");
  Check(monitor.Evaluate(status, true, 0.0, true, 0.1, "invalid command")
                .state == nav::AdapterState::kFault,
        "invalid command creates deterministic fault");
}

void InvalidConfigTest() {
  nav::AutopilotAdapterConfig config;
  config.command_timeout_s = 0.0;
  bool threw = false;
  try {
    nav::CommandTranslator invalid(config);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  Check(threw, "invalid configuration rejected");
}

}  // namespace

int main() {
  TranslatorTests();
  MonitorTests();
  InvalidConfigTest();
  return failures == 0 ? 0 : 1;
}

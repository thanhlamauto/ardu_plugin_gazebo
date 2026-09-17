#!/usr/bin/env python3
"""Thin M5 bridge from a verified ENU TwistStamped to ArduPilot SITL.

This keeps the validated velocity+yaw-rate MAVLink message in Python for M5.
It does not arm, change mode, take off, or synthesize commands when the C++
local-navigation node stops publishing. The C++ replacement belongs to M6.
"""
import math
import os
import time

os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")
os.environ["MAVLINK20"] = "1"

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


TYPE_MASK_VEL_YAWRATE = 1 | 2 | 4 | 64 | 128 | 256 | 1024
assert TYPE_MASK_VEL_YAWRATE == 1479


def enu_to_ned(vx, vy, vz):
    return float(vy), float(vx), -float(vz)


class SafeTwistToMavlink(Node):
    def __init__(self):
        super().__init__("python_sitl_command_bridge")
        self.declare_parameter("connection_url", "tcp:127.0.0.1:5762")
        self.declare_parameter("input_topic", "/control/safe_velocity_command")
        self.declare_parameter("input_frame", "odom")
        connection = self.get_parameter("connection_url").value
        topic = self.get_parameter("input_topic").value
        self.input_frame = self.get_parameter("input_frame").value
        mavutil.set_dialect("ardupilotmega")
        self.master = mavutil.mavlink_connection(
            connection, source_system=255, source_component=191,
            dialect="ardupilotmega")
        self.get_logger().info(f"waiting for ArduPilot heartbeat on {connection}")
        if self.master.wait_heartbeat(timeout=10) is None:
            raise RuntimeError(f"no ArduPilot heartbeat on {connection}")
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            TwistStamped, topic, self.on_command, qos)
        self.get_logger().info(
            f"forwarding verified commands {topic} -> {connection}; no auto-arm")

    def publish_status(self, level, message):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "python_sitl_command_bridge"
        status.hardware_id = "ardupilot_sitl"
        status.level = level
        status.message = message
        status.values = [KeyValue(key="type_mask", value=str(TYPE_MASK_VEL_YAWRATE))]
        array.status = [status]
        self.diagnostics.publish(array)

    def on_command(self, message):
        frame = message.header.frame_id or self.input_frame
        values = (message.twist.linear.x, message.twist.linear.y,
                  message.twist.linear.z, message.twist.angular.z)
        if frame != self.input_frame:
            self.publish_status(DiagnosticStatus.ERROR,
                                f"rejected frame {frame}; expected {self.input_frame}")
            return
        if not all(math.isfinite(value) for value in values):
            self.publish_status(DiagnosticStatus.ERROR, "rejected non-finite command")
            return
        vn, ve, vd = enu_to_ned(*values[:3])
        yaw_rate_ned = -float(values[3])
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK_VEL_YAWRATE,
            0.0, 0.0, 0.0, vn, ve, vd, 0.0, 0.0, 0.0,
            0.0, yaw_rate_ned)
        self.publish_status(DiagnosticStatus.OK, "safe command forwarded")


def main():
    rclpy.init()
    node = None
    try:
        node = SafeTwistToMavlink()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""MPPI local planner chạy trên companion cho ArduPilot SITL + Gazebo.

Kiến trúc (theo chốt với mentor): ArduPilot tắt avoidance riêng, chỉ làm
flight controller bám velocity setpoint; node này tiêu thụ point cloud
LiDAR 3D, chạy MPPI và gửi ``SET_POSITION_TARGET_LOCAL_NED`` ở GUIDED.

Các module:

- ``mppi_controller``: MPPI 7-state/4-action ``u = [vx, vy, vz, yaw_rate]``.
- ``pa_mppi_controller``: PA-MPPI v0 với occupancy grid và perception cost.
- ``mavlink_interface``: nối MAVLink (telemetry + velocity setpoint).
- ``lidar_preprocess``: chuyển obstacle BODY_FRD sang frame quy hoạch.
- ``mppi_local_planner_node``: vòng điều khiển + waypoint + an toàn.
"""

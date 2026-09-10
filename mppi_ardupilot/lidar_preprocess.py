"""Tiền xử lý LiDAR cho MPPI: filter/downsample rồi đưa về frame quy hoạch.

Chuỗi mới (không còn gửi LiDAR vào AP_Proximity)::

    point cloud (sensor FLU)
      -> filter/downsample (giữ nguyên thuật toán week 3: bin yaw/pitch,
         mỗi cell giữ điểm gần nhất, tối đa ~180-200 điểm)
      -> BODY_FRD
      -> LOCAL frame (NED hoặc ENU, tùy state source)
      -> MPPI obstacle cost (không cần object ID / tracking)

Với demo bay gần như level, xoay yaw-only là đủ; lên quad thật thì thay
bằng full attitude quaternion (xem hàm full-attitude trong script demo).
"""

import math
from typing import Sequence, Tuple

import numpy as np

Point = Tuple[float, float, float]


def body_frd_to_ned(
    points_frd: np.ndarray, position_ned: Sequence[float], yaw: float
) -> np.ndarray:
    """Điểm BODY_FRD (N x 3) -> LOCAL_NED tuyệt đối, xoay yaw-only.

    yaw: heading NED [rad], 0 = North, dương theo chiều kim đồng hồ.
    position_ned: [N, E, D] vị trí drone trong frame NED.
    """
    points_frd = np.asarray(points_frd, dtype=np.float64)
    if points_frd.size == 0:
        return np.zeros((0, 3))
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = points_frd[:, 0], points_frd[:, 1], points_frd[:, 2]
    out = np.column_stack((c * x - s * y, s * x + c * y, z))
    out += np.asarray(position_ned, dtype=np.float64)
    return out


def enu_to_ned_vel(vel_enu: Sequence[float]) -> Tuple[float, float, float]:
    """ENU (east, north, up) -> NED (north, east, down)."""
    vx, vy, vz = (float(vel_enu[0]), float(vel_enu[1]), float(vel_enu[2]))
    return vy, vx, -vz


def ned_to_enu_pos(pos_ned: Sequence[float]) -> np.ndarray:
    """Vị trí NED tương đối home -> ENU tương đối home (cùng gốc)."""
    n, e, d = (float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]))
    return np.array([e, n, -d], dtype=np.float64)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """Heading NED (0=N, chiều kim đồng hồ) -> yaw math ENU (0=E, CCW)."""
    return wrap_angle(math.pi / 2 - yaw_ned)


def yaw_rate_ned_to_enu(yaw_rate_ned: float) -> float:
    """Cùng một cú quay vật lý, yaw_rate hai frame ngược dấu nhau."""
    return -float(yaw_rate_ned)


def yaw_enu_to_ned_rate(yaw_rate_enu: float) -> float:
    """yaw_rate lệnh từ planner (ENU) -> yaw_rate gửi MAVLink (NED)."""
    return -float(yaw_rate_enu)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Ma trận xoay body FLU -> world ENU từ quaternion Hamilton."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def frd_to_flu(p_frd: np.ndarray) -> np.ndarray:
    """[N,3] BODY_FRD -> body FLU (đảo trục right/down)."""
    out = p_frd.copy()
    out[:, 1] = -out[:, 1]
    out[:, 2] = -out[:, 2]
    return out


def yaw_enu_from_rot(rot_flu_to_enu: np.ndarray) -> float:
    """Yaw math ENU (0=E, CCW) trích từ ma trận xoay body->world."""
    return math.atan2(rot_flu_to_enu[1, 0], rot_flu_to_enu[0, 0])


def scan_to_world_enu(raw_points_flu, pos_enu: np.ndarray,
                      rot_flu_to_enu: np.ndarray, *, max_points: int,
                      sensor_offset_body_frd: Point) -> np.ndarray:
    """Cloud FLU sensor -> obstacle world-ENU [N,3] (full attitude).

    Dùng khi có odometry full quaternion (SITL demo mặc định): chính xác
    hơn yaw-only khi quad nghiêng. Cần scripts/ trên sys.path.
    """
    obstacles_frd = downsample_to_body_frd(
        raw_points_flu, max_points=max_points,
        sensor_offset_body_frd=sensor_offset_body_frd,
    )
    if not obstacles_frd:
        return np.zeros((0, 3))
    flu = frd_to_flu(np.asarray(obstacles_frd, dtype=np.float64))
    return pos_enu + (rot_flu_to_enu @ flu.T).T


def downsample_to_body_frd(points_flu, *, max_points: int,
                            sensor_offset_body_frd: Point):
    """Dùng lại đúng thuật toán downsample week 3 (không sửa gì).

    Trả về list điểm BODY_FRD. Cần scripts/ trên sys.path để import
    lidar_to_mavlink_avoidance.
    """
    try:
        import lidar_to_mavlink_avoidance as lidar_bridge
    except ImportError as exc:
        raise SystemExit(
            "[error] cần scripts/lidar_to_mavlink_avoidance.py trên sys.path "
            f"để tái dùng downsample week 3: {exc}"
        ) from exc
    return lidar_bridge.point_cloud_to_obstacles(
        points_flu,
        max_points=max_points,
        sensor_offset_body_frd=sensor_offset_body_frd,
    )

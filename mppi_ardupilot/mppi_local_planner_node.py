"""Node MPPI local planner: LiDAR + state -> u -> MAVLink velocity target.

Pipeline (companion-side, ArduPilot OA đã tắt)::

    LiDAR 3D ---> preprocess ---> local obstacle cloud --+
    MAVLink telemetry / Gazebo odometry (pos, vel, yaw) --+--> MPPI planner
                                                            -> u = [vx, vy, vz, yaw_rate]
                                                            -> MAVLink velocity target
                                                            -> ArduPilot GUIDED

Chạy SAU khi copter đã takeoff và ở GUIDED. Ctrl+C dừng stream, copter
hold sau GUID_TIMEOUT; hạ cánh bằng ``mode land``.
"""

import threading
import time
import json
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional

import numpy as np

from .lidar_preprocess import (
    body_frd_to_ned,
    enu_to_ned_vel,
    ned_to_enu_pos,
    quat_to_rot,
    scan_to_world_enu,
    yaw_enu_from_rot,
    yaw_enu_to_ned_rate,
    yaw_ned_to_enu,
)
from .mavlink_interface import ArduPilotInterface
from .mppi_controller import MPPIConfig, QuadMPPI

DEFAULT_GOAL = "16,10,20;30,0,20"
SAFE_HOLD_EVENTS = frozenset(
    ("hold-stale", "hold-brake", "hold-timeout", "hold-await-goal", "reached")
)


@dataclass
class PlannerState:
    """State trong frame làm việc ENU: pos [3], vel [3], yaw math [rad]."""

    pos: np.ndarray
    vel: np.ndarray
    yaw: float
    quat_wxyz: Optional[np.ndarray] = None
    omega_body_flu: Optional[np.ndarray] = None
    source_age_s: float = 0.0
    source_timestamp_s: Optional[float] = None


@dataclass
class PlannerStep:
    u: np.ndarray  # [vx, vy, vz, yaw_rate] frame làm việc ENU
    nearest_m: float
    goal_dist_m: float
    event: str = ""  # hold/recover/reached/waypoint/command state
    diagnostics: Optional[dict] = None


class VelocityCommandConditioner:
    """Low-pass and slew-limit velocity commands between planner cycles."""

    def __init__(self, dt: float, alpha: float, max_accel_xy: float,
                 max_accel_z: float, max_yaw_accel: float, *,
                 u_min=None, u_max=None) -> None:
        if dt <= 0 or not 0.0 < alpha <= 1.0:
            raise ValueError("dt must be > 0 and command alpha in (0, 1]")
        limits = np.asarray([max_accel_xy, max_accel_z, max_yaw_accel], dtype=float)
        if not np.isfinite(limits).all() or np.any(limits <= 0):
            raise ValueError("command acceleration limits must be finite and > 0")
        self.dt = float(dt)
        self.alpha = float(alpha)
        self.max_accel_xy = float(max_accel_xy)
        self.max_accel_z = float(max_accel_z)
        self.max_yaw_accel = float(max_yaw_accel)
        self.u_min = None if u_min is None else np.asarray(u_min, dtype=float)
        self.u_max = None if u_max is None else np.asarray(u_max, dtype=float)
        if (self.u_min is None) != (self.u_max is None):
            raise ValueError("u_min and u_max must be provided together")
        if self.u_min is not None and (
            self.u_min.shape != (4,) or self.u_max.shape != (4,)
            or not np.isfinite(self.u_min).all() or not np.isfinite(self.u_max).all()
            or np.any(self.u_min >= self.u_max)
        ):
            raise ValueError("command bounds must be finite four-vectors with min < max")
        self.previous: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.previous = None

    def apply(self, command, measured_velocity) -> np.ndarray:
        raw = np.asarray(command, dtype=np.float64)
        if raw.shape != (4,) or not np.isfinite(raw).all():
            raise ValueError("velocity command must be a finite 4-vector")
        if self.previous is None:
            measured = np.asarray(measured_velocity, dtype=np.float64)
            if measured.shape != (3,) or not np.isfinite(measured).all():
                raise ValueError("measured velocity must be a finite 3-vector")
            self.previous = np.r_[measured, 0.0]
            if self.u_min is not None:
                self.previous = np.clip(self.previous, self.u_min, self.u_max)

        target = self.previous + self.alpha * (raw - self.previous)
        delta = target - self.previous
        horizontal_limit = self.max_accel_xy * self.dt
        horizontal_norm = float(np.linalg.norm(delta[0:2]))
        if horizontal_norm > horizontal_limit:
            delta[0:2] *= horizontal_limit / horizontal_norm
        delta[2] = np.clip(delta[2], -self.max_accel_z * self.dt,
                           self.max_accel_z * self.dt)
        delta[3] = np.clip(delta[3], -self.max_yaw_accel * self.dt,
                           self.max_yaw_accel * self.dt)
        self.previous = self.previous + delta
        if self.u_min is not None:
            self.previous = np.clip(self.previous, self.u_min, self.u_max)
        return self.previous.copy()


class LocalPlannerNode:
    """Logic planner thuần (không I/O): unit-test được qua step()."""

    def __init__(
        self,
        planner: QuadMPPI,
        waypoints: List[np.ndarray],
        *,
        wp_radius: float = 2.5,
        goal_radius: float = 1.0,
        hard_brake_m: float = 1.0,
        hard_brake_release_m: Optional[float] = None,
        hard_brake_delay_s: float = 0.25,
        brake_accel_m_s2: float = 1.5,
        recovery_speed_m_s: float = 0.4,
        planner_timeout_ms: Optional[float] = None,
        command_conditioner: Optional[VelocityCommandConditioner] = None,
        reference_path: Optional[List[np.ndarray]] = None,
        goal_slowdown_radius: Optional[float] = None,
        goal_approach_gain: float = 0.5,
        goal_min_speed: float = 0.1,
    ) -> None:
        if not waypoints:
            raise ValueError("cần ít nhất 1 waypoint")
        self.planner = planner
        self.waypoints = [np.asarray(w, dtype=np.float64) for w in waypoints]
        self.wp_radius = wp_radius
        self.goal_radius = goal_radius
        self.hard_brake_m = hard_brake_m
        self.hard_brake_release_m = (
            hard_brake_m + 0.8 if hard_brake_release_m is None
            else hard_brake_release_m
        )
        self.hard_brake_delay_s = hard_brake_delay_s
        self.brake_accel_m_s2 = brake_accel_m_s2
        self.recovery_speed_m_s = recovery_speed_m_s
        if self.hard_brake_m < 0 or self.hard_brake_release_m < self.hard_brake_m:
            raise ValueError("hard-brake release must be >= trigger distance")
        if min(self.brake_accel_m_s2, self.recovery_speed_m_s) <= 0:
            raise ValueError("brake acceleration and recovery speed must be > 0")
        if self.hard_brake_delay_s < 0:
            raise ValueError("hard-brake delay must be >= 0")
        self.planner_timeout_ms = planner_timeout_ms
        self.command_conditioner = command_conditioner
        self.reference_path = None if reference_path is None else [
            np.asarray(point, dtype=np.float64) for point in reference_path
        ]
        self.goal_slowdown_radius = goal_slowdown_radius
        self.goal_approach_gain = goal_approach_gain
        self.goal_min_speed = goal_min_speed
        self.wp_index = 0
        self.reached = False
        self.recovery_active = False
        self._recovery_has_braked = False
        planner.update_goal(self.waypoints[0])
        if self.reference_path is not None and hasattr(planner, "update_reference_path"):
            planner.update_reference_path(self.reference_path)

    def replace_route(
        self, waypoints: List[np.ndarray], *,
        reference_path: Optional[List[np.ndarray]] = None,
    ) -> None:
        """Atomically replace the active route, e.g. from an RViz goal click."""
        if not waypoints:
            raise ValueError("route mới cần ít nhất một waypoint")
        route = [np.asarray(point, dtype=np.float64) for point in waypoints]
        if any(point.shape != (3,) or not np.isfinite(point).all() for point in route):
            raise ValueError("mọi waypoint phải là finite 3-vector ENU")
        reference = None
        if reference_path is not None:
            reference = [np.asarray(point, dtype=np.float64) for point in reference_path]
            if any(point.shape != (3,) or not np.isfinite(point).all()
                   for point in reference):
                raise ValueError("reference path phải gồm finite 3-vector ENU")
            if len(reference) < 2:
                raise ValueError("reference path mới cần ít nhất hai điểm")

        self.waypoints = route
        self.reference_path = reference
        self.wp_index = 0
        self.reached = False
        self.recovery_active = False
        self._recovery_has_braked = False
        if self.command_conditioner is not None:
            self.command_conditioner.reset()
        if hasattr(self.planner, "reset_for_new_route"):
            self.planner.reset_for_new_route()
        elif hasattr(self.planner, "reset_applied_control"):
            self.planner.reset_applied_control()
        self.planner.update_goal(route[0])
        if hasattr(self.planner, "update_reference_path"):
            self.planner.update_reference_path(reference)

    def step(
        self, state: Optional[PlannerState], obstacles
    ) -> PlannerStep:
        """Một chu kỳ: state + obstacle cloud -> u (ENU, 4 chiều)."""
        if state is None:
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            return PlannerStep(np.zeros(4), float("inf"), float("inf"), "hold-stale")
        if hasattr(self.planner, "update_occupancy"):
            self.planner.update_occupancy(state.pos, obstacles)
        self.planner.update_obstacles(obstacles)
        nearest = float("inf")
        nearest_point = None
        if obstacles is not None and len(obstacles) > 0:
            obstacle_array = np.asarray(obstacles, dtype=np.float64)
            distances = np.linalg.norm(obstacle_array - state.pos, axis=1)
            nearest_index = int(np.argmin(distances))
            nearest = float(distances[nearest_index])
            nearest_point = obstacle_array[nearest_index]
        if self.reached:
            goal_dist = float(np.linalg.norm(state.pos - self.waypoints[-1]))
            return PlannerStep(np.zeros(4), nearest, goal_dist, "reached")
        if self.wp_index < len(self.waypoints) - 1 and (
            np.linalg.norm(state.pos - self.waypoints[self.wp_index]) < self.wp_radius
        ):
            self.wp_index += 1
            self.planner.update_goal(self.waypoints[self.wp_index])
            event = "waypoint"
        else:
            event = ""
        goal_dist = float(np.linalg.norm(state.pos - self.waypoints[self.wp_index]))
        if self.wp_index == len(self.waypoints) - 1 and goal_dist < self.goal_radius:
            self.reached = True
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            return PlannerStep(np.zeros(4), nearest, goal_dist, "reached")
        closing_speed = 0.0
        safety_trigger_m = self.hard_brake_m
        if nearest_point is not None and nearest > 1e-9:
            toward_obstacle = (nearest_point - state.pos) / nearest
            closing_speed = max(0.0, float(np.dot(state.vel, toward_obstacle)))
            safety_trigger_m += (
                closing_speed * self.hard_brake_delay_s
                + closing_speed**2 / (2.0 * self.brake_accel_m_s2)
            )
        if self.recovery_active and nearest >= self.hard_brake_release_m:
            self.recovery_active = False
            self._recovery_has_braked = False
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            if hasattr(self.planner, "reset_applied_control"):
                self.planner.reset_applied_control()
        if self.hard_brake_m > 0 and (
            self.recovery_active or nearest < safety_trigger_m
        ):
            newly_triggered = not self.recovery_active
            self.recovery_active = True
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            if hasattr(self.planner, "reset_applied_control"):
                self.planner.reset_applied_control()
            safety_diag = {
                "closing_speed_m_s": closing_speed,
                "safety_trigger_m": safety_trigger_m,
                "hard_brake_release_m": self.hard_brake_release_m,
            }
            horizontal_speed = float(np.linalg.norm(state.vel[0:2]))
            if newly_triggered or horizontal_speed > 0.25 or nearest_point is None:
                self._recovery_has_braked = True
                return PlannerStep(
                    np.zeros(4), nearest, goal_dist, "hold-brake", safety_diag
                )
            away_xy = state.pos[0:2] - nearest_point[0:2]
            away_norm = float(np.linalg.norm(away_xy))
            if away_norm < 1e-6:
                return PlannerStep(
                    np.zeros(4), nearest, goal_dist, "hold-brake", safety_diag
                )
            recovery = np.zeros(4)
            recovery[0:2] = self.recovery_speed_m_s * away_xy / away_norm
            safety_diag["recovery_direction_enu"] = recovery[0:3].tolist()
            return PlannerStep(recovery, nearest, goal_dist, "recover-brake", safety_diag)
        if getattr(self.planner, "requires_rigid_body_state", False):
            if state.quat_wxyz is None or state.omega_body_flu is None:
                return PlannerStep(
                    np.zeros(4), nearest, goal_dist, "hold-stale",
                    {"reason": "rigid-body state thiếu quaternion/angular rate"},
                )
            u = self.planner.command_rigid(
                state.pos, state.quat_wxyz, state.vel, state.omega_body_flu
            )
        else:
            u = self.planner.command(state.pos, state.vel, state.yaw)
            raw_u = np.asarray(u, dtype=np.float64).copy()
            if (
                self.wp_index == len(self.waypoints) - 1
                and self.goal_slowdown_radius is not None
                and goal_dist < self.goal_slowdown_radius
            ):
                speed_cap = max(
                    self.goal_min_speed,
                    self.goal_approach_gain * max(goal_dist - self.goal_radius, 0.0),
                )
                # Near the final goal, do not let a low-ESS stochastic sample
                # choose the translation direction.  Use a deterministic
                # arrival vector; MPPI remains responsible outside this gate.
                goal_error = self.waypoints[-1] - state.pos
                u = np.asarray(u, dtype=np.float64).copy()
                u[0:3] = goal_error * (speed_cap / max(goal_dist, 1e-9))
            if self.command_conditioner is not None:
                u = self.command_conditioner.apply(u, state.vel)
            if hasattr(self.planner, "accept_applied_control"):
                self.planner.accept_applied_control(u)
        diagnostics = self.planner.optimizer_diagnostics()
        if not getattr(self.planner, "requires_rigid_body_state", False):
            diagnostics["raw_control"] = raw_u.tolist()
            diagnostics["conditioned_control"] = np.asarray(u).tolist()
        if (
            self.planner_timeout_ms is not None
            and float(diagnostics.get("compute_ms", 0.0)) > self.planner_timeout_ms
        ):
            diagnostics["reason"] = "planner deadline exceeded"
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            return PlannerStep(np.zeros(4), nearest, goal_dist, "hold-timeout", diagnostics)
        return PlannerStep(u, nearest, goal_dist, event, diagnostics)


def parse_goal(text) -> List[np.ndarray]:
    """Parse ``x,y,z`` route text or a YAML list of 3-vectors (m, ENU)."""
    waypoints: List[np.ndarray] = []
    parts = text.split(";") if isinstance(text, str) else text
    if parts is None:
        raise SystemExit("[error] route rỗng")
    for part in parts:
        try:
            values = [float(v) for v in part.split(",")] if isinstance(part, str) else [float(v) for v in part]
        except (TypeError, ValueError) as exc:
            raise SystemExit(f"[error] route phải là 'x,y,z;...' (m, ENU): {exc}")
        if len(values) != 3:
            raise SystemExit("[error] mỗi waypoint phải có đúng 3 thành phần x,y,z")
        waypoints.append(np.asarray(values, dtype=np.float64))
    if not waypoints:
        raise SystemExit("[error] route rỗng")
    return waypoints


def parse_reference_path(text) -> List[np.ndarray]:
    """Parse a global reference polyline; at least two points are required."""
    path = parse_goal(text)
    if len(path) < 2:
        raise SystemExit("[error] --global-path cần ít nhất 2 điểm")
    return path


def resolve_rviz_goal(clicked_xyz, current_position, altitude_m=None) -> np.ndarray:
    """Convert RViz's planar goal into a safe 3-D ENU goal.

    RViz ``2D Goal Pose`` normally publishes z=0.  We intentionally ignore
    that z value: keep the current flight altitude unless an explicit fixed
    altitude is configured.
    """
    clicked = np.asarray(clicked_xyz, dtype=np.float64)
    current = np.asarray(current_position, dtype=np.float64)
    if (clicked.shape != (3,) or current.shape != (3,)
            or not np.isfinite(clicked).all() or not np.isfinite(current).all()):
        raise ValueError("RViz goal và current position phải là finite 3-vector ENU")
    altitude = current[2] if altitude_m is None else float(altitude_m)
    if not np.isfinite(altitude):
        raise ValueError("RViz goal altitude phải hữu hạn")
    return np.array([clicked[0], clicked[1], altitude], dtype=np.float64)


def load_config(path: Optional[str]) -> dict:
    if not path:
        return {}
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit("[error] đọc --config cần pyyaml.") from exc
    with open(path, "r", encoding="utf-8") as fh:
        return dict(yaml.safe_load(fh) or {})


def config_from_dict(d: dict) -> MPPIConfig:
    cfg = MPPIConfig()
    for key in (
        "dt", "tau", "horizon", "samples", "lambda", "vmax", "vzmax",
        "yaw_rate_max", "noise_xy", "noise_z", "noise_yaw", "margin",
        "w_goal", "w_terminal", "w_obstacle", "w_u", "w_du", "w_yaw",
        "w_path", "path_scale_m", "w_reference_velocity", "reference_speed_m_s",
        "reference_corner_radius_m", "reference_corner_samples",
        "cost_profile", "w_collision", "collision_radius_m", "paper_r_u",
        "paper_r_delta_u",
        "command_alpha", "max_accel_xy", "max_accel_z", "max_yaw_accel",
        "goal_slowdown_radius", "goal_approach_gain", "goal_min_speed",
        "device", "seed",
    ):
        if key in d:
            setattr(cfg, key if key != "lambda" else "lambda_", d[key])
    return cfg


# -- RViz: predicted trajectory ------------------------------------------------
class RvizTrajPublisher:
    """Publish MPPI paths and optionally receive ``2D Goal Pose`` clicks."""

    def __init__(self, topic: Optional[str] = "/mppi/predicted_path", *,
                 samples_topic: Optional[str] = None,
                 goal_topic: Optional[str] = None,
                 frame_id: str = "odom") -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path
            from visualization_msgs.msg import MarkerArray
        except ImportError as exc:
            raise SystemExit(
                "[error] RViz trajectory/goal I/O cần rclpy, geometry_msgs, "
                "nav_msgs và visualization_msgs trong env."
            ) from exc
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._rclpy = rclpy
        self._Path = Path
        self._MarkerArray = MarkerArray
        self.node = rclpy.create_node("mppi_traj_pub")
        self.pub = self.node.create_publisher(Path, topic, 10) if topic else None
        self.samples_pub = (
            self.node.create_publisher(MarkerArray, samples_topic, 10)
            if samples_topic else None
        )
        self.frame_id = frame_id
        self._goal_lock = threading.Lock()
        self._pending_goal = None
        self._goal_error = None
        self._goal_revision = 0
        self.goal_sub = (
            self.node.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
            if goal_topic else None
        )

    def _on_goal(self, msg) -> None:
        frame = msg.header.frame_id or self.frame_id
        with self._goal_lock:
            if frame != self.frame_id:
                self._goal_error = (
                    f"bỏ goal frame '{frame}'; cần '{self.frame_id}' "
                    "(chưa hỗ trợ TF transform goal)"
                )
                return
            xyz = np.array([
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ], dtype=np.float64)
            if not np.isfinite(xyz).all():
                self._goal_error = "bỏ RViz goal chứa NaN/Inf"
                return
            self._goal_revision += 1
            self._pending_goal = (self._goal_revision, xyz)

    def spin_once(self) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def take_goal(self):
        with self._goal_lock:
            goal = self._pending_goal
            self._pending_goal = None
            return goal

    def take_goal_error(self):
        with self._goal_lock:
            error = self._goal_error
            self._goal_error = None
            return error

    def publish(self, traj, sampled=None) -> None:
        from geometry_msgs.msg import Point, PoseStamped
        from visualization_msgs.msg import Marker

        msg = self._Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for p in np.asarray(traj, dtype=float):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                float(p[0]), float(p[1]), float(p[2]),
            )
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        if self.pub is not None:
            self.pub.publish(msg)
        if self.samples_pub is None:
            return
        markers = self._MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, path in enumerate(np.asarray(sampled if sampled is not None else [])):
            marker = Marker()
            marker.header = msg.header
            marker.ns = "mppi_samples"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.color.r = 0.15
            marker.color.g = 0.55
            marker.color.b = 1.0
            marker.color.a = 0.22
            for xyz in path:
                point = Point()
                point.x, point.y, point.z = map(float, xyz)
                marker.points.append(point)
            markers.markers.append(marker)
        self.samples_pub.publish(markers)

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()


class TimingWindow:
    """Rolling compute-time benchmark with deadline accounting."""

    def __init__(self, deadline_ms: float, window: int = 200) -> None:
        self.deadline_ms = float(deadline_ms)
        self.values = deque(maxlen=max(1, int(window)))
        self.total = 0
        self.deadline_misses = 0

    def add(self, value_ms: float) -> None:
        value_ms = float(value_ms)
        self.values.append(value_ms)
        self.total += 1
        self.deadline_misses += int(value_ms > self.deadline_ms)

    def summary(self) -> dict:
        if not self.values:
            # Safe-hold cycles (notably interactive RViz before the first
            # click) deliberately do not execute MPPI.  Keep the diagnostics
            # schema stable instead of making every consumer special-case an
            # empty timing window.
            return {
                "mean_ms": 0.0,
                "p95_ms": 0.0,
                "worst_ms": 0.0,
                "deadline_ms": self.deadline_ms,
                "deadline_misses": 0,
                "samples": 0,
            }
        values = np.asarray(self.values, dtype=float)
        return {
            "mean_ms": float(values.mean()),
            "p95_ms": float(np.percentile(values, 95)),
            "worst_ms": float(values.max()),
            "deadline_ms": self.deadline_ms,
            "deadline_misses": self.deadline_misses,
            "samples": self.total,
        }


# -- offline sim-test (không cần Gazebo/MAVLink) -------------------------------
def run_sim_test(args) -> None:
    """Lái point-mass 7-state vòng qua stack container giả lập.

    Geometry đúng warehouse thật: stack tại x=12, y trong [-6,6], z 0-24
    (box 12x2.5x6, yaw 1.57); copter ở 20 m phải vòng qua một đầu stack.
    """
    waypoints = args.goal
    route = " -> ".join(f"({w[0]:g},{w[1]:g},{w[2]:g})" for w in waypoints)
    print(f"[sim-test] start (0,0,20) yaw=0, route {route}, stack x=12, y in [-6,6]")
    planner = args.planner_factory(args.cfg)
    wall = np.array(
        [[12.0, y, z] for y in np.linspace(-6.0, 6.0, 13)
         for z in np.linspace(0.0, 24.0, 13)]
    )
    planner.update_obstacles(wall)
    pos = np.array([0.0, 0.0, 20.0])
    vel = np.zeros(3)
    yaw = 0.0
    min_clearance = float("inf")
    reached = False
    node = LocalPlannerNode(
        planner, waypoints, wp_radius=args.wp_radius,
        goal_radius=args.goal_radius, hard_brake_m=0.0,
        reference_path=args.global_path,
        command_conditioner=VelocityCommandConditioner(
            args.cfg.dt, args.cfg.command_alpha, args.cfg.max_accel_xy,
            args.cfg.max_accel_z, args.cfg.max_yaw_accel,
            u_min=args.cfg.u_min(), u_max=args.cfg.u_max(),
        ),
        goal_slowdown_radius=args.cfg.goal_slowdown_radius,
        goal_approach_gain=args.cfg.goal_approach_gain,
        goal_min_speed=args.cfg.goal_min_speed,
    )
    for step in range(600):
        st = PlannerState(pos=pos.copy(), vel=vel.copy(), yaw=yaw)
        out = node.step(st, wall)
        u = out.u
        vel = vel + (u[0:3] - vel) * (args.cfg.dt / args.cfg.tau)
        pos = pos + vel * args.cfg.dt
        yaw = yaw + float(u[3]) * args.cfg.dt
        clearance = float(np.linalg.norm(wall - pos, axis=1).min())
        min_clearance = min(min_clearance, clearance)
        if out.event == "waypoint":
            print(f"[sim-test] t={step * args.cfg.dt:4.1f}s waypoint {node.wp_index}: "
                  f"{waypoints[node.wp_index]}")
        if step % 20 == 0:
            print(f"[sim-test] t={step * args.cfg.dt:4.1f}s "
                  f"pos=({pos[0]:6.2f},{pos[1]:6.2f},{pos[2]:5.2f}) "
                  f"v=({vel[0]:5.2f},{vel[1]:5.2f},{vel[2]:5.2f}) "
                  f"yaw={yaw:5.2f} goal_dist={out.goal_dist_m:5.2f}m")
        if out.event == "reached":
            reached = True
            break
    dist_goal = float(np.linalg.norm(pos - waypoints[-1]))
    print(f"[sim-test] done: reached={reached} final_goal_dist={dist_goal:.2f}m "
          f"min_clearance={min_clearance:.2f}m")
    required_clearance = (
        args.cfg.collision_radius_m
        if args.cfg.cost_profile == "paper"
        else args.cfg.margin + 1.0
    )
    if not reached or min_clearance < required_clearance:
        raise SystemExit("[sim-test] FAILED: planner không né hoặc không tới đích")
    print("[sim-test] PASSED")


def run_rigid_dynamics_test(args) -> None:
    """Deterministic hover and quaternion checks before live attitude control."""
    planner = args.planner_factory(args.cfg)
    torch = planner.torch
    state = torch.tensor(
        [0, 0, 20, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        dtype=torch.double, device=planner.goal.device,
    )
    hover = torch.tensor(
        [args.cfg.hover_thrust_n, 0, 0, 0],
        dtype=torch.double, device=planner.goal.device,
    )
    initial = state.clone()
    for _ in range(100):
        state = planner._dynamics(state, hover)
    position_error = float(torch.linalg.vector_norm(state[0:3] - initial[0:3]).item())
    velocity_error = float(torch.linalg.vector_norm(state[7:10]).item())

    roll_command = hover.clone()
    roll_command[1] = 0.4
    rolled = planner._dynamics(initial, roll_command)
    quaternion_norm = float(torch.linalg.vector_norm(rolled[3:7]).item())
    attitude_changed = float(torch.linalg.vector_norm(rolled[4:7]).item())
    hover_norm = planner.thrust_newtons_to_normalized(args.cfg.hover_thrust_n)
    print(
        f"[rigid-test] hover position_error={position_error:.3e}m "
        f"velocity_error={velocity_error:.3e}m/s thrust_norm={hover_norm:.3f}"
    )
    print(
        f"[rigid-test] quaternion_norm={quaternion_norm:.9f} "
        f"attitude_delta={attitude_changed:.3e}"
    )
    if (
        position_error > 1e-8 or velocity_error > 1e-8
        or abs(quaternion_norm - 1.0) > 1e-9 or attitude_changed <= 0
        or abs(hover_norm - args.cfg.hover_thrust_normalized) > 1e-12
    ):
        raise SystemExit("[rigid-test] FAILED")
    print("[rigid-test] PASSED")


# -- live run (Gazebo LiDAR + odom/MAVLink state + MAVLink velocity) ------------
class _Latest:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload = None
        self._stamp = 0.0
        self.received = 0

    def update(self, payload: bytes) -> None:
        with self._lock:
            self._payload = payload
            self._stamp = time.monotonic()
            self.received += 1

    def get(self):
        with self._lock:
            if self._payload is None:
                return None, float("inf")
            return self._payload, time.monotonic() - self._stamp


def _ned_points_to_working_enu(pts_ned: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts_ned, dtype=np.float64)
    return np.column_stack((pts[:, 1], pts[:, 0], -pts[:, 2]))


def run(args) -> None:
    """Vòng điều khiển live theo đúng pipeline mentor chốt (xem module doc)."""
    try:
        import gz.transport13 as gz_transport
        from gz.msgs10 import odometry_pb2
        from gz.msgs10 import pointcloud_packed_pb2 as pc_pb2
    except ImportError as exc:
        raise SystemExit(
            "[error] node cần gz.transport13 + gz.msgs10 (Gazebo Harmonic). "
            f"Chi tiết: {exc}"
        ) from exc
    try:
        import lidar_to_mavlink_avoidance as lidar_bridge
    except ImportError as exc:
        raise SystemExit(
            "[error] cần scripts/lidar_to_mavlink_avoidance.py trên sys.path "
            f"(downsample week 3): {exc}"
        ) from exc

    cfg: MPPIConfig = args.cfg
    mav = None
    if args.no_mav:
        print("[mavlink] --no-mav: không gửi MAVLink (chỉ log MPPI)")
    else:
        mav = ArduPilotInterface(args.mav)
        mav.setup_telemetry(hz=20.0)

    planner = args.planner_factory(cfg)
    rigid_mode = bool(getattr(planner, "requires_rigid_body_state", False))
    period = 1.0 / args.hz
    if rigid_mode and mav is not None:
        mav.require_parameter("GUID_OPTIONS", 8.0)
        print("[safety] GUID_OPTIONS=8 verified: thrust-as-thrust enabled")
    node = LocalPlannerNode(
        planner, args.goal, wp_radius=args.wp_radius,
        goal_radius=args.goal_radius, hard_brake_m=args.hard_brake_m,
        hard_brake_release_m=args.hard_brake_release_m,
        hard_brake_delay_s=args.hard_brake_delay_s,
        brake_accel_m_s2=cfg.max_accel_xy,
        recovery_speed_m_s=args.recovery_speed_m_s,
        reference_path=args.global_path,
        planner_timeout_ms=args.planner_timeout_ms,
        command_conditioner=(
            None if rigid_mode else VelocityCommandConditioner(
                period,
                cfg.command_alpha, cfg.max_accel_xy,
                cfg.max_accel_z, cfg.max_yaw_accel,
                u_min=cfg.u_min(), u_max=cfg.u_max(),
            )
        ),
        goal_slowdown_radius=(None if rigid_mode else cfg.goal_slowdown_radius),
        goal_approach_gain=cfg.goal_approach_gain,
        goal_min_speed=cfg.goal_min_speed,
    )
    traj_pub = None
    if args.rviz_traj_topic or args.rviz_samples_topic or args.rviz_goal_topic:
        traj_pub = RvizTrajPublisher(
            args.rviz_traj_topic,
            samples_topic=args.rviz_samples_topic,
            goal_topic=args.rviz_goal_topic,
            frame_id=args.rviz_goal_frame,
        )
        if args.rviz_traj_topic:
            print(f"[rviz] publish predicted trajectory -> {args.rviz_traj_topic}")
        if args.rviz_samples_topic:
            print(f"[rviz] publish top-{args.rviz_top_k} samples -> {args.rviz_samples_topic}")
        if args.rviz_goal_topic:
            altitude = (
                "giữ cao độ hiện tại" if args.rviz_goal_altitude is None
                else f"z={args.rviz_goal_altitude:g}m ENU"
            )
            print(f"[rviz] subscribe goal <- {args.rviz_goal_topic} "
                  f"frame={args.rviz_goal_frame}; {altitude}")

    latest_scan = _Latest()
    latest_odom = _Latest()
    gz_node = gz_transport.Node()
    gz_node.subscribe(pc_pb2.PointCloudPacked, args.topic,
                      lambda msg: latest_scan.update(msg.SerializeToString()))
    use_odom = args.state_source == "odom"
    if use_odom:
        if not gz_node.subscribe(odometry_pb2.Odometry, args.odom_topic,
                                 lambda msg: latest_odom.update(msg.SerializeToString())):
            raise SystemExit(f"[error] không subscribe được {args.odom_topic}")
    route_text = (
        f"RViz dynamic via {args.rviz_goal_topic}"
        if args.rviz_goal_topic
        else " -> ".join(f"({w[0]:g},{w[1]:g},{w[2]:g})" for w in args.goal)
    )
    print(f"[mppi] state={args.state_source} route={route_text} ENU "
          f"vmax={cfg.vmax} margin={cfg.margin} H={cfg.horizon} N={cfg.samples} hz={args.hz} "
          f"w_path={cfg.w_path:g} cost_profile={cfg.cost_profile}")
    if cfg.w_path > 0 and args.global_path is not None:
        print(f"[path] tracking {len(args.global_path)} reference points, "
              f"scale={cfg.path_scale_m:g}m")
    elif cfg.w_path > 0 and args.rviz_goal_topic:
        print("[path] chưa có reference; mỗi RViz click sẽ tạo path current->goal")

    pos_prev: Optional[np.ndarray] = None
    pos_prev_stamp: Optional[float] = None
    vel_est = np.zeros(3)
    cycle = 0
    next_tick = time.monotonic()
    send_zero = np.zeros(4)
    timing = TimingWindow(deadline_ms=period * 1000.0, window=args.benchmark_window)
    goal_announced = False
    awaiting_rviz_goal = bool(args.rviz_goal_topic)
    goal_source = "rviz" if awaiting_rviz_goal else "cli"
    goal_revision = 0
    diag_file = None
    if args.diag_jsonl:
        diag_path = Path(args.diag_jsonl)
        diag_path.parent.mkdir(parents=True, exist_ok=True)
        diag_file = diag_path.open("a", encoding="utf-8")

    def log_missing_state(age_s: float, reason: str) -> None:
        if diag_file is None:
            return
        diag_file.write(json.dumps({
            "schema_version": 1,
            "timestamp_unix_s": time.time(),
            "timestamp_monotonic_s": time.monotonic(),
            "cycle": cycle,
            "event": "hold-stale",
            "random_seed": int(cfg.seed),
            "state_source": args.state_source,
            "state_source_timestamp_s": None,
            "state_age_s": float(age_s) if np.isfinite(age_s) else None,
            "position_enu": None,
            "velocity_enu": None,
            "attitude_quaternion_wxyz": None,
            "measured_body_rates_flu_rad_s": None,
            "nominal_control": None,
            "sent_mavlink_control": {
                "message": "SET_POSITION_TARGET_LOCAL_NED",
                "frame": "LOCAL_NED", "type_mask": 1479,
                "velocity_ned_m_s": [0.0, 0.0, 0.0],
                "yaw_rate_ned_rad_s": 0.0,
            },
            "deadline_miss": False,
            "saturation_flags": {},
            "failsafe_gate_state": reason,
        }, ensure_ascii=False, allow_nan=False) + "\n")
        diag_file.flush()

    def send_u(u: np.ndarray, *, safe_hold: bool = False) -> None:
        if mav is None:
            return
        if rigid_mode and not safe_hold:
            from .rigid_body_pa_mppi import body_flu_rates_to_frd
            rates_frd = body_flu_rates_to_frd(u[1:4])
            mav.send_attitude_target_body_rates(
                rates_frd[0], rates_frd[1], rates_frd[2],
                planner.thrust_newtons_to_normalized(float(u[0])),
            )
            return
        vn, ve, vd = enu_to_ned_vel(u[0:3])
        mav.send_velocity_ned(vn, ve, vd, yaw_rate=yaw_enu_to_ned_rate(float(u[3])))

    try:
        while True:
            now = time.monotonic()
            next_tick += period
            sleep = next_tick - now
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = now

            if traj_pub is not None:
                traj_pub.spin_once()
                goal_error = traj_pub.take_goal_error()
                if goal_error:
                    print(f"[rviz] {goal_error}")

            # 1. state: odom (SITL ground truth) hoặc MAVLink telemetry
            state: Optional[PlannerState] = None
            pos_enu = rot = yaw_ned = pos_ned = None
            if use_odom:
                odom_payload, odom_age = latest_odom.get()
                if odom_payload is None or odom_age > args.stale_after_s:
                    print("[mppi] chờ /iris/odometry...")
                    pos_prev = None
                    pos_prev_stamp = None
                    send_u(send_zero, safe_hold=True)
                    log_missing_state(odom_age, "stale-odometry")
                    cycle += 1
                    continue
                odom = odometry_pb2.Odometry()
                odom.ParseFromString(odom_payload)
                pos_enu = np.array([odom.pose.position.x, odom.pose.position.y,
                                    odom.pose.position.z])
                odom_sample_stamp = time.monotonic() - float(odom_age)
                q = odom.pose.orientation
                rot = quat_to_rot(q.x, q.y, q.z, q.w)
                yaw = yaw_enu_from_rot(rot)
                if pos_prev is not None and pos_prev_stamp is not None:
                    sample_dt = odom_sample_stamp - pos_prev_stamp
                    if 1e-3 < sample_dt <= args.stale_after_s:
                        raw_v = (pos_enu - pos_prev) / sample_dt
                        # Reject impossible differentiation spikes without
                        # forcing vertical velocity through the XY limit.
                        raw_v[0:2] = np.clip(raw_v[0:2], -2.0 * cfg.vmax, 2.0 * cfg.vmax)
                        raw_v[2] = np.clip(raw_v[2], -2.0 * cfg.vzmax, 2.0 * cfg.vzmax)
                        vel_est = 0.5 * vel_est + 0.5 * raw_v
                pos_prev = pos_enu
                pos_prev_stamp = odom_sample_stamp
                state = PlannerState(
                    pos=pos_enu,
                    vel=vel_est.copy(),
                    yaw=yaw,
                    quat_wxyz=np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
                    omega_body_flu=np.array(
                        [odom.twist.angular.x, odom.twist.angular.y, odom.twist.angular.z],
                        dtype=np.float64,
                    ),
                    source_age_s=float(odom_age),
                    source_timestamp_s=time.time() - float(odom_age),
                )
            else:
                mav.spin_once(timeout=0.0)
                got = mav.get_state(max_age_s=args.stale_after_s)
                if got is None:
                    print("[mppi] chờ MAVLink LOCAL_POSITION_NED + ATTITUDE...")
                    send_u(send_zero, safe_hold=True)
                    log_missing_state(mav.get_state_age_s(), "stale-mavlink-telemetry")
                    cycle += 1
                    continue
                pos_ned, vel_ned, yaw_ned = got
                pos_enu = ned_to_enu_pos(pos_ned)
                state = PlannerState(pos=pos_enu, vel=ned_to_enu_pos(vel_ned),
                                     yaw=yaw_ned_to_enu(yaw_ned),
                                     source_age_s=mav.get_state_age_s(),
                                     source_timestamp_s=time.time() - mav.get_state_age_s())

            if traj_pub is not None and args.rviz_goal_topic:
                pending_goal = traj_pub.take_goal()
                if pending_goal is not None:
                    goal_revision, clicked_xyz = pending_goal
                    goal = resolve_rviz_goal(
                        clicked_xyz, state.pos, args.rviz_goal_altitude
                    )
                    reference = None
                    if np.linalg.norm(goal - state.pos) > 1e-6:
                        reference = [state.pos.copy(), goal.copy()]
                    node.replace_route([goal], reference_path=reference)
                    awaiting_rviz_goal = False
                    goal_source = "rviz"
                    goal_announced = False
                    print(
                        f"[rviz] goal#{goal_revision} accepted ENU="
                        f"({goal[0]:.2f},{goal[1]:.2f},{goal[2]:.2f}); "
                        "đã thay route và reset MPPI warm start"
                    )

            # 2-3. LiDAR scan -> obstacle cloud trong frame làm việc
            scan_payload, scan_age = latest_scan.get()
            obstacles = None
            lidar_ok = scan_payload is not None and scan_age <= args.stale_after_s
            if lidar_ok:
                msg = pc_pb2.PointCloudPacked()
                msg.ParseFromString(scan_payload)
                raw_points = lidar_bridge._read_gz_points(msg, args.max_raw_points)
                if use_odom:
                    obstacles = scan_to_world_enu(
                        raw_points, pos_enu, rot, max_points=args.max_points,
                        sensor_offset_body_frd=tuple(args.sensor_offset_body_frd))
                else:
                    frd = lidar_bridge.point_cloud_to_obstacles(
                        raw_points, max_points=args.max_points,
                        sensor_offset_body_frd=tuple(args.sensor_offset_body_frd))
                    if frd:
                        pts_ned = body_frd_to_ned(np.asarray(frd), pos_ned, yaw_ned)
                        obstacles = _ned_points_to_working_enu(pts_ned)
                if obstacles is not None and len(obstacles) == 0:
                    obstacles = None

            # 4-6. MPPI -> gửi ArduPilot
            just_reached = False
            if not lidar_ok:
                out = PlannerStep(np.zeros(4), float("inf"), float("inf"), "hold-stale")
                print(f"[mppi] lidar stale/khuyết (age={scan_age:.2f}s) -> zero velocity")
            elif awaiting_rviz_goal:
                nearest = float("inf")
                if obstacles is not None and len(obstacles) > 0:
                    nearest = float(np.linalg.norm(obstacles - state.pos, axis=1).min())
                out = PlannerStep(
                    np.zeros(4), nearest, float("inf"), "hold-await-goal",
                    {"planner": "mppi", "compute_ms": 0.0, "cost": {},
                     "reason": "waiting for RViz 2D Goal Pose"},
                )
                if cycle % args.diag_every == 0:
                    print(f"[rviz] chờ goal trên {args.rviz_goal_topic}; giữ velocity zero")
            else:
                out = node.step(state, obstacles)
                just_reached = out.event == "reached" and not goal_announced
                if out.event == "waypoint":
                    print(f"[mppi] tới waypoint {node.wp_index}/{len(node.waypoints) - 1}: "
                          f"{node.waypoints[node.wp_index]}")
                elif just_reached:
                    goal_announced = True
                    print(f"[mppi] ĐÃ TỚI ĐÍCH sau {cycle} chu kỳ; giữ nguyên vị trí.")
                elif out.event == "hold-brake":
                    threshold = (out.diagnostics or {}).get(
                        "safety_trigger_m", args.hard_brake_m
                    )
                    print(f"[mppi] predictive brake: obstacle {out.nearest_m:.2f}m < "
                          f"trigger {threshold:.2f}m -> zero velocity")
                elif out.event == "recover-brake":
                    print(f"[mppi] recovery: obstacle {out.nearest_m:.2f}m -> "
                          f"retreat ({out.u[0]:+.2f},{out.u[1]:+.2f})m/s")
            send_u(
                out.u,
                safe_hold=out.event in SAFE_HOLD_EVENTS,
            )
            if traj_pub is not None and out.event not in SAFE_HOLD_EVENTS:
                traj_pub.publish(
                    planner.predict_trajectory(),
                    planner.sampled_trajectories(args.rviz_top_k),
                )
            cycle += 1
            u = out.u
            print_cycle = out.event != "reached" or cycle % args.diag_every == 0 or just_reached
            if rigid_mode and print_cycle:
                print(
                    f"[mppi] cycle={cycle} pos=({state.pos[0]:6.2f},{state.pos[1]:6.2f},"
                    f"{state.pos[2]:5.2f}) goal_dist={out.goal_dist_m:5.2f} "
                    f"nearest={out.nearest_m:5.2f}m thrust={u[0]:5.2f}N "
                    f"rates_flu=({u[1]:+.2f},{u[2]:+.2f},{u[3]:+.2f})"
                )
            elif print_cycle:
                print(f"[mppi] cycle={cycle} pos=({state.pos[0]:6.2f},{state.pos[1]:6.2f},"
                      f"{state.pos[2]:5.2f}) goal_dist={out.goal_dist_m:5.2f} "
                      f"nearest={out.nearest_m:5.2f}m "
                      f"u=({u[0]:5.2f},{u[1]:5.2f},{u[2]:5.2f}) yaw_rate={u[3]:+.2f} "
                      f"event={out.event or 'command'}")
            if out.diagnostics or diag_file is not None:
                diagnostics = out.diagnostics or {
                    "planner": getattr(planner, "command_kind", "mppi"),
                    "compute_ms": 0.0,
                    "cost": {},
                }
                compute_ms = float(diagnostics.get("compute_ms", 0.0))
                if compute_ms > 0.0:
                    timing.add(compute_ms)
                safe_hold = out.event in SAFE_HOLD_EVENTS
                if rigid_mode and not safe_hold:
                    from .rigid_body_pa_mppi import body_flu_rates_to_frd
                    sent_mavlink = {
                        "message": "SET_ATTITUDE_TARGET",
                        "type_mask": 128,
                        "body_rates_frd_rad_s": body_flu_rates_to_frd(out.u[1:4]).tolist(),
                        "thrust_normalized": planner.thrust_newtons_to_normalized(float(out.u[0])),
                    }
                else:
                    vn, ve, vd = enu_to_ned_vel(out.u[0:3])
                    sent_mavlink = {
                        "message": "SET_POSITION_TARGET_LOCAL_NED",
                        "frame": "LOCAL_NED",
                        "type_mask": 1479,
                        "velocity_ned_m_s": [vn, ve, vd],
                        "yaw_rate_ned_rad_s": yaw_enu_to_ned_rate(float(out.u[3])),
                    }
                record = {
                    "schema_version": 1,
                    "timestamp_unix_s": time.time(),
                    "timestamp_monotonic_s": time.monotonic(),
                    "cycle": cycle,
                    "event": out.event or "command",
                    "random_seed": int(cfg.seed),
                    "state_source": args.state_source,
                    "state_source_timestamp_s": state.source_timestamp_s,
                    "state_age_s": state.source_age_s,
                    "position_enu": state.pos.tolist(),
                    "velocity_enu": state.vel.tolist(),
                    "yaw_enu_rad": float(state.yaw),
                    "attitude_quaternion_wxyz": (
                        None if state.quat_wxyz is None else state.quat_wxyz.tolist()
                    ),
                    "measured_body_rates_flu_rad_s": (
                        None if state.omega_body_flu is None else state.omega_body_flu.tolist()
                    ),
                    "goal_dist_m": out.goal_dist_m if np.isfinite(out.goal_dist_m) else None,
                    "active_goal_enu": (
                        None if awaiting_rviz_goal
                        else node.waypoints[node.wp_index].tolist()
                    ),
                    "waypoint_index": int(node.wp_index),
                    "goal_source": goal_source,
                    "goal_revision": int(goal_revision),
                    "minimum_clearance_m": out.nearest_m if np.isfinite(out.nearest_m) else None,
                    "obstacle_cloud": {
                        "count": 0 if obstacles is None else int(len(obstacles)),
                        "min_enu": None if obstacles is None else np.min(obstacles, axis=0).tolist(),
                        "max_enu": None if obstacles is None else np.max(obstacles, axis=0).tolist(),
                    },
                    "nominal_control": out.u.tolist(),
                    "sent_mavlink_control": sent_mavlink,
                    "control_kind": getattr(planner, "command_kind", "velocity-enu"),
                    "thrust_newton": float(out.u[0]) if rigid_mode else None,
                    "thrust_normalized": (
                        planner.thrust_newtons_to_normalized(float(out.u[0])) if rigid_mode else None
                    ),
                    "occupancy_map_status": {
                        "mapped_fraction": diagnostics.get("mapped_fraction"),
                        "revision": getattr(getattr(planner, "map", None), "revision", None),
                    },
                    "goal_line_of_sight": diagnostics.get("goal_visible"),
                    "selected_trajectory_enu": (
                        [] if safe_hold else planner.predict_trajectory().tolist()
                    ),
                    "deadline_miss": compute_ms > timing.deadline_ms,
                    "saturation_flags": {
                        "thrust": bool(rigid_mode and (
                            out.u[0] <= cfg.u_min()[0] + 1e-9 or out.u[0] >= cfg.u_max()[0] - 1e-9
                        )),
                        "control": bool(np.any(np.isclose(out.u, cfg.u_min(), atol=1e-9))
                                        or np.any(np.isclose(out.u, cfg.u_max(), atol=1e-9))),
                    },
                    "failsafe_gate_state": out.event or "nominal",
                    **diagnostics,
                    "timing": timing.summary(),
                }
                if not rigid_mode:
                    record["u_enu"] = out.u.tolist()
                if diag_file:
                    diag_file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                    diag_file.flush()
                if out.diagnostics and cycle % args.diag_every == 0:
                    perf = timing.summary()
                    costs = diagnostics.get("cost", {})
                    print(
                        f"[diag] planner={diagnostics.get('planner')} "
                        f"compute={compute_ms:.1f}ms mean={perf['mean_ms']:.1f}ms "
                        f"p95={perf['p95_ms']:.1f}ms worst={perf['worst_ms']:.1f}ms "
                        f"miss={perf['deadline_misses']}/{perf['samples']} "
                        f"ESS={diagnostics.get('ess', float('nan')):.1f} "
                        f"cost(goal={costs.get('goal', 0.0):.1f}, "
                        f"obs={costs.get('obstacle', 0.0):.1f}, "
                        f"collision={costs.get('collision', 0.0):.1f}, "
                        f"path={costs.get('path', 0.0):.1f}, "
                        f"vref={costs.get('reference_velocity', 0.0):.1f}, "
                        f"smooth={costs.get('input_change', costs.get('smoothness', 0.0)):.1f}, "
                        f"terminal={costs.get('terminal', 0.0):.1f})"
                    )
            if out.event == "reached" and args.exit_on_goal:
                print("[mppi] --exit-on-goal: đã gửi zero setpoint, thoát vòng điều khiển.")
                break
    except KeyboardInterrupt:
        print("\n[mppi] dừng gửi setpoint; drone sẽ hold sau GUID_TIMEOUT.")
    finally:
        if diag_file is not None:
            diag_file.close()
        if traj_pub is not None:
            traj_pub.close()
        if mav is not None:
            try:
                mav.send_velocity_ned(0.0, 0.0, 0.0, yaw_rate=0.0)
            except Exception:
                pass

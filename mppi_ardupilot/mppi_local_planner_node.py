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
    event: str = ""  # "hold-stale", "hold-brake", "reached", "waypoint", ""
    diagnostics: Optional[dict] = None


class VelocityCommandConditioner:
    """Low-pass and slew-limit velocity commands between planner cycles."""

    def __init__(self, dt: float, alpha: float, max_accel_xy: float,
                 max_accel_z: float, max_yaw_accel: float) -> None:
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
        planner_timeout_ms: Optional[float] = None,
        command_conditioner: Optional[VelocityCommandConditioner] = None,
    ) -> None:
        if not waypoints:
            raise ValueError("cần ít nhất 1 waypoint")
        self.planner = planner
        self.waypoints = [np.asarray(w, dtype=np.float64) for w in waypoints]
        self.wp_radius = wp_radius
        self.goal_radius = goal_radius
        self.hard_brake_m = hard_brake_m
        self.planner_timeout_ms = planner_timeout_ms
        self.command_conditioner = command_conditioner
        self.wp_index = 0
        self.reached = False
        planner.update_goal(self.waypoints[0])

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
        if obstacles is not None and len(obstacles) > 0:
            nearest = float(np.linalg.norm(np.asarray(obstacles) - state.pos, axis=1).min())
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
        if nearest < self.hard_brake_m:
            if self.command_conditioner is not None:
                self.command_conditioner.reset()
            return PlannerStep(np.zeros(4), nearest, goal_dist, "hold-brake")
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
            if self.command_conditioner is not None:
                u = self.command_conditioner.apply(u, state.vel)
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


def parse_goal(text: str) -> List[np.ndarray]:
    """'x,y,z' hoặc tuyến 'x,y,z;x,y,z;...' (m, ENU)."""
    waypoints: List[np.ndarray] = []
    for part in text.split(";"):
        try:
            values = [float(v) for v in part.split(",")]
        except ValueError as exc:
            raise SystemExit(f"[error] --goal phải là 'x,y,z;...' (m, ENU): {exc}")
        if len(values) != 3:
            raise SystemExit("[error] mỗi waypoint phải có đúng 3 thành phần x,y,z")
        waypoints.append(np.asarray(values, dtype=np.float64))
    if not waypoints:
        raise SystemExit("[error] --goal rỗng")
    return waypoints


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
        "command_alpha", "max_accel_xy", "max_accel_z", "max_yaw_accel",
        "device", "seed",
    ):
        if key in d:
            setattr(cfg, key if key != "lambda" else "lambda_", d[key])
    return cfg


# -- RViz: predicted trajectory ------------------------------------------------
class RvizTrajPublisher:
    """Publish nominal Path and optional top sampled rollouts to RViz."""

    def __init__(self, topic: str = "/mppi/predicted_path", *,
                 samples_topic: Optional[str] = None, frame_id: str = "odom") -> None:
        try:
            import rclpy
            from nav_msgs.msg import Path
            from visualization_msgs.msg import MarkerArray
        except ImportError as exc:
            raise SystemExit(
                "[error] --rviz-traj-topic cần rclpy + nav_msgs trong env."
            ) from exc
        rclpy.init()
        self._rclpy = rclpy
        self._Path = Path
        self._MarkerArray = MarkerArray
        self.node = rclpy.create_node("mppi_traj_pub")
        self.pub = self.node.create_publisher(Path, topic, 10)
        self.samples_pub = (
            self.node.create_publisher(MarkerArray, samples_topic, 10)
            if samples_topic else None
        )
        self.frame_id = frame_id

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
            return {}
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
        command_conditioner=VelocityCommandConditioner(
            args.cfg.dt, args.cfg.command_alpha, args.cfg.max_accel_xy,
            args.cfg.max_accel_z, args.cfg.max_yaw_accel,
        ),
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
    if not reached or min_clearance < args.cfg.margin + 1.0:
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
        planner_timeout_ms=args.planner_timeout_ms,
        command_conditioner=(
            None if rigid_mode else VelocityCommandConditioner(
                period,
                cfg.command_alpha, cfg.max_accel_xy,
                cfg.max_accel_z, cfg.max_yaw_accel,
            )
        ),
    )
    traj_pub = None
    if args.rviz_traj_topic:
        traj_pub = RvizTrajPublisher(
            args.rviz_traj_topic, samples_topic=args.rviz_samples_topic
        )
        print(f"[rviz] publish predicted trajectory -> {args.rviz_traj_topic}")
        if args.rviz_samples_topic:
            print(f"[rviz] publish top-{args.rviz_top_k} samples -> {args.rviz_samples_topic}")

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
    print(f"[mppi] state={args.state_source} "
          f"route={' -> '.join(f'({w[0]:g},{w[1]:g},{w[2]:g})' for w in args.goal)} ENU "
          f"vmax={cfg.vmax} margin={cfg.margin} H={cfg.horizon} N={cfg.samples} hz={args.hz}")

    pos_prev: Optional[np.ndarray] = None
    vel_est = np.zeros(3)
    cycle = 0
    next_tick = time.monotonic()
    send_zero = np.zeros(4)
    timing = TimingWindow(deadline_ms=period * 1000.0, window=args.benchmark_window)
    goal_announced = False
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

            # 1. state: odom (SITL ground truth) hoặc MAVLink telemetry
            state: Optional[PlannerState] = None
            pos_enu = rot = yaw_ned = pos_ned = None
            if use_odom:
                odom_payload, odom_age = latest_odom.get()
                if odom_payload is None or odom_age > args.stale_after_s:
                    print("[mppi] chờ /iris/odometry...")
                    pos_prev = None
                    send_u(send_zero, safe_hold=True)
                    log_missing_state(odom_age, "stale-odometry")
                    cycle += 1
                    continue
                odom = odometry_pb2.Odometry()
                odom.ParseFromString(odom_payload)
                pos_enu = np.array([odom.pose.position.x, odom.pose.position.y,
                                    odom.pose.position.z])
                q = odom.pose.orientation
                rot = quat_to_rot(q.x, q.y, q.z, q.w)
                yaw = yaw_enu_from_rot(rot)
                if pos_prev is not None:
                    raw_v = np.clip((pos_enu - pos_prev) * args.hz, -cfg.vmax, cfg.vmax)
                    vel_est = 0.5 * vel_est + 0.5 * raw_v
                pos_prev = pos_enu
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
                    print(f"[mppi] obstacle {out.nearest_m:.2f}m < "
                          f"{args.hard_brake_m}m -> zero velocity")
            send_u(
                out.u,
                safe_hold=out.event in ("hold-stale", "hold-brake", "hold-timeout", "reached"),
            )
            if traj_pub is not None and out.event not in (
                "hold-stale", "hold-brake", "hold-timeout", "reached"
            ):
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
                if out.diagnostics:
                    timing.add(compute_ms)
                safe_hold = out.event in ("hold-stale", "hold-brake", "hold-timeout", "reached")
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
                    "minimum_clearance_m": out.nearest_m if np.isfinite(out.nearest_m) else None,
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
                        f"smooth={costs.get('smoothness', 0.0):.1f}, "
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
        if mav is not None:
            try:
                mav.send_velocity_ned(0.0, 0.0, 0.0, yaw_rate=0.0)
            except Exception:
                pass

"""Vanilla MPPI cho quadrotor bám velocity setpoint (companion-side planner).

ArduPilot lo attitude control + velocity tracking, nên mô hình quy hoạch
chỉ cần point-mass với first-order lag (quad "có inertia nhẹ"):

    state   x = [px, py, pz, vx, vy, vz, yaw, u_applied] (11)
    control u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd] (4)

Không điều khiển wx, wy: chúng thuộc tầng SET_ATTITUDE_TARGET, thấp hơn
và khó hơn nhiều so với mục tiêu "để ArduPilot bám vận tốc".
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class MPPIConfig:
    seed: int = 7
    dt: float = 0.1
    tau: float = 0.5  # hằng số bám tốc của mô hình [s]
    horizon: int = 30  # 30 x 0.1s = 3s lookahead
    samples: int = 500  # rollout CPU; GPU dùng 1024-2048
    lambda_: float = 1.0  # temperature MPPI
    vmax: float = 2.0  # giới hạn tốc ngang [m/s], demo đầu cứ chậm
    vzmax: float = 1.0  # giới hạn tốc đứng [m/s]
    yaw_rate_max: float = 0.6  # giới hạn yaw rate [rad/s]
    noise_xy: float = 0.8  # sigma nhiễu tốc ngang [m/s]
    noise_z: float = 0.3
    noise_yaw: float = 0.3  # sigma nhiễu yaw rate [rad/s]
    margin: float = 4.0  # bán kính an toàn obstacle [m]
    w_goal: float = 1.0
    w_terminal: float = 5.0
    w_obstacle: float = 300.0
    w_u: float = 0.05
    w_du: float = 0.2
    w_yaw: float = 0.2  # ưu tiên yaw hướng theo chiều bay
    # Optional global-path tracking term.  Zero preserves the historical
    # point-to-waypoint baseline.  The path is supplied at runtime as an ENU
    # polyline; this adapts the position part of the reference cost in
    # Minařík et al. (2024) to the current velocity-level state.
    w_path: float = 0.0
    path_scale_m: float = 1.0
    w_reference_velocity: float = 0.0
    reference_speed_m_s: float = 1.0
    # ``project`` keeps the current proximity-softplus objective.  ``paper``
    # selects the subset mapped from Minařík et al.: input effort, input-change
    # effort, position reference and collision indicator.
    cost_profile: str = "project"
    w_collision: float = 1.0e6
    collision_radius_m: float = 0.5
    paper_r_u: tuple = (0.01, 0.05, 0.05, 0.10)
    paper_r_delta_u: tuple = (0.05, 0.10, 0.10, 0.30)
    # Conditioning of the receding-horizon command before it is sent to AP.
    # This is an interface constraint, not part of the PA-MPPI paper objective.
    command_alpha: float = 0.45
    max_accel_xy: float = 1.5  # [m/s^2]
    max_accel_z: float = 0.8  # [m/s^2]
    max_yaw_accel: float = 1.2  # [rad/s^2]
    goal_slowdown_radius: float = 4.0  # [m]
    goal_approach_gain: float = 0.5  # speed cap [m/s] per metre remaining
    goal_min_speed: float = 0.10  # [m/s], avoids asymptotic stall at the gate
    device: str = "cpu"

    def u_min(self):
        return [-self.vmax, -self.vmax, -self.vzmax, -self.yaw_rate_max]

    def u_max(self):
        return [self.vmax, self.vmax, self.vzmax, self.yaw_rate_max]


class QuadMPPI:
    """MPPI trên mô hình point-mass + yaw, frame làm việc do caller chọn.

    Frame làm việc là mặt phẳng level bất kỳ (ENU tuyệt đối của Gazebo
    hoặc ENU tương đối home); chỉ yêu cầu yaw là góc math (CCW từ trục x).
    """

    # The last four states retain the velocity/yaw-rate setpoint that passed
    # through the same low-pass and slew limiter used at the MAVLink boundary.
    # Without these states MPPI predicts an instantaneous command reversal
    # that the real interface cannot execute.
    NX = 11
    NU = 4

    def __init__(self, cfg: Optional[MPPIConfig] = None) -> None:
        import torch
        from pytorch_mppi import MPPI

        self.torch = torch
        self.cfg = cfg or MPPIConfig()
        torch.manual_seed(int(self.cfg.seed))
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(int(self.cfg.seed))
        device = torch.device(self.cfg.device)
        dtype = torch.double

        self.goal = torch.zeros(3, dtype=dtype, device=device)
        self.obstacles = None  # [N,3] trong frame làm việc, None khi trời quang
        self.reference_path = None  # [M,3] ENU polyline, optional
        self.reference_positions = None  # [H,3], time-indexed from current progress
        self.reference_velocities = None  # [H,3]
        self._path_progress_m = 0.0
        self._path_segment_lengths = None
        self._path_cumulative_lengths = None
        if self.cfg.path_scale_m <= 0 or not np.isfinite(self.cfg.path_scale_m):
            raise ValueError("path_scale_m phải hữu hạn và lớn hơn 0")
        if self.cfg.w_path < 0 or not np.isfinite(self.cfg.w_path):
            raise ValueError("w_path phải hữu hạn và không âm")
        if self.cfg.w_reference_velocity < 0 or not np.isfinite(self.cfg.w_reference_velocity):
            raise ValueError("w_reference_velocity phải hữu hạn và không âm")
        if self.cfg.reference_speed_m_s <= 0 or not np.isfinite(self.cfg.reference_speed_m_s):
            raise ValueError("reference_speed_m_s phải hữu hạn và lớn hơn 0")
        if self.cfg.cost_profile not in ("project", "paper"):
            raise ValueError("cost_profile phải là 'project' hoặc 'paper'")
        if self.cfg.w_collision < 0 or not np.isfinite(self.cfg.w_collision):
            raise ValueError("w_collision phải hữu hạn và không âm")
        if self.cfg.collision_radius_m < 0 or not np.isfinite(self.cfg.collision_radius_m):
            raise ValueError("collision_radius_m phải hữu hạn và không âm")
        paper_r_u = np.asarray(self.cfg.paper_r_u, dtype=np.float64)
        if (paper_r_u.shape != (self.NU,) or not np.isfinite(paper_r_u).all()
                or np.any(paper_r_u < 0)):
            raise ValueError("paper_r_u phải là 4 trọng số hữu hạn và không âm")
        paper_r_delta_u = np.asarray(self.cfg.paper_r_delta_u, dtype=np.float64)
        if (paper_r_delta_u.shape != (self.NU,)
                or not np.isfinite(paper_r_delta_u).all()
                or np.any(paper_r_delta_u < 0)):
            raise ValueError("paper_r_delta_u phải là 4 trọng số hữu hạn và không âm")
        self._paper_r_u = torch.as_tensor(paper_r_u, dtype=dtype, device=device)
        self._paper_r_delta_u = torch.as_tensor(
            paper_r_delta_u, dtype=dtype, device=device
        )

        noise_diag = torch.tensor(
            [
                self.cfg.noise_xy**2,
                self.cfg.noise_xy**2,
                self.cfg.noise_z**2,
                self.cfg.noise_yaw**2,
            ],
            dtype=dtype,
            device=device,
        )
        self.ctrl = MPPI(
            self._dynamics,
            self._running_cost,
            self.NX,
            noise_sigma=torch.diag(noise_diag),
            noise_mu=torch.zeros(self.NU, dtype=dtype, device=device),
            num_samples=self.cfg.samples,
            horizon=self.cfg.horizon,
            lambda_=self.cfg.lambda_,
            device=device,
            u_min=torch.tensor(self.cfg.u_min(), dtype=dtype, device=device),
            u_max=torch.tensor(self.cfg.u_max(), dtype=dtype, device=device),
            terminal_state_cost=self._terminal_cost,
            step_dependent_dynamics=True,
        )

    # -- mô hình ---------------------------------------------------------
    def _condition_action_torch(self, previous, requested):
        """Apply the MAVLink-side command conditioner inside every rollout."""
        torch = self.torch
        cfg = self.cfg
        target = previous + cfg.command_alpha * (requested - previous)
        delta = target - previous
        xy_limit = cfg.max_accel_xy * cfg.dt
        xy_norm = torch.linalg.vector_norm(delta[..., 0:2], dim=-1, keepdim=True)
        xy_scale = torch.clamp(xy_limit / xy_norm.clamp_min(1e-12), max=1.0)
        delta_xy = delta[..., 0:2] * xy_scale
        delta_z = delta[..., 2:3].clamp(-cfg.max_accel_z * cfg.dt,
                                        cfg.max_accel_z * cfg.dt)
        delta_yaw = delta[..., 3:4].clamp(-cfg.max_yaw_accel * cfg.dt,
                                          cfg.max_yaw_accel * cfg.dt)
        applied = previous + torch.cat((delta_xy, delta_z, delta_yaw), dim=-1)
        lower = torch.as_tensor(cfg.u_min(), dtype=applied.dtype, device=applied.device)
        upper = torch.as_tensor(cfg.u_max(), dtype=applied.dtype, device=applied.device)
        return torch.maximum(torch.minimum(applied, upper), lower)

    def _dynamics(self, state, action, t=None):
        torch = self.torch
        cfg = self.cfg
        p = state[..., 0:3]
        v = state[..., 3:6]
        yaw = state[..., 6]
        previous_applied = (
            state[..., 7:11]
            if state.shape[-1] >= self.NX
            else torch.cat((v, torch.zeros_like(yaw).unsqueeze(-1)), dim=-1)
        )
        applied = self._condition_action_torch(previous_applied, action)
        v_cmd = applied[..., 0:3]
        yaw_rate = applied[..., 3]
        alpha = min(cfg.dt / cfg.tau, 1.0)
        v_next = v + alpha * (v_cmd - v)
        p_next = p + v_next * cfg.dt
        yaw_next = yaw + yaw_rate * cfg.dt
        return torch.cat((p_next, v_next, yaw_next.unsqueeze(-1), applied), dim=-1)

    # -- cost ------------------------------------------------------------
    def wrap_angle(self, x):
        torch = self.torch
        return torch.atan2(torch.sin(x), torch.cos(x))

    def _obstacle_cost(self, p):
        torch = self.torch
        if self.obstacles is None or self.obstacles.shape[0] == 0:
            return torch.zeros(p.shape[:-1], dtype=p.dtype, device=p.device)
        nearest = torch.cdist(p.reshape(-1, 3), self.obstacles).min(dim=1).values
        nearest = nearest.reshape(p.shape[:-1])
        # softplus có offset để cost đúng bằng 0 NGAY tại biên margin;
        # không offset thì mọi rollout trên biên cùng chịu một bậc ثابت
        # làm planner đứng yên.
        beta = 2.0
        return torch.nn.functional.softplus(self.cfg.margin - nearest, beta=beta) - math.log(2.0) / beta

    def _collision_indicator(self, p):
        """Paper-style collision-set membership adapted to point obstacles.

        Minařík et al. use a geometry/collision module.  This velocity planner
        receives point samples, so ``collision_radius_m`` is an explicit
        project adapter and is not a claim about the paper's geometry.
        """
        torch = self.torch
        if self.obstacles is None or self.obstacles.shape[0] == 0:
            return torch.zeros(p.shape[:-1], dtype=p.dtype, device=p.device)
        nearest = torch.cdist(p.reshape(-1, 3), self.obstacles).min(dim=1).values
        return (nearest <= self.cfg.collision_radius_m).to(p.dtype).reshape(p.shape[:-1])

    def _path_distance(self, p):
        """Khoảng cách ngắn nhất tới polyline tham chiếu [M,3].

        ``p`` có thể là [K,T,3], [T,3] hoặc [3].  Projection lên từng đoạn
        thẳng tránh tạo ra các điểm hút nhân tạo chỉ tại các waypoint.
        """
        torch = self.torch
        if self.reference_path is None or self.reference_path.shape[0] == 0:
            return torch.zeros(p.shape[:-1], dtype=p.dtype, device=p.device)
        flat = p.reshape(-1, 3)
        if self.reference_path.shape[0] == 1:
            d2 = ((flat - self.reference_path[0]) ** 2).sum(dim=-1)
        else:
            a = self.reference_path[:-1]
            ab = self.reference_path[1:] - a
            denom = (ab * ab).sum(dim=-1).clamp_min(1e-12)
            rel = flat[:, None, :] - a[None, :, :]
            t = (rel * ab[None, :, :]).sum(dim=-1) / denom[None, :]
            t = t.clamp(0.0, 1.0)
            residual = rel - t[..., None] * ab[None, :, :]
            d2 = (residual * residual).sum(dim=-1).min(dim=-1).values
        return torch.sqrt(d2.clamp_min(0.0)).reshape(p.shape[:-1])

    def _running_cost(self, state, action, t=None):
        torch = self.torch
        cfg = self.cfg
        p = state[..., 0:3]
        v = state[..., 3:6]
        yaw = state[..., 6]
        goal_cost = torch.linalg.vector_norm(p - self.goal, dim=-1)
        obstacle_term = (
            cfg.w_collision * self._collision_indicator(p)
            if cfg.cost_profile == "paper"
            else cfg.w_obstacle * self._obstacle_cost(p)
        )
        if (cfg.cost_profile == "paper" and t is not None
                and self.reference_positions is not None):
            p_ref = self.reference_positions[min(int(t), len(self.reference_positions) - 1)]
            v_ref = self.reference_velocities[min(int(t), len(self.reference_velocities) - 1)]
            path_cost = ((p - p_ref) / cfg.path_scale_m).square().sum(dim=-1)
            reference_velocity_cost = (v - v_ref).square().sum(dim=-1)
        else:
            path_distance = self._path_distance(p)
            path_cost = (path_distance / cfg.path_scale_m) ** 2
            reference_velocity_cost = torch.zeros_like(path_cost)
        feasible_action = state[..., 7:11] if state.shape[-1] >= self.NX else action
        effort_term = (
            (feasible_action ** 2 * self._paper_r_u).sum(dim=-1)
            if cfg.cost_profile == "paper"
            else cfg.w_u * (action[..., 0:3] ** 2).sum(dim=-1)
        )
        accel_cost = ((action[..., 0:3] - v) ** 2).sum(dim=-1)
        # ưu tiên yaw hướng theo chiều bay, tỉ lệ với tốc ngang
        desired_yaw = torch.atan2(action[..., 1], action[..., 0])
        yaw_error = self.wrap_angle(desired_yaw - yaw)
        speed_xy = torch.sqrt(action[..., 0] ** 2 + action[..., 1] ** 2)
        yaw_cost = speed_xy * yaw_error**2
        return (
            cfg.w_goal * goal_cost
            + obstacle_term
            + effort_term
            + cfg.w_du * accel_cost
            + cfg.w_yaw * yaw_cost
            + cfg.w_path * path_cost
            + cfg.w_reference_velocity * reference_velocity_cost
        )

    def _terminal_cost(self, states, actions):
        torch = self.torch
        p_T = states[..., -1, 0:3]
        terminal_reference = (
            self.reference_positions[-1]
            if self.cfg.cost_profile == "paper" and self.reference_positions is not None
            else self.goal
        )
        cost = self.cfg.w_terminal * (
            torch.linalg.vector_norm(p_T - terminal_reference, dim=-1) ** 2
        )
        if self.cfg.cost_profile == "paper" and actions is not None:
            # Paper Eq. (16): sum of weighted input changes.  The installed
            # MPPI library passes the complete action sequence here, so this
            # term is evaluated once per rollout at the terminal callback.
            feasible_actions = states[..., 7:11] if states.shape[-1] >= self.NX else actions
            deltas = feasible_actions[..., 1:, :] - feasible_actions[..., :-1, :]
            cost = cost + (deltas.square() * self._paper_r_delta_u).sum(dim=(-2, -1))
        return cost

    # -- giao tiếp ngoài -------------------------------------------------
    def update_obstacles(self, points) -> None:
        if points is None or len(points) == 0:
            self.obstacles = None
            return
        self.obstacles = self.torch.tensor(
            np.asarray(points, dtype=np.float64),
            dtype=self.torch.double,
            device=self.goal.device,
        )

    def update_goal(self, goal) -> None:
        self.goal = self.torch.tensor(
            np.asarray(goal, dtype=np.float64),
            dtype=self.torch.double,
            device=self.goal.device,
        )

    def update_reference_path(self, path) -> None:
        """Đặt global reference polyline trong frame làm việc ENU.

        Đây là reference để local optimizer bám theo, không phải chứng nhận
        path đã collision-free. Global/mission layer phải tạo hoặc kiểm tra
        path trước khi truyền vào đây.
        """
        if path is None:
            self.reference_path = None
            self.reference_positions = None
            self.reference_velocities = None
            self._path_segment_lengths = None
            self._path_cumulative_lengths = None
            self._path_progress_m = 0.0
            return
        arr = np.asarray(path, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
            raise ValueError("reference path phải có dạng [M,3], M >= 1")
        if not np.isfinite(arr).all():
            raise ValueError("reference path chứa NaN/Inf")
        self.reference_path = self.torch.as_tensor(
            arr, dtype=self.torch.double, device=self.goal.device
        )
        segment_lengths = np.linalg.norm(np.diff(arr, axis=0), axis=1)
        if not np.any(segment_lengths > 1e-9):
            raise ValueError("reference path phải có ít nhất một đoạn khác 0")
        self._path_segment_lengths = segment_lengths
        self._path_cumulative_lengths = np.r_[0.0, np.cumsum(segment_lengths)]
        self._path_progress_m = 0.0

    def _project_path_progress(self, position: np.ndarray) -> float:
        path = self.reference_path.detach().cpu().numpy()
        a = path[:-1]
        ab = path[1:] - a
        denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
        rel = position[None, :] - a
        fractions = np.clip(np.sum(rel * ab, axis=1) / denom, 0.0, 1.0)
        projections = a + fractions[:, None] * ab
        index = int(np.argmin(np.sum((projections - position) ** 2, axis=1)))
        return float(self._path_cumulative_lengths[index]
                     + fractions[index] * self._path_segment_lengths[index])

    def _sample_path_at_progress(self, progress) -> np.ndarray:
        progress = np.asarray(progress, dtype=np.float64)
        path = self.reference_path.detach().cpu().numpy()
        total = float(self._path_cumulative_lengths[-1])
        clipped = np.clip(progress, 0.0, total)
        indices = np.searchsorted(self._path_cumulative_lengths, clipped, side="right") - 1
        indices = np.clip(indices, 0, len(path) - 2)
        lengths = self._path_segment_lengths[indices]
        fractions = np.divide(
            clipped - self._path_cumulative_lengths[indices], lengths,
            out=np.zeros_like(clipped), where=lengths > 1e-12,
        )
        return path[indices] + fractions[..., None] * (path[indices + 1] - path[indices])

    def _update_reference_trajectory(self, position: np.ndarray) -> None:
        if self.reference_path is None or self.reference_path.shape[0] < 2:
            self.reference_positions = None
            self.reference_velocities = None
            return
        projected = self._project_path_progress(position)
        self._path_progress_m = max(self._path_progress_m, projected)
        offsets = np.arange(1, self.cfg.horizon + 1, dtype=np.float64)
        progress = self._path_progress_m + offsets * self.cfg.reference_speed_m_s * self.cfg.dt
        positions = self._sample_path_at_progress(progress)
        previous_progress = np.r_[self._path_progress_m, progress[:-1]]
        previous_positions = self._sample_path_at_progress(previous_progress)
        velocities = (positions - previous_positions) / self.cfg.dt
        self.reference_positions = self.torch.as_tensor(
            positions, dtype=self.torch.double, device=self.goal.device
        )
        self.reference_velocities = self.torch.as_tensor(
            velocities, dtype=self.torch.double, device=self.goal.device
        )

    def command(self, pos, vel, yaw: float) -> np.ndarray:
        """Trả về u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]."""
        pos_np = np.asarray(pos, dtype=np.float64)
        vel_np = np.asarray(vel, dtype=np.float64)
        self._update_reference_trajectory(pos_np)
        if hasattr(self, "_applied_control_np"):
            applied = self._applied_control_np.copy()
        else:
            applied = np.r_[vel_np, 0.0]
            applied = np.clip(applied, self.cfg.u_min(), self.cfg.u_max())
        state_np = np.concatenate((pos_np, vel_np, [float(yaw)], applied))
        if not np.isfinite(state_np).all():
            raise ValueError("MPPI state chứa NaN/Inf")
        state = self.torch.tensor(
            state_np, dtype=self.torch.double, device=self.goal.device
        )
        self._last_state_np = state_np
        started = time.perf_counter()
        u = self.ctrl.command(state)
        self.last_compute_ms = (time.perf_counter() - started) * 1000.0
        return u.detach().cpu().numpy().astype(np.float64)

    def accept_applied_control(self, control) -> None:
        """Feed the post-conditioned command back into the MPPI warm start.

        ``pytorch_mppi`` shifts its nominal sequence on the next call under
        the assumption that ``U[0]`` was executed.  The companion interface
        low-pass/slew-limits that action, so U[0] must reflect the action
        actually sent to ArduPilot rather than the raw optimizer output.
        """
        applied = np.asarray(control, dtype=np.float64)
        if applied.shape != (self.NU,) or not np.isfinite(applied).all():
            raise ValueError("applied MPPI control must be a finite 4-vector")
        applied = np.clip(applied, self.cfg.u_min(), self.cfg.u_max())
        self._applied_control_np = applied.copy()
        self.ctrl.U[0] = self.torch.as_tensor(applied, dtype=self.torch.double,
                                             device=self.goal.device)

    def reset_applied_control(self) -> None:
        """Re-anchor the next rollout to measured velocity after a safety hold."""
        if hasattr(self, "_applied_control_np"):
            del self._applied_control_np

    def reset_for_new_route(self) -> None:
        """Drop warm-start controls when an operator replaces the mission."""
        self.reset_applied_control()
        self.ctrl.U.zero_()

    def predict_trajectory(self) -> np.ndarray:
        """Vị trí dự đoán của nominal rollout, [T+1, 3], để vẽ lên RViz.

        Tái tạo bằng dynamics numpy từ nominal action sequence U của
        lần command() gần nhất và state đã lưu lúc đó.
        """
        U = self.ctrl.get_action_sequence().detach().cpu().numpy()
        if not hasattr(self, "_last_state_np"):
            return np.zeros((U.shape[0] + 1, 3))
        state = self.torch.as_tensor(
            self._last_state_np, dtype=self.torch.double, device=self.goal.device
        )
        traj = [state[0:3].detach().cpu().numpy().copy()]
        for k in range(U.shape[0]):
            action = self.torch.as_tensor(U[k], dtype=self.torch.double,
                                          device=self.goal.device)
            state = self._dynamics(state, action, k)
            traj.append(state[0:3].detach().cpu().numpy().copy())
        return np.stack(traj)

    def sampled_trajectories(self, top_k: int = 20) -> np.ndarray:
        """Lowest-cost rollouts from the most recent MPPI iteration.

        Returns ``[K, T+1, 3]`` and prepends the measured initial position so
        RViz can show how the sample fan leaves the current vehicle state.
        """
        states = getattr(self.ctrl, "states", None)
        costs = getattr(self.ctrl, "cost_total", None)
        if states is None or costs is None or not hasattr(self, "_last_state_np"):
            return np.zeros((0, 0, 3), dtype=np.float64)
        states = states[0] if states.ndim == 4 else states
        count = min(max(int(top_k), 0), states.shape[0])
        if count == 0:
            return np.zeros((0, states.shape[1] + 1, 3), dtype=np.float64)
        indices = self.torch.argsort(costs)[:count]
        trajectories = states[indices, :, 0:3].detach().cpu().numpy()
        initial = np.broadcast_to(
            self._last_state_np[None, None, 0:3], (count, 1, 3)
        )
        return np.concatenate((initial, trajectories), axis=1)

    def nominal_cost_breakdown(self) -> dict:
        """Human-readable cost components along the nominal action sequence."""
        if not hasattr(self, "_last_state_np"):
            return {}
        cfg = self.cfg
        U = self.ctrl.get_action_sequence().detach().cpu().numpy()
        state = self.torch.as_tensor(
            self._last_state_np, dtype=self.torch.double, device=self.goal.device
        )
        totals = {"goal": 0.0, "obstacle": 0.0, "collision": 0.0, "path": 0.0,
                  "reference_velocity": 0.0, "effort": 0.0,
                  "smoothness": 0.0, "yaw": 0.0}
        obstacles = None if self.obstacles is None else self.obstacles.detach().cpu().numpy()
        reference_path = (
            None if self.reference_path is None
            else self.reference_path.detach().cpu().numpy()
        )
        paper_r_u = self._paper_r_u.detach().cpu().numpy()
        paper_r_delta_u = self._paper_r_delta_u.detach().cpu().numpy()
        goal_np = self.goal.detach().cpu().numpy()
        feasible_actions = []
        for index, requested_action in enumerate(U):
            action_t = self.torch.as_tensor(
                requested_action, dtype=self.torch.double, device=self.goal.device
            )
            state = self._dynamics(state, action_t, index)
            state_np = state.detach().cpu().numpy()
            p, v, yaw = state_np[0:3], state_np[3:6], float(state_np[6])
            action = state_np[7:11]
            feasible_actions.append(action.copy())
            totals["goal"] += cfg.w_goal * float(np.linalg.norm(p - goal_np))
            if (cfg.cost_profile == "paper" and self.reference_positions is not None):
                p_ref = self.reference_positions[index].detach().cpu().numpy()
                v_ref = self.reference_velocities[index].detach().cpu().numpy()
                totals["path"] += cfg.w_path * float(
                    np.sum(((p - p_ref) / cfg.path_scale_m) ** 2)
                )
                totals["reference_velocity"] += cfg.w_reference_velocity * float(
                    np.sum((v - v_ref) ** 2)
                )
            elif reference_path is not None and len(reference_path):
                if len(reference_path) == 1:
                    path_distance = float(np.linalg.norm(p - reference_path[0]))
                else:
                    a = reference_path[:-1]
                    ab = reference_path[1:] - a
                    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
                    rel = p[None, :] - a
                    t = np.clip(np.sum(rel * ab, axis=1) / denom, 0.0, 1.0)
                    residual = rel - t[:, None] * ab
                    path_distance = float(np.sqrt(np.min(np.sum(residual * residual, axis=1))))
                totals["path"] += cfg.w_path * (path_distance / cfg.path_scale_m) ** 2
            if obstacles is not None and len(obstacles):
                nearest = float(np.linalg.norm(obstacles - p, axis=1).min())
                if cfg.cost_profile == "paper":
                    totals["collision"] += cfg.w_collision * float(
                        nearest <= cfg.collision_radius_m
                    )
                else:
                    raw = float(np.logaddexp(0.0, 2.0 * (cfg.margin - nearest)) / 2.0
                                - math.log(2.0) / 2.0)
                    totals["obstacle"] += cfg.w_obstacle * raw
            if cfg.cost_profile == "paper":
                totals["effort"] += float(np.dot(action * action, paper_r_u))
            else:
                totals["effort"] += cfg.w_u * float(np.dot(action[0:3], action[0:3]))
            totals["smoothness"] += cfg.w_du * float(np.dot(action[0:3] - v, action[0:3] - v))
            desired = math.atan2(float(action[1]), float(action[0]))
            yaw_error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            totals["yaw"] += cfg.w_yaw * float(np.linalg.norm(action[0:2])) * yaw_error**2
        terminal_reference = (
            self.reference_positions[-1].detach().cpu().numpy()
            if cfg.cost_profile == "paper" and self.reference_positions is not None
            else goal_np
        )
        totals["terminal"] = cfg.w_terminal * float(
            np.linalg.norm(p - terminal_reference) ** 2
        )
        if cfg.cost_profile == "paper" and len(feasible_actions) > 1:
            feasible_actions = np.asarray(feasible_actions)
            delta_u = feasible_actions[1:] - feasible_actions[:-1]
            totals["input_change"] = float(
                np.sum(delta_u * delta_u * paper_r_delta_u[None, :])
            )
        else:
            totals["input_change"] = 0.0
        totals["total_nominal"] = float(sum(totals.values()))
        return totals

    def optimizer_diagnostics(self) -> dict:
        """Timing, optimizer concentration and nominal cost diagnostics."""
        out = {"planner": "mppi", "compute_ms": float(getattr(self, "last_compute_ms", 0.0))}
        omega = getattr(self.ctrl, "omega", None)
        costs = getattr(self.ctrl, "cost_total", None)
        if omega is not None:
            out["ess"] = float((1.0 / (omega.square().sum().clamp_min(1e-12))).item())
        if costs is not None:
            out["best_sample_cost"] = float(costs.min().item())
            out["mean_sample_cost"] = float(costs.mean().item())
        if self.reference_positions is not None:
            out["reference_progress_m"] = float(self._path_progress_m)
            out["reference_position_enu"] = (
                self.reference_positions[0].detach().cpu().numpy().tolist()
            )
            out["reference_velocity_enu"] = (
                self.reference_velocities[0].detach().cpu().numpy().tolist()
            )
        out["collision_radius_m"] = float(self.cfg.collision_radius_m)
        out["cost"] = self.nominal_cost_breakdown()
        return out

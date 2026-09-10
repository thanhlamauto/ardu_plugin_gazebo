"""Vanilla MPPI cho quadrotor bám velocity setpoint (companion-side planner).

ArduPilot lo attitude control + velocity tracking, nên mô hình quy hoạch
chỉ cần point-mass với first-order lag (quad "có inertia nhẹ"):

    state   x = [px, py, pz, vx, vy, vz, yaw]          (7)
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
    # Conditioning of the receding-horizon command before it is sent to AP.
    # This is an interface constraint, not part of the PA-MPPI paper objective.
    command_alpha: float = 0.45
    max_accel_xy: float = 1.5  # [m/s^2]
    max_accel_z: float = 0.8  # [m/s^2]
    max_yaw_accel: float = 1.2  # [rad/s^2]
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

    NX = 7
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
        )

    # -- mô hình ---------------------------------------------------------
    def _dynamics(self, state, action):
        torch = self.torch
        cfg = self.cfg
        p = state[..., 0:3]
        v = state[..., 3:6]
        yaw = state[..., 6]
        v_cmd = action[..., 0:3]
        yaw_rate = action[..., 3]
        alpha = min(cfg.dt / cfg.tau, 1.0)
        v_next = v + alpha * (v_cmd - v)
        p_next = p + v_next * cfg.dt
        yaw_next = yaw + yaw_rate * cfg.dt
        return torch.cat((p_next, v_next, yaw_next.unsqueeze(-1)), dim=-1)

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

    def _running_cost(self, state, action):
        torch = self.torch
        cfg = self.cfg
        p = state[..., 0:3]
        v = state[..., 3:6]
        yaw = state[..., 6]
        goal_cost = torch.linalg.vector_norm(p - self.goal, dim=-1)
        obstacle_cost = self._obstacle_cost(p)
        effort_cost = (action[..., 0:3] ** 2).sum(dim=-1)
        accel_cost = ((action[..., 0:3] - v) ** 2).sum(dim=-1)
        # ưu tiên yaw hướng theo chiều bay, tỉ lệ với tốc ngang
        desired_yaw = torch.atan2(action[..., 1], action[..., 0])
        yaw_error = self.wrap_angle(desired_yaw - yaw)
        speed_xy = torch.sqrt(action[..., 0] ** 2 + action[..., 1] ** 2)
        yaw_cost = speed_xy * yaw_error**2
        return (
            cfg.w_goal * goal_cost
            + cfg.w_obstacle * obstacle_cost
            + cfg.w_u * effort_cost
            + cfg.w_du * accel_cost
            + cfg.w_yaw * yaw_cost
        )

    def _terminal_cost(self, states, actions):
        torch = self.torch
        p_T = states[..., -1, 0:3]
        return self.cfg.w_terminal * (torch.linalg.vector_norm(p_T - self.goal, dim=-1) ** 2)

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

    def command(self, pos, vel, yaw: float) -> np.ndarray:
        """Trả về u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]."""
        state_np = np.concatenate((np.asarray(pos, dtype=np.float64),
                                   np.asarray(vel, dtype=np.float64),
                                   [float(yaw)]))
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

    def predict_trajectory(self) -> np.ndarray:
        """Vị trí dự đoán của nominal rollout, [T+1, 3], để vẽ lên RViz.

        Tái tạo bằng dynamics numpy từ nominal action sequence U của
        lần command() gần nhất và state đã lưu lúc đó.
        """
        U = self.ctrl.get_action_sequence().detach().cpu().numpy()
        if not hasattr(self, "_last_state_np"):
            return np.zeros((U.shape[0] + 1, 3))
        p = self._last_state_np[0:3].copy()
        v = self._last_state_np[3:6].copy()
        traj = [p.copy()]
        alpha = min(self.cfg.dt / self.cfg.tau, 1.0)
        for k in range(U.shape[0]):
            v = v + alpha * (U[k, 0:3] - v)
            p = p + v * self.cfg.dt
            traj.append(p.copy())
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
        p = self._last_state_np[0:3].copy()
        v = self._last_state_np[3:6].copy()
        yaw = float(self._last_state_np[6])
        totals = {"goal": 0.0, "obstacle": 0.0, "effort": 0.0,
                  "smoothness": 0.0, "yaw": 0.0}
        alpha = min(cfg.dt / cfg.tau, 1.0)
        obstacles = None if self.obstacles is None else self.obstacles.detach().cpu().numpy()
        for action in U:
            v_next = v + alpha * (action[0:3] - v)
            p = p + v_next * cfg.dt
            yaw += float(action[3]) * cfg.dt
            totals["goal"] += cfg.w_goal * float(np.linalg.norm(p - self.goal.detach().cpu().numpy()))
            if obstacles is not None and len(obstacles):
                nearest = float(np.linalg.norm(obstacles - p, axis=1).min())
                raw = float(np.logaddexp(0.0, 2.0 * (cfg.margin - nearest)) / 2.0
                            - math.log(2.0) / 2.0)
                totals["obstacle"] += cfg.w_obstacle * raw
            totals["effort"] += cfg.w_u * float(np.dot(action[0:3], action[0:3]))
            totals["smoothness"] += cfg.w_du * float(np.dot(action[0:3] - v, action[0:3] - v))
            desired = math.atan2(float(action[1]), float(action[0]))
            yaw_error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            totals["yaw"] += cfg.w_yaw * float(np.linalg.norm(action[0:2])) * yaw_error**2
            v = v_next
        totals["terminal"] = cfg.w_terminal * float(
            np.linalg.norm(p - self.goal.detach().cpu().numpy()) ** 2
        )
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
        out["cost"] = self.nominal_cost_breakdown()
        return out

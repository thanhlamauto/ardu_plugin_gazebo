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
    w_stopping: float = 0.0  # anticipatory stopping-clearance cost
    stopping_margin_m: float = 2.0
    stopping_delay_s: float = 0.25
    w_du: float = 0.2
    w_yaw: float = 0.2  # ưu tiên yaw hướng theo chiều bay
    # Optional global-path tracking term.  Zero preserves the historical
    # point-to-waypoint baseline.  The path is supplied at runtime as an ENU
    # polyline; this adapts the position part of the reference cost in
    # Minařík et al. (2024) to the current velocity-level state.
    path_progress_objective: bool = False
    w_progress: float = 0.0
    w_speed_limit: float = 0.0
    w_path: float = 0.0
    path_scale_m: float = 1.0
    w_reference_velocity: float = 0.0
    reference_speed_m_s: float = 1.0
    reference_accel_m_s2: float = 0.0  # opt-in retiming; zero preserves baseline
    reference_ramp_from_measured: bool = True  # false starts at the path speed cap
    validate_final_trajectory: bool = False  # opt-in observed-cloud swept check
    validate_stopping_trajectory: bool = False  # check every predicted state after conditioning
    feasible_sample_weighting: bool = False  # mask samples with the same predicate as final gate
    stopping_guard_uncertainty_m: float = 0.0  # explicit margin; requires calibration
    known_obstacle_sdf: Optional[str] = None  # explicit prior map, never inferred from LiDAR
    response_accel_model: bool = False
    response_accel_xy: float = 3.0
    response_jerk_xy: float = 5.0
    reference_lateral_accel_m_s2: float = 0.6
    proactive_proposals: bool = False  # sample braking/turning before rejection
    reference_warm_start: bool = False  # opt-in proposal initialization/tail
    brake_swept_path: bool = False  # opt-in stopping-segment cloud geometry
    # Optional geometric rounding of interior global-path corners.  Zero keeps
    # the original polyline.  This is a project reference generator, not an
    # obstacle-aware global planner; collision cost remains authoritative.
    reference_corner_radius_m: float = 0.0
    reference_corner_samples: int = 5
    # ``project`` keeps the current proximity-softplus objective.  ``paper``
    # selects the subset mapped from Minařík et al.: input effort, input-change
    # effort, position reference and collision indicator.
    cost_profile: str = "project"
    w_collision: float = 1.0e6
    collision_cost_buffer_m: float = 0.0  # extra optimization margin; gate radius unchanged
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
        from pytorch_mppi.mppi import SpecificActionSampler

        self.torch = torch
        self.cfg = cfg or MPPIConfig()
        if self.cfg.validate_stopping_trajectory and not self.cfg.validate_final_trajectory:
            raise ValueError('stopping trajectory guard requires final trajectory validation')
        if self.cfg.feasible_sample_weighting and not self.cfg.validate_stopping_trajectory:
            raise ValueError('feasible sample weighting requires the stopping trajectory guard')
        if (not np.isfinite(self.cfg.stopping_guard_uncertainty_m)
                or self.cfg.stopping_guard_uncertainty_m < 0):
            raise ValueError('stopping guard uncertainty must be finite and nonnegative')
        response_limits = [self.cfg.tau, self.cfg.response_accel_xy, self.cfg.response_jerk_xy]
        if not np.isfinite(response_limits).all() or min(response_limits) <= 0:
            raise ValueError('response limits and tau must be positive')
        if self.cfg.response_accel_model:
            self.NX = 14  # add measured acceleration memory
        self.known_geometry = None
        if self.cfg.known_obstacle_sdf:
            from .known_geometry import KnownGeometry
            self.known_geometry = KnownGeometry(self.cfg.known_obstacle_sdf)
        if (not np.isfinite(self.cfg.reference_accel_m_s2)
                or self.cfg.reference_accel_m_s2 < 0
                or not np.isfinite(self.cfg.reference_lateral_accel_m_s2)
                or self.cfg.reference_lateral_accel_m_s2 <= 0):
            raise ValueError('Invalid reference acceleration budgets')
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
        if (self.cfg.reference_corner_radius_m < 0
                or not np.isfinite(self.cfg.reference_corner_radius_m)):
            raise ValueError("reference_corner_radius_m phải hữu hạn và không âm")
        if (not isinstance(self.cfg.reference_corner_samples, int)
                or self.cfg.reference_corner_samples < 2):
            raise ValueError("reference_corner_samples phải là số nguyên >= 2")
        if self.cfg.cost_profile not in ("project", "paper"):
            raise ValueError("cost_profile phải là 'project' hoặc 'paper'")
        if self.cfg.w_collision < 0 or not np.isfinite(self.cfg.w_collision):
            raise ValueError("w_collision phải hữu hạn và không âm")
        if not np.isfinite(self.cfg.collision_cost_buffer_m) or self.cfg.collision_cost_buffer_m < 0:
            raise ValueError("collision_cost_buffer_m must be finite and nonnegative")
        if (not np.isfinite([self.cfg.w_stopping, self.cfg.stopping_margin_m, self.cfg.stopping_delay_s]).all()
                or min(self.cfg.w_stopping, self.cfg.stopping_margin_m, self.cfg.stopping_delay_s) < 0
                or (self.cfg.w_stopping and self.cfg.max_accel_xy <= 0)):
            raise ValueError("invalid stopping cost parameters")
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
        original_weighting = self.ctrl._compute_weighting
        def safety_weighting(cost_total):
            if not self.cfg.feasible_sample_weighting:
                return original_weighting(cost_total)
            result = self._evaluate_latest_sample_safety()
            mask = result['safe']
            self._last_sample_safety = result
            self._last_sample_feasible_mask = mask
            if bool(mask.any().item()):
                weights = original_weighting(
                    self.torch.where(mask, cost_total,
                                     self.torch.full_like(cost_total, float('inf'))))
            else:
                weights = self.torch.zeros_like(cost_total)
                self.ctrl.omega = weights
            self._last_safe_weight_mass = float(weights[mask].sum().item())
            return weights
        self.ctrl._compute_weighting = safety_weighting
        owner = self
        class RecoverySampler(SpecificActionSampler):
            def sample_trajectories(self, state, info):
                return owner.recovery_proposals()
        self.ctrl.specific_action_sampler = RecoverySampler()

    def configure_trajectory_safety(self, *, collision_radius, acceleration,
                                    delay, stopping_clearance, uncertainty=0.0,
                                    sample_spacing=0.1):
        """Install the predicate parameters shared by sample and final checks."""
        self._trajectory_safety_parameters = dict(
            collision_radius=float(collision_radius), acceleration=float(acceleration),
            delay=float(delay), stopping_clearance=float(stopping_clearance),
            uncertainty=float(uncertainty), sample_spacing=float(sample_spacing))

    def _evaluate_safety_states(self, states, *, detailed=False):
        from .trajectory_safety import evaluate_trajectory_safety_batch
        if not hasattr(self, '_trajectory_safety_parameters'):
            raise RuntimeError('trajectory safety predicate is not configured')
        return evaluate_trajectory_safety_batch(
            states, [] if self.obstacles is None else self.obstacles.detach().cpu().numpy(),
            self.known_geometry, torch_module=self.torch, detailed=detailed,
            **self._trajectory_safety_parameters)

    def _evaluate_latest_sample_safety(self):
        states = self.ctrl.states
        states = states[0] if states.ndim == 4 else states
        initial = self.torch.as_tensor(
            self._last_state_np, dtype=states.dtype, device=states.device)
        initial = initial.view(1, 1, -1).expand(states.shape[0], 1, -1)
        return self._evaluate_safety_states(self.torch.cat((initial, states), dim=1))

    def recovery_proposals(self):
        """Explicit alternatives in the MPPI sample pool; reference/cost unchanged."""
        torch = self.torch
        if not self.cfg.proactive_proposals and not getattr(self, '_rejection_recovery', False):
            return torch.zeros((0, self.cfg.horizon, self.NU), dtype=torch.double, device=self.goal.device)
        proposals = [np.zeros((self.cfg.horizon, self.NU))]
        if self.reference_path is not None:
            for speed in (.5, 1., 2., 4.):
                progress = self._path_progress_m + np.arange(self.cfg.horizon+1)*speed*self.cfg.dt
                positions = self._sample_path_at_progress(progress)
                actions = np.zeros((self.cfg.horizon, self.NU))
                actions[:, :3] = np.diff(positions, axis=0)/self.cfg.dt
                proposals.append(actions)
        if self.cfg.proactive_proposals and self.reference_path is not None:
            # Sample multiple braking schedules; none changes the tracking reference.
            initial = float(np.linalg.norm(getattr(self, '_last_state_np', np.zeros(11))[3:5]))
            for target in (1., 2., 4., 6., self.cfg.reference_speed_m_s):
                for decel in (1.5, 3.):
                    speed = initial
                    distances = [self._path_progress_m]
                    for _ in range(self.cfg.horizon):
                        speed += np.clip(target-speed, -decel*self.cfg.dt, decel*self.cfg.dt)
                        distances.append(distances[-1]+speed*self.cfg.dt)
                    positions = self._sample_path_at_progress(np.asarray(distances))
                    actions = np.zeros((self.cfg.horizon, self.NU))
                    actions[:, :3] = np.diff(positions, axis=0)/self.cfg.dt
                    proposals.append(actions)
        result = np.asarray(proposals[:self.cfg.samples])
        result = np.clip(result, self.cfg.u_min(), self.cfg.u_max())
        return torch.as_tensor(result, dtype=torch.double, device=self.goal.device)

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
        return self._advance_applied(state, applied)

    def _advance_applied(self, state, applied):
        """Advance with an already-conditioned command, exactly once."""
        torch, cfg = self.torch, self.cfg
        p, v, yaw = state[..., :3], state[..., 3:6], state[..., 6]
        v_cmd = applied[..., 0:3]
        yaw_rate = applied[..., 3]
        alpha = min(cfg.dt / cfg.tau, 1.0)
        v_next = v + alpha * (v_cmd - v)
        if self.cfg.response_accel_model:
            previous_accel = state[..., 11:14]
            desired = (v_cmd-v)/cfg.tau
            def limit_xy(x, bound):
                xy=x[..., :2]
                xy=xy*torch.clamp(bound/torch.linalg.vector_norm(xy,dim=-1,keepdim=True).clamp_min(1e-12),max=1.)
                return torch.cat((xy,x[...,2:3]),dim=-1)
            desired=limit_xy(desired,cfg.response_accel_xy)
            change=limit_xy(desired-previous_accel,cfg.response_jerk_xy*cfg.dt)
            acceleration=previous_accel+change
            # Vertical response remains first order; XY uses acceleration memory.
            acceleration=torch.cat((acceleration[...,:2],desired[...,2:3]),dim=-1)
            v_next=v+acceleration*cfg.dt
        p_next = p + v_next * cfg.dt
        yaw_next = yaw + yaw_rate * cfg.dt
        result=torch.cat((p_next, v_next, yaw_next.unsqueeze(-1), applied), dim=-1)
        return torch.cat((result,acceleration),dim=-1) if cfg.response_accel_model else result

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
        nearest=torch.full(p.shape[:-1],float('inf'),dtype=p.dtype,device=p.device)
        if self.obstacles is not None and self.obstacles.shape[0]:
            nearest=torch.cdist(p.reshape(-1,3),self.obstacles).min(dim=1).values.reshape(p.shape[:-1])
        if self.known_geometry is not None:
            nearest=torch.minimum(nearest,self.known_geometry.torch_clearance(p))
        return (nearest <= self.cfg.collision_radius_m + self.cfg.collision_cost_buffer_m).to(p.dtype)

    def _stopping_cost(self, p, v):
        """Penalty for predicted states whose straight stopping segment is crowded."""
        torch = self.torch
        speed = torch.linalg.vector_norm(v, dim=-1)
        length = speed*self.cfg.stopping_delay_s + speed.square()/(2*self.cfg.max_accel_xy)
        direction = v / speed.clamp_min(1e-9).unsqueeze(-1)
        nearest = torch.full_like(speed, float('inf'))
        if self.obstacles is not None and len(self.obstacles):
            offsets = self.obstacles - p.unsqueeze(-2)
            along = (offsets*direction.unsqueeze(-2)).sum(dim=-1).clamp_min(0)
            along = torch.minimum(along, length.unsqueeze(-1))
            nearest = torch.linalg.vector_norm(offsets-along.unsqueeze(-1)*direction.unsqueeze(-2),dim=-1).min(dim=-1).values
        return (self.cfg.stopping_margin_m-nearest).clamp_min(0).square()

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
            a = self._reference_a
            ab = self._reference_ab
            denom = self._reference_ab_denom
            rel = flat[:, None, :] - a[None, :, :]
            t = (rel * ab[None, :, :]).sum(dim=-1) / denom[None, :]
            t = t.clamp(0.0, 1.0)
            residual = rel - t[..., None] * ab[None, :, :]
            d2 = (residual * residual).sum(dim=-1).min(dim=-1).values
        return torch.sqrt(d2.clamp_min(0.0)).reshape(p.shape[:-1])

    def _geometric_progress(self, p):
        """Arc length of nearest projection; no timestamp or speed schedule."""
        torch = self.torch
        if self.reference_path is None or len(self.reference_path) < 2:
            return torch.zeros(p.shape[:-1], dtype=p.dtype, device=p.device)
        a = self._reference_a
        ab = self._reference_ab
        lengths = self._reference_lengths_torch
        rel = p.unsqueeze(-2) - a
        fraction = ((rel * ab).sum(-1) / lengths.square().clamp_min(1e-12)).clamp(0, 1)
        distance2 = (rel - fraction.unsqueeze(-1)*ab).square().sum(-1)
        index = distance2.argmin(-1, keepdim=True)
        return (self._reference_cumulative_torch + fraction*lengths).gather(-1,index).squeeze(-1)

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
        if (not cfg.path_progress_objective and cfg.cost_profile == "paper" and t is not None
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
            + (cfg.w_stopping*self._stopping_cost(p, v) if cfg.w_stopping and (t is None or int(t) % 5 == 0) else 0)
            + effort_term
            + cfg.w_du * accel_cost
            + cfg.w_yaw * yaw_cost
            + cfg.w_path * path_cost
            + cfg.w_reference_velocity * reference_velocity_cost
            + (cfg.w_speed_limit * (torch.linalg.vector_norm(v[..., :2], dim=-1)-cfg.vmax).clamp_min(0).square() if cfg.path_progress_objective else 0)
        )

    def _terminal_cost(self, states, actions):
        torch = self.torch
        p_T = states[..., -1, 0:3]
        terminal_reference = (
            self.reference_positions[-1]
            if not self.cfg.path_progress_objective and self.cfg.cost_profile == "paper" and self.reference_positions is not None
            else self.goal
        )
        cost = self.cfg.w_terminal * (
            torch.linalg.vector_norm(p_T - terminal_reference, dim=-1) ** 2
        )
        if self.cfg.path_progress_objective:
            start = torch.as_tensor(self._last_state_np[:3], dtype=p_T.dtype, device=p_T.device)
            cost = cost - self.cfg.w_progress * (self._geometric_progress(p_T)-self._geometric_progress(start))
        if self.cfg.cost_profile == "paper" and actions is not None:
            # Paper Eq. (16): sum of weighted input changes.  The installed
            # MPPI library passes the complete action sequence here, so this
            # term is evaluated once per rollout at the terminal callback.
            feasible_actions = states[..., 7:11] if states.shape[-1] >= self.NX else actions
            # The first command is the only command executed this cycle, so
            # include its jump from the command actually sent on the previous
            # cycle.  Omitting this boundary term lets receding-horizon output
            # jitter even when the remaining in-horizon sequence is smooth.
            if hasattr(self, "_last_state_np"):
                previous = self.torch.as_tensor(
                    self._last_state_np[7:11], dtype=feasible_actions.dtype,
                    device=feasible_actions.device,
                )
            else:
                previous = self.torch.zeros(
                    self.NU, dtype=feasible_actions.dtype,
                    device=feasible_actions.device,
                )
            previous = previous.reshape(
                *([1] * (feasible_actions.ndim - 2)), self.NU
            )
            first_delta = feasible_actions[..., 0, :] - previous
            later_deltas = feasible_actions[..., 1:, :] - feasible_actions[..., :-1, :]
            deltas = self.torch.cat((first_delta.unsqueeze(-2), later_deltas), dim=-2)
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
            self.reference_path_raw = None
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
        self.reference_path_raw = self.torch.as_tensor(
            arr, dtype=self.torch.double, device=self.goal.device
        )
        arr = self._round_reference_corners(arr)
        self.reference_path = self.torch.as_tensor(
            arr, dtype=self.torch.double, device=self.goal.device
        )
        self._reference_a = self.reference_path[:-1]
        self._reference_ab = self.reference_path[1:] - self.reference_path[:-1]
        self._reference_ab_denom = (
            self._reference_ab*self._reference_ab).sum(-1).clamp_min(1e-12)
        self._reference_lengths_torch = self.torch.linalg.vector_norm(
            self._reference_ab, dim=-1)
        self._reference_cumulative_torch = self.torch.cat((
            self._reference_lengths_torch.new_zeros(1),
            self._reference_lengths_torch.cumsum(0)[:-1]))
        segment_lengths = np.linalg.norm(np.diff(arr, axis=0), axis=1)
        if not np.any(segment_lengths > 1e-9):
            raise ValueError("reference path phải có ít nhất một đoạn khác 0")
        self._path_segment_lengths = segment_lengths
        self._path_cumulative_lengths = np.r_[0.0, np.cumsum(segment_lengths)]
        self._path_progress_m = 0.0

    def _round_reference_corners(self, path: np.ndarray) -> np.ndarray:
        """Replace interior polyline vertices by small quadratic Bezier arcs.

        Endpoints are preserved.  Each trim distance is capped at 45% of both
        adjacent segments, so short segments cannot be consumed completely.
        The result only smooths the supplied reference geometry; it does not
        certify obstacle clearance.
        """
        radius = float(self.cfg.reference_corner_radius_m)
        if radius <= 0.0 or len(path) < 3:
            return path.copy()

        rounded = [path[0].copy()]

        def append_unique(point):
            if np.linalg.norm(point - rounded[-1]) > 1e-9:
                rounded.append(point.copy())

        for index in range(1, len(path) - 1):
            previous, corner, following = path[index - 1:index + 2]
            incoming = corner - previous
            outgoing = following - corner
            len_in = float(np.linalg.norm(incoming))
            len_out = float(np.linalg.norm(outgoing))
            if len_in <= 1e-9 or len_out <= 1e-9:
                append_unique(corner)
                continue
            direction_in = incoming / len_in
            direction_out = outgoing / len_out
            cosine = float(np.clip(np.dot(direction_in, direction_out), -1.0, 1.0))
            if abs(cosine) > 0.9999:
                append_unique(corner)
                continue
            trim = min(radius, 0.45 * len_in, 0.45 * len_out)
            entry = corner - trim * direction_in
            exit_ = corner + trim * direction_out
            append_unique(entry)
            for t in np.linspace(0.0, 1.0, self.cfg.reference_corner_samples + 1)[1:]:
                point = ((1.0 - t) ** 2 * entry
                         + 2.0 * (1.0 - t) * t * corner
                         + t ** 2 * exit_)
                append_unique(point)
        append_unique(path[-1])
        return np.asarray(rounded, dtype=np.float64)

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

    def _update_reference_trajectory(self, position: np.ndarray, velocity=None) -> None:
        if self.reference_path is None or self.reference_path.shape[0] < 2:
            self.reference_positions = None
            self.reference_velocities = None
            return
        projected = self._project_path_progress(position)
        self._path_progress_m = max(self._path_progress_m, projected)
        offsets = np.arange(1, self.cfg.horizon + 1, dtype=np.float64)
        progress = self._path_progress_m + offsets * self.cfg.reference_speed_m_s * self.cfg.dt
        if self.cfg.reference_accel_m_s2 > 0:
            # Geometry-based cruise cap, backward braking pass and forward
            # acceleration from measured speed. This is reference scheduling,
            # not a flight-safety certificate or a change to MPPI costs.
            total = float(self._path_cumulative_lengths[-1])
            s = np.linspace(0, total, max(3, int(np.ceil(total / .1)) + 1))
            xyz = self._sample_path_at_progress(s)
            tangent = np.gradient(xyz, s, axis=0)
            tangent /= np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-9)
            curvature = np.linalg.norm(np.gradient(tangent, s, axis=0), axis=1)
            caps = np.minimum(min(self.cfg.reference_speed_m_s, self.cfg.vmax),
                np.sqrt(self.cfg.reference_lateral_accel_m_s2 / np.maximum(curvature, 1e-6)))
            caps[-1] = 0
            accel = min(self.cfg.reference_accel_m_s2, self.cfg.max_accel_xy)
            for j in range(len(s) - 2, -1, -1):
                caps[j] = min(caps[j], np.sqrt(caps[j+1]**2 + 2*accel*(s[j+1]-s[j])))
            at = self._path_progress_m
            speed = float(np.linalg.norm(velocity)) if velocity is not None else 0.0
            if not self.cfg.reference_ramp_from_measured:
                # Request cruise immediately where geometry permits, retaining
                # curvature, backward braking and forward acceleration limits.
                speed = float(np.interp(at, s, caps))
            scheduled = []
            for _ in offsets:
                target = float(np.interp(at, s, caps))
                speed = max(0., min(target, speed + accel*self.cfg.dt))
                at = min(total, at + speed*self.cfg.dt)
                scheduled.append(at)
            progress = np.asarray(scheduled)
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

    def observe_motion(self, velocity, stamp):
        if not self.cfg.response_accel_model:
            return
        v=np.asarray(velocity,dtype=float)
        if stamp is not None and hasattr(self,'_motion_observation'):
            last_v,last_t=self._motion_observation
            dt=stamp-last_t
            if 0 < dt <= .5:
                a=(v-last_v)/dt
                a[:2]*=min(1.,self.cfg.response_accel_xy/max(np.linalg.norm(a[:2]),1e-12))
                self._observed_acceleration=a
            elif dt < 0 or dt > .5:
                self._observed_acceleration=np.zeros(3)
        if stamp is not None:
            self._motion_observation=(v.copy(),stamp)

    def command(self, pos, vel, yaw: float) -> np.ndarray:
        """Trả về u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]."""
        pos_np = np.asarray(pos, dtype=np.float64)
        vel_np = np.asarray(vel, dtype=np.float64)
        self._update_reference_trajectory(pos_np, vel_np)
        if hasattr(self, "_applied_control_np"):
            applied = self._applied_control_np.copy()
        else:
            applied = np.r_[vel_np, 0.0]
            applied = np.clip(applied, self.cfg.u_min(), self.cfg.u_max())
        state_np = np.concatenate((pos_np, vel_np, [float(yaw)], applied))
        if self.cfg.response_accel_model:
            state_np=np.r_[state_np,getattr(self,'_observed_acceleration',np.zeros(3))]
        if not np.isfinite(state_np).all():
            raise ValueError("MPPI state chứa NaN/Inf")
        state = self.torch.tensor(
            state_np, dtype=self.torch.double, device=self.goal.device
        )
        self._last_state_np = state_np
        if getattr(self, '_rejection_recovery', False):
            # Do not overwrite a refined nominal every rejected cycle.
            self.ctrl.u_init.zero_()
            if getattr(self, '_recovery_seed_pending', False):
                self.ctrl.U.zero_()
                self._recovery_seed_pending = False
        elif self.cfg.reference_warm_start and self.reference_velocities is not None:
            # Seed the proposal, not the command actually applied. Rollout and
            # MAVLink conditioning still impose the same acceleration limits.
            reference_actions = self.torch.zeros_like(self.ctrl.U)
            reference_actions[:, :3] = self.reference_velocities
            lower = self.torch.as_tensor(self.cfg.u_min(), device=self.goal.device)
            upper = self.torch.as_tensor(self.cfg.u_max(), device=self.goal.device)
            reference_actions = reference_actions.clamp(lower, upper)
            self.ctrl.u_init = reference_actions[-1].clone()
            if not getattr(self, '_reference_warm_started', False):
                self.ctrl.U.copy_(reference_actions)
                self._reference_warm_started = True
        started = time.perf_counter()
        if getattr(self, '_capture_rng', False):
            self._rng_state_before_command = self.torch.get_rng_state().clone()
            self._nominal_before_command = self.ctrl.U.detach().clone()
        u = self.ctrl.command(state)
        self._selected_feasible_sample_index = None
        self.last_compute_ms = (time.perf_counter() - started) * 1000.0
        return u.detach().cpu().numpy().astype(np.float64)

    def select_best_feasible_sample(self):
        """Replace U by the lowest-cost sample already certified this cycle."""
        mask = getattr(self, '_last_sample_feasible_mask', None)
        if mask is None or not bool(mask.any().item()):
            return None
        masked_cost = self.torch.where(
            mask, self.ctrl.cost_total,
            self.torch.full_like(self.ctrl.cost_total, float('inf')))
        index = int(self.torch.argmin(masked_cost).item())
        self.ctrl.U.copy_(self.ctrl.perturbed_action[index])
        states = self.ctrl.states[0] if self.ctrl.states.ndim == 4 else self.ctrl.states
        applied = states[index, 0, 7:11].detach().cpu().numpy().astype(np.float64)
        raw = self.ctrl.U[0].detach().cpu().numpy().astype(np.float64)
        self._selected_feasible_sample_index = index
        return raw, applied, index

    def accept_applied_control(self, control) -> None:
        """Remember the applied command separately from raw nominal U.

        The next command shifts U[0] out; its applied value belongs in the
        augmented state, not in the raw action sequence used by diagnostics.
        """
        applied = np.asarray(control, dtype=np.float64)
        if applied.shape != (self.NU,) or not np.isfinite(applied).all():
            raise ValueError("applied MPPI control must be a finite 4-vector")
        applied = np.clip(applied, self.cfg.u_min(), self.cfg.u_max())
        self._applied_control_np = applied.copy()

    def reset_applied_control(self) -> None:
        """Re-anchor the next rollout to measured velocity after a safety hold."""
        if hasattr(self, "_applied_control_np"):
            del self._applied_control_np
        self._reference_warm_started = False
        if self.cfg.reference_warm_start:
            self.ctrl.u_init.zero_()

    def reject_nominal(self) -> None:
        """Zero was sent, but do not discard optimizer progress after every rejection."""
        self.accept_applied_control(np.zeros(self.NU))
        if not getattr(self, '_rejection_recovery', False):
            self._recovery_seed_pending = True
        self._rejection_recovery = True

    def reset_for_new_route(self) -> None:
        """Drop warm-start controls when an operator replaces the mission."""
        self.reset_applied_control()
        self._rejection_recovery = False
        self._recovery_seed_pending = False
        self.ctrl.U.zero_()

    def predict_states(self, first_applied=None) -> np.ndarray:
        """Nominal state rollout using the optimizer's exact dynamics transition.

        Tái tạo bằng dynamics numpy từ nominal action sequence U của
        lần command() gần nhất và state đã lưu lúc đó.
        """
        U = self.ctrl.get_action_sequence().detach().cpu().numpy()
        if not hasattr(self, "_last_state_np"):
            return np.zeros((U.shape[0] + 1, self.NX))
        state = self.torch.as_tensor(
            self._last_state_np, dtype=self.torch.double, device=self.goal.device
        )
        traj = [state.detach().cpu().numpy().copy()]
        for k in range(U.shape[0]):
            action = self.torch.as_tensor(U[k], dtype=self.torch.double,
                                          device=self.goal.device)
            if k == 0 and first_applied is not None:
                applied = self.torch.as_tensor(first_applied, dtype=self.torch.double,
                                               device=self.goal.device)
                state = self._advance_applied(state, applied)
            else:
                state = self._dynamics(state, action, k)
            traj.append(state.detach().cpu().numpy().copy())
        return np.stack(traj)

    def predict_trajectory(self, first_applied=None) -> np.ndarray:
        """Positions of the same rollout, [T+1, 3], for gate and RViz."""
        return self.predict_states(first_applied)[:, :3]

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
        totals["stopping"] = 0.0
        totals["speed_limit"] = 0.0
        feasible_actions = []
        for index, requested_action in enumerate(U):
            action_t = self.torch.as_tensor(
                requested_action, dtype=self.torch.double, device=self.goal.device
            )
            state = self._dynamics(state, action_t, index)
            state_np = state.detach().cpu().numpy()
            p, v, yaw = state_np[0:3], state_np[3:6], float(state_np[6])
            if cfg.w_stopping and index % 5 == 0:
                totals["stopping"] += cfg.w_stopping * float(self._stopping_cost(state[:3], state[3:6]).item())
            if cfg.path_progress_objective:
                totals["speed_limit"] += cfg.w_speed_limit * max(0, np.linalg.norm(v[:2])-cfg.vmax)**2
            action = state_np[7:11]
            feasible_actions.append(action.copy())
            totals["goal"] += cfg.w_goal * float(np.linalg.norm(p - goal_np))
            if (not cfg.path_progress_objective and cfg.cost_profile == "paper" and self.reference_positions is not None):
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
            if (obstacles is not None and len(obstacles)) or self.known_geometry is not None:
                nearest = float(np.linalg.norm(obstacles - p, axis=1).min()) if obstacles is not None and len(obstacles) else float('inf')
                if self.known_geometry is not None and cfg.cost_profile == 'paper':
                    nearest = min(nearest, float(self.known_geometry.clearance(p)))
                if cfg.cost_profile == "paper":
                    totals["collision"] += cfg.w_collision * float(
                        nearest <= cfg.collision_radius_m + cfg.collision_cost_buffer_m
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
            if not cfg.path_progress_objective and cfg.cost_profile == "paper" and self.reference_positions is not None
            else goal_np
        )
        totals["terminal"] = cfg.w_terminal * float(
            np.linalg.norm(p - terminal_reference) ** 2
        )
        if cfg.cost_profile == "paper" and len(feasible_actions) > 1:
            feasible_actions = np.asarray(feasible_actions)
            previous_applied = np.asarray(self._last_state_np[7:11], dtype=np.float64)
            delta_u = np.diff(np.vstack((previous_applied, feasible_actions)), axis=0)
            totals["input_change"] = float(
                np.sum(delta_u * delta_u * paper_r_delta_u[None, :])
            )
        else:
            totals["input_change"] = 0.0
        if cfg.path_progress_objective:
            start = self.torch.as_tensor(self._last_state_np[:3],dtype=state.dtype,device=state.device)
            totals["progress"] = -cfg.w_progress * float((self._geometric_progress(state[:3])-self._geometric_progress(start)).item())
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
        mask = getattr(self, '_last_sample_feasible_mask', None)
        if self.cfg.feasible_sample_weighting and mask is not None:
            safe_count = int(mask.sum().item())
            out['feasible_sample_count'] = safe_count
            out['unsafe_sample_count'] = int(len(mask)-safe_count)
            out['weight_mass_on_feasible_samples'] = float(
                getattr(self, '_last_safe_weight_mass', 0.0))
            out['selected_feasible_sample_index'] = self._selected_feasible_sample_index
            if safe_count:
                safe_weights = self.ctrl.omega[mask]
                out['feasible_weight_ess'] = float(
                    1.0/safe_weights.square().sum().clamp_min(1e-300).item())
                positive = safe_weights[safe_weights > 0]
                out['feasible_weight_entropy'] = float(
                    -(positive*positive.log()).sum().item()) if len(positive) else 0.0
                safe_cost = self.ctrl.cost_total[mask]
                out['best_feasible_cost'] = float(safe_cost.min().item())
        if self.reference_positions is not None:
            out["reference_progress_m"] = float(self._path_progress_m)
            out["reference_position_enu"] = (
                self.reference_positions[0].detach().cpu().numpy().tolist()
            )
            out["reference_velocity_enu"] = (
                self.reference_velocities[0].detach().cpu().numpy().tolist()
            )
        out["collision_radius_m"] = float(self.cfg.collision_radius_m)
        out['rejection_recovery'] = bool(getattr(self, '_rejection_recovery', False))
        if self.known_geometry is not None:
            out['known_map'] = {'source': str(self.known_geometry.path), 'sha256':self.known_geometry.sha256}
        if self.cfg.validate_final_trajectory and hasattr(self, '_last_state_np'):
            out['nominal_snapshot'] = {
                'state': self._last_state_np.tolist(),
                'raw_actions': self.ctrl.get_action_sequence().detach().cpu().numpy().tolist(),
                'obstacles_enu': [] if self.obstacles is None else self.obstacles.detach().cpu().numpy().tolist(),
                'dt': self.cfg.dt, 'tau': self.cfg.tau,
                'response_accel_model': self.cfg.response_accel_model,
                'response_accel_xy': self.cfg.response_accel_xy,
                'response_jerk_xy': self.cfg.response_jerk_xy,
                'semantics': 'raw nominal before external conditioning; applied command stored separately',
            }
        out["cost"] = self.nominal_cost_breakdown()
        return out

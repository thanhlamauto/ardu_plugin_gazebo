"""Experimental rigid-body PA-MPPI for Gazebo/SITL.

The planner state is ``[p_W(3), q_WB(wxyz,4), v_W(3), omega_B_FLU(3)]`` and
the sampled control is ``[collective_thrust_N, body_rates_B_FLU(3)]``.  The
orientation maps the Gazebo body FLU frame into world ENU.  ArduPilot still
closes the motor and body-rate loops; ``rate_tau`` models that tracking with a
first-order lag.

This is the next porting stage toward Zhai et al. (RA-L 2026).  It is kept in
an explicit experimental class because switching from velocity targets to
thrust/body-rate targets removes ArduPilot's position-control safety layer.
"""

from dataclasses import dataclass
from typing import Optional
import math
import time

import numpy as np

from .occupancy_grid import OccupancyGrid3D, OccupancyGridConfig
from .pa_mppi_controller import PAMPPIConfig


def body_flu_rates_to_frd(rates) -> np.ndarray:
    """Gazebo body FLU angular rates -> MAVLink/ArduPilot body FRD rates."""
    p, q, r = np.asarray(rates, dtype=np.float64)
    return np.array([p, -q, -r], dtype=np.float64)


@dataclass
class RigidBodyPAMPPIConfig(PAMPPIConfig):
    dt: float = 0.05
    horizon: int = 20
    samples: int = 512
    mass_kg: float = 2.10
    gravity: float = 9.80665
    rate_tau: float = 0.08
    thrust_min_ratio: float = 0.25
    thrust_max_ratio: float = 1.80
    body_rate_max_xy: float = 1.5
    body_rate_max_z: float = 1.0
    noise_thrust_ratio: float = 0.18
    noise_body_rate_xy: float = 0.35
    noise_body_rate_z: float = 0.25
    hover_thrust_normalized: float = 0.38
    w_rigid_thrust: float = 2.0
    w_rigid_rate: float = 1.0
    w_rigid_rate_track: float = 0.5
    w_rigid_tilt: float = 12.0
    w_rigid_velocity: float = 0.5
    w_rigid_altitude: float = 8.0
    w_rigid_vertical_speed: float = 3.0

    @property
    def hover_thrust_n(self) -> float:
        return self.mass_kg * self.gravity

    def u_min(self):
        return [
            self.thrust_min_ratio * self.hover_thrust_n,
            -self.body_rate_max_xy,
            -self.body_rate_max_xy,
            -self.body_rate_max_z,
        ]

    def u_max(self):
        return [
            self.thrust_max_ratio * self.hover_thrust_n,
            self.body_rate_max_xy,
            self.body_rate_max_xy,
            self.body_rate_max_z,
        ]


class RigidBodyPAMPPI:
    """Rigid-body/rate-tracking PA-MPPI with thrust and body-rate output."""

    NX = 13
    NU = 4
    requires_rigid_body_state = True
    command_kind = "thrust-body-rates"

    def __init__(self, cfg: Optional[RigidBodyPAMPPIConfig] = None) -> None:
        import torch
        from pytorch_mppi import MPPI

        self.torch = torch
        self.cfg = cfg or RigidBodyPAMPPIConfig()
        torch.manual_seed(int(self.cfg.seed))
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(int(self.cfg.seed))
        if self.cfg.mass_kg <= 0 or self.cfg.rate_tau <= 0:
            raise ValueError("mass_kg và rate_tau phải lớn hơn 0")
        device = torch.device(self.cfg.device)
        dtype = torch.double
        self.goal = torch.zeros(3, dtype=dtype, device=device)
        self.obstacles = None
        self.map = OccupancyGrid3D(
            OccupancyGridConfig(
                resolution=self.cfg.map_resolution,
                size_m=(self.cfg.map_size_x, self.cfg.map_size_y, self.cfg.map_size_z),
                free_inflation_m=self.cfg.map_free_inflation,
                occupied_inflation_m=self.cfg.map_occupied_inflation,
                sensor_free_radius_m=self.cfg.map_sensor_free_radius,
            )
        )
        self.goal_visible = False
        hover = self.cfg.hover_thrust_n
        noise = torch.tensor(
            [
                (self.cfg.noise_thrust_ratio * hover) ** 2,
                self.cfg.noise_body_rate_xy**2,
                self.cfg.noise_body_rate_xy**2,
                self.cfg.noise_body_rate_z**2,
            ], dtype=dtype, device=device,
        )
        self.ctrl = MPPI(
            self._dynamics,
            self._running_cost,
            self.NX,
            noise_sigma=torch.diag(noise),
            noise_mu=torch.zeros(self.NU, dtype=dtype, device=device),
            num_samples=self.cfg.samples,
            horizon=self.cfg.horizon,
            lambda_=self.cfg.lambda_,
            device=device,
            u_min=torch.tensor(self.cfg.u_min(), dtype=dtype, device=device),
            u_max=torch.tensor(self.cfg.u_max(), dtype=dtype, device=device),
            terminal_state_cost=self._terminal_cost,
            u_init=torch.tensor(
                [hover, 0.0, 0.0, 0.0], dtype=dtype, device=device
            ),
        )
        # pytorch-mppi initializes U from noise_mu (zero).  For a thrust
        # actuator, zero is not a neutral command, so seed the whole nominal
        # sequence at hover before the first optimization iteration.
        self.ctrl.U[:] = torch.tensor(
            [hover, 0.0, 0.0, 0.0], dtype=dtype, device=device
        )

    def _quat_derivative(self, quat, omega):
        w, x, y, z = quat.unbind(dim=-1)
        p, q, r = omega.unbind(dim=-1)
        return 0.5 * self.torch.stack(
            (
                -x * p - y * q - z * r,
                w * p + y * r - z * q,
                w * q + z * p - x * r,
                w * r + x * q - y * p,
            ), dim=-1,
        )

    def _body_axes_world(self, quat):
        """Return body x and z axes expressed in world ENU."""
        w, x, y, z = quat.unbind(dim=-1)
        body_x = self.torch.stack(
            (1 - 2 * (y*y + z*z), 2 * (x*y + w*z), 2 * (x*z - w*y)),
            dim=-1,
        )
        body_z = self.torch.stack(
            (2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x*x + y*y)),
            dim=-1,
        )
        return body_x, body_z

    def _dynamics(self, state, action):
        cfg = self.cfg
        p = state[..., 0:3]
        quat = state[..., 3:7]
        quat = quat / self.torch.linalg.vector_norm(quat, dim=-1, keepdim=True).clamp_min(1e-9)
        vel = state[..., 7:10]
        omega = state[..., 10:13]
        thrust = action[..., 0]
        omega_cmd = action[..., 1:4]

        alpha = min(cfg.dt / cfg.rate_tau, 1.0)
        omega_next = omega + alpha * (omega_cmd - omega)
        quat_next = quat + cfg.dt * self._quat_derivative(quat, omega_next)
        quat_next = quat_next / self.torch.linalg.vector_norm(
            quat_next, dim=-1, keepdim=True
        ).clamp_min(1e-9)
        _, body_z = self._body_axes_world(quat_next)
        gravity = self.torch.tensor(
            [0.0, 0.0, -cfg.gravity], dtype=state.dtype, device=state.device
        )
        accel = body_z * (thrust / cfg.mass_kg).unsqueeze(-1) + gravity
        p_next = p + vel * cfg.dt + 0.5 * accel * cfg.dt**2
        vel_next = vel + accel * cfg.dt
        return self.torch.cat((p_next, quat_next, vel_next, omega_next), dim=-1)

    def _obstacle_cost(self, position):
        if self.obstacles is None or self.obstacles.shape[0] == 0:
            return self.torch.zeros(
                position.shape[:-1], dtype=position.dtype, device=position.device
            )
        nearest = self.torch.cdist(position.reshape(-1, 3), self.obstacles).min(dim=1).values
        nearest = nearest.reshape(position.shape[:-1])
        beta = 2.0
        return self.torch.nn.functional.softplus(
            self.cfg.margin - nearest, beta=beta
        ) - math.log(2.0) / beta

    def _running_cost(self, state, action):
        cfg = self.cfg
        p = state[..., 0:3]
        quat = state[..., 3:7]
        vel = state[..., 7:10]
        omega = state[..., 10:13]
        _, body_z = self._body_axes_world(quat)
        goal_distance = self.torch.linalg.vector_norm(p - self.goal, dim=-1)
        hover_error = (action[..., 0] / cfg.hover_thrust_n - 1.0) ** 2
        rate_effort = (action[..., 1:4] ** 2).sum(dim=-1)
        rate_track = ((action[..., 1:4] - omega) ** 2).sum(dim=-1)
        tilt = (1.0 - body_z[..., 2]).clamp_min(0.0)
        near_goal = self.torch.exp(-0.5 * goal_distance.square())
        velocity = near_goal * (vel.square().sum(dim=-1))
        altitude = (p[..., 2] - self.goal[2]).square()
        vertical_speed = vel[..., 2].square()
        cost = (
            cfg.w_goal * goal_distance
            + cfg.w_obstacle * self._obstacle_cost(p)
            + cfg.w_rigid_thrust * hover_error
            + cfg.w_rigid_rate * rate_effort
            + cfg.w_rigid_rate_track * rate_track
            + cfg.w_rigid_tilt * tilt
            + cfg.w_rigid_velocity * velocity
            + cfg.w_rigid_altitude * altitude
            + cfg.w_rigid_vertical_speed * vertical_speed
        )
        if self.map.origin is not None:
            status = self.map.lookup_torch(p)
            cost = cost + cfg.w_pa_collision * (status != 0).to(cost.dtype)
        return cost

    def _terminal_cost(self, states, actions):
        cfg = self.cfg
        terminal = states[..., -1, :]
        p = terminal[..., 0:3]
        quat = terminal[..., 3:7]
        cost = cfg.w_terminal * self.torch.linalg.vector_norm(p - self.goal, dim=-1).square()
        if self.map.origin is None or self.goal_visible:
            return cost
        body_x, _ = self._body_axes_world(quat)
        direction = self.goal - p
        distance = self.torch.linalg.vector_norm(direction, dim=-1).clamp_min(1e-6)
        alignment = (body_x * (direction / distance.unsqueeze(-1))).sum(dim=-1).clamp(-1, 1)
        poi = cfg.w_pa_poi * (1.0 - alignment).square()
        poi = poi * (distance > cfg.pa_goal_threshold).to(poi.dtype)
        status = self.map.first_nonfree_on_rays_torch(p.reshape(-1, 3), self.goal)
        ray_cost = (
            cfg.w_pa_occupied * (status == 1).to(cost.dtype)
            + cfg.w_pa_unknown * (status == -1).to(cost.dtype)
        ).reshape(cost.shape)
        return cost + poi + ray_cost

    def update_goal(self, goal) -> None:
        self.goal = self.torch.as_tensor(
            np.asarray(goal, dtype=np.float64), dtype=self.torch.double,
            device=self.goal.device,
        )

    def update_obstacles(self, points) -> None:
        if points is None or len(points) == 0:
            self.obstacles = None
        else:
            self.obstacles = self.torch.as_tensor(
                np.asarray(points, dtype=np.float64), dtype=self.torch.double,
                device=self.goal.device,
            )

    def update_occupancy(self, sensor_origin, obstacle_endpoints) -> None:
        self.map.update_rays(
            sensor_origin,
            np.empty((0, 3)) if obstacle_endpoints is None else obstacle_endpoints,
        )

    def command_rigid(self, pos, quat_wxyz, vel, omega_body_flu) -> np.ndarray:
        quat = np.asarray(quat_wxyz, dtype=np.float64)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm < 1e-9:
            raise ValueError("quaternion state không hợp lệ")
        quat = quat / norm
        state_np = np.concatenate(
            (np.asarray(pos, dtype=np.float64), quat,
             np.asarray(vel, dtype=np.float64),
             np.asarray(omega_body_flu, dtype=np.float64))
        )
        if not np.isfinite(state_np).all():
            raise ValueError("rigid-body state chứa NaN/Inf")
        self.goal_visible = self.map.line_of_sight(pos, self.goal.detach().cpu().numpy())
        self._last_state_np = state_np
        state = self.torch.as_tensor(
            state_np, dtype=self.torch.double, device=self.goal.device
        )
        started = time.perf_counter()
        control = self.ctrl.command(state)
        self.last_compute_ms = (time.perf_counter() - started) * 1000.0
        return control.detach().cpu().numpy().astype(np.float64)

    def thrust_newtons_to_normalized(self, thrust_n: float) -> float:
        value = self.cfg.hover_thrust_normalized * float(thrust_n) / self.cfg.hover_thrust_n
        return float(np.clip(value, 0.0, 1.0))

    def predict_trajectory(self) -> np.ndarray:
        actions = self.ctrl.get_action_sequence()
        if not hasattr(self, "_last_state_np"):
            return np.zeros((actions.shape[0] + 1, 3))
        state = self.torch.as_tensor(
            self._last_state_np, dtype=self.torch.double, device=self.goal.device
        )
        positions = [state[0:3].detach().cpu().numpy()]
        for action in actions:
            state = self._dynamics(state, action)
            positions.append(state[0:3].detach().cpu().numpy())
        return np.stack(positions)

    def sampled_trajectories(self, top_k: int = 20) -> np.ndarray:
        states = getattr(self.ctrl, "states", None)
        costs = getattr(self.ctrl, "cost_total", None)
        if states is None or costs is None or not hasattr(self, "_last_state_np"):
            return np.zeros((0, 0, 3))
        states = states[0] if states.ndim == 4 else states
        count = min(max(int(top_k), 0), states.shape[0])
        if count == 0:
            return np.zeros((0, states.shape[1] + 1, 3))
        indices = self.torch.argsort(costs)[:count]
        paths = states[indices, :, 0:3].detach().cpu().numpy()
        initial = np.broadcast_to(self._last_state_np[None, None, 0:3], (count, 1, 3))
        return np.concatenate((initial, paths), axis=1)

    def optimizer_diagnostics(self) -> dict:
        out = {
            "planner": "rigid-pa-mppi",
            "model": "rigid-body-with-first-order-rate-tracking",
            "compute_ms": float(getattr(self, "last_compute_ms", 0.0)),
            "goal_visible": bool(self.goal_visible),
            "mapped_fraction": self.map.mapped_fraction,
        }
        omega = getattr(self.ctrl, "omega", None)
        costs = getattr(self.ctrl, "cost_total", None)
        if omega is not None:
            out["ess"] = float((1.0 / omega.square().sum().clamp_min(1e-12)).item())
        if costs is not None:
            out["best_sample_cost"] = float(costs.min().item())
            out["mean_sample_cost"] = float(costs.mean().item())
        out["cost"] = self.nominal_cost_breakdown()
        return out

    def nominal_cost_breakdown(self) -> dict:
        if not hasattr(self, "_last_state_np"):
            return {}
        torch = self.torch
        cfg = self.cfg
        state = torch.as_tensor(
            self._last_state_np, dtype=torch.double, device=self.goal.device
        )
        totals = {
            "goal": 0.0, "obstacle": 0.0, "thrust": 0.0, "body_rate": 0.0,
            "rate_tracking": 0.0, "tilt": 0.0, "velocity": 0.0,
            "altitude": 0.0, "vertical_speed": 0.0, "collision": 0.0,
            "perception": 0.0,
        }
        for action in self.ctrl.get_action_sequence():
            state = self._dynamics(state, action)
            p, quat, vel, omega = state[0:3], state[3:7], state[7:10], state[10:13]
            _, body_z = self._body_axes_world(quat)
            distance = torch.linalg.vector_norm(p - self.goal)
            totals["goal"] += cfg.w_goal * float(distance.item())
            totals["obstacle"] += cfg.w_obstacle * float(self._obstacle_cost(p).item())
            totals["thrust"] += cfg.w_rigid_thrust * float((action[0] / cfg.hover_thrust_n - 1.0).square().item())
            totals["body_rate"] += cfg.w_rigid_rate * float(action[1:4].square().sum().item())
            totals["rate_tracking"] += cfg.w_rigid_rate_track * float((action[1:4] - omega).square().sum().item())
            totals["tilt"] += cfg.w_rigid_tilt * float((1.0 - body_z[2]).clamp_min(0.0).item())
            near_goal = torch.exp(-0.5 * distance.square())
            totals["velocity"] += cfg.w_rigid_velocity * float((near_goal * vel.square().sum()).item())
            totals["altitude"] += cfg.w_rigid_altitude * float((p[2] - self.goal[2]).square().item())
            totals["vertical_speed"] += cfg.w_rigid_vertical_speed * float(vel[2].square().item())
            if self.map.origin is not None:
                totals["collision"] += cfg.w_pa_collision * float(int(self.map.lookup_torch(p).item()) != 0)
        totals["terminal"] = cfg.w_terminal * float(torch.linalg.vector_norm(state[0:3] - self.goal).square().item())
        if self.map.origin is not None and not self.goal_visible:
            p, quat = state[0:3], state[3:7]
            body_x, _ = self._body_axes_world(quat)
            direction = self.goal - p
            distance = torch.linalg.vector_norm(direction).clamp_min(1e-6)
            alignment = torch.dot(body_x, direction / distance).clamp(-1.0, 1.0)
            if float(distance.item()) > cfg.pa_goal_threshold:
                totals["perception"] += cfg.w_pa_poi * float((1.0 - alignment).square().item())
            status = int(self.map.first_nonfree_on_rays_torch(p.reshape(1, 3), self.goal)[0].item())
            totals["perception"] += cfg.w_pa_occupied * float(status == 1)
            totals["perception"] += cfg.w_pa_unknown * float(status == -1)
        totals["total_nominal"] = float(sum(totals.values()))
        return totals

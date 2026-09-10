"""Perception-Aware MPPI v0 cho pipeline velocity-level hiện tại.

Port này giữ interface và dynamics 7-state của ``QuadMPPI`` nhưng bổ sung hai
thành phần cốt lõi của PA-MPPI (Zhai et al., RA-L 2026): occupancy grid ba
trạng thái và perception cost ở endpoint của rollout. Đây chưa phải bản tái
tạo paper-faithful vì paper dùng full rigid-body dynamics, body-rate/thrust và
ROG-Map/JAX.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .mppi_controller import MPPIConfig, QuadMPPI
from .occupancy_grid import OccupancyGrid3D, OccupancyGridConfig


@dataclass
class PAMPPIConfig(MPPIConfig):
    map_resolution: float = 0.5
    map_size_x: float = 80.0
    map_size_y: float = 80.0
    map_size_z: float = 40.0
    map_free_inflation: float = 0.5
    map_occupied_inflation: float = 0.5
    map_sensor_free_radius: float = 4.0
    w_pa_collision: float = 500.0
    w_pa_poi: float = 20.0
    w_pa_occupied: float = 100.0
    w_pa_unknown: float = -50.0
    pa_goal_threshold: float = 0.5


class PerceptionAwareMPPI(QuadMPPI):
    """PA-MPPI port v0 with paper-style two-phase perception objective."""

    def __init__(self, cfg: Optional[PAMPPIConfig] = None) -> None:
        self.pa_cfg = cfg or PAMPPIConfig()
        self.map = OccupancyGrid3D(
            OccupancyGridConfig(
                resolution=self.pa_cfg.map_resolution,
                size_m=(self.pa_cfg.map_size_x, self.pa_cfg.map_size_y, self.pa_cfg.map_size_z),
                free_inflation_m=self.pa_cfg.map_free_inflation,
                occupied_inflation_m=self.pa_cfg.map_occupied_inflation,
                sensor_free_radius_m=self.pa_cfg.map_sensor_free_radius,
            )
        )
        self.goal_visible = False
        super().__init__(self.pa_cfg)

    def update_occupancy(self, sensor_origin, obstacle_endpoints) -> None:
        if obstacle_endpoints is None:
            return
        self.map.update_rays(sensor_origin, obstacle_endpoints)

    def command(self, pos, vel, yaw: float) -> np.ndarray:
        goal = self.goal.detach().cpu().numpy()
        self.goal_visible = self.map.line_of_sight(pos, goal)
        return super().command(pos, vel, yaw)

    def _running_cost(self, state, action):
        cost = super()._running_cost(state, action)
        if self.map.origin is None:
            return cost
        status = self.map.lookup_torch(state[..., 0:3])
        return cost + self.pa_cfg.w_pa_collision * (status != 0).to(cost.dtype)

    def _terminal_cost(self, states, actions):
        cost = super()._terminal_cost(states, actions)
        if self.map.origin is None or self.goal_visible:
            return cost

        torch = self.torch
        terminal = states[..., -1, :]
        p = terminal[..., 0:3]
        yaw = terminal[..., 6]
        direction = self.goal - p
        distance = torch.linalg.vector_norm(direction, dim=-1).clamp_min(1e-6)
        goal_hat = direction / distance.unsqueeze(-1)
        heading = torch.stack((torch.cos(yaw), torch.sin(yaw), torch.zeros_like(yaw)), dim=-1)
        alignment = (heading * goal_hat).sum(dim=-1).clamp(-1.0, 1.0)
        poi = self.pa_cfg.w_pa_poi * (1.0 - alignment) ** 2
        poi = poi * (distance > self.pa_cfg.pa_goal_threshold).to(poi.dtype)

        flat_p = p.reshape(-1, 3)
        status = self.map.first_nonfree_on_rays_torch(flat_p, self.goal)
        ray_cost = (
            self.pa_cfg.w_pa_occupied * (status == 1).to(cost.dtype)
            + self.pa_cfg.w_pa_unknown * (status == -1).to(cost.dtype)
        ).reshape(cost.shape)
        return cost + poi + ray_cost

    def optimizer_diagnostics(self) -> dict:
        out = super().optimizer_diagnostics()
        out.update(
            planner="pa-mppi",
            goal_visible=bool(self.goal_visible),
            mapped_fraction=self.map.mapped_fraction,
        )
        return out

    def nominal_cost_breakdown(self) -> dict:
        totals = super().nominal_cost_breakdown()
        if not totals or self.map.origin is None:
            totals.setdefault("collision", 0.0)
            totals.setdefault("perception", 0.0)
            return totals
        torch = self.torch
        state = torch.as_tensor(
            self._last_state_np, dtype=torch.double, device=self.goal.device
        )
        collision = 0.0
        for action in self.ctrl.get_action_sequence():
            state = self._dynamics(state, action)
            status = int(self.map.lookup_torch(state[0:3]).item())
            collision += self.pa_cfg.w_pa_collision * float(status != 0)
        perception = 0.0
        if not self.goal_visible:
            p, yaw = state[0:3], state[6]
            direction = self.goal - p
            distance = torch.linalg.vector_norm(direction).clamp_min(1e-6)
            heading = torch.stack((torch.cos(yaw), torch.sin(yaw), torch.zeros_like(yaw)))
            alignment = torch.dot(heading, direction / distance).clamp(-1.0, 1.0)
            if float(distance.item()) > self.pa_cfg.pa_goal_threshold:
                perception += self.pa_cfg.w_pa_poi * float((1.0 - alignment).square().item())
            status = int(self.map.first_nonfree_on_rays_torch(p.reshape(1, 3), self.goal)[0].item())
            perception += self.pa_cfg.w_pa_occupied * float(status == 1)
            perception += self.pa_cfg.w_pa_unknown * float(status == -1)
        totals["collision"] = float(collision)
        totals["perception"] = float(perception)
        totals["total_nominal"] = float(totals.get("total_nominal", 0.0) + collision + perception)
        return totals

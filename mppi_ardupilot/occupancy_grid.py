"""Bản đồ voxel ba trạng thái cho PA-MPPI.

Quy ước theo paper PA-MPPI: ``-1`` unknown, ``0`` free, ``1`` occupied.
LiDAR ray đánh dấu các voxel trước endpoint là free và endpoint là occupied.
Đây là backend NumPy/Torch gọn để bắt đầu port; có thể thay bằng ROG-Map mà
không đổi interface của planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(1)


@dataclass
class OccupancyGridConfig:
    resolution: float = 0.5
    size_m: Tuple[float, float, float] = (80.0, 80.0, 40.0)
    free_inflation_m: float = 0.5
    occupied_inflation_m: float = 0.5
    sensor_free_radius_m: float = 4.0


class OccupancyGrid3D:
    """Fixed-size world-frame grid centred at the first vehicle position."""

    def __init__(self, cfg: Optional[OccupancyGridConfig] = None) -> None:
        self.cfg = cfg or OccupancyGridConfig()
        if self.cfg.resolution <= 0:
            raise ValueError("resolution phải lớn hơn 0")
        self.shape = tuple(
            int(np.ceil(v / self.cfg.resolution)) for v in self.cfg.size_m
        )
        self.origin: Optional[np.ndarray] = None
        self.grid = np.full(self.shape, UNKNOWN, dtype=np.int8)
        self._torch_grid = None
        self._torch_device = None
        self.revision = 0

    def reset(self, center: Sequence[float]) -> None:
        center = np.asarray(center, dtype=np.float64)
        self.origin = center - 0.5 * np.asarray(self.cfg.size_m)
        self.grid.fill(UNKNOWN)
        self._invalidate()

    def _invalidate(self) -> None:
        self.revision += 1
        self._torch_grid = None
        self._torch_device = None

    def world_to_grid(self, points) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        if self.origin is None:
            raise RuntimeError("occupancy grid chưa có origin")
        return np.floor((points - self.origin) / self.cfg.resolution).astype(np.int64)

    def _inside(self, indices: np.ndarray) -> np.ndarray:
        return np.logical_and(indices >= 0, indices < np.asarray(self.shape)).all(axis=-1)

    def lookup(self, points) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        flat = points.reshape(-1, 3)
        if self.origin is None:
            return np.full(points.shape[:-1], UNKNOWN, dtype=np.int8)
        idx = self.world_to_grid(flat)
        inside = self._inside(idx)
        out = np.full(len(flat), UNKNOWN, dtype=np.int8)
        good = idx[inside]
        out[inside] = self.grid[good[:, 0], good[:, 1], good[:, 2]]
        return out.reshape(points.shape[:-1])

    def _paint(self, indices: np.ndarray, value: np.int8, radius_m: float) -> None:
        radius = max(0, int(np.ceil(radius_m / self.cfg.resolution)))
        for idx in np.asarray(indices, dtype=np.int64).reshape(-1, 3):
            lo = np.maximum(idx - radius, 0)
            hi = np.minimum(idx + radius + 1, np.asarray(self.shape))
            if np.any(lo >= hi):
                continue
            self.grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] = value

    def update_rays(self, sensor_origin, endpoints) -> None:
        """Fuse one downsampled LiDAR scan into the 3-state grid."""
        sensor_origin = np.asarray(sensor_origin, dtype=np.float64)
        endpoints = np.asarray(endpoints, dtype=np.float64).reshape(-1, 3)
        if self.origin is None:
            self.reset(sensor_origin)

        free_indices = []
        occupied_indices = []
        # The Gazebo point cloud contains hit endpoints but does not preserve
        # explicit max-range misses after downsampling. A 360-degree 3D scan
        # still certifies the near field as observed. Mark that local volume
        # free first; occupied endpoints are painted last and take precedence.
        sensor_idx = self.world_to_grid(sensor_origin.reshape(1, 3))
        self._paint(sensor_idx, FREE, self.cfg.sensor_free_radius_m)
        if len(endpoints) == 0:
            self._invalidate()
            return
        step_m = 0.5 * self.cfg.resolution
        for endpoint in endpoints:
            delta = endpoint - sensor_origin
            distance = float(np.linalg.norm(delta))
            if not np.isfinite(distance) or distance < self.cfg.resolution:
                continue
            count = max(2, int(np.ceil(distance / step_m)) + 1)
            samples = sensor_origin + np.linspace(0.0, 1.0, count)[:, None] * delta
            ray_idx = self.world_to_grid(samples)
            inside = self._inside(ray_idx)
            ray_idx = ray_idx[inside]
            if len(ray_idx) < 1:
                continue
            free_indices.append(ray_idx[:-1])
            occupied_indices.append(ray_idx[-1:])

        if free_indices:
            self._paint(np.concatenate(free_indices), FREE, self.cfg.free_inflation_m)
        if occupied_indices:
            # Paint occupied last so obstacle endpoints cannot be erased by a
            # neighbouring free ray from the same scan.
            self._paint(
                np.concatenate(occupied_indices), OCCUPIED,
                self.cfg.occupied_inflation_m,
            )
        self._invalidate()

    def first_nonfree_on_ray(self, start, end) -> int:
        """Return FREE, UNKNOWN or OCCUPIED at the first non-free ray voxel."""
        start = np.asarray(start, dtype=np.float64)
        end = np.asarray(end, dtype=np.float64)
        distance = float(np.linalg.norm(end - start))
        count = max(2, int(np.ceil(distance / self.cfg.resolution)) + 1)
        points = start + np.linspace(0.0, 1.0, count)[:, None] * (end - start)
        values = self.lookup(points[1:])
        nonfree = np.flatnonzero(values != FREE)
        return int(values[nonfree[0]]) if len(nonfree) else int(FREE)

    def line_of_sight(self, start, end) -> bool:
        return self.first_nonfree_on_ray(start, end) == int(FREE)

    @property
    def mapped_fraction(self) -> float:
        return float(np.count_nonzero(self.grid != UNKNOWN) / self.grid.size)

    def as_torch(self, device):
        import torch

        if self._torch_grid is None or self._torch_device != str(device):
            self._torch_grid = torch.as_tensor(self.grid, dtype=torch.int8, device=device)
            self._torch_device = str(device)
        return self._torch_grid

    def lookup_torch(self, points):
        """Vectorized voxel lookup preserving the leading tensor dimensions."""
        import torch

        shape = points.shape[:-1]
        if self.origin is None:
            return torch.full(shape, -1, dtype=torch.int8, device=points.device)
        origin = torch.as_tensor(self.origin, dtype=points.dtype, device=points.device)
        idx = torch.floor((points - origin) / self.cfg.resolution).long()
        bounds = torch.as_tensor(self.shape, dtype=torch.long, device=points.device)
        inside = ((idx >= 0) & (idx < bounds)).all(dim=-1)
        safe = torch.maximum(torch.minimum(idx, bounds - 1), torch.zeros_like(idx))
        grid = self.as_torch(points.device)
        values = grid[safe[..., 0], safe[..., 1], safe[..., 2]]
        return torch.where(inside, values, torch.full_like(values, -1))

    def first_nonfree_on_rays_torch(self, starts, end, max_steps: int = 192):
        """First non-free status for a batch of rays from ``starts`` to ``end``."""
        import torch

        starts = starts.reshape(-1, 3)
        end = end.reshape(1, 3)
        distances = torch.linalg.vector_norm(end - starts, dim=-1)
        steps = torch.clamp(
            torch.ceil(distances / self.cfg.resolution).long(), min=1, max=max_steps
        )
        t = torch.arange(1, max_steps + 1, device=starts.device).reshape(1, -1)
        alpha = torch.minimum(t / steps.reshape(-1, 1), torch.ones_like(t, dtype=starts.dtype))
        points = starts[:, None, :] + alpha[..., None] * (end - starts)[:, None, :]
        values = self.lookup_torch(points)
        valid = t <= steps.reshape(-1, 1)
        nonfree = (values != 0) & valid
        has_nonfree = nonfree.any(dim=1)
        first_idx = nonfree.to(torch.int64).argmax(dim=1)
        first = values.gather(1, first_idx[:, None]).squeeze(1)
        return torch.where(has_nonfree, first, torch.zeros_like(first))

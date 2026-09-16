"""Static-map A* global planner for the Gazebo MPPI experiments.

The planner deliberately has a narrow contract: it reads primitive static
collision geometry from an SDF world and searches in the horizontal plane at
the requested ENU altitude.  It is therefore a 2.5-D *known-map* planner, not
an online mapper and not a replacement for PA-MPPI's unknown/free/occupied
representation.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Iterable, Optional
import xml.etree.ElementTree as ET

import numpy as np


def _pose(element: Optional[ET.Element]) -> np.ndarray:
    if element is None or not element.text:
        return np.zeros(6, dtype=np.float64)
    values = [float(value) for value in element.text.split()]
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError("SDF pose phải gồm 6 số hữu hạn: x y z roll pitch yaw")
    return np.asarray(values, dtype=np.float64)


def _compose_planar_pose(parent: np.ndarray, child: np.ndarray) -> np.ndarray:
    """Compose SDF poses for the level/yaw-only worlds used by this project."""
    c, s = math.cos(parent[5]), math.sin(parent[5])
    xy = parent[:2] + np.array([
        c * child[0] - s * child[1],
        s * child[0] + c * child[1],
    ])
    result = parent + child
    result[:2] = xy
    result[5] = parent[5] + child[5]
    return result


@dataclass(frozen=True)
class StaticObstacle2D:
    """Horizontal footprint and vertical interval of one SDF collision."""

    name: str
    kind: str
    center_xy: tuple[float, float]
    z_min: float
    z_max: float
    yaw: float = 0.0
    half_size_xy: tuple[float, float] = (0.0, 0.0)
    radius: float = 0.0

    def active_at(self, altitude_m: float, vertical_clearance_m: float = 0.0) -> bool:
        return (
            self.z_min - vertical_clearance_m
            <= altitude_m
            <= self.z_max + vertical_clearance_m
        )

    def contains_xy(self, point_xy: np.ndarray, clearance_m: float) -> bool:
        delta = np.asarray(point_xy, dtype=np.float64) - np.asarray(self.center_xy)
        if self.kind == "cylinder":
            return float(np.dot(delta, delta)) <= (self.radius + clearance_m) ** 2
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
        half = np.asarray(self.half_size_xy) + clearance_m
        return bool(np.all(np.abs(local) <= half))

    def extent_xy(self, clearance_m: float) -> np.ndarray:
        if self.kind == "cylinder":
            return np.full(2, self.radius + clearance_m)
        half = np.asarray(self.half_size_xy) + clearance_m
        c, s = abs(math.cos(self.yaw)), abs(math.sin(self.yaw))
        return np.array([c * half[0] + s * half[1], s * half[0] + c * half[1]])


def load_sdf_obstacles(path: str | Path) -> list[StaticObstacle2D]:
    """Load box/cylinder collisions directly declared in an SDF world.

    Unsupported scenery includes and mesh collisions raise an error rather
    than silently treating unparsed geometry as free space.
    """
    sdf_path = Path(path).expanduser().resolve()
    if not sdf_path.is_file():
        raise ValueError(f"không tìm thấy SDF world: {sdf_path}")
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"SDF không có phần tử <world>: {sdf_path}")
    unsupported_includes = []
    for include in world.findall("include"):
        uri = (include.findtext("uri") or "").strip()
        # The vehicle is dynamic and must not be inserted into the static map.
        if "iris_with_" not in uri:
            unsupported_includes.append(uri or "<missing uri>")
    if unsupported_includes:
        raise ValueError(
            "A* chưa resolve collision của scenery <include>: "
            + ", ".join(unsupported_includes)
        )
    obstacles: list[StaticObstacle2D] = []
    for model in world.findall("model"):
        if (model.findtext("static") or "").strip().lower() != "true":
            continue
        model_pose = _pose(model.find("pose"))
        for link_index, link in enumerate(model.findall("link")):
            link_pose = _compose_planar_pose(model_pose, _pose(link.find("pose")))
            for collision_index, collision in enumerate(link.findall("collision")):
                pose = _compose_planar_pose(link_pose, _pose(collision.find("pose")))
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                name = (
                    f"{model.get('name', 'model')}/"
                    f"{link.get('name', link_index)}/"
                    f"{collision.get('name', collision_index)}"
                )
                if abs(float(pose[3])) > 1e-9 or abs(float(pose[4])) > 1e-9:
                    raise ValueError(f"A* 2.5D không hỗ trợ collision roll/pitch tại {name}")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None:
                    size_text = box.findtext("size")
                    if not size_text:
                        continue
                    size = np.asarray([float(v) for v in size_text.split()], dtype=np.float64)
                    if size.shape != (3,) or np.any(size <= 0) or not np.isfinite(size).all():
                        raise ValueError(f"box size không hợp lệ tại {name}")
                    obstacles.append(StaticObstacle2D(
                        name=name,
                        kind="box",
                        center_xy=(float(pose[0]), float(pose[1])),
                        z_min=float(pose[2] - 0.5 * size[2]),
                        z_max=float(pose[2] + 0.5 * size[2]),
                        yaw=float(pose[5]),
                        half_size_xy=(float(0.5 * size[0]), float(0.5 * size[1])),
                    ))
                elif cylinder is not None:
                    radius = float(cylinder.findtext("radius", "nan"))
                    length = float(cylinder.findtext("length", "nan"))
                    if radius <= 0 or length <= 0 or not np.isfinite([radius, length]).all():
                        raise ValueError(f"cylinder không hợp lệ tại {name}")
                    obstacles.append(StaticObstacle2D(
                        name=name,
                        kind="cylinder",
                        center_xy=(float(pose[0]), float(pose[1])),
                        z_min=float(pose[2] - 0.5 * length),
                        z_max=float(pose[2] + 0.5 * length),
                        radius=radius,
                    ))
                else:
                    raise ValueError(
                        f"A* chưa hỗ trợ geometry không phải box/cylinder tại {name}"
                    )
    return obstacles


@dataclass(frozen=True)
class AStarConfig:
    resolution_m: float = 0.5
    clearance_m: float = 1.8
    bounds_padding_m: float = 4.0
    vertical_clearance_m: float = 0.3
    allow_diagonal: bool = True
    max_expansions: int = 250_000


@dataclass(frozen=True)
class AStarResult:
    path_enu: np.ndarray
    raw_grid_points: int
    expanded_nodes: int
    path_length_m: float
    active_obstacles: int


class AStarGlobalPlanner:
    """A* over an XY lattice with continuous primitive collision checks."""

    def __init__(self, obstacles: Iterable[StaticObstacle2D], cfg: Optional[AStarConfig] = None):
        self.obstacles = list(obstacles)
        self.cfg = cfg or AStarConfig()
        if self.cfg.resolution_m <= 0 or self.cfg.clearance_m < 0:
            raise ValueError("A* resolution phải > 0 và clearance phải >= 0")
        if self.cfg.bounds_padding_m <= 0 or self.cfg.max_expansions <= 0:
            raise ValueError("A* padding/max_expansions phải > 0")

    @classmethod
    def from_sdf(cls, path: str | Path, cfg: Optional[AStarConfig] = None):
        return cls(load_sdf_obstacles(path), cfg)

    def _active(self, altitude_m: float) -> list[StaticObstacle2D]:
        return [
            obstacle for obstacle in self.obstacles
            if obstacle.active_at(altitude_m, self.cfg.vertical_clearance_m)
        ]

    def obstacle_points_enu(self, altitude_m: float, spacing_m: float = 0.35) -> np.ndarray:
        """Sample active primitive boundaries for the offline/local collision adapter."""
        if spacing_m <= 0:
            raise ValueError("obstacle sample spacing phải > 0")
        clouds: list[np.ndarray] = []
        for obstacle in self._active(altitude_m):
            if obstacle.kind == "cylinder":
                count = max(12, int(math.ceil(2.0 * math.pi * obstacle.radius / spacing_m)))
                theta = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
                xy = np.column_stack((np.cos(theta), np.sin(theta))) * obstacle.radius
                xy += np.asarray(obstacle.center_xy)
            else:
                hx, hy = obstacle.half_size_xy
                local_segments = (
                    ([-hx, -hy], [hx, -hy]), ([hx, -hy], [hx, hy]),
                    ([hx, hy], [-hx, hy]), ([-hx, hy], [-hx, -hy]),
                )
                pieces = []
                for a, b in local_segments:
                    a_arr, b_arr = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
                    count = max(2, int(math.ceil(np.linalg.norm(b_arr - a_arr) / spacing_m)) + 1)
                    pieces.append(np.linspace(a_arr, b_arr, count, endpoint=False))
                local = np.vstack(pieces)
                c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
                rotation = np.array([[c, -s], [s, c]])
                xy = local @ rotation.T + np.asarray(obstacle.center_xy)
            clouds.append(np.column_stack((xy, np.full(len(xy), altitude_m))))
        return np.vstack(clouds) if clouds else np.empty((0, 3), dtype=np.float64)

    def _blocked(self, point_xy: np.ndarray, active: list[StaticObstacle2D]) -> bool:
        return any(
            obstacle.contains_xy(point_xy, self.cfg.clearance_m)
            for obstacle in active
        )

    def _line_free(self, a: np.ndarray, b: np.ndarray, active: list[StaticObstacle2D]) -> bool:
        # Continuous segment tests: sampling can miss thin walls between cells.
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        for obstacle in active:
            center = np.asarray(obstacle.center_xy)
            delta = b - a
            if obstacle.kind == 'cylinder':
                denominator = float(delta @ delta)
                t = 0.0 if denominator == 0 else np.clip(float((center - a) @ delta) / denominator, 0, 1)
                if np.linalg.norm(a + t * delta - center) <= obstacle.radius + self.cfg.clearance_m:
                    return False
            else:
                c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
                rotation = np.array([[c, s], [-s, c]])
                origin, direction = rotation @ (a - center), rotation @ delta
                half = np.asarray(obstacle.half_size_xy) + self.cfg.clearance_m
                lo, hi = 0.0, 1.0
                for axis in range(2):
                    if abs(direction[axis]) < 1e-12:
                        if abs(origin[axis]) > half[axis]:
                            lo, hi = 1.0, 0.0
                            break
                    else:
                        t0 = (-half[axis] - origin[axis]) / direction[axis]
                        t1 = (half[axis] - origin[axis]) / direction[axis]
                        lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
                if lo <= hi:
                    return False
        return True

    def _simplify(self, points: np.ndarray, active: list[StaticObstacle2D]) -> np.ndarray:
        if len(points) <= 2:
            return points
        simplified = [points[0]]
        anchor = 0
        while anchor < len(points) - 1:
            candidate = len(points) - 1
            while candidate > anchor + 1:
                if self._line_free(points[anchor], points[candidate], active):
                    break
                candidate -= 1
            simplified.append(points[candidate])
            anchor = candidate
        return np.asarray(simplified, dtype=np.float64)

    def plan(self, start_enu, goal_enu) -> AStarResult:
        start = np.asarray(start_enu, dtype=np.float64)
        goal = np.asarray(goal_enu, dtype=np.float64)
        if start.shape != (3,) or goal.shape != (3,) or not np.isfinite([*start, *goal]).all():
            raise ValueError("A* start/goal phải là finite 3-vector ENU")
        if abs(float(start[2] - goal[2])) > 0.25:
            raise ValueError("A* 2.5D yêu cầu start và goal cùng cao độ (sai khác <= 0.25 m)")
        altitude = float(goal[2])
        active = self._active(altitude)
        if self._blocked(start[:2], active):
            raise ValueError("A* start nằm trong obstacle đã inflate")
        if self._blocked(goal[:2], active):
            raise ValueError("A* goal nằm trong obstacle đã inflate")

        lower = np.minimum(start[:2], goal[:2]) - self.cfg.bounds_padding_m
        upper = np.maximum(start[:2], goal[:2]) + self.cfg.bounds_padding_m
        for obstacle in active:
            extent = obstacle.extent_xy(self.cfg.clearance_m)
            center = np.asarray(obstacle.center_xy)
            lower = np.minimum(lower, center - extent - self.cfg.bounds_padding_m)
            upper = np.maximum(upper, center + extent + self.cfg.bounds_padding_m)
        resolution = self.cfg.resolution_m
        shape = np.ceil((upper - lower) / resolution).astype(int) + 1
        if np.prod(shape, dtype=np.int64) > self.cfg.max_expansions * 20:
            raise ValueError(f"A* search bounds quá lớn: grid {tuple(shape)}")

        def to_index(point: np.ndarray) -> tuple[int, int]:
            idx = np.rint((point - lower) / resolution).astype(int)
            idx = np.clip(idx, 0, shape - 1)
            return int(idx[0]), int(idx[1])

        def to_point(index: tuple[int, int]) -> np.ndarray:
            return lower + resolution * np.asarray(index, dtype=np.float64)

        start_idx, goal_idx = to_index(start[:2]), to_index(goal[:2])
        if self._blocked(to_point(start_idx), active) or self._blocked(to_point(goal_idx), active):
            raise ValueError("A* lattice cell của start/goal bị obstacle chiếm; giảm resolution")
        cardinal = ((1, 0), (-1, 0), (0, 1), (0, -1))
        diagonal = ((1, 1), (1, -1), (-1, 1), (-1, -1)) if self.cfg.allow_diagonal else ()
        moves = cardinal + diagonal
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_idx)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score = {start_idx: 0.0}
        closed: set[tuple[int, int]] = set()
        expanded = 0
        while queue:
            _, current_g, current = heapq.heappop(queue)
            if current in closed:
                continue
            closed.add(current)
            expanded += 1
            if expanded > self.cfg.max_expansions:
                raise RuntimeError(f"A* vượt max_expansions={self.cfg.max_expansions}")
            if current == goal_idx:
                break
            for dx, dy in moves:
                neighbour = current[0] + dx, current[1] + dy
                if not (0 <= neighbour[0] < shape[0] and 0 <= neighbour[1] < shape[1]):
                    continue
                if self._blocked(to_point(neighbour), active):
                    continue
                if not self._line_free(to_point(current), to_point(neighbour), active):
                    continue
                if dx and dy:
                    # No diagonal corner cutting between two occupied cells.
                    side_x, side_y = (current[0] + dx, current[1]), (current[0], current[1] + dy)
                    if self._blocked(to_point(side_x), active) or self._blocked(to_point(side_y), active):
                        continue
                tentative = current_g + resolution * math.hypot(dx, dy)
                if tentative >= g_score.get(neighbour, float("inf")):
                    continue
                came_from[neighbour] = current
                g_score[neighbour] = tentative
                heuristic = float(np.linalg.norm(to_point(neighbour) - to_point(goal_idx)))
                heapq.heappush(queue, (tentative + heuristic, tentative, neighbour))
        else:
            raise RuntimeError("A* không tìm thấy đường trong search bounds")
        if goal_idx not in closed:
            raise RuntimeError("A* không tìm thấy đường tới goal")

        indices = [goal_idx]
        while indices[-1] != start_idx:
            indices.append(came_from[indices[-1]])
        indices.reverse()
        raw_xy = np.asarray([to_point(index) for index in indices])
        # Keep endpoint-to-lattice connectors; replacing endpoints can cut a wall.
        raw_xy = np.vstack((start[:2], raw_xy, goal[:2]))
        if not all(self._line_free(a, b, active) for a, b in zip(raw_xy[:-1], raw_xy[1:])):
            raise ValueError('A* endpoint connector crosses inflated obstacle; reduce resolution')
        simplified_xy = self._simplify(raw_xy, active)
        path = np.column_stack((simplified_xy, np.full(len(simplified_xy), altitude)))
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        return AStarResult(path, len(raw_xy), expanded, length, len(active))

#!/usr/bin/env python3
"""Generate committed C++ Milestone-2 fixtures from the Python baseline."""
import json
import math
from pathlib import Path

import numpy as np
import torch

from mppi_ardupilot.mppi_local_planner_node import VelocityCommandConditioner
from mppi_ardupilot.trajectory_safety import (
    evaluate_trajectory_safety_batch, single_safety_diagnostics)


class SphereGeometry:
    def __init__(self, spheres):
        self.spheres = np.asarray(spheres, dtype=float).reshape(-1, 4)

    def clearance(self, points):
        p = np.asarray(points, dtype=float)
        values = [np.maximum(np.linalg.norm(p-s[:3], axis=-1)-s[3], 0.0)
                  for s in self.spheres]
        return np.min(values, axis=0)

    def torch_clearance(self, points):
        values = [torch.clamp(torch.linalg.vector_norm(
            points-torch.as_tensor(s[:3], dtype=points.dtype, device=points.device), dim=-1)-s[3], min=0)
            for s in self.spheres]
        return torch.stack(values).amin(0)

    def torch_segments_safe(self, starts, ends, radius):
        ab = ends-starts
        denom = (ab*ab).sum(-1).clamp_min(1e-12)
        safe = torch.ones(starts.shape[:-1], dtype=torch.bool, device=starts.device)
        for sphere in self.spheres:
            center = torch.as_tensor(sphere[:3], dtype=starts.dtype, device=starts.device)
            t = (((center-starts)*ab).sum(-1)/denom).clamp(0, 1)
            nearest = starts+t.unsqueeze(-1)*ab
            distance = torch.linalg.vector_norm(center-nearest, dim=-1)-sphere[3]
            safe &= distance > radius
        return safe


CONFIG = dict(collision_radius=1.0, acceleration=3.0, delay=0.25,
              stopping_clearance=1.0, uncertainty=0.2,
              sample_spacing=0.1, cloud_map_tolerance=0.1)


def write_safety(output, name, states, cloud, spheres, expected_reason=None):
    geometry = SphereGeometry(spheres) if spheres else None
    tensor = torch.as_tensor([states], dtype=torch.double)
    result = evaluate_trajectory_safety_batch(
        tensor, cloud, geometry, torch_module=torch, detailed=True, **CONFIG)
    diagnostic = single_safety_diagnostics(result)
    reason = diagnostic["reason"]
    if expected_reason is not None and reason != expected_reason:
        raise RuntimeError(f"{name}: expected {expected_reason}, got {reason}")
    combined_collision = min(float(result["cloud_min"][0]), float(result["map_min"][0]))
    combined_stop = float(result["stop_min"][0])
    payload = {
        "state_width": 6, "states_flat": np.asarray(states).reshape(-1).tolist(),
        "cloud_flat": np.asarray(cloud, dtype=float).reshape(-1).tolist(),
        "spheres_flat": np.asarray(spheres, dtype=float).reshape(-1).tolist(),
        **{k: float(v) for k, v in CONFIG.items()},
        "expected_safe": bool(diagnostic["valid"]), "expected_reason": reason,
        "expected_collision_finite": math.isfinite(combined_collision),
        "expected_collision_clearance": combined_collision if math.isfinite(combined_collision) else 0.0,
        "expected_stopping_finite": math.isfinite(combined_stop),
        "expected_stopping_clearance": combined_stop if math.isfinite(combined_stop) else 0.0,
    }
    (output/f"{name}.json").write_text(json.dumps(payload, indent=2)+"\n")


def main():
    output = Path(__file__).resolve().parents[1]/"uav_navigation_core"/"test"/"fixtures"
    output.mkdir(parents=True, exist_ok=True)
    far_cloud = [[0, 12, 5]]
    far_sphere = [[30, 30, 5, 1]]
    write_safety(output, "safety_clear", [[0,0,5,1,0,0],[2,0,5,1,0,0]], far_cloud, far_sphere, "clear_trajectory")
    write_safety(output, "safety_collision", [[0,0,5,1,0,0],[2,0,5,1,0,0]], [[1,0,5]], far_sphere, "swept_collision")
    write_safety(output, "safety_stopping", [[0,0,5,8,0,0],[.2,0,5,8,0,0]], [[7,0,5]], far_sphere, "predicted_stopping_clearance")
    write_safety(output, "safety_static_collision", [[0,0,5,1,0,0],[4,0,5,1,0,0]], far_cloud, [[2,0,5,.2]], "known_map_collision")
    write_safety(output, "safety_overlap", [[0,0,5,0,0,0],[1,0,5,0,0,0]], [[10,0,5]], [[10,0,5,1]], "clear_trajectory")
    write_safety(output, "safety_empty_cloud", [[0,0,5,0,0,0],[1,0,5,0,0,0]], [], far_sphere, "swept_collision")

    config = dict(dt=0.1, alpha=0.45, max_accel_xy=1.5,
                  max_accel_z=0.8, max_yaw_accel=1.2,
                  u_min=[-3,-3,-1,-1], u_max=[3,3,1,1])
    measured = [0.4, -0.2, 0.1]
    requests = [[3,2,1,1], [3,2,1,1], [-3,-3,-1,-1], [0,0,0,0]]
    conditioner = VelocityCommandConditioner(**config)
    expected = [conditioner.apply(command, measured).tolist() for command in requests]
    (output/"conditioner_sequence.json").write_text(json.dumps({
        **config, "measured_velocity": measured,
        "requests_flat": np.asarray(requests).reshape(-1).tolist(),
        "expected_flat": np.asarray(expected).reshape(-1).tolist()}, indent=2)+"\n")


if __name__ == "__main__":
    main()

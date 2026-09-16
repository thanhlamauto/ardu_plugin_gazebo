#!/usr/bin/env python3
"""Audit one opt-in MPPI pool snapshot; no optimizer tuning or flight control."""
import argparse
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')
os.environ.setdefault('OMP_NUM_THREADS', '1')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch
import yaml

from mppi_ardupilot.braking import validate_stopping_states
from mppi_ardupilot.known_geometry import KnownGeometry
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI
from mppi_ardupilot.trajectory_validation import validate_trajectory
from mppi_ardupilot.trajectory_safety import evaluate_trajectory_safety_batch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    raw = yaml.safe_load(args.config.read_text()) or {}
    cfg = MPPIConfig()
    for name, value in raw.items():
        key = 'lambda_' if name == 'lambda' else name
        if hasattr(cfg, key):
            setattr(cfg, key, value)
    q = QuadMPPI(cfg)
    with np.load(args.snapshot, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    states = arrays['predicted_states']
    applied = arrays['conditioned_sample_actions']
    if states.ndim == 4:
        if states.shape[0] != 1:
            raise ValueError('expected one initial-state batch')
        states, applied = states[0], applied[0]
    actions = arrays['raw_actions']
    if actions.ndim == 4:
        actions = actions[0]
    n, horizon, nx = states.shape
    if actions.shape != (n, horizon, 4) or applied.shape != actions.shape:
        raise ValueError('sample actions/states have incompatible shapes')
    x0 = arrays['initial_state']
    cloud = arrays['obstacle_cloud']
    path = arrays['reference_path_enu']
    q._last_state_np = x0
    q.update_reference_path(path)
    q.update_goal(arrays['goal_enu'])
    q.update_obstacles(cloud)
    geometry = None
    if cfg.known_obstacle_sdf:
        source = Path(cfg.known_obstacle_sdf)
        geometry = KnownGeometry(source if source.is_absolute() else ROOT/source)
        q.known_geometry = geometry

    st = torch.as_tensor(states, dtype=torch.double)
    act = torch.as_tensor(actions, dtype=torch.double)
    app = torch.as_tensor(applied, dtype=torch.double)
    components = {}
    positions, velocities = st[..., :3], st[..., 3:6]
    components['path'] = cfg.w_path * q._path_distance(positions).square().sum(-1) / cfg.path_scale_m**2
    components['collision'] = cfg.w_collision * q._collision_indicator(positions).sum(-1)
    stop = torch.zeros(n, dtype=torch.double)
    if cfg.w_stopping:
        for k in range(0, horizon, 5):
            stop += cfg.w_stopping * q._stopping_cost(positions[:, k], velocities[:, k])
    components['stop_soft'] = stop
    components['effort'] = (app.square()*q._paper_r_u).sum((-1, -2))
    components['smoothness'] = cfg.w_du * (act[..., :3]-velocities).square().sum((-1, -2))
    desired = torch.atan2(act[..., 1], act[..., 0])
    yaw_error = q.wrap_angle(desired-st[..., 6])
    components['yaw'] = (cfg.w_yaw * torch.linalg.vector_norm(act[..., :2], dim=-1)
                         * yaw_error.square()).sum(-1)
    components['speed_limit'] = (cfg.w_speed_limit *
        (torch.linalg.vector_norm(velocities[..., :2], dim=-1)-cfg.vmax).clamp_min(0).square()).sum(-1)
    components['terminal'] = cfg.w_terminal * (positions[:, -1]-q.goal).square().sum(-1)
    start = torch.as_tensor(x0[:3], dtype=torch.double)
    components['progress'] = -cfg.w_progress * (q._geometric_progress(positions[:, -1])
                                               -q._geometric_progress(start))
    previous = torch.as_tensor(x0[7:11], dtype=torch.double).expand(n, 1, 4)
    deltas = torch.diff(torch.cat((previous, app), dim=1), dim=1)
    components['input_change'] = (deltas.square()*q._paper_r_delta_u).sum((-1, -2))
    values = {name: value.detach().cpu().numpy() for name, value in components.items()}
    objective = np.sum(np.array(list(values.values())), axis=0)
    optimizer_cost = arrays['total_sample_cost'].reshape(-1)
    weights = arrays['normalized_weights'].reshape(-1)
    if len(optimizer_cost) != n or len(weights) != n:
        raise ValueError('cost and weight array lengths do not match sample count')

    def feasibility(full_states):
        trajectory = full_states[:, :3]
        cloud_check = validate_trajectory(trajectory, cloud, cfg.collision_radius_m)
        map_check = (None if geometry is None else
                     geometry.validate(trajectory, cfg.collision_radius_m))
        stop_check = validate_stopping_states(
            full_states, cloud, geometry, cfg.max_accel_xy,
            raw.get('hard_brake_delay_s', .25), raw.get('hard_brake_m', cfg.collision_radius_m),
            cfg.stopping_guard_uncertainty_m)
        return cloud_check, map_check, stop_check

    all_states = np.concatenate((np.broadcast_to(x0, (n, 1, len(x0))), states), axis=1)
    shared = evaluate_trajectory_safety_batch(
        all_states, cloud, geometry, collision_radius=cfg.collision_radius_m,
        acceleration=cfg.max_accel_xy, delay=raw.get('hard_brake_delay_s', .25),
        stopping_clearance=raw.get('hard_brake_m', cfg.collision_radius_m),
        uncertainty=cfg.stopping_guard_uncertainty_m, torch_module=torch)
    cloud_ok = shared['cloud_safe'].cpu().numpy()
    map_ok = shared['map_safe'].cpu().numpy()
    stop_ok = shared['stopping_safe'].cpu().numpy()
    min_stop_clearance = np.empty(n)
    for index in range(n):
        full_states = np.vstack((x0, states[index]))
        _, _, checked = feasibility(full_states)
        min_stop_clearance[index] = (float('nan') if checked['min_clearance_m'] is None
                                     else checked['min_clearance_m'])
    safe = cloud_ok & map_ok & stop_ok
    q.ctrl.U.copy_(torch.as_tensor(arrays['nominal_actions'], dtype=torch.double))
    nominal_states = q.predict_states(first_applied=arrays['conditioned_nominal'])
    nominal_shared = evaluate_trajectory_safety_batch(
        nominal_states, cloud, geometry, collision_radius=cfg.collision_radius_m,
        acceleration=cfg.max_accel_xy, delay=raw.get('hard_brake_delay_s', .25),
        stopping_clearance=raw.get('hard_brake_m', cfg.collision_radius_m),
        uncertainty=cfg.stopping_guard_uncertainty_m, torch_module=torch,
        detailed=True)
    best = int(np.argmin(optimizer_cost))
    ranked = np.argsort(optimizer_cost)
    report = {
        'snapshot': str(args.snapshot.resolve()), 'config': str(args.config.resolve()),
        'samples': n, 'horizon': horizon, 'initial_state': x0.tolist(),
        'event': str(arrays['event']), 'sent_command': arrays['sent_command'].tolist(),
        'conditioned_nominal': arrays['conditioned_nominal'].tolist(),
        'lambda': float(arrays['lambda_value']),
        'optimizer_cost_min': float(optimizer_cost.min()),
        'optimizer_cost_max': float(optimizer_cost.max()),
        'ess': float(1/np.sum(weights**2)),
        'safe_sample_count': int(safe.sum()), 'unsafe_sample_count': int((~safe).sum()),
        'weight_mass_on_safe_samples': float(weights[safe].sum()),
        'cloud_safe_count': int(cloud_ok.sum()), 'map_safe_count': int(map_ok.sum()),
        'stopping_safe_count': int(stop_ok.sum()),
        'best_sample': {'index': best, 'safe': bool(safe[best]),
                        'optimizer_cost': float(optimizer_cost[best]),
                        'weight': float(weights[best])},
        'conditioned_nominal_feasibility': {
            'cloud': bool(nominal_shared['cloud_safe'][0]),
            'map': bool(nominal_shared['map_safe'][0]),
            'stopping': bool(nominal_shared['stopping_safe'][0]),
            'min_stopping_clearance_m': float(nominal_shared['stop_min'][0]),
        },
        'top_samples': [{'index': int(i), 'safe': bool(safe[i]),
                         'optimizer_cost': float(optimizer_cost[i]),
                         'weight': float(weights[i]),
                         'min_stop_clearance_m': float(min_stop_clearance[i])}
                        for i in ranked[:min(10, n)]],
        'objective_vs_optimizer_cost': {
            'note': 'optimizer cost includes MPPI sampling/control correction; component sum is physical objective only',
            'difference_min': float((optimizer_cost-objective).min()),
            'difference_max': float((optimizer_cost-objective).max())},
        'cost_component_mean': {k: float(v.mean()) for k, v in values.items()},
    }
    output = args.output or args.snapshot.with_name(args.snapshot.stem+'_analysis.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    np.savez_compressed(output.with_suffix('.npz'),
        **{f'cost_{k}': v for k, v in values.items()}, objective_cost=objective,
        optimizer_cost=optimizer_cost, weights=weights,
        cloud_feasible=cloud_ok, map_feasible=map_ok, stopping_feasible=stop_ok,
        safe=safe, min_stopping_clearance_m=min_stop_clearance)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

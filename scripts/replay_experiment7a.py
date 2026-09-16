#!/usr/bin/env python3
"""Offline sample-count/RNG replay for the fixed Experiment 7A snapshots."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch

from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI


def config_from_json(text, samples):
    raw = json.loads(str(text))
    cfg = MPPIConfig()
    for key, value in raw.items():
        if hasattr(cfg, key):
            setattr(cfg, key, value)
    cfg.samples = samples
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('selection', type=Path)
    parser.add_argument('--samples', nargs='+', type=int, default=[80, 160, 320, 640])
    parser.add_argument('--realizations', type=int, default=20)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.selection/'manifest.json').read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for snap_index, entry in enumerate(manifest['snapshots']):
        with np.load(args.selection/entry['file'], allow_pickle=False) as data:
            arrays = {key: data[key] for key in data.files}
        x0 = arrays['initial_state'].astype(float)
        runtime = json.loads(str(arrays['runtime_json']))
        for sample_count in args.samples:
            cfg = config_from_json(arrays['config_json'], sample_count)
            planner = QuadMPPI(cfg)
            planner.update_reference_path(arrays['reference_path_enu'])
            planner.update_goal(arrays['goal_enu'])
            planner.update_obstacles(arrays['obstacle_cloud'])
            planner.configure_trajectory_safety(
                collision_radius=cfg.collision_radius_m,
                acceleration=cfg.max_accel_xy,
                delay=runtime['hard_brake_delay_s'],
                stopping_clearance=runtime['hard_brake_m'],
                uncertainty=cfg.stopping_guard_uncertainty_m)
            for realization in range(args.realizations):
                seed = 7_000_000 + snap_index*10_000 + sample_count*20 + realization
                torch.manual_seed(seed)
                planner.ctrl.U.copy_(torch.as_tensor(
                    arrays['nominal_actions_before'], dtype=torch.double,
                    device=planner.goal.device))
                planner._reference_warm_started = True
                planner._path_progress_m = float(arrays['path_progress_m'])
                planner._applied_control_np = x0[7:11].copy()
                if cfg.response_accel_model:
                    planner._observed_acceleration = x0[11:14].copy()
                planner._rejection_recovery = bool(entry['rejection_recovery'])
                planner._recovery_seed_pending = False
                started = time.perf_counter()
                planner.command(x0[:3], x0[3:6], float(x0[6]))
                solve_ms = (time.perf_counter()-started)*1000
                mask = planner._last_sample_feasible_mask.detach().cpu().numpy().astype(bool)
                costs = planner.ctrl.cost_total.detach().cpu().numpy()
                weights = planner.ctrl.omega.detach().cpu().numpy()
                safe_indices = np.flatnonzero(mask)
                row = {**{k: entry[k] for k in ('file','seed','kind','label','cycle',
                        'speed_m_s','distance_to_first_turn_m')},
                       'K': sample_count, 'realization': realization,
                       'rng_seed': seed, 'n_safe': int(mask.sum()),
                       'hit': bool(mask.any()), 'solve_ms': solve_ms,
                       'best_feasible_cost': None, 'delta_j_safe': None,
                       'delta_j_safe_over_lambda': None, 'ess_safe': 0.0,
                       'best_stop_clearance_m': None,
                       'best_collision_clearance_m': None,
                       'best_first_raw_control': None,
                       'best_first_conditioned_control': None}
                if len(safe_indices):
                    ranked = safe_indices[np.argsort(costs[safe_indices])]
                    best = int(ranked[0])
                    row['best_feasible_cost'] = float(costs[best])
                    if len(ranked) > 1:
                        delta = float(costs[ranked[1]]-costs[best])
                        row['delta_j_safe'] = delta
                        row['delta_j_safe_over_lambda'] = delta/cfg.lambda_
                    safe_weights = weights[mask]
                    row['ess_safe'] = float(1/max(np.sum(safe_weights**2), 1e-300))
                    states = planner.ctrl.states
                    states = states[0] if states.ndim == 4 else states
                    initial = torch.as_tensor(x0, dtype=torch.double).view(1, 1, -1)
                    detailed = planner._evaluate_safety_states(
                        torch.cat((initial, states[best:best+1]), dim=1), detailed=True)
                    collision = min(float(detailed['cloud_min'][0]),
                                    float(detailed['map_min'][0]))
                    row['best_collision_clearance_m'] = collision if np.isfinite(collision) else None
                    stop = float(detailed['stop_min'][0])
                    row['best_stop_clearance_m'] = stop if np.isfinite(stop) else None
                    row['best_first_raw_control'] = planner.ctrl.perturbed_action[
                        best, 0].detach().cpu().numpy().tolist()
                    row['best_first_conditioned_control'] = states[
                        best, 0, 7:11].detach().cpu().numpy().tolist()
                rows.append(row)
            print(f"{entry['file']} K={sample_count} "
                  f"P_hit={sum(r['hit'] for r in rows[-args.realizations:])/args.realizations:.2f}",
                  flush=True)

    fields = list(rows[0])
    with (args.output/'solves.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            cooked = {k: json.dumps(v) if isinstance(v, list) else v for k, v in row.items()}
            writer.writerow(cooked)
    grouped = []
    for kind in ('failure', 'control'):
        for sample_count in args.samples:
            subset = [r for r in rows if r['kind']==kind and r['K']==sample_count]
            grouped.append({'kind': kind, 'K': sample_count, 'solves': len(subset),
                            'p_hit': float(np.mean([r['hit'] for r in subset])),
                            'median_n_safe': float(np.median([r['n_safe'] for r in subset])),
                            'median_solve_ms': float(np.median([r['solve_ms'] for r in subset]))})
    per_snapshot = []
    for entry in manifest['snapshots']:
        for sample_count in args.samples:
            subset = [r for r in rows if r['file']==entry['file'] and r['K']==sample_count]
            per_snapshot.append({'file': entry['file'], 'seed': entry['seed'],
                                 'kind': entry['kind'], 'label': entry['label'],
                                 'K': sample_count,
                                 'p_hit': float(np.mean([r['hit'] for r in subset])),
                                 'median_n_safe': float(np.median([r['n_safe'] for r in subset]))})
    report = {'schema_version': 1, 'selection': str(args.selection.resolve()),
              'sample_counts': args.samples, 'realizations': args.realizations,
              'total_solves': len(rows), 'grouped': grouped,
              'per_snapshot': per_snapshot}
    (args.output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report['grouped'], indent=2))


if __name__ == '__main__':
    main()

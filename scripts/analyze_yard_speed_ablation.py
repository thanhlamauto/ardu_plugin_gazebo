#!/usr/bin/env python3
"""Summarize actual headless runs; never substitute offline/ideal trajectories.

Speed/acceleration and flight time use Gazebo timestamps. Compute uses wall
time. Clearance is center-to-SDF-solid distance, NOT rotor clearance/contact.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mppi_ardupilot.global_planner import load_sdf_obstacles


def read_jsonl(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # a partial last line while monitoring is not a measurement
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    meta = json.loads((directory/'manifest.json').read_text())
    obstacles = load_sdf_obstacles(directory/meta.get('world_file', 'iris_mppi_industrial_yard.sdf'))
    reference = np.asarray(meta['reference_enu'])
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    ax = axes[0, 0]
    for o in obstacles:
        if o.kind == 'box' and o.z_max > .1:
            hx, hy = o.half_size_xy
            ax.add_patch(Rectangle((o.center_xy[0]-hx, o.center_xy[1]-hy),
                2*hx, 2*hy, color='gray', alpha=.4))
    ax.plot(reference[:,0], reference[:,1], 'k--', lw=1, label='fixed A* raw path')
    ax.set(xlabel='East [m]', ylabel='North [m]', title='Measured trajectories', ylim=(-12,12))
    ax.set_aspect('equal')
    summary = []
    for trial in sorted(directory.glob('v*_seed*')):
        result_path = trial/'result.json'
        if not result_path.exists():
            continue
        result = json.loads(result_path.read_text())
        rows = read_jsonl(trial/'planner.jsonl')
        if not rows:
            summary.append({'trial': trial.name, 'status': result['status']})
            continue
        start_wall = rows[0]['timestamp_monotonic_s'] - rows[0].get('cycle_to_send_ms', 0)/1000
        end_wall = rows[-1]['timestamp_monotonic_s']
        gt = [r for r in read_jsonl(trial/'ground_truth.jsonl')
              if start_wall <= r['wall_monotonic_s'] <= end_wall]
        if len(gt) < 3:
            continue
        times = np.array([r['sim_time_s'] for r in gt])
        points = np.array([r['position_enu'] for r in gt])
        keep = np.r_[True, np.diff(times)>0]
        times, points = times[keep], points[keep]
        # Resample at .1 simulation seconds to avoid arrival jitter in derivatives.
        uniform = np.arange(times[0], times[-1], .1)
        p = np.column_stack([np.interp(uniform, times, points[:,i]) for i in range(3)])
        v = np.gradient(p, .1, axis=0)
        speed = np.linalg.norm(v[:,:2], axis=1)
        acceleration = np.gradient(v[:,:2], .1, axis=0)
        clearance = np.full(len(p), np.inf)
        for o in obstacles:
            c, s = np.cos(o.yaw), np.sin(o.yaw)
            local = (p[:,:2]-o.center_xy) @ np.array([[c,-s],[s,c]])
            if o.kind == 'box':
                outside = np.maximum(np.abs(local)-o.half_size_xy, 0)
                dxy = np.linalg.norm(outside, axis=1)
            else:
                dxy = np.maximum(np.linalg.norm(local, axis=1)-o.radius, 0)
            dz = np.maximum(np.maximum(o.z_min-p[:,2], p[:,2]-o.z_max), 0)
            clearance = np.minimum(clearance, np.hypot(dxy,dz))
        compute = np.array([r.get('compute_ms',0) for r in rows if r.get('compute_ms',0)>0])
        cycle = np.array([r.get('cycle_to_send_ms',0) for r in rows])
        control_rows = [r for r in rows if r.get('state_simulation_timestamp_s') is not None and r.get('u_enu') is not None]
        commands = np.array([r['u_enu'][:2] for r in control_rows])
        sim_times = np.array([r['state_simulation_timestamp_s'] for r in control_rows])
        valid_dt = np.diff(sim_times)>1e-6
        du = np.linalg.norm(np.diff(commands,axis=0),axis=1)[valid_dt]/np.diff(sim_times)[valid_dt]
        duration = times[-1]-times[0]
        summary.append({
            'trial': trial.name, 'status': result['status'], 'reached': result.get('reached',False),
            'reference_speed_m_s': result['speed_reference'], 'seed': result['seed'],
            'flight_time_sim_s': duration, 'flight_time_wall_s': end_wall-start_wall,
            'rtf_measured': duration/(end_wall-start_wall),
            'path_length_m': np.linalg.norm(np.diff(p,axis=0),axis=1).sum(),
            'mean_xy_speed_m_s': speed.mean(), 'p95_xy_speed_m_s': np.percentile(speed,95),
            'max_xy_speed_m_s': speed.max(), 'min_center_surface_clearance_m': clearance.min(),
            'time_at_or_above_95pct_request_sim_s': float(np.count_nonzero(speed >= .95*result['speed_reference'])*.1),
            'rms_xy_acceleration_m_s2': np.sqrt(np.mean(np.sum(acceleration**2,axis=1))),
            'rms_command_slew_m_s2': np.sqrt(np.mean(du**2)) if len(du) else None,
            'slow_time_sim_s': float(np.count_nonzero(speed<.1)*.1),
            'max_altitude_error_m': np.max(np.abs(p[:,2]-5)),
            'goal_error_m': float(np.linalg.norm(points[-1]-meta['goal_enu'])),
            'planner_final_goal_error_m': next((r['goal_dist_m'] for r in reversed(rows) if r.get('goal_dist_m') is not None), None),
            'planner_cycles': len(rows),
            'compute_mean_ms': compute.mean() if len(compute) else None,
            'compute_p95_ms': np.percentile(compute,95) if len(compute) else None,
            'compute_worst_ms': compute.max() if len(compute) else None,
            'cycle_to_send_p95_ms': np.percentile(cycle,95),
            'optimizer_deadline_misses': sum(r.get('deadline_miss',False) for r in rows),
            'cycle_deadline_misses': sum(r.get('cycle_deadline_miss',False) for r in rows),
            'brake_cycles': sum(r.get('event') in ('hold-brake','recover-brake') for r in rows),
            'stopping_guard_hold_cycles': sum(r.get('event') == 'hold-invalid-stopping-trajectory' for r in rows),
            'timeout_hold_cycles': sum(r.get('event')=='hold-timeout' for r in rows),
        })
        label = trial.name
        line, = ax.plot(p[:,0],p[:,1],label=label,lw=1)
        color = line.get_color()
        t = uniform-uniform[0]
        axes[0,1].plot(t,speed,label=label,color=color,lw=1)
        axes[1,0].plot(t,clearance,label=label,color=color,lw=1)
        axes[1,1].plot(t,p[:,2],label=label,color=color,lw=1)
    axes[0,1].set(xlabel='Simulation time [s]',ylabel='Horizontal speed [m/s]')
    axes[1,0].set(xlabel='Simulation time [s]',ylabel='Center-to-solid clearance [m]')
    axes[1,0].axhline(1.5,color='red',linestyle='--',label='local safety radius')
    axes[1,1].set(xlabel='Simulation time [s]',ylabel='Altitude [m]')
    for a in axes.flat:
        a.grid(alpha=.2)
    ax.legend(fontsize=6,loc='upper left')
    fig.tight_layout()
    fig.savefig(directory/'comparison.png',dpi=160)
    (directory/'summary.json').write_text(json.dumps(summary,indent=2))
    if summary:
        keys = list(dict.fromkeys(k for row in summary for k in row))
        with (directory/'summary.csv').open('w') as handle:
            writer = csv.DictWriter(handle,fieldnames=keys)
            writer.writeheader()
            writer.writerows(summary)
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()

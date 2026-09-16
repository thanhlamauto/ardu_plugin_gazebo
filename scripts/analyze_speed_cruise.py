#!/usr/bin/env python3
"""Measure continuous cruise from Gazebo positions, without bridging telemetry gaps."""
import argparse
import json
from pathlib import Path

import numpy as np


def cruise_windows(rows, target, tolerance=.05, step=.1):
    """Return in-band intervals; split at nonmonotonic time or >0.5s data gaps.

    Velocity is the XY displacement over each resampled simulation-time interval.
    A wall-time stall breaks continuity even when simulation time barely moves.
    """
    segments, segment = [], []
    for row in rows:
        if segment:
            ds = row['sim_time_s'] - segment[-1]['sim_time_s']
            dw = row['wall_monotonic_s'] - segment[-1]['wall_monotonic_s']
            if not (0 < ds <= .5 and 0 < dw <= .5):
                segments.append(segment)
                segment = []
        segment.append(row)
    segments.append(segment)
    windows = []
    for segment in segments:
        if len(segment) < 2:
            continue
        t = np.array([r['sim_time_s'] for r in segment])
        p = np.array([r['position_enu'][:2] for r in segment])
        grid = np.arange(t[0], t[-1], step)
        if len(grid) < 2:
            continue
        xy = np.column_stack([np.interp(grid, t, p[:, i]) for i in range(2)])
        speed = np.linalg.norm(np.diff(xy, axis=0), axis=1)/step
        inside = np.abs(speed-target) <= tolerance*target + 1e-9
        edges = np.diff(np.r_[False, inside, False].astype(int))
        for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            windows.append({'start_sim_s': float(grid[start]),
                            'end_sim_s': float(grid[end]),
                            'duration_sim_s': float(grid[end]-grid[start]),
                            'mean_xy_m_s': float(speed[start:end].mean())})
    return windows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    reports = []
    for trial in sorted(args.directory.glob('v*_seed*')):
        if not (trial/'result.json').exists():
            continue
        result = json.loads((trial/'result.json').read_text())
        planner = [json.loads(line) for line in (trial/'planner.jsonl').read_text().splitlines()] if (trial/'planner.jsonl').exists() else []
        gt = []
        if planner:
            start = planner[0]['timestamp_monotonic_s']-planner[0].get('cycle_to_send_ms', 0)/1000
            end = planner[-1]['timestamp_monotonic_s']
            gt = [r for line in (trial/'ground_truth.jsonl').read_text().splitlines()
                  if start <= (r := json.loads(line))['wall_monotonic_s'] <= end]
        windows = cruise_windows(gt, result['speed_reference'])
        longest = max((w['duration_sim_s'] for w in windows), default=0.)
        reports.append({'trial': trial.name, 'target_m_s': result['speed_reference'],
                        'status': result['status'], 'reached': result.get('reached', False),
                        'landed_disarmed': result.get('landed_disarmed', False),
                        'tolerance_fraction': .05, 'required_continuous_sim_s': 5.,
                        'longest_in_band_sim_s': longest,
                        'speed_criterion_pass': longest >= 5.-1e-9,
                        'windows': windows})
    (args.directory/'cruise_check.json').write_text(json.dumps(reports, indent=2))
    print(json.dumps([{k: v for k, v in r.items() if k != 'windows'} for r in reports], indent=2))


if __name__ == '__main__':
    main()

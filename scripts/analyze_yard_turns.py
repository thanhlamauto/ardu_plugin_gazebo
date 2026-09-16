#!/usr/bin/env python3
"""Measured speeds near sharp A* vertices; not an estimate of collision safety."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    meta = json.loads((directory/'manifest.json').read_text())
    path = np.array(meta['reference_enu'])[:, :2]
    direction = np.diff(path, axis=0)
    direction /= np.linalg.norm(direction, axis=1)[:, None]
    angles = np.degrees(np.arccos(np.clip(np.sum(direction[:-1]*direction[1:], axis=1), -1, 1)))
    corners = path[1:-1][angles >= 60]
    if not len(corners):
        raise SystemExit('No A* heading change >=60 degrees in this experiment')
    results = []
    for trial in sorted(directory.glob('v*_seed*')):
        if not (trial/'result.json').exists():
            continue
        result = json.loads((trial/'result.json').read_text())
        if not (trial/'planner.jsonl').exists():
            continue
        planner = read(trial/'planner.jsonl')
        if not planner:
            continue
        start = planner[0]['timestamp_monotonic_s']-planner[0].get('cycle_to_send_ms', 0)/1000
        end = planner[-1]['timestamp_monotonic_s']
        gt = [r for r in read(trial/'ground_truth.jsonl') if start <= r['wall_monotonic_s'] <= end]
        # Do not interpolate across a stall or a reset of simulation time.
        segments, current = [], []
        for row in gt:
            if current and not (0 < row['sim_time_s']-current[-1]['sim_time_s'] <= .5
                               and 0 < row['wall_monotonic_s']-current[-1]['wall_monotonic_s'] <= .5):
                segments.append(current)
                current = []
            current.append(row)
        segments.append(current)
        turn_speeds = []
        for segment in segments:
            if len(segment) < 3:
                continue
            t = np.array([r['sim_time_s'] for r in segment])
            p = np.array([r['position_enu'] for r in segment])
            grid = np.arange(t[0], t[-1], .1)
            if len(grid) < 3:
                continue
            xy = np.column_stack([np.interp(grid, t, p[:, k]) for k in range(2)])
            speed = np.linalg.norm(np.gradient(xy, .1, axis=0), axis=1)
            near = np.min(np.linalg.norm(xy[:, None, :]-corners, axis=2), axis=1) <= 2.
            turn_speeds.extend(speed[near].tolist())
        results.append({'trial': trial.name, 'status': result['status'],
                        'reference_speed_m_s': result['speed_reference'],
                        'turn_zone_sample_time_sim_s': len(turn_speeds)*.1,
                        'turn_zone_min_xy_m_s': min(turn_speeds) if turn_speeds else None,
                        'turn_zone_mean_xy_m_s': float(np.mean(turn_speeds)) if turn_speeds else None,
                        'turn_zone_max_xy_m_s': max(turn_speeds) if turn_speeds else None})
    report = {'zone_definition': 'within 2m XY of raw A* vertices with heading change >=60deg',
              'corner_centers_xy': corners.tolist(), 'trials': results}
    (directory/'turn_check.json').write_text(json.dumps(report, indent=2))
    source = Path(__file__).read_bytes()
    (directory/'analyze_yard_turns.py').write_bytes(source)
    (directory/'turn_analysis_sha256.txt').write_text(hashlib.sha256(source).hexdigest()+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

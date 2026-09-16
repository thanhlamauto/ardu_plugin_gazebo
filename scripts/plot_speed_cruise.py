#!/usr/bin/env python3
"""Plot measured cruise against requested speed and the demo tolerance band."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    directory = parser.parse_args().directory
    reports = json.loads((directory/'cruise_check.json').read_text())
    speeds = sorted({r['target_m_s'] for r in reports})
    fig, axes = plt.subplots(1, len(speeds), figsize=(6*len(speeds), 4.5), squeeze=False)
    for ax, target in zip(axes[0], speeds):
        ax.axhspan(.95*target, 1.05*target, color='#5dba91', alpha=.18,
                   label='Target band (+/-5%)')
        ax.axhline(target, color='#326a50', linestyle='--', linewidth=1)
        for report in (r for r in reports if r['target_m_s'] == target):
            trial = directory/report['trial']
            planner = [json.loads(line) for line in (trial/'planner.jsonl').read_text().splitlines()]
            start = planner[0]['timestamp_monotonic_s']-planner[0].get('cycle_to_send_ms', 0)/1000
            end = planner[-1]['timestamp_monotonic_s']
            gt = [r for line in (trial/'ground_truth.jsonl').read_text().splitlines()
                  if start <= (r := json.loads(line))['wall_monotonic_s'] <= end]
            if len(gt) < 2:
                continue
            times = np.array([r['sim_time_s'] for r in gt])
            wall = np.array([r['wall_monotonic_s'] for r in gt])
            xy = np.array([r['position_enu'][:2] for r in gt])
            split = np.flatnonzero((np.diff(times) <= 0) | (np.diff(times) > .5)
                                   | (np.diff(wall) <= 0) | (np.diff(wall) > .5))+1
            seed = report['trial'].split('_seed')[-1]
            label = f"Seed {seed}: longest in band {report['longest_in_band_sim_s']:.1f} s"
            color = None
            for indices in np.split(np.arange(len(gt)), split):
                if len(indices) < 2:
                    continue
                t, p = times[indices], xy[indices]
                grid = np.arange(t[0], t[-1], .1)
                if len(grid) < 2:
                    continue
                positions = np.column_stack([np.interp(grid, t, p[:, k]) for k in range(2)])
                speed = np.linalg.norm(np.diff(positions, axis=0), axis=1)/.1
                line, = ax.plot(grid[:-1]+.05-times[0], speed, lw=1.4, label=label, color=color)
                color, label = line.get_color(), '_nolegend_'
        ax.set(title=f'Requested cruise: {target:g} m/s', xlabel='Simulation time [s]',
               ylabel='Measured horizontal speed [m/s]', ylim=(0, target*1.15))
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='lower center')
    fig.suptitle('Gazebo / ArduPilot SITL — 300 m straight flight', fontsize=13)
    fig.tight_layout()
    fig.savefig(directory/'cruise_speed.png', dpi=170)
    source = Path(__file__).read_bytes()
    (directory/'plot_speed_cruise.py').write_bytes(source)
    (directory/'plot_source_sha256.txt').write_text(hashlib.sha256(source).hexdigest()+'\n')


if __name__ == '__main__':
    main()

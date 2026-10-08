#!/usr/bin/env python3
"""Plot comparable CPU/latency/throughput measurements from saved JSON runs."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', action='append', required=True,
                        help='Label=path/to/benchmark.json; repeat for each run')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    labels, runs = [], []
    for item in args.result:
        if '=' not in item:
            parser.error('--result must be Label=path/to/benchmark.json')
        label, path = item.split('=', 1)
        labels.append(label)
        runs.append(json.loads(Path(path).read_text()))
    y = np.arange(len(runs))
    colors = ['#287eb6' if r['device'] != 'cpu' else '#d7793c' for r in runs]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.1), sharey=True)
    metrics = [
        ('cpu_percent_one_core', 'Process CPU (% of one core)', 110, None),
        ('pipeline_latency_ms', 'Depth pipeline p95 (ms)', 300, 100),
        ('achieved_rate_hz', 'Processed frames / second', 11, 10),
    ]
    for ax, (key, title, limit, guide) in zip(axes, metrics):
        values = [r[key]['p95'] if key == 'pipeline_latency_ms' else r[key]
                  for r in runs]
        ax.barh(y, values, height=.55, color=colors)
        ax.set_xlim(0, max(limit, max(values) * 1.17))
        ax.set_title(title, fontsize=11)
        ax.set_yticks(y, labels)
        ax.invert_yaxis()
        ax.grid(axis='x', color='#dddddd', linewidth=.7)
        ax.set_axisbelow(True)
        if guide is not None:
            ax.axvline(guide, color='#a52e2e', linestyle='--', linewidth=1.4)
        for i, value in enumerate(values):
            ax.text(value + max(limit, max(values) * 1.17) * .015, i,
                    f'{value:.1f}', va='center', fontsize=10)
    fig.suptitle('Depth Anything V2 Metric Outdoor Small — saved RGB replay at 10 Hz',
                 fontsize=13, fontweight='bold')
    fig.text(.5, .025, 'MacBook Air M2; MPS uses GPU. CPU-only run uses one Torch thread. '
             'Red lines: 100 ms/frame and 10 fps targets.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .06, 1, .92))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Select the fixed 12 failure + 8 control snapshots for Experiment 7A."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np


def cycle_of(path):
    return int(path.stem.rsplit('cycle', 1)[1])


def nearest_unique(rows, targets, used):
    chosen = []
    for target in targets:
        available = [r for r in rows if r['cycle'] not in used]
        row = min(available, key=lambda r: abs(r['position_enu'][0]-target))
        used.add(row['cycle'])
        chosen.append(row)
    return chosen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema_version': 1, 'source': str(args.capture.resolve()), 'snapshots': []}
    for trial in sorted(args.capture.glob('v10.0_seed*')):
        seed = int(trial.name.rsplit('seed', 1)[1])
        rows = [json.loads(line) for line in (trial/'planner.jsonl').read_text().splitlines()]
        by_cycle = {r['cycle']: r for r in rows}
        pools = {cycle_of(p): p for p in trial.glob('mppi_sample_pool_cycle*.npz')}
        failures = [r for r in rows if r.get('feasible_sample_count') == 0 and r['cycle'] in pools]
        if len(failures) < 6:
            raise RuntimeError(f'{trial.name}: need at least 6 N_safe=0 snapshots')
        zero_cycles = {r['cycle'] for r in failures}
        pre_recovery = [r for r in failures if r['cycle']+1 not in zero_cycles]
        selected = []
        used = set()
        candidates = [
            ('first_occurrence', failures[0]),
            ('after_first_zero_hold', next((r for r in failures if r['cycle']==failures[0]['cycle']+1), failures[1])),
            ('early_cluster', failures[max(1, len(failures)//4)]),
            ('middle', failures[len(failures)//2]),
            ('nearest_first_turn', min(failures, key=lambda r: abs(r['position_enu'][0]-60.0))),
            ('before_recovery', pre_recovery[-1] if pre_recovery else failures[-1]),
        ]
        for label, row in candidates:
            if row['cycle'] in used:
                replacement = next(r for r in failures if r['cycle'] not in used)
                row = replacement
            used.add(row['cycle'])
            selected.append((label, row, 'failure'))

        controls = [r for r in rows if r['cycle'] in pools
                    and r.get('feasible_sample_count', 0) > 0 and r.get('event') == 'command']
        used_controls = set()
        control_rows = nearest_unique(controls, [15.0, 35.0, 65.0, 74.0], used_controls)
        selected += [(f'control_{index+1}', row, 'control')
                     for index, row in enumerate(control_rows)]

        for label, row, kind in selected:
            source = pools[row['cycle']]
            name = f'seed{seed}_{kind}_{label}_cycle{row["cycle"]}.npz'
            shutil.copy2(source, args.output/name)
            manifest['snapshots'].append({
                'file': name, 'seed': seed, 'kind': kind, 'label': label,
                'cycle': row['cycle'], 'event': row['event'],
                'n_safe_original': row.get('feasible_sample_count'),
                'position_enu': row['position_enu'],
                'speed_m_s': float(np.linalg.norm(row['velocity_enu'][:2])),
                'distance_to_first_turn_m': float(np.linalg.norm(
                    np.asarray(row['position_enu'][:2])-np.asarray([60.0, 0.0]))),
                'rejection_recovery': bool(row.get('rejection_recovery', False)),
            })
    counts = {kind: sum(x['kind']==kind for x in manifest['snapshots'])
              for kind in ('failure', 'control')}
    if counts != {'failure': 12, 'control': 8}:
        raise RuntimeError(f'unexpected selection counts: {counts}')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({'output': str(args.output), **counts}, indent=2))


if __name__ == '__main__':
    main()

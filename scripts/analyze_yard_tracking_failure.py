#!/usr/bin/env python3
"""Read-only diagnostics of archived failed flights; no controller tuning.

Logged nominal trajectories are reconstructed after applied-control feedback,
not immutable optimizer samples. Future tracking error includes replanning.
Zero-command model comparison is illustrative, not a replay of safety logic.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read(p):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory', type=Path)
    args = ap.parse_args()
    directory = args.directory
    report = {'limitations': __doc__, 'trials': []}
    trials = sorted(directory.glob('v*_seed*'))
    fig, axes = plt.subplots(len(trials), 3, figsize=(17, 5*len(trials)), squeeze=False)
    for row, trial in enumerate(trials):
        records = read(trial/'planner.jsonl')
        commands = [r for r in records if r['event'] == 'command']
        brake = next(r for r in records if r['event'] == 'hold-brake')
        last = next(r for r in reversed(commands) if r['cycle'] < brake['cycle'])
        bt = brake['state_simulation_timestamp_s']
        gt = read(trial/'ground_truth.jsonl')
        # Restrict to logged planner activity: do not include LAND after abort.
        start = records[0]['state_simulation_timestamp_s']
        end = records[-1]['state_simulation_timestamp_s']
        gt = [r for r in gt if start <= r['sim_time_s'] <= end]
        t = np.array([r['sim_time_s'] for r in gt])
        p = np.array([r['position_enu'] for r in gt])
        assert np.all(np.diff(t) > 0), 'non-monotonic ground truth'
        def actual(times):
            return np.column_stack([np.interp(times, t, p[:, k]) for k in range(3)])
        grid = np.arange(t[0], t[-1], .02)
        positions = actual(grid)
        speed = np.linalg.norm(np.gradient(positions, .02, axis=0)[:, :2], axis=1)
        nominal = np.array(last['selected_trajectory_enu'])
        nt = last['state_simulation_timestamp_s'] + np.arange(len(nominal))*.1
        valid = (nt >= t[0]) & (nt <= t[-1])
        errors = np.linalg.norm(nominal[valid,:2]-actual(nt[valid])[:,:2],axis=1)
        after = (grid >= bt) & (grid <= end-.04)
        peak_index = np.flatnonzero(after)[np.argmax(speed[after])]
        v0 = np.linalg.norm(brake['velocity_enu'][:2])
        horizon = min(1., t[-1]-bt)
        distance = np.linalg.norm(actual(np.array([bt+horizon]))[0,:2]-actual(np.array([bt]))[0,:2])
        # Exact discrete velocity model, with already-applied zero command.
        model_time = np.arange(0,1.1001,.1)
        model_speed = v0*.8**np.arange(len(model_time))
        item = {'trial':trial.name, 'first_brake_cycle':brake['cycle'],
            'first_brake_position_enu':brake['position_enu'],
            'first_brake_measured_xy_m_s':v0,
            'ground_truth_peak_after_brake_m_s':float(speed[peak_index]),
            'peak_delay_from_brake_state_sim_s':float(grid[peak_index]-bt),
            'displacement_after_brake_m':float(distance), 'displacement_window_s':horizon,
            'zero_command_model_xy_at_1s_m_s':float(model_speed[10]),
            'actual_xy_at_1s_m_s':float(np.interp(bt+1,grid,speed)) if bt+1<=grid[-1] else None,
            'last_command_cycle':last['cycle'], 'raw_control':last['raw_control'],
            'sent_control':last['nominal_control'],
            'last_logged_rollout_xy_error_max_over_observed_future_m':float(max(errors)),
            'logged_nominal_collision_cost_positive_cycles': [r['cycle'] for r in commands if r.get('cost',{}).get('collision',0)>0],
            'ess_min':min(r.get('ess',float('inf')) for r in commands),
            'first_brake_geometry':{k:brake.get(k) for k in ['stopping_distance_m','swept_clearance_m','closing_speed_m_s','state_age_at_send_s']}}
        report['trials'].append(item)
        ax=axes[row,0]
        ax.plot(p[:,0],p[:,1],label='Gazebo, until abort')
        ax.plot(nominal[:,0],nominal[:,1],'--',label=f'Logged nominal, cycle {last["cycle"]}')
        path=np.array(json.loads((directory/'manifest.json').read_text())['reference_enu'])
        ax.plot(path[:,0],path[:,1],':',color='gray',label='Raw A* path')
        from matplotlib.patches import Rectangle
        for x,y in [(9,-2),(21,2)]:
            ax.add_patch(Rectangle((x-3,y-1.22),6,2.44,color='gray',alpha=.4))
        ax.scatter(*brake['position_enu'][:2],color='red',label='First brake')
        ax.set(title=trial.name,xlabel='East (m)',ylabel='North (m)',ylim=(-5,5))
        ax.legend(fontsize=8);ax.grid(alpha=.2)
        ax=axes[row,1]
        rt=np.array([r['state_simulation_timestamp_s'] for r in records])-bt
        ax.plot(rt,[np.linalg.norm(r['velocity_enu'][:2]) for r in records],label='Measured state XY')
        ax.plot(rt,[np.linalg.norm(r['nominal_control'][:2]) for r in records],label='Sent XY command')
        ax.plot([r['state_simulation_timestamp_s']-bt for r in commands],
                [np.linalg.norm(r['raw_control'][:2]) for r in commands],label='Raw XY command',alpha=.65)
        ax.axvline(0,color='red',ls=':');ax.set(xlabel='Sim seconds relative to first brake state',ylabel='m/s')
        ax.legend(fontsize=8);ax.grid(alpha=.2)
        ax=axes[row,2]
        ax.plot(grid[after]-bt,speed[after],label='Gazebo XY speed')
        ax.plot(model_time,model_speed,'--',label='tau=.5, applied zero (illustration)')
        ax.set(xlabel='Sim seconds after first brake state',ylabel='m/s',title='Brake response: model vs actual')
        ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(directory/'tracking_failure.png',dpi=150);plt.close(fig)
    report['analysis_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (directory/'tracking_failure.json').write_text(json.dumps(report,indent=2))
    (directory/'source_snapshot/scripts'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    main()

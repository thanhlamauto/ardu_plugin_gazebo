#!/usr/bin/env python3
"""Compare first-order model with logged sent commands and Gazebo motion.
Offline one-step check, not identification of ArduPilot's internal target states.
"""
import json
from pathlib import Path
import numpy as np

def read(p):return [json.loads(l) for l in p.read_text().splitlines()]
def main():
    root=Path('output/benchmark/yard_free10_20260915_v1');data={}
    for seed in [7,17]:
        d=root/f'v10.0_seed{seed}';rows=read(d/'planner.jsonl');gt=read(d/'ground_truth.jsonl')
        wall=np.array([r['wall_monotonic_s'] for r in gt]);sim=np.array([r['sim_time_s'] for r in gt]);p=np.array([r['position_enu'][:2] for r in gt])
        assert np.all(np.diff(wall)>0) and np.all(np.diff(sim)>0)
        grid=np.arange(sim[0],sim[-1],.02)
        xy=np.column_stack([np.interp(grid,sim,p[:,i]) for i in range(2)])
        v=np.gradient(xy,.02,axis=0)
        def vel(t):return np.array([np.interp(t,grid,v[:,i]) for i in range(2)])
        first=next(r for r in rows if r['event']=='hold-brake')
        bt=float(np.interp(first['timestamp_monotonic_s'],wall,sim))
        pairs=[]
        for r,next_r in zip(rows[:-1],rows[1:]):
            t=float(np.interp(r['timestamp_monotonic_s'],wall,sim))
            nt=float(np.interp(next_r['timestamp_monotonic_s'],wall,sim))
            if not .05<=nt-t<=.15 or r['event']!='command':continue
            pairs.append((vel(t),vel(nt),np.array(r['nominal_control'][:2]),nt-t))
        def error(tau):
            return float(np.sqrt(np.mean([np.sum((v0+min(dt/tau,1)*(u-v0)-v1)**2) for v0,v1,u,dt in pairs])))
        data[seed]={'pairs':pairs,'error':error,'bt':bt,'velocity':vel,'first':first}
        # Bind per-trial closure values before moving to the next seed.
        data[seed]['errors']={str(tau):error(tau) for tau in [.3,.5,.8,1.,1.5,2.]}
        data[seed]['brake']={'v_gt_at_send_xy_m_s':float(np.linalg.norm(vel(bt))),
          'v_gt_after_1s_xy_m_s':float(np.linalg.norm(vel(bt+1))),
          'zero_model_tau05_after_1s_xy_m_s':float(np.linalg.norm(vel(bt))*.8**10),
          'initial_0p2s_actual_delta_speed_m_s':float(np.linalg.norm(vel(bt+.2))-np.linalg.norm(vel(bt)))}
    best=min(data[7]['errors'],key=data[7]['errors'].get)
    result={'source':'archived baseline, sent timestamps mapped via ground truth wall/sim pairs',
      'limitations':'one-step velocities from 0.02s position differentiation; send timestamp is not ArduPilot receipt; no internal target/acceleration telemetry',
      'train_seed':7,'heldout_seed':17,'best_tau_grid_on_normal_commands_s':float(best),
      'trials':{str(s):{'normal_command_pairs':len(data[s]['pairs']),'normal_one_step_xy_velocity_rmse_m_s':data[s]['errors'],'brake':data[s]['brake']} for s in data},
      'conclusion':'first-order zero-input model cannot represent continued acceleration after zero; tau sensitivity is not a validated replacement model'}
    out=Path('output/benchmark/mppi_implementation_audit_fixed_20260915');(out/'response_check.json').write_text(json.dumps(result,indent=2));(out/Path(__file__).name).write_bytes(Path(__file__).read_bytes());print(json.dumps(result,indent=2))
if __name__=='__main__':main()

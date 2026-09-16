#!/usr/bin/env python3
"""Frozen-state comparison; not an exact replay of the flight RNG or a new flight."""
import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
import mppi_velocity_avoidance as cli
from mppi_ardupilot.mppi_controller import QuadMPPI
from mppi_ardupilot.trajectory_validation import validate_trajectory

def main():
 root=ROOT/'output/benchmark/yard_stopping10_20260916_v3'
 meta=json.loads((root/'manifest.json').read_text()); results=[]
 for seed in (7,17):
  rows=[json.loads(x) for x in (root/f'v10.0_seed{seed}/planner.jsonl').read_text().splitlines()]
  for cycle in (70,80):
   row=next(x for x in rows if x['cycle']==cycle);snap=row.get('nominal_snapshot')
   if snap is None:continue
   for config in ('mppi_yard_stopping_long10.yaml','mppi_yard_progress10.yaml'):
    args=cli.build_parser().parse_args(['--config',str(ROOT/'config/experiments'/config),'--seed',str(seed)])
    q=QuadMPPI(cli.build_cfg(args));q.update_goal(meta['goal_enu']);q.update_reference_path(np.array(meta['reference_enu']));q.update_obstacles(np.array(snap['obstacles_enu']))
    state=np.array(snap['state']);q.accept_applied_control(state[7:11]);q._observed_acceleration=state[11:14].copy();q._path_progress_m=row['reference_progress_m']
    raw=np.array(snap['raw_actions']);raw=np.concatenate((raw,np.repeat(raw[-1:],max(0,q.cfg.horizon-len(raw)),axis=0)))[:q.cfg.horizon]
    q.ctrl.U=q.torch.tensor(raw,dtype=q.torch.double);q._reference_warm_started=True
    q.command(state[:3],state[3:6],state[6])
    trajectory=q.predict_trajectory();roll=q.torch.tensor(q._last_state_np,dtype=q.torch.double);velocities=[]
    for i,u in enumerate(q.ctrl.U):
     roll=q._dynamics(roll,u,i);velocities.append(roll[3:6].detach().cpu().numpy().tolist())
    results.append({'seed':seed,'cycle':cycle,'config':config,'position':state[:3].tolist(),'initial_velocity':state[3:6].tolist(),'cost':q.nominal_cost_breakdown(),'predicted_velocity':velocities,'trajectory':trajectory.tolist(),'cloud_gate':validate_trajectory(trajectory,snap['obstacles_enu'],q.cfg.collision_radius_m),'map_gate':q.known_geometry.validate(trajectory,q.cfg.collision_radius_m)})
 dest=ROOT/'output/benchmark/progress_replay_20260916';dest.mkdir(exist_ok=True)
 (dest/'replay.json').write_text(json.dumps(results,indent=2));(dest/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
 print(json.dumps([{'seed':x['seed'],'cycle':x['cycle'],'config':x['config'],'valid':x['cloud_gate']['valid'] and x['map_gate']['valid'],'speed_1s':float(np.linalg.norm(x['predicted_velocity'][9][:2])),'speed_2s':float(np.linalg.norm(x['predicted_velocity'][19][:2]))} for x in results],indent=2))
if __name__=='__main__':main()

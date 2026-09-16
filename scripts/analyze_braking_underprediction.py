#!/usr/bin/env python3
"""One-sided braking metric from logged safety holds; observational, not a dedicated step-input test."""
import json,argparse
from pathlib import Path
import numpy as np

def rows(path):return [json.loads(s) for s in path.read_text().splitlines()]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);args=ap.parse_args()
 output=[]
 for trial in sorted(args.directory.glob('v*_seed*')):
  f=trial/'planner.jsonl';g=trial/'ground_truth.jsonl'
  if not f.exists() or not g.exists():continue
  logs=rows(f);gt=rows(g);wall=np.array([r['wall_monotonic_s'] for r in gt]);t=np.array([r['sim_time_s'] for r in gt]);p=np.array([r['position_enu'] for r in gt]);keep=np.r_[True,np.diff(t)>0];t,p=t[keep],p[keep]
  grid=np.arange(t[0],t[-1],.1);pos=np.column_stack([np.interp(grid,t,p[:,k]) for k in range(3)]);v=np.gradient(pos,.1,axis=0)
  for idx,r in enumerate(logs):
   if r['event']!='hold-brake' or idx and logs[idx-1]['event']=='hold-brake':continue
   event_t=float(np.interp(r['timestamp_monotonic_s'],wall,np.array([x['sim_time_s'] for x in gt])))
   i=int(np.searchsorted(grid,event_t));initial=v[i,:2];speed=float(np.linalg.norm(initial));direction=initial/max(speed,1e-9)
   after=[z for z in logs[idx:] if z.get('state_simulation_timestamp_s') is not None and z['state_simulation_timestamp_s']>=event_t]
   first_nonzero=next((z for z in after if z['event'] not in ('hold-brake','hold-invalid-trajectory','hold-invalid-stopping-trajectory','hold-timeout')),None)
   end_t=first_nonzero['state_simulation_timestamp_s'] if first_nonzero else grid[-1]
   stop=next((j for j in range(i+1,len(grid)-2) if grid[j]<end_t and all(np.linalg.norm(v[j+k,:2])<=.25 for k in range(3))),None)
   predicted=speed*.25+speed**2/(2*3.0)
   row={'trial':trial.name,'cycle':r['cycle'],'event_sim_s':event_t,'initial_speed_gt_m_s':speed,'predicted_stop_distance_model_formula_m':predicted,'consecutive_hold_end_sim_s':end_t,'stop_reached_before_nonzero_command':stop is not None}
   if stop is not None:
    delta=pos[stop,:2]-pos[i,:2];actual=float(delta@direction);lateral=float(np.abs(np.cross(direction,(pos[i:stop+1,:2]-pos[i,:2]))).max())
    row.update(actual_stop_distance_along_initial_direction_m=actual,stop_distance_underprediction_m=actual-predicted,max_lateral_deviation_m=lateral,actual_stop_time_s=float(grid[stop]-event_t))
   output.append(row)
 dest=args.directory/'braking_underprediction.json';dest.write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
if __name__=='__main__':main()

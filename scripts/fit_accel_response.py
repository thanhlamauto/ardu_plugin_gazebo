#!/usr/bin/env python3
"""Offline acceleration-memory surrogate fit, baseline seed7 training, seed17 holdout."""
import json
from pathlib import Path
import numpy as np

def load(seed):
 d=Path(f'output/benchmark/yard_free10_20260915_v1/v10.0_seed{seed}')
 r=[json.loads(l) for l in (d/'planner.jsonl').read_text().splitlines()]
 gt=[json.loads(l) for l in (d/'ground_truth.jsonl').read_text().splitlines()]
 wall=np.array([x['wall_monotonic_s'] for x in gt]);t=np.array([x['sim_time_s'] for x in gt]);p=np.array([x['position_enu'][:2] for x in gt])
 sent=np.array([np.interp(x['timestamp_monotonic_s'],wall,t) for x in r]);u=np.array([x['nominal_control'][:2] for x in r])
 grid=np.arange(sent[0],sent[-1],.1);fine=np.arange(t[0],t[-1],.02)
 xy=np.column_stack([np.interp(fine,t,p[:,i]) for i in range(2)]);vf=np.gradient(xy,.02,axis=0)
 v=np.column_stack([np.interp(grid,fine,vf[:,i]) for i in range(2)]);a=np.gradient(v,.1,axis=0)
 controls=u[np.clip(np.searchsorted(sent,grid,side='right')-1,0,len(u)-1)]
 return v,a,controls

def limit(x,b):return x*min(1.,b/max(np.linalg.norm(x),1e-12))
def score(data,tau,acc,jerk,mode):
 v,a,u=data;errors=[]
 for i in range(len(v)-10):
  pred=v[i].copy();mem=limit(a[i],acc)
  for k in range(10):
   if mode=='first':pred=pred+min(.1/tau,1)*(u[i+k]-pred)
   else:
    desired=limit((u[i+k]-pred)/tau,acc)
    mem=mem+limit(desired-mem,jerk*.1);pred=pred+mem*.1
  errors.append(np.sum((pred-v[i+10])**2))
 return float(np.sqrt(np.mean(errors)))
def main():
 train=load(7);test=load(17)
 candidates=[(score(train,t,a,j,'accel'),t,a,j) for t in [.3,.5,.8,1.,1.5] for a in [2.,3.,4.] for j in [2.,4.,6.,8.,12.]]
 best=min(candidates);_,t,a,j=best
 result={'fit_scope':'1s sliding-window XY velocity prediction; initial acceleration from ground truth differentiation; sent timestamps not receipt',
 'train_seed':7,'heldout_seed':17,'tau':t,'accel_xy':a,'jerk_xy':j,
 'rmse_m_s':{'train_first_tau05':score(train,.5,3,5,'first'),'heldout_first_tau05':score(test,.5,3,5,'first'),'train_accel':best[0],'heldout_accel':score(test,t,a,j,'accel')},
 'limitations':'correlated windows, only two baseline runs; flight estimator acceleration differs from ground truth; surrogate not exact ArduPilot model'}
 out=Path('output/benchmark/accel_response_fit_20260915');out.mkdir(exist_ok=True)
 (out/'fit.json').write_text(json.dumps(result,indent=2));(out/Path(__file__).name).write_bytes(Path(__file__).read_bytes());print(json.dumps(result,indent=2))
if __name__=='__main__':main()

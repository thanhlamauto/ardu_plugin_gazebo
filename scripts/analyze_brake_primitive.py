#!/usr/bin/env python3
"""Report one-sided brake distance and target echo latency from dedicated steps."""
import json,argparse
from pathlib import Path
import numpy as np
import os
import sys
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI

def surrogate_stop(speed, initial_acceleration, previous_command, delay):
 q=QuadMPPI(MPPIConfig(samples=4,horizon=2,response_accel_model=True,
     tau=.5,response_accel_xy=3.,response_jerk_xy=4.,dt=.1,vmax=10))
 state=q.torch.zeros(q.NX,dtype=q.torch.double)
 state[3]=speed;state[7]=previous_command;state[11]=initial_acceleration
 elapsed=0.
 while elapsed < delay-1e-9:
  dt=min(.1,delay-elapsed);q.cfg.dt=dt
  state=q._advance_applied(state,q.torch.tensor([previous_command,0.,0.,0.],dtype=q.torch.double))
  elapsed+=dt
 while elapsed < 20 and state[3]>.25:
  q.cfg.dt=.1;state=q._advance_applied(state,q.torch.zeros(4,dtype=q.torch.double));elapsed+=.1
 return float(state[0]),elapsed

def main():
 ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);args=ap.parse_args();root=args.directory
 trials=json.loads((root/'trials.json').read_text());events=[json.loads(x) for x in (root/'mavlink_events.jsonl').read_text().splitlines()];gt=[json.loads(x) for x in (root/'ground_truth.jsonl').read_text().splitlines()]
 wall=np.array([x['wall_monotonic_s'] for x in gt]);t=np.array([x['sim_time_s'] for x in gt]);p=np.array([x['position_enu'] for x in gt]);keep=np.r_[True,np.diff(t)>0];t,p=t[keep],p[keep]
 fine=np.arange(t[0],t[-1],.02);pos=np.column_stack([np.interp(fine,t,p[:,j]) for j in range(3)])
 # Central difference over .2 s; acceleration over .2 s to suppress sensor jitter.
 velocity=(np.roll(pos,-5,axis=0)-np.roll(pos,5,axis=0))/.2
 accel=(np.roll(velocity,-5,axis=0)-np.roll(velocity,5,axis=0))/.2
 out=[]
 for r in trials:
  sent=r['brake_send_wall_s'];t0=float(np.interp(sent,wall,t));i=int(np.searchsorted(fine,t0));i=max(i,10)
  v0=velocity[i,:2];speed=float(np.linalg.norm(v0));direction=v0/max(speed,1e-12)
  end_t=float(np.interp(r['stop_wall_s'],wall,t));end=min(len(fine)-11,int(np.searchsorted(fine,end_t)))
  stop=next((j for j in range(i+1,end) if all(np.linalg.norm(velocity[j+k,:2])<.25 for k in range(6))),None)
  if stop is None:stop=end;observed_stop=False
  else:observed_stop=True
  delta=pos[stop,:2]-pos[i,:2];actual=float(delta@direction);lateral=float(np.abs(direction[0]*(pos[i:stop+1,1]-pos[i,1])-direction[1]*(pos[i:stop+1,0]-pos[i,0])).max())
  predicted=speed*.25+speed*speed/6.
  echo=next((x for x in events if x['type']=='POSITION_TARGET_LOCAL_NED' and x['wall_monotonic_s']>=sent and abs(x['data'].get('vy',999))<.05),None)
  echo_latency=echo['wall_monotonic_s']-sent if echo else None
  before=float(np.median(accel[i-10:i,0]))
  model_distance,model_time=surrogate_stop(speed,before,r['speed_target_m_s'],.25)
  response=next((j for j in range(i,end-8) if np.median(accel[j:j+8,:2]@direction)<-.5),None)
  row={'speed_target_m_s':r['speed_target_m_s'],'repeat':r['repeat'],'actual_initial_speed_m_s':speed,'brake_send_sim_s':t0,'target_echo_latency_s':echo_latency,'acceleration_before_brake_m_s2':before,'first_deceleration_time_from_send_s':float(fine[response]-t0) if response is not None else None,'acceleration_onset_ambiguous':before<-.5,'predicted_stop_distance_guard_formula_m':predicted,'predicted_stop_distance_accel_memory_model_m':model_distance,'predicted_stop_time_accel_memory_model_s':model_time,'actual_stop_distance_m':actual,'guard_formula_underprediction_m':actual-predicted,'accel_memory_underprediction_m':actual-model_distance,'actual_stop_time_s':float(fine[stop]-t0),'stop_observed_below_025_m_s':observed_stop,'max_lateral_deviation_m':lateral}
  out.append(row)
 summary={}
 for speed in sorted(set(x['speed_target_m_s'] for x in out)):
  selected=[x for x in out if x['speed_target_m_s']==speed]
  def stats(key):
   values=np.asarray([x[key] for x in selected if x[key] is not None],dtype=float)
   return {'count':int(len(values)),'median':float(np.median(values)),
           'p95':float(np.percentile(values,95)),'min':float(values.min()),
           'max':float(values.max())}
  summary[str(speed)]={'trials':len(selected),
      'stops_observed':sum(x['stop_observed_below_025_m_s'] for x in selected),
      **{key:stats(key) for key in ('actual_initial_speed_m_s','actual_stop_distance_m',
          'guard_formula_underprediction_m','accel_memory_underprediction_m',
          'target_echo_latency_s','first_deceleration_time_from_send_s',
          'actual_stop_time_s','max_lateral_deviation_m')}}
 report={'method':'Position resampled .02 s; velocity and acceleration from central .2 s differences. Target echo is autopilot telemetry, not motor execution time. Guard formula v*.25+v²/(2*3); surrogate uses the same _advance_applied with observed prebrake acceleration, prior cruise command for .25 s, zero thereafter. This is an offline best-case initialization, not a live estimator.',
         'independence_limit':'Repetitions share one continuous Gazebo/SITL boot; they are repeated phases, not independent restarts.',
         'summary_by_requested_speed_m_s':summary,'rows':out}
 (root/'analysis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Protocol A/B/C analysis, unchanged controller. No new flight or RNG replay."""
import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import sys,json,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
import mppi_velocity_avoidance as cli
from mppi_ardupilot.mppi_controller import QuadMPPI
from mppi_ardupilot.trajectory_validation import validate_trajectory
from mppi_ardupilot.braking import stopping_segment_clearance

def read(p):return [json.loads(x) for x in p.read_text().splitlines()]
def command(r):
 s=r['sent_mavlink_control'];v=s['velocity_ned_m_s']
 return np.array([v[1],v[0],-v[2],-s['yaw_rate_ned_rad_s']])
def planner(root,config,seed):
 args=cli.build_parser().parse_args(['--config',str(root/config),'--seed',str(seed)])
 q=QuadMPPI(cli.build_cfg(args));meta=json.loads((root/'manifest.json').read_text())
 q.update_goal(meta['goal_enu']);q.update_reference_path(np.array(meta['reference_enu']));return q

def measured(rows,gt):
 wall=np.array([r['wall_monotonic_s'] for r in gt]);times=np.array([r['sim_time_s'] for r in gt]);p=np.array([r['position_enu'] for r in gt])
 keep=np.r_[True,np.diff(times)>0];times,p=times[keep],p[keep]
 grid=np.arange(times[0],times[-1],.1)
 pos=np.column_stack([np.interp(grid,times,p[:,j]) for j in range(3)])
 vel=np.gradient(pos,.1,axis=0);acc=np.gradient(vel,.1,axis=0)
 def sample(t):return [np.array([np.interp(t,grid,a[:,j]) for j in range(3)]) for a in (pos,vel,acc)]
 od=[r for r in rows if r.get('state_simulation_timestamp_s') is not None and r.get('velocity_enu') is not None]
 ot=np.array([r['state_simulation_timestamp_s'] for r in od]);ov=np.array([r['velocity_enu'] for r in od])
 def odom(t):return np.array([np.interp(t,ot,ov[:,j]) for j in range(3)])
 commands=[r for r in rows if r.get('sent_mavlink_control',{}).get('velocity_ned_m_s') is not None]
 send=np.array([np.interp(r['timestamp_monotonic_s'],wall,np.array([x['sim_time_s'] for x in gt])) for r in commands])
 u=np.array([command(r) for r in commands])
 return sample,odom,send,u,grid

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--straight',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
 out={'scope':'No controller tuning. Sent commands replayed without conditioner. Timestamp mapping uses GT wall/sim; receipt unknown. No sample-pool capture in historical logs.', 'model':[],'decision':[],'cruise_candidates':[]}
 root=ROOT/'output/benchmark/yard_progress_cruise10_20260916_v1'
 for seed in (7,17):
  q=planner(root,'mppi_yard_progress_cruise10.yaml',seed)
  rows=read(root/f'v10.0_seed{seed}/planner.jsonl');gt=read(root/f'v10.0_seed{seed}/ground_truth.jsonl')
  sample,odom,send,commands,grid=measured(rows,gt)
  available=[r for r in rows if r.get('nominal_snapshot')]
  seen=set()
  for x in (40,45,50,53):
   r=min(available,key=lambda r:abs(r['position_enu'][0]-x))
   if r['cycle'] in seen:continue
   seen.add(r['cycle']);snap=r['nominal_snapshot'];initial=np.array(snap['state']);t0=r['state_simulation_timestamp_s']
   if t0<grid[0] or t0+2>grid[-1]:continue
   q._last_state_np=initial.copy();q._path_progress_m=r['reference_progress_m'];q._update_reference_trajectory(initial[:3],initial[3:6]);q.update_obstacles(np.array(snap['obstacles_enu']));q.ctrl.U=q.torch.tensor(snap['raw_actions'],dtype=q.torch.double)
   nominal=[];state=q.torch.tensor(initial,dtype=q.torch.double)
   for i,u in enumerate(q.ctrl.U):state=q._dynamics(state,u,i);nominal.append(state.detach().numpy().copy())
   path=np.vstack((initial[:3],np.array(nominal)[:,:3]));nominal=np.array(nominal)
   out['decision'].append({'seed':seed,'cycle':r['cycle'],'requested_x':x,'event':r['event'],'predicted_min_stopping_cloud_clearance_m':min(stopping_segment_clearance(z[:3],z[3:6],snap['obstacles_enu'],q.cfg.max_accel_xy,.25)[0] for z in nominal),'state_time':t0,'position':initial[:3].tolist(),'raw_u0':snap['raw_actions'][0],'conditioned':r.get('conditioned_control'),'sent':command(r).tolist(),'recorded_gate':r.get('final_trajectory_validation'),'replayed_cloud_gate':validate_trajectory(path,snap['obstacles_enu'],q.cfg.collision_radius_m),'replayed_map_gate':q.known_geometry.validate(path,q.cfg.collision_radius_m),'cost':q.nominal_cost_breakdown(),'nominal_velocity_at_05_1_2s':[nominal[k,3:6].tolist() for k in (4,9,19)],'actual_velocity_at_05_1_2s':[sample(t0+h)[1].tolist() for h in (.5,1,2)],'future_events':sorted(set(z['event'] for z in rows if z.get('state_simulation_timestamp_s') is not None and t0<z['state_simulation_timestamp_s']<=t0+2))})
   for step in (.02,.1):
    for delay in (0.,.1,.2):
     state=q.torch.tensor(initial,dtype=q.torch.double);elapsed=0.;predpath=[initial[:3]];records=[]
     for horizon in (.5,1.,2.):
      while elapsed<horizon-1e-9:
       now=t0+elapsed;index=int(np.searchsorted(send+delay,now+1e-9,side='right')-1)
       applied=commands[index] if index>=0 else initial[7:11]
       next_event=(send[index+1]+delay-now) if index+1<len(send) else float('inf')
       dt=min(step,horizon-elapsed,next_event if next_event>1e-8 else step)
       q.cfg.dt=dt;state=q._advance_applied(state,q.torch.tensor(applied,dtype=q.torch.double));elapsed+=dt;predpath.append(state[:3].numpy().copy())
      pred=state.numpy();mp,mv,ma=sample(t0+horizon);mp0=sample(t0)[0]
      pv=pred[3:6];ang=None
      if min(np.linalg.norm(pv[:2]),np.linalg.norm(mv[:2]))>.5:
       a=np.arctan2(pv[1],pv[0])-np.arctan2(mv[1],mv[0]);ang=float(np.degrees(np.arctan2(np.sin(a),np.cos(a))))
      actualpath=np.array([sample(t)[0] for t in np.linspace(t0,t0+horizon,51)])
      records.append({'horizon_s':horizon,'predicted_position':pred[:3].tolist(),'measured_position_gt':mp.tolist(),'position_error_m':float(np.linalg.norm(pred[:3]-mp)),'displacement_error_m':float(np.linalg.norm((pred[:3]-initial[:3])-(mp-mp0))),'predicted_velocity':pv.tolist(),'measured_velocity_gt':mv.tolist(),'velocity_xy_error_m_s':float(np.linalg.norm(pv[:2]-mv[:2])),'velocity_xy_error_vs_odom_m_s':float(np.linalg.norm(pv[:2]-odom(t0+horizon)[:2])),'predicted_acceleration':pred[11:14].tolist(),'measured_acceleration_gt':ma.tolist(),'velocity_direction_error_deg':ang,'predicted_sampled_map_clearance_m':float(q.known_geometry.clearance(np.array(predpath)).min()),'measured_sampled_map_clearance_m':float(q.known_geometry.clearance(actualpath).min())})
     out['model'].append({'seed':seed,'cycle':r['cycle'],'requested_x':x,'actual_x':float(initial[0]),'max_step_s':step,'delay_s':delay,'samples':records})
   q.cfg.dt=.1
 if args.straight:
  root=args.straight
  for seed in (7,17):
   q=planner(root,'mppi_protocol_straight10.yaml',seed);rows=read(root/f'v10.0_seed{seed}/planner.jsonl')
   available=[r for r in rows if r.get('nominal_snapshot') and r['event']=='command' and 15<r['position_enu'][0]<260]
   # Report whether each selected snapshot is steady; do not zero acceleration memory.
   picked={}
   for target in (4,8,10):
    if not available:continue
    r=min(available,key=lambda r:abs(np.linalg.norm(r['velocity_enu'][:2])-target));picked[r['cycle']]=r
   for r in picked.values():
    snap=r['nominal_snapshot'];initial=np.array(snap['state']);q._last_state_np=initial.copy();q._path_progress_m=r['reference_progress_m'];q._update_reference_trajectory(initial[:3],initial[3:6]);q.update_obstacles(np.array(snap['obstacles_enu']))
    prior=[z for z in rows if z.get('state_simulation_timestamp_s') is not None and r['state_simulation_timestamp_s']-1<=z['state_simulation_timestamp_s']<=r['state_simulation_timestamp_s']]
    speeds=[np.linalg.norm(z['velocity_enu'][:2]) for z in prior if z.get('velocity_enu')]
    steady=len(speeds)>=5 and np.ptp(speeds)<.3 and np.linalg.norm(initial[11:13])<.3
    candidates=[]
    for target in (8,10):
     actions=np.tile([target,0,0,0],(q.cfg.horizon,1));q.ctrl.U=q.torch.tensor(actions,dtype=q.torch.double);state=q.torch.tensor(initial,dtype=q.torch.double)
     for i,u in enumerate(q.ctrl.U):state=q._dynamics(state,u,i)
     candidates.append({'raw_target':target,'terminal_velocity':state[3:6].tolist(),'cost':q.nominal_cost_breakdown()})
    out['cruise_candidates'].append({'seed':seed,'cycle':r['cycle'],'state':initial.tolist(),'steady_criterion_met':bool(steady),'previous_1s_speed_range':float(np.ptp(speeds)) if speeds else None,'candidates':candidates})
 args.output.mkdir(parents=True,exist_ok=True);(args.output/'analysis.json').write_text(json.dumps(out,indent=2));(args.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes());print(json.dumps({'model_windows':len(out['model']),'decision_states':len(out['decision']),'cruise_states':len(out['cruise_candidates'])}))
if __name__=='__main__':main()

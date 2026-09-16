#!/usr/bin/env python3
"""Frozen-state probe from recorded hold; not a new flight or exact RNG replay."""
import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE');os.environ.setdefault('OMP_NUM_THREADS','1')
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import mppi_velocity_avoidance as cli
from mppi_ardupilot.mppi_controller import QuadMPPI
from mppi_ardupilot.trajectory_validation import validate_trajectory

def main():
 root=Path('output/benchmark/yard_map_response10_20260915_v1')
 meta=json.loads((root/'manifest.json').read_text());out=[]
 for seed in [7,17]:
  rows=[json.loads(l) for l in (root/f'v10.0_seed{seed}/planner.jsonl').read_text().splitlines()]
  r=next(x for x in rows if x['cycle']==250);snap=r['nominal_snapshot']
  args=cli.build_parser().parse_args(['--config',str(root/'mppi_yard_map_response10.yaml')]);q=QuadMPPI(cli.build_cfg(args))
  q.update_goal(meta['goal_enu']);q.update_reference_path(np.array(meta['reference_enu']));q.update_obstacles(np.array(snap['obstacles_enu']))
  q._path_progress_m=r['reference_progress_m']
  q._last_state_np=np.array(snap['state']);q._update_reference_trajectory(q._last_state_np[:3],q._last_state_np[3:6])
  probes=[]
  for name,actions in [('logged',snap['raw_actions']),('zero',np.zeros((q.cfg.horizon,4))),('slow_forward',np.tile([1.,0,0,0],(q.cfg.horizon,1)))]:
   q.ctrl.U=q.torch.tensor(actions,dtype=q.torch.double)
   path=q.predict_trajectory();cost=q.nominal_cost_breakdown();cloud=validate_trajectory(path,snap['obstacles_enu'],q.cfg.collision_radius_m);known=q.known_geometry.validate(path,q.cfg.collision_radius_m)
   probes.append({'proposal':name,'nominal_cost':cost['total_nominal'],'collision_cost':cost['collision'],'map_valid':known['valid'],'cloud_valid':cloud['valid'],'endpoint':path[-1].tolist()})
  # Capture the initial proposal on two successive calls separated by rejection reset.
  starts=[]
  def capture(state):
   starts.append(q.ctrl.U.detach().cpu().numpy().copy());q.ctrl.U.zero_();return q.ctrl.U[0]
  q.ctrl.command=capture
  for _ in range(2):
   q.command(q._last_state_np[:3],q._last_state_np[3:6],q._last_state_np[6]);q.reset_applied_control()
  out.append({'seed':seed,'cycle':250,'position':r['position_enu'],'probes':probes,
   'initial_proposal_repeated_after_reset':bool(np.allclose(starts[0],starts[1])),
   'initial_raw_xy_speed':float(np.linalg.norm(starts[0][0,:2])),
   'note':'cost excludes MPPI sampling perturbation correction; safe short horizon is not a complete route'})
 dest=Path('output/benchmark/hold_loop_diagnosis_20260915');dest.mkdir(exist_ok=True)
 (dest/'probe.json').write_text(json.dumps(out,indent=2));(dest/Path(__file__).name).write_bytes(Path(__file__).read_bytes());print(json.dumps(out,indent=2))
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Deterministic implementation audit; synthetic probes, not flight validation."""
import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import sys,json,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import torch
from pytorch_mppi import mppi
from mppi_ardupilot.mppi_controller import MPPIConfig,QuadMPPI

def main():
    # Exercise installed optimizer update with controlled two-sample costs.
    c=mppi.MPPI(lambda x,u:x+u,lambda x,u:(x*x).sum(-1),2,
        noise_sigma=torch.eye(2,dtype=torch.double),num_samples=2,horizon=1,
        lambda_=1,device='cpu')
    c.U.zero_()
    def controlled_batch():
        c.noise=torch.tensor([[[1.,1.]],[[1.,-1.]]],dtype=torch.double)
        return torch.zeros(2,dtype=torch.double)
    c._compute_total_cost_batch=controlled_batch
    u=c.command(torch.zeros(2,dtype=torch.double),shift_nominal_trajectory=False)
    # Two line segments clear a disk centered (1,0), radius .4; average hits it.
    def clearance(end):
        end=np.asarray(end);obs=np.array([1.,0.]);t=np.clip(obs@end/(end@end),0,1)
        return float(np.linalg.norm(t*end-obs))
    samples=[clearance([1,1]),clearance([1,-1])];average=clearance(u.numpy())
    assert min(samples)>.4 and average<.4
    # Mirror feedback order in the real node: raw -> conditioner -> accept -> log.
    q=QuadMPPI(MPPIConfig(samples=4,horizon=2,vmax=10,command_alpha=.3,max_accel_xy=3))
    q._last_state_np=np.array([0,0,5,0,0,0,0,0,0,0,0],dtype=float)
    q.ctrl.U.zero_();q.ctrl.U[0,0]=10
    before=q.predict_trajectory()[1,0]
    sent=q._condition_action_torch(torch.zeros(4,dtype=torch.double),q.ctrl.U[0]).numpy()
    q.accept_applied_control(sent)
    after=q.predict_trajectory()[1,0]
    assert abs(sent[0]-.3)<1e-9 and abs(before-.006)<1e-9 and abs(after-before)<1e-9
    report={'kind':'synthetic implementation probes, not flight replay',
      'weighted_update':{'sample_segment_clearances':samples,'returned_control':u.tolist(),
          'returned_segment_clearance':average,'disk_radius':.4,
          'meaning':'weighted mean of feasible samples need not be feasible; no final filter in installed update'},
      'diagnostic_feedback':{'sent_x_m_s':float(sent[0]),'predicted_first_x_before_feedback_m':float(before),
          'logged_first_x_after_feedback_m':float(after),
          'meaning':'applied feedback leaves raw nominal diagnostics unchanged'},
      'files':{}}
    out=Path('output/benchmark/mppi_implementation_audit_fixed_20260915')
    out.mkdir(parents=True, exist_ok=True)
    for source in [Path(__file__),Path(mppi.__file__),Path('mppi_ardupilot/mppi_controller.py'),
                   Path('mppi_ardupilot/mppi_local_planner_node.py')]:
        report['files'][str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
        (out/source.name).write_bytes(source.read_bytes())
    (out/'probes.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()

import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import unittest
import numpy as np
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI
class ProgressTests(unittest.TestCase):
 def planner(self):
  q=QuadMPPI(MPPIConfig(samples=16,horizon=5,cost_profile='paper',path_progress_objective=True,w_progress=20,w_speed_limit=100,vmax=10,w_goal=0,w_terminal=0,w_path=4,w_reference_velocity=999,w_du=0,w_yaw=0,paper_r_u=(0,0,0,0),paper_r_delta_u=(0,0,0,0)))
  q.update_reference_path(np.array([[0.,0,5],[20.,0,5],[20.,20,5]]));q._update_reference_trajectory(np.array([0.,0,5]),np.zeros(3));q._last_state_np=np.array([0.,0,5,0,0,0,0,0,0,0,0]);return q
 def test_no_below_cap_or_timestamp_penalty(self):
  q=self.planner();t=q.torch
  for speed in (0.,4.,7.,10.):
   state=t.tensor([5.,0,5,speed,0,0,0,0,0,0,0],dtype=t.double)
   self.assertEqual(q._running_cost(state,t.zeros(4),0).item(),0)
   self.assertEqual(q._running_cost(state,t.zeros(4),4).item(),0)
  state[3]=11
  self.assertEqual(q._running_cost(state,t.zeros(4),0).item(),100)
 def test_arc_progress_bounded_and_terminal_reward(self):
  q=self.planner();t=q.torch
  p=t.tensor([[5.,0,5],[20.,5,5],[20.,30,5]],dtype=t.double)
  np.testing.assert_allclose(q._geometric_progress(p).numpy(),[5,25,40])
  state=t.tensor(q._last_state_np,dtype=t.double).repeat(1,5,1);state[0,-1,0]=5
  self.assertEqual(q._terminal_cost(state,t.zeros((1,5,4))).item(),-100)
 def test_diagnostics_match_rollout_objective(self):
  q=self.planner();q.command([0,0,5],[1,0,0],0)
  state=q.torch.tensor(q._last_state_np,dtype=q.torch.double);states=[];cost=0
  for i,a in enumerate(q.ctrl.U):
   state=q._dynamics(state,a,i);states.append(state);cost+=q._running_cost(state,a,i).item()
  cost+=q._terminal_cost(q.torch.stack(states).unsqueeze(0),q.ctrl.U.unsqueeze(0)).item()
  self.assertAlmostEqual(q.nominal_cost_breakdown()['total_nominal'],cost,places=7)

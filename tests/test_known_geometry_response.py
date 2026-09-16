import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import unittest
import numpy as np
from mppi_ardupilot.known_geometry import KnownGeometry
from mppi_ardupilot.mppi_controller import QuadMPPI,MPPIConfig
from mppi_ardupilot.mppi_local_planner_node import LocalPlannerNode,PlannerState

WORLD='worlds/iris_mppi_yard_runup60.sdf'
class MapResponseTests(unittest.TestCase):
 def test_map_detects_container_missing_from_cloud(self):
  q=QuadMPPI(MPPIConfig(samples=4,horizon=3,known_obstacle_sdf=WORLD,validate_final_trajectory=True))
  node=LocalPlannerNode(q,[np.array([77.1,-1.9,5])],hard_brake_m=0)
  q.predict_trajectory=lambda first_applied=None:np.array([[60,0,5],[68,0,5]])
  out=node.step(PlannerState(np.array([60,0,5]),np.zeros(3),0),np.array([[57,8,5]]))
  self.assertEqual(out.event,'hold-invalid-trajectory')
  self.assertEqual(out.diagnostics['final_trajectory_validation']['reason'],'known_map_collision')
 def test_map_cost_and_numpy_geometry_agree(self):
  import torch
  g=KnownGeometry(WORLD);p=np.array([[66.1,.1,5],[60,0,5],[0,0,5]],dtype=float)
  np.testing.assert_allclose(g.clearance(p),g.torch_clearance(torch.tensor(p)).numpy())
  q=QuadMPPI(MPPIConfig(samples=4,horizon=2,known_obstacle_sdf=WORLD))
  self.assertEqual(q._collision_indicator(torch.tensor(p)).tolist(),[1.,0.,0.])
 def test_accel_memory_preserves_initial_acceleration_during_brake(self):
  import torch
  q=QuadMPPI(MPPIConfig(samples=4,horizon=2,response_accel_model=True,response_jerk_xy=4))
  state=torch.zeros(q.NX,dtype=torch.double);state[3]=10;state[11]=3
  nxt=q._advance_applied(state,torch.zeros(4,dtype=torch.double))
  self.assertAlmostEqual(nxt[11].item(),2.6)
  self.assertAlmostEqual(nxt[3].item(),10.26)
 def test_observer_uses_timestamp_and_includes_holds(self):
  q=QuadMPPI(MPPIConfig(samples=4,horizon=2,response_accel_model=True))
  q.observe_motion([1,0,0],1);q.observe_motion([1.2,0,0],1.1)
  np.testing.assert_allclose(q._observed_acceleration,[2,0,0])
  q.observe_motion([9,0,0],3)
  np.testing.assert_allclose(q._observed_acceleration,0)
  self.assertEqual(q.command([0,0,5],[0,0,0],0).shape,(4,))
  self.assertEqual(q._last_state_np.shape,(14,))

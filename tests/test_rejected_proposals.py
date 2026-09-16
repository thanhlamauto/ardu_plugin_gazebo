import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE');os.environ.setdefault('OMP_NUM_THREADS','1')
import unittest
import numpy as np
from mppi_ardupilot.mppi_controller import MPPIConfig,QuadMPPI
class RejectionTests(unittest.TestCase):
 def test_repeated_rejection_keeps_progress_and_zero_memory(self):
  q=QuadMPPI(MPPIConfig(samples=8,horizon=5,vmax=10,reference_speed_m_s=10,reference_warm_start=True))
  q.update_reference_path(np.array([[0.,0,5],[100.,0,5]]))
  seen=[]
  def optimizer(state):
   seen.append(q.ctrl.U.clone());q.ctrl.U.fill_(.7);return q.ctrl.U[0]
  q.ctrl.command=optimizer
  q.reject_nominal();q.command([0,0,5],[0,0,0],0)
  np.testing.assert_allclose(seen[0].numpy(),0)
  q.reject_nominal();q.command([0,0,5],[0,0,0],0)
  np.testing.assert_allclose(seen[1].numpy(),.7)
  np.testing.assert_allclose(q._applied_control_np,0)
  proposals=q.recovery_proposals().numpy()
  np.testing.assert_allclose(proposals[:,0,0],[0,.5,1,2,4])
  self.assertEqual(q.cfg.reference_speed_m_s,10)
  q.reset_for_new_route();self.assertFalse(q._rejection_recovery)
 def test_real_sampler_includes_slow_proposals(self):
  q=QuadMPPI(MPPIConfig(samples=8,horizon=5,vmax=10,reference_speed_m_s=10))
  q.update_reference_path(np.array([[0.,0,5],[100.,0,5]]));q.reject_nominal()
  q.command([0,0,5],[0,0,0],0)
  np.testing.assert_allclose(q.ctrl.perturbed_action[:5].numpy(),q.recovery_proposals().numpy())

 def test_proactive_before_rejection_keeps_reference_and_slews_speed(self):
  q=QuadMPPI(MPPIConfig(samples=32,horizon=30,vmax=10,reference_speed_m_s=10,proactive_proposals=True))
  q.update_reference_path(np.array([[0.,0,5],[100.,0,5]]))
  q.command([0,0,5],[10,0,0],0)
  proposals=q.recovery_proposals().numpy()
  self.assertEqual(proposals.shape,(15,30,4))
  self.assertFalse(getattr(q,'_rejection_recovery',False))
  self.assertEqual(q.cfg.reference_speed_m_s,10)
  np.testing.assert_allclose(proposals[5,0,0],9.85)
  self.assertLessEqual(np.abs(np.diff(proposals[5,:,0])).max(),.15000001)
  np.testing.assert_allclose(q.ctrl.perturbed_action[:15].numpy(),proposals)

 def test_cost_buffer_does_not_change_gate_radius(self):
  q=QuadMPPI(MPPIConfig(samples=8,horizon=5,collision_radius_m=1.5,collision_cost_buffer_m=.55))
  q.obstacles=q.torch.tensor([[0.,0.,0.]],dtype=q.torch.double)
  point=q.torch.tensor([[1.8,0.,0.]],dtype=q.torch.double)
  self.assertEqual(q._collision_indicator(point).item(),1)
  self.assertEqual(q.cfg.collision_radius_m,1.5)
  q.cfg.collision_cost_buffer_m=0
  self.assertEqual(q._collision_indicator(point).item(),0)

 def test_proactive_proposals_follow_turn_without_retiming(self):
  q=QuadMPPI(MPPIConfig(samples=32,horizon=30,vmax=10,reference_speed_m_s=10,proactive_proposals=True))
  q.update_reference_path(np.array([[0.,0,5],[2.,0,5],[2.,20,5]]))
  q.command([0,0,5],[4,0,0],0)
  a=q.recovery_proposals().numpy()
  self.assertTrue(np.any(a[5:,:,1]>0))
  self.assertEqual(q.cfg.reference_accel_m_s2,0)
  self.assertTrue(np.isfinite(a).all())

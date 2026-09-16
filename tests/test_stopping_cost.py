import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'TRUE')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import unittest
import numpy as np
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI
from mppi_ardupilot.braking import stopping_segment_clearance

class StoppingCostTests(unittest.TestCase):
    def test_matches_cloud_stopping_geometry_and_direction(self):
        q = QuadMPPI(MPPIConfig(samples=8, horizon=5, max_accel_xy=3,
                               w_stopping=1, stopping_margin_m=2))
        points = np.array([[8., 1., 5.], [3., 4., 5.]])
        q.update_obstacles(points)
        for velocity in ([6.,0.,0.], [-6.,0.,0.], [0.,0.,0.]):
            distance, _, _ = stopping_segment_clearance([0,0,5],velocity,points,3,.25)
            p=q.torch.tensor([0.,0.,5.],dtype=q.torch.double)
            v=q.torch.tensor(velocity,dtype=q.torch.double)
            self.assertAlmostEqual(q._stopping_cost(p,v).item(),max(0,2-distance)**2)
    def test_empty_cloud_has_no_stopping_penalty(self):
        q=QuadMPPI(MPPIConfig(samples=8,horizon=5,w_stopping=1))
        p=q.torch.zeros((2,3),dtype=q.torch.double)
        v=q.torch.ones((2,3),dtype=q.torch.double)
        np.testing.assert_allclose(q._stopping_cost(p,v).numpy(),0)
    def test_running_cost_uses_stopping_term_on_scheduled_steps(self):
        q=QuadMPPI(MPPIConfig(samples=8,horizon=5,w_stopping=10,w_goal=0,
            w_obstacle=0,w_u=0,w_du=0,w_yaw=0,max_accel_xy=3))
        q.update_obstacles(np.array([[8.,1.,5.]]))
        state=q.torch.tensor([0.,0.,5.,6.,0.,0.,0.,0.,0.,0.,0.],dtype=q.torch.double)
        action=q.torch.zeros(4,dtype=q.torch.double)
        self.assertGreater(q._running_cost(state,action,0).item(),0)
        self.assertEqual(q._running_cost(state,action,1).item(),0)

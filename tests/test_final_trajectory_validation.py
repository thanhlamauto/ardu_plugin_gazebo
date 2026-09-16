import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import unittest
import numpy as np
from mppi_ardupilot.trajectory_validation import validate_trajectory
from mppi_ardupilot.mppi_controller import QuadMPPI, MPPIConfig
from mppi_ardupilot.mppi_local_planner_node import LocalPlannerNode, PlannerState, SAFE_HOLD_EVENTS

class FinalValidationTests(unittest.TestCase):
    def test_segment_catches_obstacle_between_clear_endpoints(self):
        self.assertFalse(validate_trajectory([[0,0,5],[2,0,5]],[[1,0,5]],.2)['valid'])
        self.assertTrue(validate_trajectory([[0,0,5],[2,0,5]],[[1,1,5]],.2)['valid'])
    def test_unknown_and_nonfinite_fail_closed(self):
        self.assertFalse(validate_trajectory([[0,0,0],[1,0,0]],[],.2)['valid'])
        self.assertFalse(validate_trajectory([[0,0,0],[np.nan,0,0]],[[1,0,0]],.2)['valid'])
    def test_feedback_preserves_raw_rollout_and_cost(self):
        q=QuadMPPI(MPPIConfig(samples=4,horizon=3,vmax=10))
        q.command([0,0,5],[0,0,0],0)
        before=q.predict_trajectory();cost=q.nominal_cost_breakdown();raw=q.ctrl.U.clone()
        q.accept_applied_control([.3,0,0,0])
        np.testing.assert_allclose(before,q.predict_trajectory())
        np.testing.assert_allclose(raw.numpy(),q.ctrl.U.numpy())
        self.assertEqual(cost,q.nominal_cost_breakdown())
        # Actual first command must bypass a second round of conditioning.
        checked=q.predict_trajectory(first_applied=[.3,0,0,0])
        self.assertAlmostEqual(checked[1,0],.006)
    def test_node_rejects_final_path_and_sends_hold(self):
        q=QuadMPPI(MPPIConfig(samples=4,horizon=3,validate_final_trajectory=True))
        node=LocalPlannerNode(q,[np.array([20,0,5])],hard_brake_m=0)
        q.predict_trajectory=lambda first_applied=None: np.array([[0,0,5],[4,0,5]])
        out=node.step(PlannerState(np.array([0,0,5]),np.zeros(3),0),np.array([[2,0,5]]))
        self.assertEqual(out.event,'hold-invalid-trajectory')
        self.assertIn(out.event,SAFE_HOLD_EVENTS)
        np.testing.assert_allclose(out.u,0)
        self.assertEqual(out.diagnostics['final_trajectory_validation']['reason'],'swept_collision')

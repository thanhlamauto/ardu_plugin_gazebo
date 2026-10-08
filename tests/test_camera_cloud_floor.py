import unittest
import numpy as np
from mppi_ardupilot.mppi_local_planner_node import LocalPlannerNode,PlannerState,VelocityCommandConditioner,SAFE_HOLD_EVENTS

class Planner:
    def __init__(self):self.calls=0;self.applied=np.zeros(4)
    def update_goal(self,g):self.goal=g
    def update_obstacles(self,p):self.points=p
    def command(self,*state):self.calls+=1;return np.array([1.,0,0,.1])
    def optimizer_diagnostics(self):return {'compute_ms':1.}
    def accept_applied_control(self,u):self.applied=u.copy()
    def reject_nominal(self):self.applied=np.zeros(4)

class CameraFloorTests(unittest.TestCase):
    def setUp(self):
        self.planner=Planner()
        self.conditioner=VelocityCommandConditioner(.2,.45,1.5,.8,1.2)
        self.node=LocalPlannerNode(self.planner,[np.array([12.,0,3])],min_obstacle_points=12,command_conditioner=self.conditioner)
        self.state=PlannerState(np.array([0.,0,3]),np.zeros(3),0.)
        self.cloud=np.column_stack((np.full(12,50.),np.linspace(-2,2,12),np.full(12,3.)))

    def test_sparse_after_motion_holds_and_resumes(self):
        moving=self.node.step(self.state,self.cloud)
        self.assertGreater(np.linalg.norm(moving.u),0)
        held=self.node.step(self.state,np.array([[7.1,-1.5,2.7]]))
        self.assertEqual(held.event,'hold-insufficient-cloud')
        self.assertIn(held.event,SAFE_HOLD_EVENTS)
        np.testing.assert_array_equal(held.u,np.zeros(4))
        np.testing.assert_array_equal(self.planner.applied,np.zeros(4))
        self.assertIsNone(self.conditioner.previous)
        self.assertEqual(self.planner.calls,1)
        resumed=self.node.step(self.state,self.cloud)
        self.assertGreater(np.linalg.norm(resumed.u),0)
        self.assertEqual(self.planner.calls,2)

    def test_empty_and_nonfinite_points_do_not_satisfy_floor(self):
        corrupt=np.vstack((self.cloud[:11],[np.nan,0,3],[np.inf,0,3]))
        for points,count in ((None,0),(np.empty((0,3)),0),(self.cloud[:1],1),(corrupt,11)):
            with self.subTest(count=count):
                out=self.node.step(self.state,points)
                self.assertEqual(out.event,'hold-insufficient-cloud')
                self.assertEqual(out.diagnostics['finite_points'],count)
                np.testing.assert_array_equal(out.u,np.zeros(4))
        self.assertEqual(self.planner.calls,0)

    def test_nonfinite_point_is_removed_when_enough_valid_points_exist(self):
        out=self.node.step(self.state,np.vstack((self.cloud,[np.nan,0,3])))
        self.assertGreater(np.linalg.norm(out.u),0)
        self.assertEqual(len(self.planner.points),12)
        self.assertTrue(np.isfinite(self.planner.points).all())

    def test_invalid_floor_is_rejected(self):
        for bad in (-1,1.5,True):
            with self.assertRaises(ValueError):LocalPlannerNode(Planner(),[np.array([12.,0,3])],min_obstacle_points=bad)

    def test_runtime_yaml_and_cli_precedence(self):
        from scripts.mppi_velocity_avoidance import build_parser,apply_runtime_config
        defaults=build_parser().parse_args([]);apply_runtime_config(defaults,{})
        self.assertEqual(defaults.min_obstacle_points,0)
        configured=build_parser().parse_args([]);apply_runtime_config(configured,{'min_obstacle_points':12})
        self.assertEqual(configured.min_obstacle_points,12)
        cli=build_parser().parse_args(['--min-obstacle-points','20']);apply_runtime_config(cli,{'min_obstacle_points':12})
        self.assertEqual(cli.min_obstacle_points,20)

if __name__=='__main__':unittest.main()

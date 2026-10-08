import unittest
import numpy as np
from mppi_ardupilot.monocular_evaluation import scene_clearance,gate_crossings
class SceneEvaluationTests(unittest.TestCase):
    def test_second_obstacle_and_rotation(self):
        boxes=[dict(center=[8,0,3],size=[2,3,6]),dict(center=[12,0,3],size=[2,3,6])]
        self.assertEqual(float(scene_clearance([12,0,3],boxes)),0.)
        self.assertAlmostEqual(float(scene_clearance([10,0,3],boxes)),1.)
        self.assertAlmostEqual(float(scene_clearance([0,3,0],[dict(center=[0,0,0],size=[2,4,2],yaw=np.pi/2)])),2.)
    def test_bad_size_rejected(self):
        with self.assertRaises(ValueError):scene_clearance([0,0,0],[dict(center=[0,0,0],size=[0,1,1])])

class GateEvaluationTests(unittest.TestCase):
    def test_goal_via_outside_does_not_count_as_gap(self):
        boxes=[dict(center=[8,-3.2,3],size=[2,3,6]),dict(center=[8,3.2,3],size=[2,3,6])]
        self.assertTrue(gate_crossings([[6,0,3],[10,0,3],[14,0,3]],boxes)['through_gap'])
        self.assertFalse(gate_crossings([[6,6,3],[10,6,3],[14,0,3]],boxes)['through_gap'])
        self.assertFalse(gate_crossings([[6,0,3],[7,0,3]],boxes)['through_gap'])

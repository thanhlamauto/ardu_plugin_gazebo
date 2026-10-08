import unittest
import numpy as np
from mppi_ardupilot.camera_motion_primitives import camera_motion_primitives
from mppi_ardupilot.mppi_local_planner_node import config_from_dict

class CameraPrimitiveTests(unittest.TestCase):
    def test_goal_rotation_speed_and_yaw_bounds(self):
        actions=camera_motion_primitives([2,3,3],np.pi-.1,[2,13,3],50,.2,.5,.6)
        self.assertEqual(actions.shape,(30,50,4))
        self.assertTrue(np.all(np.linalg.norm(actions[:,:,:2],axis=-1)<=.500001))
        self.assertTrue(np.all(actions[:,:,2]==0))
        self.assertTrue(np.all(abs(actions[:,:,3])<=.6))
        # Straight primitive rotates with the goal; it does not assume a world-X route.
        np.testing.assert_allclose(actions[4,0,:2],[0,.3],atol=1e-8)
    def test_side_then_forward_alternative_has_clearance_beyond_a_front_wall(self):
        # This is a proposal-coverage test, not a flight or safety certificate.
        actions=camera_motion_primitives([5,0,3],0,[12,0,3],50,.2,.5,.6)
        paths=np.cumsum(actions[:,:,:2],axis=1)*.2+np.array([5,0])
        delta=np.maximum(np.maximum(np.array([7,-1.5])-paths,paths-np.array([9,1.5])),0)
        safe=(np.linalg.norm(delta,axis=-1)>=1.25).all(axis=1)
        goal_progress=np.linalg.norm(paths[:,-1]-[12,0],axis=1)<7
        self.assertTrue(np.any(safe & goal_progress))
    def test_configuration_is_opt_in(self):
        self.assertFalse(config_from_dict({}).camera_primitives)
        self.assertTrue(config_from_dict({'camera_primitives':True}).camera_primitives)

if __name__=='__main__':unittest.main()

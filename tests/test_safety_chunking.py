import unittest
import numpy as np
from mppi_ardupilot.trajectory_safety import evaluate_trajectory_safety_batch

class SafetyChunkingTests(unittest.TestCase):
    def test_masks_preserved_for_mixed_trajectories(self):
        import torch
        torch.set_num_threads(2)
        rng=np.random.default_rng(91)
        states=np.zeros((160,51,11))
        states[:,:,:3]=rng.normal(size=(160,1,3))+np.cumsum(rng.normal(0,.08,(160,51,3)),axis=1)
        states[:,:,3:6]=rng.normal(0,.4,(160,51,3))
        states[80:,:,:3] += 10
        cloud=rng.normal(size=(200,3))*2
        kwargs=dict(collision_radius=.5,acceleration=1.5,delay=.25,stopping_clearance=.7)
        baseline=evaluate_trajectory_safety_batch(states,cloud,None,chunk_size=256,**kwargs)
        self.assertTrue(baseline['safe'].any());self.assertTrue((~baseline['safe']).any())
        for chunk in (1,16,32):
            result=evaluate_trajectory_safety_batch(states,cloud,None,chunk_size=chunk,**kwargs)
            for key in ('safe','cloud_safe','stopping_safe','map_safe'):
                self.assertTrue(torch.equal(result[key],baseline[key]),(chunk,key))
    def test_invalid_chunk_rejected(self):
        for chunk in (0,-1,True,1.5):
            with self.assertRaises(ValueError):
                evaluate_trajectory_safety_batch(np.zeros((1,2,11)),[[2,0,0]],None,collision_radius=.5,acceleration=1,delay=.2,stopping_clearance=.5,chunk_size=chunk)

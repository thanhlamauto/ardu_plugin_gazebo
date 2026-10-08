import unittest
import numpy as np
from mppi_ardupilot.monocular_scale_tracker import MonocularScaleTracker

class ScaleTrackerTests(unittest.TestCase):
    def test_metric_scale_from_translated_textured_plane_without_reference_depth(self):
        import cv2
        rng=np.random.default_rng(11)
        gray=rng.integers(0,256,(240,320),dtype=np.uint8)
        gray=cv2.GaussianBlur(gray,(3,3),0)
        rgb=np.repeat(gray[:,:,None],3,axis=2)
        # f=300, baseline lateral .5, plane camera-Z=5 -> pixel translation 30.
        moved=cv2.warpAffine(rgb,np.float32([[1,0,30],[0,1,0]]),(320,240))
        tracker=MonocularScaleTracker([[300,0,159.5],[0,300,119.5],[0,0,1]])
        depth=np.full((240,320),1.25)
        self.assertFalse(tracker.observe(rgb,depth,[0,0,3],np.eye(3))['accepted'])
        result=tracker.observe(moved,depth,[0,.5,3],np.eye(3))
        self.assertTrue(result['accepted'],result);self.assertAlmostEqual(result['scale'],4.,delta=.05)
    def test_blank_images_and_stationary_camera_provide_no_scale(self):
        tracker=MonocularScaleTracker(np.eye(3));rgb=np.zeros((100,100,3),np.uint8);depth=np.ones((100,100))
        tracker.observe(rgb,depth,[0,0,0],np.eye(3))
        self.assertEqual(tracker.observe(rgb,depth,[0,.01,0],np.eye(3))['reason'],'low-baseline')
        self.assertEqual(tracker.observe(rgb,depth,[0,.5,0],np.eye(3))['reason'],'no-texture')

if __name__=='__main__':unittest.main()

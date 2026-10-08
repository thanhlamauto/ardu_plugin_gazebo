import unittest
from unittest.mock import patch
import numpy as np
from mppi_ardupilot.monocular_point_tracker import MonocularPointTracker
from mppi_ardupilot.camera_local_map import CameraLocalMap

class PointTrackerTests(unittest.TestCase):
    def test_temporal_rgb_recovers_surface_distance_from_metric_motion(self):
        import cv2
        cv2.setNumThreads(1)
        rng=np.random.default_rng(17);gray=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),0)
        rgb=np.repeat(gray[:,:,None],3,axis=2)
        moved=cv2.warpAffine(rgb,np.float32([[1,0,30],[0,1,0]]),(320,240))
        tracker=MonocularPointTracker([[300,0,159.5],[0,300,119.5],[0,0,1]])
        first,_=tracker.observe(rgb,[0,0,3],np.eye(3));self.assertEqual(len(first),0)
        points,diag=tracker.observe(moved,[0,.5,3],np.eye(3))
        self.assertGreater(len(points),40,diag)
        self.assertAlmostEqual(float(np.median(points[:,0])),5.17,delta=.03)
        grid=CameraLocalMap(min_hits=1,altitude=3.)
        cloud=grid.update_world(points,1.)
        self.assertTrue(len(cloud)>10)
        self.assertTrue(np.all(abs(cloud[:,2]-3)<=.85))
        # World points are not translated by the camera mount a second time.
        self.assertAlmostEqual(float(np.median(cloud[:,0])),5.1,delta=.11)
    def test_textureless_translation_remains_unknown(self):
        tracker=MonocularPointTracker(np.eye(3));rgb=np.zeros((100,100,3),np.uint8)
        tracker.observe(rgb,[0,0,0],np.eye(3))
        points,diag=tracker.observe(rgb,[0,.5,0],np.eye(3))
        self.assertEqual(len(points),0);self.assertEqual(diag['reason'],'no-texture')

    def test_failed_triangulation_keeps_baseline_until_good_frame(self):
        import cv2
        cv2.setNumThreads(1)
        rng=np.random.default_rng(24)
        gray=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),0)
        rgb=np.repeat(gray[:,:,None],3,axis=2)
        shift=lambda px: cv2.warpAffine(rgb,np.float32([[1,0,px],[0,1,0]]),(320,240))
        tracker=MonocularPointTracker([[300,0,159.5],[0,300,119.5],[0,0,1]],min_baseline=.4)
        tracker.observe(rgb,[0,0,3],np.eye(3))
        with patch('mppi_ardupilot.monocular_point_tracker.triangulate_matches',
                   return_value=(np.empty((0,3)),np.zeros(1,dtype=bool))):
            points,diag=tracker.observe(shift(30),[0,.5,3],np.eye(3))
        self.assertEqual(len(points),0)
        self.assertFalse(diag['keyframe_promoted'])
        np.testing.assert_array_equal(tracker.keyframe[1],[0,0,3])
        points,diag=tracker.observe(shift(36),[0,.6,3],np.eye(3))
        self.assertGreaterEqual(diag['valid_points'],12)
        self.assertTrue(diag['keyframe_promoted'])
        self.assertAlmostEqual(diag['baseline_m'],.6)
        np.testing.assert_array_equal(tracker.keyframe[1],[0,.6,3])

if __name__=='__main__':unittest.main()

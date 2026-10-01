import unittest

import numpy as np

from mppi_ardupilot.monocular_multiframe_depth import (
    MultiFrameLandmarkTracker, camera_observation, estimate_inverse_depth,
)
from mppi_ardupilot.monocular_triangulation import CAMERA_OFFSET_FLU, OPTICAL_TO_FLU


K = np.array([[300., 0., 159.5], [0., 300., 119.5], [0., 0., 1.]])


def projected_observation(point, position):
    optical = OPTICAL_TO_FLU.T @ (np.asarray(point)-np.asarray(position)-CAMERA_OFFSET_FLU)
    uvw = K @ optical
    return camera_observation(uvw[:2]/uvw[2], position, np.eye(3), K)


class MultiFrameDepthTests(unittest.TestCase):
    def test_more_independent_views_reduce_inverse_depth_uncertainty(self):
        point = np.array([5.17, .25, 3.1])
        views = [projected_observation(point, [0., y, 3.]) for y in (0., .15, .3, .45, .6)]
        early = estimate_inverse_depth(views[:3], K, feature_id=17,
                                       pose_sigma_m=.05,max_depth_sigma_m=2.)
        late = estimate_inverse_depth(views, K, feature_id=17,
                                      pose_sigma_m=.05,max_depth_sigma_m=2.)
        self.assertIsNotNone(early)
        self.assertIsNotNone(late)
        np.testing.assert_allclose(late.point_enu, point, atol=.02)
        self.assertEqual(late.feature_id, 17)
        self.assertLess(late.inverse_depth_sigma_1_m, early.inverse_depth_sigma_1_m)
        self.assertLess(late.depth_sigma_m, early.depth_sigma_m)

    def test_stationary_views_do_not_create_depth(self):
        point = np.array([5.17, .25, 3.])
        views = [projected_observation(point, [0., 0., 3.]) for _ in range(5)]
        self.assertIsNone(estimate_inverse_depth(views, K))

    def test_pure_yaw_does_not_create_depth(self):
        point = np.array([5.17, .25, 3.])
        views = []
        for angle in (0., .05, .1, .15):
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            optical = OPTICAL_TO_FLU.T @ rotation.T @ (point-[0.,0.,3.]-CAMERA_OFFSET_FLU)
            uv = (K @ optical)[:2]/optical[2]
            views.append(camera_observation(uv,[0.,0.,3.],rotation,K))
        self.assertIsNone(estimate_inverse_depth(views,K))

    def test_inconsistent_pixel_is_rejected(self):
        point = np.array([5.17, .25, 3.])
        views = [projected_observation(point, [0., y, 3.]) for y in (0., .15, .3, .45)]
        views[-1].pixel += [12., 0.]
        self.assertIsNone(estimate_inverse_depth(views, K))

    def test_metric_uncertainty_gate_waits_for_usable_baseline(self):
        point = np.array([7., .25, 3.])
        near = [projected_observation(point,[0.,y,3.]) for y in (0.,.15,.3)]
        far = [projected_observation(point,[0.,y,3.]) for y in (0.,.75,1.5)]
        self.assertIsNone(estimate_inverse_depth(near,K))
        self.assertIsNotNone(estimate_inverse_depth(far,K))

    def test_persistent_klt_tracks_keep_depth_without_fake_precision_at_hover(self):
        import cv2
        cv2.setNumThreads(1)
        rng = np.random.default_rng(17)
        gray = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        rgb = np.repeat(gray[:, :, None], 3, axis=2)
        def shift(px):
            return cv2.warpAffine(rgb, np.float32([[1, 0, px], [0, 1, 0]]), (320, 240))
        tracker = MultiFrameLandmarkTracker(K, max_features=300,
                                            pose_sigma_m=.05,max_depth_sigma_m=2.,
                                            min_pose_step_m=.05)
        tracker.observe(rgb, [0., 0., 3.], np.eye(3))
        tracker.observe(shift(9), [0., .15, 3.], np.eye(3))
        landmarks, moving = tracker.observe(shift(18), [0., .3, 3.], np.eye(3))
        self.assertGreater(len(landmarks), 30, moving)
        depths = np.array([landmark.point_enu[0] for landmark in landmarks])
        self.assertAlmostEqual(float(np.median(depths)), 5.17, delta=.25)
        before = {item.feature_id: item.inverse_depth_sigma_1_m for item in landmarks}
        landmarks, hover = tracker.observe(shift(18), [0., .3, 3.], np.eye(3))
        self.assertEqual(hover['geometry_observations_added'], 0)
        for item in landmarks:
            if item.feature_id in before:
                self.assertEqual(item.inverse_depth_sigma_1_m, before[item.feature_id])

    def test_track_retains_anchor_after_observation_window_fills(self):
        import cv2
        rng = np.random.default_rng(23)
        gray = cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),0)
        rgb = np.repeat(gray[:,:,None],3,axis=2)
        tracker = MultiFrameLandmarkTracker(K,max_features=100,max_observations=4,
                                            pose_sigma_m=.05,max_depth_sigma_m=2.,
                                            min_pose_step_m=.05)
        for i in range(9):
            moved = cv2.warpAffine(rgb,np.float32([[1,0,i*4],[0,1,0]]),(320,240))
            tracker.observe(moved,[0.,i*.07,3.],np.eye(3))
        long_tracks = [track for track in tracker.tracks.values() if len(track.observations)==4]
        self.assertTrue(long_tracks)
        self.assertGreater(max(np.linalg.norm(t.observations[-1].center-t.observations[0].center)
                               for t in long_tracks),.5)


if __name__ == '__main__':
    unittest.main()

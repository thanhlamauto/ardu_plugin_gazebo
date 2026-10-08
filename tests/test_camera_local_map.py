import unittest

import numpy as np

from mppi_ardupilot.camera_local_map import CameraLocalMap


class CameraLocalMapTests(unittest.TestCase):
    def test_neighboring_voxel_evidence_confirms_one_obstacle(self):
        grid = CameraLocalMap(fusion_radius=.3, lifetime=3., publish_age=1.,
                              altitude=3., half_band=.65)
        first = np.array([[5.19, 0., 3.], [5.18, .01, 3.]])
        shifted = np.array([[5.22, 0., 3.]])
        self.assertEqual(len(grid.update_world(first, 1.)), 0)
        self.assertEqual(len(grid.update_world(shifted, 1.1)), 1)
        self.assertEqual(max(hits for _, hits in grid.voxels.values()), 2)

    def test_one_frame_cannot_confirm_duplicate_features(self):
        grid = CameraLocalMap(fusion_radius=.3, altitude=3., half_band=.65)
        points = np.array([[5.01, 0., 3.], [5.21, 0., 3.], [5.21, .01, 3.]])
        self.assertEqual(len(grid.update_world(points, 1.)), 0)
        self.assertTrue(all(hits == 1 for _, hits in grid.voxels.values()))

    def test_old_confirmed_obstacle_stops_publishing(self):
        grid = CameraLocalMap(fusion_radius=.3, lifetime=3., publish_age=1.,
                              altitude=3., half_band=.65)
        point = np.array([[5.01, 0., 3.]])
        grid.update_world(point, 1.)
        self.assertEqual(len(grid.update_world(point, 1.1)), 1)
        self.assertEqual(len(grid.update_world([], 2.2)), 0)
        self.assertEqual(len(grid.voxels), 1)
        self.assertEqual(len(grid.update_world([], 4.2)), 0)
        self.assertEqual(len(grid.voxels), 0)


if __name__ == '__main__':
    unittest.main()

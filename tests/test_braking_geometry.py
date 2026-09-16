import unittest
import numpy as np
from mppi_ardupilot.braking import stopping_segment_clearance


class BrakingGeometryTests(unittest.TestCase):
    def check(self, points, velocity=(10, 0, 0)):
        return stopping_segment_clearance([0, 0, 5], velocity, points, .6, .25)[0]

    def test_horizontal_flight_does_not_intersect_ground(self):
        self.assertAlmostEqual(self.check([[15, 0, 0], [25, 0, 0]]), 5)

    def test_forward_obstacle_is_seen_despite_closer_side_point(self):
        self.assertEqual(self.check([[0, 3, 5], [20, 0, 5]]), 0)

    def test_obstacle_beyond_stop_or_behind_not_on_segment(self):
        self.assertGreater(self.check([[100, 0, 5], [-10, 0, 5]]), 1.5)

    def test_descent_still_detects_ground(self):
        self.assertEqual(self.check([[0, 0, 0]], velocity=(0, 0, -5)), 0)

    def test_stationary_preserves_proximity(self):
        self.assertAlmostEqual(self.check([[.5, 0, 5]], velocity=(0, 0, 0)), .5)

    def test_empty_cloud(self):
        self.assertTrue(np.isinf(self.check([])))

import unittest
import numpy as np
from mppi_ardupilot.global_planner import AStarConfig, AStarGlobalPlanner, StaticObstacle2D
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI


class IndustrialAStarTests(unittest.TestCase):
    def test_thin_wall_segment_not_skipped(self):
        wall = StaticObstacle2D('thin', 'box', (.123, 0), 0, 8, half_size_xy=(.001, 1))
        planner = AStarGlobalPlanner([wall], AStarConfig(clearance_m=0))
        self.assertFalse(planner._line_free(np.array([0, 0]), np.array([1, 0]), [wall]))
        self.assertTrue(planner._line_free(np.array([0, 2]), np.array([1, 2]), [wall]))

    def test_same_cell_preserves_both_endpoints(self):
        planner = AStarGlobalPlanner([])
        result = planner.plan([0, 0, 5], [.01, 0, 5])
        np.testing.assert_allclose(result.path_enu[[0, -1]], [[0, 0, 5], [.01, 0, 5]])

    def test_industrial_smoothed_reference_clearance(self):
        planner = AStarGlobalPlanner.from_sdf('worlds/iris_mppi_industrial_yard.sdf', AStarConfig(clearance_m=2.6))
        result = planner.plan([0, 0, 5], [32, 0, 5])
        self.assertGreater(len(result.path_enu), 3)
        controller = QuadMPPI(MPPIConfig(samples=8, horizon=5,
            reference_corner_radius_m=.8, reference_corner_samples=6))
        controller.update_reference_path(result.path_enu)
        rounded = controller.reference_path.cpu().numpy()
        # Check the actual rounded polyline against safety radius, not A* inflation.
        safety = AStarGlobalPlanner(planner.obstacles, AStarConfig(clearance_m=1.5))
        for a, b in zip(rounded[:-1], rounded[1:]):
            self.assertTrue(safety._line_free(a[:2], b[:2], safety._active(5)))


if __name__ == '__main__':
    unittest.main()

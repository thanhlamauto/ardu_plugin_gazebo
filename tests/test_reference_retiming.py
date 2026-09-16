import unittest
import numpy as np
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI


class ReferenceRetimingTests(unittest.TestCase):
    def test_immediate_cruise_preserves_corner_and_terminal_braking(self):
        c = QuadMPPI(MPPIConfig(samples=4, horizon=30, vmax=10,
            reference_speed_m_s=10, reference_accel_m_s2=1.5,
            reference_lateral_accel_m_s2=1,
            reference_ramp_from_measured=False,
            reference_corner_radius_m=1.5, reference_corner_samples=12))
        c.update_reference_path(np.array([[0., 0, 5], [100., 0, 5], [100., 100, 5]]))
        c._update_reference_trajectory(np.array([0., 0, 5]), np.zeros(3))
        np.testing.assert_allclose(c.reference_velocities.cpu().numpy(),
                                   np.tile([10., 0, 0], (30, 1)), atol=1e-8)
        c._update_reference_trajectory(np.array([95., 0, 5]), np.zeros(3))
        speeds = np.linalg.norm(c.reference_velocities.cpu().numpy(), axis=1)
        self.assertGreater(speeds[0], 0)
        self.assertLess(speeds[0], 5)
        self.assertLess(np.min(speeds), speeds[0])
        c._update_reference_trajectory(np.array([100., 100, 5]), np.zeros(3))
        np.testing.assert_allclose(c.reference_velocities.cpu().numpy(), 0)

    def test_opt_in_reference_proposal_and_reset(self):
        c = QuadMPPI(MPPIConfig(samples=4, horizon=5, vmax=10,
            reference_speed_m_s=10, reference_warm_start=True))
        c.update_reference_path(np.array([[0., 0, 5], [300., 0, 5]]))
        captured = []
        def optimizer(state):
            captured.append(c.ctrl.U.cpu().numpy().copy())
            return c.ctrl.U[0]
        c.ctrl.command = optimizer
        c.command([0, 0, 5], [0, 0, 0], 0)
        np.testing.assert_allclose(captured[0][:, 0], 10)
        np.testing.assert_allclose(c.ctrl.u_init.cpu().numpy(), [10, 0, 0, 0])
        # Later optimization retains its sequence, rather than being erased.
        c.ctrl.U.fill_(.2)
        c.command([1, 0, 5], [1, 0, 0], 0)
        np.testing.assert_allclose(captured[-1], .2)
        c.reset_for_new_route()
        c.command([2, 0, 5], [1, 0, 0], 0)
        np.testing.assert_allclose(captured[-1][:, 0], 10)

    def test_acceleration_from_rest_and_terminal_braking(self):
        c = QuadMPPI(MPPIConfig(samples=4, horizon=30, vmax=10,
            reference_speed_m_s=10, reference_accel_m_s2=.6))
        c.update_reference_path(np.array([[0., 0, 5], [32., 0, 5]]))
        c._update_reference_trajectory(np.array([0., 0, 5]), np.zeros(3))
        v = c.reference_velocities.cpu().numpy()[:, 0]
        self.assertLessEqual(v[0], .060001)
        self.assertLessEqual(np.max(np.diff(v)), .060001)
        self.assertTrue(np.isfinite(v).all())
        c._update_reference_trajectory(np.array([32., 0, 5]), np.zeros(3))
        np.testing.assert_allclose(c.reference_velocities.cpu().numpy(), 0)

    def test_baseline_unchanged(self):
        c = QuadMPPI(MPPIConfig(samples=4, horizon=5, reference_speed_m_s=1.2))
        c.update_reference_path(np.array([[0., 0, 5], [32., 0, 5]]))
        c._update_reference_trajectory(np.array([0., 0, 5]))
        np.testing.assert_allclose(c.reference_velocities.cpu().numpy()[:, 0], 1.2)

    def test_invalid_budget(self):
        with self.assertRaises(ValueError):
            QuadMPPI(MPPIConfig(reference_accel_m_s2=float('nan')))

    def test_corner_and_endpoint_reduce_reference_speed(self):
        c = QuadMPPI(MPPIConfig(samples=4, horizon=30, vmax=10,
            reference_speed_m_s=10, reference_accel_m_s2=.6,
            reference_corner_radius_m=.8, reference_corner_samples=6))
        c.update_reference_path(np.array([[0., 0, 5], [10., 0, 5], [10., 10, 5]]))
        c._update_reference_trajectory(np.array([8., 0, 5]), np.array([5., 0, 0]))
        speed = np.linalg.norm(c.reference_velocities.cpu().numpy(), axis=1)
        self.assertLess(speed[0], 2.)
        self.assertTrue(np.isfinite(speed).all())
        c._update_reference_trajectory(np.array([10., 9.9, 5]), np.zeros(3))
        p = c.reference_positions.cpu().numpy()
        self.assertTrue(np.all(p[:, 1] <= 10.))
        self.assertTrue(np.all(np.diff(p[:, 1]) >= 0.))

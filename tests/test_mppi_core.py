import math
import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np

from mppi_ardupilot.lidar_preprocess import (
    body_frd_to_ned,
    enu_to_ned_vel,
    ned_to_enu_pos,
    quat_to_rot,
    yaw_enu_to_ned_rate,
    yaw_ned_to_enu,
)
from mppi_ardupilot.mavlink_interface import (
    ArduPilotInterface,
    TYPE_MASK_BODY_RATES_THRUST,
    TYPE_MASK_VEL_ONLY,
    TYPE_MASK_VEL_YAWRATE,
    check_mask_against_dialect,
    parse_state,
    validate_body_rate_thrust_mask,
)
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI
from mppi_ardupilot.mppi_local_planner_node import (
    LocalPlannerNode,
    PlannerState,
    TimingWindow,
    VelocityCommandConditioner,
    parse_goal,
)
from mppi_ardupilot.occupancy_grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid3D
from mppi_ardupilot.pa_mppi_controller import PAMPPIConfig, PerceptionAwareMPPI
from mppi_ardupilot.rigid_body_pa_mppi import (
    RigidBodyPAMPPI,
    RigidBodyPAMPPIConfig,
    body_flu_rates_to_frd,
)
from scripts.mppi_velocity_avoidance import apply_runtime_config, build_parser


class FakePlanner:
    def __init__(self):
        self.goal = None
        self.obstacles = None
        self.command_calls = 0

    def update_goal(self, goal):
        self.goal = np.asarray(goal)

    def update_obstacles(self, obstacles):
        self.obstacles = obstacles

    def command(self, pos, vel, yaw):
        self.command_calls += 1
        return np.array([1.0, 0.0, 0.0, 0.1])

    def optimizer_diagnostics(self):
        return {"compute_ms": 1.0, "ess": 5.0, "cost": {}}


class TestMavlinkAndFrames(unittest.TestCase):
    def test_velocity_masks(self):
        self.assertEqual(TYPE_MASK_VEL_YAWRATE, 1479)
        self.assertEqual(TYPE_MASK_VEL_ONLY, 3527)
        check_mask_against_dialect()
        self.assertEqual(TYPE_MASK_BODY_RATES_THRUST, 128)
        validate_body_rate_thrust_mask(TYPE_MASK_BODY_RATES_THRUST)
        for partial in (1, 2, 4, 3, 5, 6):
            with self.assertRaises(ValueError):
                validate_body_rate_thrust_mask(128 | partial)

    def test_attitude_target_body_rates_and_thrust(self):
        class FakeMav:
            def set_attitude_target_send(inner_self, *args):
                inner_self.args = args

        interface = object.__new__(ArduPilotInterface)
        fake_mav = FakeMav()
        interface.master = SimpleNamespace(
            mav=fake_mav, target_system=1, target_component=1
        )
        interface.send_attitude_target_body_rates(0.1, -0.2, 0.3, 1.2)
        self.assertEqual(fake_mav.args[3], TYPE_MASK_BODY_RATES_THRUST)
        self.assertEqual(fake_mav.args[4], [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(fake_mav.args[-1], 1.0)
        with self.assertRaises(ValueError):
            interface.send_attitude_target_body_rates(float("nan"), 0, 0, 0.5)
        with self.assertRaises(ValueError):
            interface.send_attitude_target_body_rates(0, 0, 0, float("inf"))

    def test_guid_options_safety_gate(self):
        class FakeMav:
            def param_request_read_send(inner_self, *args):
                inner_self.request = args

        class FakeMaster:
            target_system = 1
            target_component = 1
            mav = FakeMav()

            def __init__(self, value):
                self.value = value

            def recv_match(self, **kwargs):
                return SimpleNamespace(param_id="GUID_OPTIONS", param_value=self.value)

        interface = object.__new__(ArduPilotInterface)
        interface.master = FakeMaster(8.0)
        self.assertEqual(interface.require_parameter("GUID_OPTIONS", 8.0), 8.0)
        interface.master = FakeMaster(0.0)
        with self.assertRaises(SystemExit):
            interface.require_parameter("GUID_OPTIONS", 8.0)

    def test_body_flu_to_frd_rates(self):
        np.testing.assert_allclose(body_flu_rates_to_frd([1, 2, 3]), [1, -2, -3])

    def test_parse_state(self):
        lned = SimpleNamespace(x=1, y=2, z=-3, vx=4, vy=5, vz=-6)
        att = SimpleNamespace(yaw=0.7)
        pos, vel, yaw = parse_state(lned, att)
        np.testing.assert_allclose(pos, [1, 2, -3])
        np.testing.assert_allclose(vel, [4, 5, -6])
        self.assertAlmostEqual(yaw, 0.7)

    def test_enu_ned_roundtrip_helpers(self):
        self.assertEqual(enu_to_ned_vel([1, 2, 3]), (2.0, 1.0, -3.0))
        np.testing.assert_allclose(ned_to_enu_pos([2, 1, -3]), [1, 2, 3])
        self.assertAlmostEqual(yaw_ned_to_enu(0), math.pi / 2)
        self.assertAlmostEqual(yaw_enu_to_ned_rate(0.2), -0.2)
        np.testing.assert_allclose(quat_to_rot(0, 0, 0, 1), np.eye(3))
        vectors = np.array([[1.2, -3.4, 5.6], [-2.0, 0.5, -1.0]])
        for vector in vectors:
            ned = np.asarray(enu_to_ned_vel(vector))
            np.testing.assert_allclose(ned_to_enu_pos(ned), vector)
        # One physical positive ENU yaw rate is negative in NED and round-trips.
        self.assertAlmostEqual(-yaw_enu_to_ned_rate(0.37), 0.37)

    def test_body_frd_to_ned(self):
        points = body_frd_to_ned(np.array([[1.0, 0.0, 0.0]]), [10, 20, -3], math.pi / 2)
        np.testing.assert_allclose(points, [[10, 21, -3]], atol=1e-9)


class TestPlannerSafety(unittest.TestCase):
    def setUp(self):
        self.fake = FakePlanner()
        self.node = LocalPlannerNode(self.fake, [np.array([5.0, 0.0, 2.0])], hard_brake_m=1.0)
        self.state = PlannerState(np.array([0.0, 0.0, 2.0]), np.zeros(3), 0.0)

    def test_stale_state_holds(self):
        out = self.node.step(None, None)
        self.assertEqual(out.event, "hold-stale")
        np.testing.assert_allclose(out.u, 0)

    def test_near_obstacle_brakes(self):
        out = self.node.step(self.state, np.array([[0.5, 0.0, 2.0]]))
        self.assertEqual(out.event, "hold-brake")
        np.testing.assert_allclose(out.u, 0)

    def test_goal_reached_holds(self):
        state = PlannerState(np.array([4.5, 0.0, 2.0]), np.zeros(3), 0.0)
        out = self.node.step(state, None)
        self.assertEqual(out.event, "reached")
        np.testing.assert_allclose(out.u, 0)

        # Terminal is latched: drift after acceptance must not restart MPPI.
        drifted = PlannerState(np.array([7.0, 0.0, 2.0]), np.zeros(3), 0.0)
        out = self.node.step(drifted, None)
        self.assertEqual(out.event, "reached")
        np.testing.assert_allclose(out.u, 0)
        self.assertEqual(self.fake.command_calls, 0)

    def test_tight_goal_radius_does_not_stop_at_041_m(self):
        node = LocalPlannerNode(
            self.fake, [np.array([5.0, 0.0, 2.0])], goal_radius=0.25
        )
        state = PlannerState(np.array([4.59, 0.0, 2.0]), np.zeros(3), 0.0)
        out = node.step(state, None)
        self.assertNotEqual(out.event, "reached")
        np.testing.assert_allclose(out.u, [1, 0, 0, 0.1])

    def test_normal_command_has_diagnostics(self):
        out = self.node.step(self.state, None)
        np.testing.assert_allclose(out.u, [1, 0, 0, 0.1])
        self.assertEqual(out.diagnostics["ess"], 5.0)

    def test_planner_timeout_holds(self):
        node = LocalPlannerNode(
            self.fake, [np.array([5.0, 0.0, 2.0])], planner_timeout_ms=0.5
        )
        out = node.step(self.state, None)
        self.assertEqual(out.event, "hold-timeout")
        np.testing.assert_allclose(out.u, 0)

    def test_velocity_command_conditioner_limits_inter_cycle_change(self):
        conditioner = VelocityCommandConditioner(
            dt=0.1, alpha=1.0, max_accel_xy=1.0,
            max_accel_z=0.5, max_yaw_accel=2.0,
        )
        first = conditioner.apply([1.0, 1.0, 1.0, 1.0], np.zeros(3))
        self.assertLessEqual(np.linalg.norm(first[:2]), 0.1 + 1e-12)
        self.assertAlmostEqual(first[2], 0.05)
        self.assertAlmostEqual(first[3], 0.2)
        second = conditioner.apply([-1.0, -1.0, -1.0, -1.0], np.zeros(3))
        self.assertLessEqual(np.linalg.norm(second[:2] - first[:2]), 0.1 + 1e-12)
        self.assertLessEqual(abs(second[2] - first[2]), 0.05 + 1e-12)
        self.assertLessEqual(abs(second[3] - first[3]), 0.2 + 1e-12)

    def test_hard_brake_bypasses_and_resets_conditioner(self):
        conditioner = VelocityCommandConditioner(0.1, 1.0, 1.0, 1.0, 1.0)
        node = LocalPlannerNode(
            self.fake, [np.array([5.0, 0.0, 2.0])], hard_brake_m=1.0,
            command_conditioner=conditioner,
        )
        node.step(self.state, None)
        self.assertIsNotNone(conditioner.previous)
        out = node.step(self.state, np.array([[0.5, 0.0, 2.0]]))
        self.assertEqual(out.event, "hold-brake")
        np.testing.assert_allclose(out.u, 0)
        self.assertIsNone(conditioner.previous)


class TestOccupancyGrid(unittest.TestCase):
    def test_unknown_free_occupied_and_ray_status(self):
        grid = OccupancyGrid3D()
        grid.update_rays([0, 0, 0], np.array([[5.0, 0.0, 0.0]]))
        self.assertEqual(grid.lookup([[20, 20, 10]])[0], UNKNOWN)
        self.assertEqual(grid.lookup([[2, 0, 0]])[0], FREE)
        self.assertEqual(grid.lookup([[5, 0, 0]])[0], OCCUPIED)
        self.assertEqual(grid.first_nonfree_on_ray([0, 0, 0], [8, 0, 0]), OCCUPIED)
        self.assertEqual(grid.first_nonfree_on_ray([0, 0, 0], [0, 8, 0]), UNKNOWN)
        self.assertFalse(grid.line_of_sight([0, 0, 0], [8, 0, 0]))
        self.assertTrue(grid.line_of_sight([0, 0, 0], [3, 0, 0]))

    def test_ray_traversal_and_unknown_collision_policy(self):
        grid = OccupancyGrid3D()
        grid.update_rays([0, 0, 0], np.array([[4.0, 0.0, 0.0]]))
        statuses = grid.lookup([[1.0, 0, 0], [4.0, 0, 0], [12.0, 0, 0]])
        np.testing.assert_array_equal(statuses, [FREE, OCCUPIED, UNKNOWN])
        self.assertTrue(np.all(statuses[1:] != FREE))


class TestDiagnostics(unittest.TestCase):
    def test_timing_window(self):
        timing = TimingWindow(deadline_ms=10, window=3)
        for value in (5, 8, 12, 7):
            timing.add(value)
        result = timing.summary()
        self.assertAlmostEqual(result["mean_ms"], 9.0)
        self.assertEqual(result["deadline_misses"], 1)
        self.assertEqual(result["samples"], 4)

    def test_real_mppi_exposes_rollouts_and_costs(self):
        cfg = MPPIConfig(horizon=5, samples=32, device="cpu")
        planner = QuadMPPI(cfg)
        planner.update_goal([3, 0, 2])
        planner.update_obstacles(np.array([[1.5, 3.0, 2.0]]))
        action = planner.command([0, 0, 2], [0, 0, 0], 0)
        self.assertEqual(action.shape, (4,))
        self.assertEqual(planner.predict_trajectory().shape, (6, 3))
        self.assertEqual(planner.sampled_trajectories(3).shape, (3, 6, 3))
        diagnostics = planner.optimizer_diagnostics()
        self.assertGreater(diagnostics["compute_ms"], 0)
        self.assertGreater(diagnostics["ess"], 0)
        self.assertIn("terminal", diagnostics["cost"])

    def test_seeded_rollout_is_deterministic_and_finite(self):
        cfg = MPPIConfig(horizon=5, samples=32, device="cpu", seed=19)
        first = QuadMPPI(cfg)
        first.update_goal([3, 0, 2])
        u_first = first.command([0, 0, 2], [0, 0, 0], 0)
        second = QuadMPPI(cfg)
        second.update_goal([3, 0, 2])
        u_second = second.command([0, 0, 2], [0, 0, 0], 0)
        self.assertEqual(u_first.shape, (4,))
        self.assertTrue(np.isfinite(u_first).all())
        np.testing.assert_allclose(u_first, u_second, atol=0, rtol=0)
        with self.assertRaises(ValueError):
            second.command([float("nan"), 0, 2], [0, 0, 0], 0)

    def test_pa_mppi_fuses_map_and_reports_perception_state(self):
        cfg = PAMPPIConfig(horizon=5, samples=32, device="cpu")
        planner = PerceptionAwareMPPI(cfg)
        planner.update_goal([3, 0, 2])
        planner.update_occupancy([0, 0, 2], np.array([[1.5, 2.0, 2.0]]))
        planner.update_obstacles(np.array([[1.5, 2.0, 2.0]]))
        action = planner.command([0, 0, 2], [0, 0, 0], 0)
        diagnostics = planner.optimizer_diagnostics()
        self.assertEqual(action.shape, (4,))
        self.assertEqual(diagnostics["planner"], "pa-mppi")
        self.assertGreater(diagnostics["mapped_fraction"], 0)
        self.assertIn("goal_visible", diagnostics)
        self.assertIn("collision", diagnostics["cost"])
        self.assertIn("perception", diagnostics["cost"])

    def test_pa_perception_reward_and_baseline_dynamics_match(self):
        import torch

        base = QuadMPPI(MPPIConfig(horizon=4, samples=16, device="cpu", seed=3))
        pa = PerceptionAwareMPPI(
            PAMPPIConfig(horizon=4, samples=16, device="cpu", seed=3)
        )
        state = torch.tensor([0, 0, 2, 0.1, 0, 0, 0], dtype=torch.double)
        action = torch.tensor([1, 0, 0, 0.1], dtype=torch.double)
        np.testing.assert_allclose(
            base._dynamics(state, action).numpy(), pa._dynamics(state, action).numpy()
        )
        pa.update_goal([6, 0, 2])
        pa.update_occupancy([0, 0, 2], np.array([[2, 2, 2]]))
        states = torch.zeros((1, 4, 7), dtype=torch.double)
        states[..., 2] = 2
        # Unknown endpoint receives the configured negative exploration term.
        terminal = float(pa._terminal_cost(states, None).item())
        old_reward = pa.pa_cfg.w_pa_unknown
        pa.pa_cfg.w_pa_unknown = 0.0
        without_reward = float(pa._terminal_cost(states, None).item())
        self.assertLess(terminal, without_reward)
        pa.pa_cfg.w_pa_unknown = old_reward

    def test_rigid_body_hover_and_optimizer(self):
        cfg = RigidBodyPAMPPIConfig(horizon=5, samples=32, device="cpu")
        planner = RigidBodyPAMPPI(cfg)
        state = planner.torch.tensor(
            [0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            dtype=planner.torch.double,
        )
        hover = planner.torch.tensor(
            [cfg.hover_thrust_n, 0, 0, 0], dtype=planner.torch.double
        )
        next_state = planner._dynamics(state, hover)
        np.testing.assert_allclose(next_state.detach().numpy(), state.numpy(), atol=1e-12)
        self.assertAlmostEqual(
            planner.thrust_newtons_to_normalized(cfg.hover_thrust_n),
            cfg.hover_thrust_normalized,
        )

        planner.update_goal([2, 0, 2])
        planner.update_occupancy([0, 0, 2], np.empty((0, 3)))
        control = planner.command_rigid(
            [0, 0, 2], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0]
        )
        self.assertEqual(control.shape, (4,))
        self.assertGreaterEqual(control[0], cfg.u_min()[0])
        self.assertLessEqual(control[0], cfg.u_max()[0])
        rigid_diagnostics = planner.optimizer_diagnostics()
        self.assertEqual(rigid_diagnostics["planner"], "rigid-pa-mppi")
        self.assertIn("terminal", rigid_diagnostics["cost"])
        self.assertIn("collision", rigid_diagnostics["cost"])
        self.assertIn("perception", rigid_diagnostics["cost"])
        with self.assertRaises(ValueError):
            planner.command_rigid(
                [float("nan"), 0, 2], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0]
            )

    def test_rigid_closed_loop_clear_air_progress(self):
        import torch

        torch.manual_seed(7)
        cfg = RigidBodyPAMPPIConfig(
            horizon=15, samples=128, dt=0.05, device="cpu", w_pa_unknown=0.0
        )
        planner = RigidBodyPAMPPI(cfg)
        goal = np.array([1.0, 0.0, 2.0])
        planner.update_goal(goal)
        state = np.array(
            [0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=float
        )
        altitudes = []
        for _ in range(80):
            planner.update_occupancy(state[0:3], np.empty((0, 3)))
            control = planner.command_rigid(
                state[0:3], state[3:7], state[7:10], state[10:13]
            )
            state = planner._dynamics(
                torch.tensor(state, dtype=torch.double),
                torch.tensor(control, dtype=torch.double),
            ).detach().numpy()
            altitudes.append(state[2])
        self.assertLess(np.linalg.norm(state[0:3] - goal), 0.5)
        self.assertGreater(min(altitudes), 1.5)
        self.assertLess(max(altitudes), 2.5)


class TestCliParsing(unittest.TestCase):
    def test_route(self):
        route = parse_goal("1,2,3;4,5,6")
        self.assertEqual(len(route), 2)
        np.testing.assert_allclose(route[1], [4, 5, 6])

    def test_yaml_runtime_values_and_cli_precedence(self):
        args = build_parser().parse_args([])
        apply_runtime_config(args, {"hz": 7.0, "goal": "1,2,3", "max_points": 99})
        self.assertEqual(args.hz, 7.0)
        self.assertEqual(args.goal, "1,2,3")
        self.assertEqual(args.max_points, 99)

        explicit = build_parser().parse_args(["--hz", "12"])
        apply_runtime_config(explicit, {"hz": 7.0})
        self.assertEqual(explicit.hz, 12.0)


if __name__ == "__main__":
    unittest.main()

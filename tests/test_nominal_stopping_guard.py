import os
os.environ.setdefault('KMP_DUPLICATE_LIB_OK','TRUE')
os.environ.setdefault('OMP_NUM_THREADS','1')
import json,unittest
from pathlib import Path
import numpy as np
from mppi_ardupilot.braking import validate_stopping_states
from mppi_ardupilot.mppi_controller import MPPIConfig,QuadMPPI
from mppi_ardupilot.mppi_local_planner_node import LocalPlannerNode,PlannerState,VelocityCommandConditioner,SAFE_HOLD_EVENTS
from mppi_ardupilot.known_geometry import KnownGeometry

ROOT=Path(__file__).resolve().parents[1]
class NominalStopGuardTests(unittest.TestCase):
 def test_hard_guard_checks_every_state(self):
  states=np.array([[0.,0,5.,0.,0,0],[1.,0,5.,6.,0,0],[2.,0,5.,0.,0,0]])
  points=np.array([[5.,0.,5.]])
  result=validate_stopping_states(states,points,None,3,.25,1.5)
  self.assertFalse(result['valid']);self.assertEqual(result['first_failing_step'],1)
  self.assertEqual(result['checked_states'],3)
 def test_known_map_segment_between_sampled_points(self):
  g=KnownGeometry(ROOT/'worlds/iris_mppi_yard_runup60.sdf')
  states=np.array([[54.,0.,5.,6.5,0,0]])
  result=validate_stopping_states(states,np.array([[0.,20.,5.]]),g,3,.25,1.5)
  self.assertFalse(result['valid']);self.assertTrue(result['map_checked'])
 def test_seed7_cycle70_regression_rejects_old_sent_nominal(self):
  root=ROOT/'output/benchmark/yard_progress_cruise10_20260916_v1'
  meta=json.loads((root/'manifest.json').read_text())
  row=next(json.loads(x) for x in (root/'v10.0_seed7/planner.jsonl').read_text().splitlines() if json.loads(x)['cycle']==70)
  self.assertEqual(row['event'],'command')
  snap=row['nominal_snapshot']
  from scripts.mppi_velocity_avoidance import build_cfg,build_parser
  args=build_parser().parse_args(['--config',str(ROOT/'config/experiments/mppi_yard_progress_stopguard10.yaml'),'--seed','7'])
  q=QuadMPPI(build_cfg(args));q.update_goal(meta['goal_enu'])
  path=np.array(meta['reference_enu']);q.update_reference_path(path)
  state=np.array(snap['state']);q._last_state_np=state.copy();q._observed_acceleration=state[11:14].copy()
  q.ctrl.U=q.torch.tensor(snap['raw_actions'],dtype=q.torch.double)
  q.command=lambda pos,vel,yaw:np.array(row['raw_control'])
  conditioner=VelocityCommandConditioner(q.cfg.dt,q.cfg.command_alpha,q.cfg.max_accel_xy,q.cfg.max_accel_z,q.cfg.max_yaw_accel,u_min=q.cfg.u_min(),u_max=q.cfg.u_max())
  conditioner.previous=state[7:11].copy()
  node=LocalPlannerNode(q,[np.array(meta['goal_enu'])],hard_brake_m=1.5,hard_brake_release_m=2.,hard_brake_delay_s=.25,brake_accel_m_s2=3.,brake_swept_path=True,reference_path=path,command_conditioner=conditioner)
  cloud=np.array(snap['obstacles_enu'])
  observed=PlannerState(state[:3],state[3:6],float(state[6]),simulation_timestamp_s=row['state_simulation_timestamp_s'])
  out=node.step(observed,cloud)
  self.assertEqual(out.event,'hold-invalid-stopping-trajectory')
  self.assertIn(out.event,SAFE_HOLD_EVENTS)
  np.testing.assert_allclose(out.u,0)
  self.assertEqual(out.diagnostics['final_trajectory_validation']['reason'],'predicted_stopping_clearance')
  self.assertTrue(out.diagnostics['final_trajectory_validation']['known_map']['valid'])
  np.testing.assert_allclose(out.diagnostics['conditioned_control'],row['conditioned_control'],atol=1e-9)
  self.assertLess(out.diagnostics['final_trajectory_validation']['stopping']['min_clearance_m'],1.5)

class FeasibleWeightingRegressionTests(unittest.TestCase):
 def test_cycle160_safe_samples_receive_all_weight_and_output_is_safe(self):
  snapshot=ROOT/'output/benchmark/yard_stopguard_pool_cycle160_20260916_v1/v10.0_seed7/mppi_sample_pool_cycle160.npz'
  with np.load(snapshot,allow_pickle=False) as data:
   saved={key:data[key] for key in data.files}
  from scripts.mppi_velocity_avoidance import build_cfg,build_parser
  args=build_parser().parse_args(['--config',str(ROOT/'config/experiments/mppi_yard_progress_feasible10.yaml'),'--seed','7'])
  q=QuadMPPI(build_cfg(args));q.update_goal(saved['goal_enu']);q.update_reference_path(saved['reference_path_enu'])
  q.update_obstacles(saved['obstacle_cloud'])
  q.configure_trajectory_safety(collision_radius=1.5,acceleration=3.,delay=.25,stopping_clearance=1.5)
  x0=saved['initial_state'];q._applied_control_np=x0[7:11].copy();q._observed_acceleration=x0[11:14].copy()
  states=q.torch.as_tensor(saved['predicted_states'],dtype=q.torch.double)
  actions=q.torch.as_tensor(saved['raw_actions'],dtype=q.torch.double)
  costs=q.torch.as_tensor(saved['total_sample_cost'],dtype=q.torch.double)
  def controlled_command(state):
   q.ctrl.states=states;q.ctrl.perturbed_action=actions;q.ctrl.cost_total=costs
   q.ctrl._compute_weighting(costs)
   unsafe=int(q.torch.argmin(costs).item())
   q.ctrl.U.copy_(actions[unsafe])
   return q.ctrl.U[0]
  q.ctrl.command=controlled_command
  returned=q.command(x0[:3],x0[3:6],float(x0[6]))
  mask=q._last_sample_feasible_mask.detach().cpu().numpy()
  self.assertEqual(int(mask.sum()),148)
  self.assertAlmostEqual(q._last_safe_weight_mass,1.,places=12)
  self.assertTrue(mask[int(q.torch.argmax(q.ctrl.omega).item())])
  # The final gate requests this fallback only if the weighted nominal fails.
  fallback=q.select_best_feasible_sample();self.assertIsNotNone(fallback)
  returned=fallback[1]
  self.assertIsNotNone(q._selected_feasible_sample_index)
  self.assertTrue(mask[q._selected_feasible_sample_index])
  np.testing.assert_allclose(returned,states[0,q._selected_feasible_sample_index,0,7:11].numpy())
  self.assertTrue(bool(q._evaluate_safety_states(q.predict_states(first_applied=returned))['safe'][0].item()))

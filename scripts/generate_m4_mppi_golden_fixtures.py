#!/usr/bin/env python3
"""Generate deterministic M4 cost/update fixtures from the Python baseline."""
import json
from pathlib import Path

import numpy as np
import torch

from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI


OUT = (Path(__file__).resolve().parents[1] / "uav_navigation_core" / "test" /
       "fixtures" / "mppi_m4")
TERMS = ("goal", "obstacle", "collision", "stopping", "effort",
         "smoothness", "yaw", "path", "reference_velocity", "speed_limit",
         "terminal", "progress", "input_change")
COMMON = dict(
    dt=.1, tau=.5, horizon=6, samples=4, lambda_=1.3,
    vmax=5., vzmax=1., yaw_rate_max=.6,
    noise_xy=.8, noise_z=.3, noise_yaw=.3,
    margin=2., w_goal=1.2, w_terminal=4., w_obstacle=30., w_u=.07,
    w_stopping=2.5, stopping_margin_m=1.5, stopping_delay_s=.25,
    w_du=.13, w_yaw=.17, w_progress=2.2, w_speed_limit=3.1,
    w_path=1.7, path_scale_m=1.2, w_reference_velocity=.8,
    reference_speed_m_s=3., w_collision=500., collision_radius_m=.6,
    collision_cost_buffer_m=.1,
    paper_r_u=(.01, .05, .05, .10),
    paper_r_delta_u=(.05, .10, .10, .30),
    command_alpha=.45, max_accel_xy=1.5, max_accel_z=.8,
    max_yaw_accel=1.2, response_accel_model=True,
    response_accel_xy=3., response_jerk_xy=5., device="cpu")


def flat_controls(values):
    return np.asarray(values, dtype=np.float64).reshape(-1).tolist()


def state():
    return np.asarray([.5, -.4, 2., 1.1, -.2, .1, .3,
                       .8, -.1, .05, .1, .2, -.1, .03], dtype=np.float64)


def controls(horizon=6):
    base = np.asarray([[2.5, .3, .2, .3], [3., .8, .1, .4],
                       [3.5, 1.2, 0., .5], [2.8, 1.8, -.1, .2],
                       [2.2, 2., -.2, -.2], [1.5, 2.2, 0., -.4]])
    return base[:horizon].copy()


def rollout(planner, initial, actions):
    current = torch.as_tensor(initial, dtype=torch.double)
    states = []
    for t, action in enumerate(actions):
        current = planner._dynamics(current, torch.as_tensor(action), t)
        states.append(current)
    return torch.stack(states)


def breakdown(planner, initial, states, actions):
    cfg = planner.cfg
    result = {name: 0. for name in TERMS}
    for t in range(len(actions)):
        s, action = states[t], torch.as_tensor(actions[t], dtype=torch.double)
        p, v, yaw = s[:3], s[3:6], s[6]
        result["goal"] += cfg.w_goal * torch.linalg.vector_norm(p-planner.goal).item()
        if cfg.cost_profile == "paper":
            result["collision"] += cfg.w_collision * planner._collision_indicator(p).item()
            feasible = s[7:11]
            result["effort"] += (feasible.square()*planner._paper_r_u).sum().item()
        else:
            result["obstacle"] += cfg.w_obstacle * planner._obstacle_cost(p).item()
            result["effort"] += cfg.w_u * action[:3].square().sum().item()
        if cfg.w_stopping and t % 5 == 0:
            result["stopping"] += cfg.w_stopping*planner._stopping_cost(p, v).item()
        result["smoothness"] += cfg.w_du*(action[:3]-v).square().sum().item()
        desired = torch.atan2(action[1], action[0])
        yaw_error = planner.wrap_angle(desired-yaw)
        result["yaw"] += cfg.w_yaw*torch.linalg.vector_norm(action[:2]).item()*yaw_error.item()**2
        if (not cfg.path_progress_objective and cfg.cost_profile == "paper"
                and planner.reference_positions is not None):
            result["path"] += cfg.w_path*(((p-planner.reference_positions[t])/cfg.path_scale_m).square().sum()).item()
            result["reference_velocity"] += cfg.w_reference_velocity*((v-planner.reference_velocities[t]).square().sum()).item()
        else:
            result["path"] += cfg.w_path*(planner._path_distance(p)/cfg.path_scale_m).square().item()
        if cfg.path_progress_objective:
            result["speed_limit"] += cfg.w_speed_limit*max(0., torch.linalg.vector_norm(v[:2]).item()-cfg.vmax)**2
    terminal_ref = (planner.reference_positions[-1]
                    if not cfg.path_progress_objective and cfg.cost_profile == "paper"
                    and planner.reference_positions is not None else planner.goal)
    result["terminal"] = cfg.w_terminal*torch.linalg.vector_norm(states[-1,:3]-terminal_ref).item()**2
    if cfg.path_progress_objective:
        start = torch.as_tensor(initial[:3], dtype=torch.double)
        result["progress"] = -cfg.w_progress*(planner._geometric_progress(states[-1,:3])-planner._geometric_progress(start)).item()
    if cfg.cost_profile == "paper":
        feasible = states[:, 7:11]
        previous = torch.as_tensor(initial[7:11], dtype=torch.double)
        delta = torch.diff(torch.cat((previous[None], feasible)), dim=0)
        result["input_change"] = (delta.square()*planner._paper_r_delta_u).sum().item()
    return result


def payload_config(cfg):
    return {key: getattr(cfg, key) for key in (
        "dt", "tau", "command_alpha", "max_accel_xy", "max_accel_z",
        "max_yaw_accel", "vmax", "vzmax", "yaw_rate_max",
        "response_accel_xy", "response_jerk_xy", "margin", "w_goal",
        "w_terminal", "w_obstacle", "w_collision", "collision_radius_m",
        "collision_cost_buffer_m", "w_u", "w_stopping", "stopping_margin_m",
        "stopping_delay_s", "w_du", "w_yaw", "w_path", "path_scale_m",
        "w_reference_velocity", "w_progress", "w_speed_limit")}


def write_cost(name, **overrides):
    params = dict(COMMON)
    params.update(overrides)
    planner = QuadMPPI(MPPIConfig(**params))
    planner.update_goal([8., 3., 2.])
    planner.update_obstacles([[3., .4, 2.], [6., 2.4, 2.]])
    path = np.asarray([[0., 0., 2.], [4., 0., 2.], [8., 4., 2.]])
    planner.update_reference_path(path)
    initial = state()
    planner._last_state_np = initial.copy()
    planner._update_reference_trajectory(initial[:3], initial[3:6])
    actions = controls(params["horizon"])
    states = rollout(planner, initial, actions)
    terms = breakdown(planner, initial, states, actions)
    payload = payload_config(planner.cfg)
    payload.update(
        cost_profile=planner.cfg.cost_profile,
        path_progress_objective=planner.cfg.path_progress_objective,
        response_accel_model=planner.cfg.response_accel_model,
        paper_r_u=list(planner.cfg.paper_r_u),
        paper_r_delta_u=list(planner.cfg.paper_r_delta_u),
        initial_state_flat=initial.tolist(), controls_flat=flat_controls(actions),
        goal_flat=planner.goal.tolist(), obstacles_flat=planner.obstacles.flatten().tolist(),
        path_flat=planner.reference_path.flatten().tolist(),
        reference_positions_flat=([] if planner.reference_positions is None else planner.reference_positions.flatten().tolist()),
        reference_velocities_flat=([] if planner.reference_velocities is None else planner.reference_velocities.flatten().tolist()),
        expected_terms_flat=[terms[key] for key in TERMS],
        expected_total=sum(terms.values()))
    (OUT/f"cost_{name}.json").write_text(json.dumps(payload, indent=2)+"\n")


def write_optimizer(name, profile):
    params = dict(COMMON, horizon=5, samples=4, cost_profile=profile,
                  path_progress_objective=(profile == "project"))
    planner = QuadMPPI(MPPIConfig(**params))
    planner.update_goal([7., 2., 2.])
    planner.update_obstacles([[2.2, .1, 2.], [5., 1.8, 2.]])
    planner.update_reference_path([[0., 0., 2.], [4., 0., 2.], [8., 3., 2.]])
    initial = state()
    planner._last_state_np = initial.copy()
    planner._update_reference_trajectory(initial[:3], initial[3:6])
    nominal = np.asarray([[1., -.2, .1, .1], [2., .1, .2, .2],
                          [4.8, .5, .3, .4], [5., 1., 0., .5],
                          [4., 1.5, -.2, .3]], dtype=np.float64)
    tail = np.asarray([.2, -.1, 0., 0.])
    noise = np.asarray([
        [[0.,0.,0.,0.]]*5,
        [[2.,-1.,.5,.4],[-1.,.3,0.,-.2],[2.,2.,1.,.5],[1.,-2.,-1.,-.8],[.4,.2,.1,.1]],
        [[-1.,.5,-.2,-.1],[.4,-.6,.2,.3],[-3.,1.,-.4,-.3],[-1.,.2,.1,.2],[-.2,.1,0.,0.]],
        [[.2,.1,0.,0.],[.3,.2,.1,.1],[.4,.3,.1,.1],[.5,.4,.2,.2],[8.,-8.,3.,2.]],
    ], dtype=np.float64)
    planner.ctrl.U.copy_(torch.as_tensor(nominal))
    planner.ctrl.u_init.copy_(torch.as_tensor(tail))
    injected = torch.as_tensor(noise, dtype=torch.double)
    planner.ctrl._sample_noise = lambda shape: injected.clone()
    updated_first = planner.ctrl.command(torch.as_tensor(initial), shift_nominal_trajectory=True)
    shifted = np.vstack((nominal[1:], tail))
    effective = planner.ctrl.noise.detach().numpy()
    perturb = np.sum(shifted[None]*(
        planner.cfg.lambda_*effective/np.asarray([
            planner.cfg.noise_xy**2, planner.cfg.noise_xy**2,
            planner.cfg.noise_z**2, planner.cfg.noise_yaw**2])[None,None]), axis=(1,2))
    payload = payload_config(planner.cfg)
    payload.update(
        cost_profile=profile, path_progress_objective=planner.cfg.path_progress_objective,
        response_accel_model=True, lambda_=planner.cfg.lambda_,
        noise_sigma=[planner.cfg.noise_xy, planner.cfg.noise_xy,
                     planner.cfg.noise_z, planner.cfg.noise_yaw],
        paper_r_u=list(planner.cfg.paper_r_u), paper_r_delta_u=list(planner.cfg.paper_r_delta_u),
        initial_state_flat=initial.tolist(), nominal_before_shift_flat=nominal.flatten().tolist(),
        controls_flat=nominal.flatten().tolist(),
        tail_control_flat=tail.tolist(), injected_noise_flat=noise.flatten().tolist(),
        goal_flat=planner.goal.tolist(), obstacles_flat=planner.obstacles.flatten().tolist(),
        path_flat=planner.reference_path.flatten().tolist(),
        reference_positions_flat=planner.reference_positions.flatten().tolist(),
        reference_velocities_flat=planner.reference_velocities.flatten().tolist(),
        expected_shifted_nominal_flat=shifted.flatten().tolist(),
        expected_perturbed_actions_flat=planner.ctrl.perturbed_action.detach().numpy().flatten().tolist(),
        expected_effective_noise_flat=effective.flatten().tolist(),
        expected_states_flat=planner.ctrl.states[0].detach().numpy().flatten().tolist(),
        expected_rollout_costs_flat=(planner.ctrl.cost_total.detach().numpy()-perturb).tolist(),
        expected_perturbation_costs_flat=perturb.tolist(),
        expected_total_costs_flat=planner.ctrl.cost_total.detach().numpy().tolist(),
        expected_weights_flat=planner.ctrl.omega.detach().numpy().tolist(),
        expected_updated_nominal_flat=planner.ctrl.U.detach().numpy().flatten().tolist(),
        expected_first_command_flat=updated_first.detach().numpy().tolist())
    (OUT/f"optimizer_{name}.json").write_text(json.dumps(payload, indent=2)+"\n")


def write_recovery():
    params = dict(COMMON, horizon=6, samples=16, proactive_proposals=True,
                  reference_speed_m_s=5.)
    planner = QuadMPPI(MPPIConfig(**params))
    planner.update_reference_path([[0., 0., 2.], [4., 0., 2.], [8., 4., 2.]])
    initial = state()
    planner._last_state_np = initial.copy()
    planner._path_progress_m = planner._project_path_progress(initial[:3])
    proposals = planner.recovery_proposals().detach().numpy()
    payload = dict(
        horizon=planner.cfg.horizon, samples=planner.cfg.samples,
        dt=planner.cfg.dt, reference_speed_m_s=planner.cfg.reference_speed_m_s,
        vmax=planner.cfg.vmax, vzmax=planner.cfg.vzmax,
        yaw_rate_max=planner.cfg.yaw_rate_max,
        path_progress_m=planner._path_progress_m,
        initial_horizontal_speed_m_s=float(np.linalg.norm(initial[3:5])),
        path_flat=planner.reference_path.flatten().tolist(),
        expected_count=len(proposals),
        expected_proposals_flat=proposals.flatten().tolist())
    (OUT/"recovery_proposals.json").write_text(json.dumps(payload, indent=2)+"\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    write_cost("project_open_space", cost_profile="project", w_obstacle=0., w_stopping=0.)
    write_cost("project_obstacle", cost_profile="project", w_stopping=0.)
    write_cost("project_path_tracking", cost_profile="project", w_obstacle=0., w_stopping=0.)
    write_cost("paper_open_space", cost_profile="paper", w_collision=0., w_stopping=0.)
    write_cost("paper_collision", cost_profile="paper", w_stopping=0.)
    write_cost("paper_reference_tracking", cost_profile="paper", w_collision=0., w_stopping=0.)
    write_cost("paper_input_change", cost_profile="paper", w_collision=0., w_stopping=0.)
    write_cost("progress_objective", cost_profile="project", path_progress_objective=True, w_obstacle=0., w_stopping=0.)
    write_cost("stopping_cost", cost_profile="project", w_obstacle=0.)
    write_cost("speed_limit", cost_profile="project", path_progress_objective=True,
               vmax=1., w_obstacle=0., w_stopping=0.)
    write_optimizer("project", "project")
    write_optimizer("paper", "paper")
    write_recovery()


if __name__ == "__main__":
    main()

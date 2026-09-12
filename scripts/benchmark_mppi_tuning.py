#!/usr/bin/env python3
"""Deterministic offline benchmark for velocity-level MPPI tuning.

This exercises the same QuadMPPI, LocalPlannerNode, command conditioner and
first-order tracking model used by ``--sim-test``.  It does not represent a
Gazebo or ArduPilot flight result.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
os.environ.setdefault("OMP_NUM_THREADS", "1")

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI
from mppi_ardupilot.mppi_local_planner_node import (
    LocalPlannerNode,
    PlannerState,
    VelocityCommandConditioner,
)


@dataclass(frozen=True)
class Scenario:
    name: str
    start: tuple[float, float, float]
    waypoints: tuple[tuple[float, float, float], ...]
    obstacles: np.ndarray
    max_steps: int = 450


def _line(a, b, spacing=0.35, z=20.0) -> np.ndarray:
    a2, b2 = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    count = max(2, int(np.linalg.norm(b2 - a2) / spacing) + 1)
    xy = np.linspace(a2, b2, count)
    return np.column_stack((xy, np.full(count, z)))


def _circle(center, radius=1.0, count=28, z=20.0) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    return np.column_stack((
        center[0] + radius * np.cos(theta),
        center[1] + radius * np.sin(theta),
        np.full(count, z),
    ))


def scenarios() -> dict[str, Scenario]:
    # Alternating disks require repeated changes of curvature.  Guide points
    # specify only the homotopy; MPPI still produces the local motion.
    slalom_obs = np.vstack([
        _circle((7.0, 1.8), 1.2),
        _circle((13.0, -1.8), 1.2),
        _circle((19.0, 1.8), 1.2),
        _circle((25.0, -1.8), 1.2),
    ])
    slalom = Scenario(
        "multi_obstacle_slalom", (0.0, 0.0, 20.0),
        ((5.5, -2.2, 20.0), (11.5, 2.2, 20.0),
         (17.5, -2.2, 20.0), (23.5, 2.2, 20.0), (30.0, 0.0, 20.0)),
        slalom_obs, max_steps=600,
    )

    # Two wall segments leave a 5.5 m opening.  With margin=2 m, the nominal
    # safe part of the opening is only 1.5 m wide.
    gate_obs = np.vstack([
        _line((10.0, -8.0), (10.0, -2.75)),
        _line((10.0, 2.75), (10.0, 8.0)),
        _circle((16.0, 2.2), 1.0),
        _circle((20.0, -2.2), 1.0),
    ])
    gate = Scenario(
        "narrow_gate", (0.0, 0.0, 20.0),
        ((13.0, 0.0, 20.0), (12.5, 5.8, 20.0),
         (23.0, 5.8, 20.0), (24.0, 0.0, 20.0)),
        gate_obs,
    )

    # An L-shaped corridor.  The inner corner forces an approximately
    # 90-degree change in flight direction without using a global spline.
    corner_obs = np.vstack([
        _line((-1.0, -3.0), (9.0, -3.0)),
        _line((-1.0, 3.0), (7.0, 3.0)),
        _line((7.0, 3.0), (7.0, 15.0)),
        _line((13.0, -1.0), (13.0, 15.0)),
    ])
    corner = Scenario(
        "right_angle_corridor", (0.0, 0.0, 20.0),
        ((5.5, 0.0, 20.0), (10.0, 0.0, 20.0),
         (10.0, 5.0, 20.0), (10.0, 13.0, 20.0)),
        corner_obs,
    )
    return {s.name: s for s in (slalom, gate, corner)}


PROFILES = {
    "baseline": {},
    "smooth_slow": {
        "vmax": 1.2, "noise_xy": 0.55, "lambda_": 2.0,
        "max_accel_xy": 0.7, "command_alpha": 0.35,
    },
    "long_horizon": {
        "horizon": 40, "samples": 500, "lambda_": 2.0,
        "noise_xy": 0.9,
    },
    "high_exploration": {
        "horizon": 35, "samples": 500, "lambda_": 3.0,
        "noise_xy": 1.25, "max_accel_xy": 1.0,
    },
    "balanced_cost": {
        "horizon": 40, "samples": 500, "lambda_": 3.0,
        "noise_xy": 0.9, "vmax": 1.4, "max_accel_xy": 0.8,
        "w_obstacle": 60.0,
    },
    "tight_passage": {
        "horizon": 40, "samples": 500, "lambda_": 3.0,
        "noise_xy": 0.9, "vmax": 1.2, "max_accel_xy": 0.7,
        "w_goal": 2.0, "w_terminal": 10.0, "w_obstacle": 12.0,
    },
    "safe_tight": {
        "horizon": 40, "samples": 500, "lambda_": 3.0,
        "noise_xy": 0.9, "vmax": 1.1, "max_accel_xy": 0.6,
        "w_goal": 2.0, "w_terminal": 10.0, "w_obstacle": 24.0,
    },
    "paper_reference": {
        "cost_profile": "paper", "horizon": 30, "samples": 500,
        "lambda_": 100.0, "vmax": 1.5, "vzmax": 0.8,
        "w_goal": 0.0, "w_terminal": 0.0, "w_obstacle": 0.0,
        "w_u": 0.0, "w_du": 0.0, "w_yaw": 0.0,
        "w_path": 400.0, "w_reference_velocity": 40.0,
        "reference_speed_m_s": 1.0, "w_collision": 1.0e6,
        "collision_radius_m": 1.5,
    },
}


def run_case(scenario: Scenario, profile: str, seed: int) -> dict:
    base = MPPIConfig(
        seed=seed, margin=2.0, horizon=30, samples=350, vmax=1.8,
        w_obstacle=300.0, goal_slowdown_radius=3.0,
        goal_approach_gain=0.45, max_accel_xy=1.2,
    )
    cfg = replace(base, **PROFILES[profile])
    planner = QuadMPPI(cfg)
    reference_path = np.vstack((scenario.start, scenario.waypoints))
    paper_profile = cfg.cost_profile == "paper"
    node = LocalPlannerNode(
        planner,
        [np.asarray(w, dtype=float) for w in scenario.waypoints],
        wp_radius=1.25,
        goal_radius=0.25,
        hard_brake_m=1.0 if paper_profile else 0.0,
        hard_brake_release_m=1.8 if paper_profile else None,
        hard_brake_delay_s=0.25,
        brake_accel_m_s2=cfg.max_accel_xy,
        recovery_speed_m_s=0.4,
        reference_path=reference_path if paper_profile else None,
        command_conditioner=VelocityCommandConditioner(
            cfg.dt, cfg.command_alpha, cfg.max_accel_xy,
            cfg.max_accel_z, cfg.max_yaw_accel,
            u_min=cfg.u_min(), u_max=cfg.u_max(),
        ),
        goal_slowdown_radius=cfg.goal_slowdown_radius,
        goal_approach_gain=cfg.goal_approach_gain,
        goal_min_speed=cfg.goal_min_speed,
    )
    pos = np.asarray(scenario.start, dtype=float)
    vel = np.zeros(3)
    yaw = 0.0
    previous_u = np.zeros(4)
    path_length = 0.0
    control_variation = 0.0
    command_accel: list[float] = []
    min_clearance = float("inf")
    compute_ms: list[float] = []
    ess: list[float] = []
    reached = False
    waypoint_switches = 0
    started = time.perf_counter()

    for step in range(scenario.max_steps):
        old_pos = pos.copy()
        out = node.step(PlannerState(pos.copy(), vel.copy(), yaw), scenario.obstacles)
        u = np.asarray(out.u, dtype=float)
        if out.event == "waypoint":
            waypoint_switches += 1
        if out.event == "reached":
            reached = True
            break
        alpha = min(cfg.dt / cfg.tau, 1.0)
        vel += alpha * (u[:3] - vel)
        pos += vel * cfg.dt
        yaw += float(u[3]) * cfg.dt
        path_length += float(np.linalg.norm(pos - old_pos))
        control_variation += float(np.linalg.norm(u - previous_u))
        command_accel.append(float(np.linalg.norm(u[:3] - previous_u[:3])) / cfg.dt)
        previous_u = u
        min_clearance = min(
            min_clearance,
            float(np.linalg.norm(scenario.obstacles - pos, axis=1).min()),
        )
        diag = out.diagnostics
        compute_ms.append(float(diag.get("compute_ms", 0.0)))
        if "ess" in diag:
            ess.append(float(diag["ess"]))

    elapsed = time.perf_counter() - started
    direct = float(np.linalg.norm(np.asarray(scenario.waypoints[-1]) - np.asarray(scenario.start)))
    return {
        "scenario": scenario.name,
        "profile": profile,
        "seed": seed,
        "offline_only": True,
        "reached": reached,
        "steps": step + 1,
        "sim_time_s": (step + 1) * cfg.dt,
        "final_error_m": float(np.linalg.norm(pos - np.asarray(scenario.waypoints[-1]))),
        "min_clearance_m": min_clearance,
        "safety_margin_m": cfg.margin,
        "margin_violated": min_clearance < cfg.margin,
        "path_length_m": path_length,
        "path_ratio": path_length / max(direct, 1e-9),
        "control_variation": control_variation,
        "command_accel_rms_mps2": float(np.sqrt(np.mean(np.square(command_accel)))) if command_accel else 0.0,
        "command_accel_max_mps2": float(np.max(command_accel)) if command_accel else 0.0,
        "waypoint_switches": waypoint_switches,
        "compute_mean_ms": float(np.mean(compute_ms)) if compute_ms else 0.0,
        "compute_p95_ms": float(np.percentile(compute_ms, 95)) if compute_ms else 0.0,
        "compute_worst_ms": float(np.max(compute_ms)) if compute_ms else 0.0,
        "ess_mean": float(np.mean(ess)) if ess else 0.0,
        "ess_p05": float(np.percentile(ess, 5)) if ess else 0.0,
        "wall_elapsed_s": elapsed,
        "config": asdict(cfg),
    }


def _split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", default=",".join(scenarios()))
    parser.add_argument("--profiles", default=",".join(PROFILES))
    parser.add_argument("--seeds", default="7", help="comma-separated integers")
    parser.add_argument("--output", default="output/benchmark/mppi_tuning")
    args = parser.parse_args()

    known_scenarios = scenarios()
    selected_scenarios = _split_csv(args.scenarios)
    selected_profiles = _split_csv(args.profiles)
    seeds = [int(item) for item in _split_csv(args.seeds)]
    unknown = set(selected_scenarios) - set(known_scenarios)
    unknown_profiles = set(selected_profiles) - set(PROFILES)
    if unknown or unknown_profiles:
        raise SystemExit(f"unknown scenarios={sorted(unknown)}, profiles={sorted(unknown_profiles)}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario_name in selected_scenarios:
        for profile in selected_profiles:
            for seed in seeds:
                print(f"[benchmark] {scenario_name} / {profile} / seed={seed}", flush=True)
                row = run_case(known_scenarios[scenario_name], profile, seed)
                rows.append(row)
                print(
                    f"  reached={row['reached']} clearance={row['min_clearance_m']:.2f}m "
                    f"path={row['path_length_m']:.1f}m p95={row['compute_p95_ms']:.1f}ms "
                    f"ESS={row['ess_mean']:.1f}", flush=True,
                )

    json_path = output.with_suffix(".json")
    csv_path = output.with_suffix(".csv")
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    flat_keys = [key for key in rows[0] if key != "config"]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=flat_keys)
        writer.writeheader()
        writer.writerows({key: row[key] for key in flat_keys} for row in rows)
    print(f"[benchmark] wrote {json_path} and {csv_path}")


if __name__ == "__main__":
    main()

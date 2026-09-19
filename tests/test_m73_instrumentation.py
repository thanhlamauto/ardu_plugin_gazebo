import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_script(name: str):
    path = ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write_run(tmp_path, scenario, rows, manifest=None):
    run = tmp_path / scenario.lower() / "run_01_seed_7"
    run.mkdir(parents=True)
    (run / "manifest.json").write_text(json.dumps(
        manifest or {"scenario": scenario, "variant": "base", "run": 1, "seed": 7}))
    (run / "events.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows))
    return run


def test_timeout_classification_covers_each_component():
    analyzer = load_script("analyze_m73_root_cause")
    assert analyzer.classify_timeout(
        {"t_total_ms": 65.0, "t_scheduling_delay_ms": 60.0,
         "t_rollout_ms": 1.0}) == "A_callback_started_late"
    assert analyzer.classify_timeout(
        {"t_total_ms": 90.0, "t_scheduling_delay_ms": 1.0,
         "t_rollout_ms": 80.0}) == "B_rollout"
    assert analyzer.classify_timeout(
        {"t_total_ms": 90.0, "t_rollout_ms": 1.0,
         "t_safety_ms": 80.0}) == "C_safety"
    assert analyzer.classify_timeout(
        {"t_total_ms": 90.0, "t_rollout_ms": 1.0,
         "t_cost_ms": 80.0}) == "D_cost"
    assert analyzer.classify_timeout(
        {"t_total_ms": 90.0, "t_rollout_ms": 1.0,
         "t_conditioner_ms": 50.0}) == "E_io_logging"
    assert analyzer.classify_timeout({}) == "F_other"


def test_run_metrics_identify_collapse_and_rejection_reason(tmp_path):
    analyzer = load_script("analyze_m73_root_cause")
    rows = [
        {"event": "run_started", "elapsed_s": 0.0},
        {"event": "control_cycle", "elapsed_s": 1.0,
         "state": {"position_enu_m": [0.0, 0.0, 5.0],
                   "velocity_enu_m_s": [4.0, 0.0, 0.0]},
         "controller": {"mode": "ACTIVE", "samples": 80, "N_safe": 80,
                        "speed_xy_m_s": 4.0, "path_curvature_per_m": 0.01,
                        "heading_error_rad": 0.1,
                        "minimum_collision_clearance_m": 5.0,
                        "minimum_stopping_clearance_m": 5.0,
                        "t_total_ms": 10.0, "t_scheduling_delay_ms": 0.0}},
        {"event": "control_cycle", "elapsed_s": 2.0,
         "state": {"position_enu_m": [1.0, 0.0, 5.0],
                   "velocity_enu_m_s": [7.0, 0.0, 0.0]},
         "controller": {"mode": "NO_SAFE_TRAJECTORY", "samples": 80, "N_safe": 0,
                        "speed_xy_m_s": 7.0, "path_curvature_per_m": 0.5,
                        "heading_error_rad": 0.8,
                        "minimum_collision_clearance_m": 0.01,
                        "minimum_stopping_clearance_m": 0.01,
                        "reject_swept_collision": 80, "gaussian_safe": 0,
                        "reference_safe": 0, "braking_safe": 0,
                        "recovery_safe": 0, "specific_safe": 0,
                        "t_total_ms": 95.0, "t_scheduling_delay_ms": 0.0}},
        {"event": "run_result", "elapsed_s": 3.0, "success": False,
         "reason": "scenario timeout"},
    ]
    run = write_run(tmp_path, "S03", rows)
    metrics = analyzer.extract_run_metrics(
        run, analyzer.read_jsonl(run / "events.jsonl"), {"scenario": "S03", "seed": 7})
    assert metrics["no_safe_cycles"] == 1
    assert metrics["first_no_safe_elapsed_s"] == 2.0
    assert metrics["speed_at_collapse_m_s"] == 7.0
    assert metrics["heading_error_at_collapse_rad"] == 0.8
    assert metrics["reject_dominant_at_collapse"] == "reject_swept_collision"
    assert metrics["reject_swept_collision_total"] == 80
    assert metrics["gaussian_safe_at_collapse"] == 0.0
    assert metrics["min_collision_clearance_m"] == 0.01


def test_handoff_join_links_global_and_local_path_ids(tmp_path):
    analyzer = load_script("analyze_m73_root_cause")
    rows = [
        {"event": "run_started", "elapsed_s": 0.0},
        {"event": "global_planner", "elapsed_s": 5.0,
         "global_planner": {"status": "OK", "path_stamp_ns": 111,
                            "goal_id": 1.0, "global_path_id": 1.0}},
        {"event": "control_cycle", "elapsed_s": 5.1,
         "controller": {"mode": "ACTIVE", "samples": 80,
                        "local_active_path_id": 1.0,
                        "active_path_stamp_ns": 111.0,
                        "first_solve_on_active_path": True,
                        "first_safe_command_on_active_path": True}},
        {"event": "global_planner", "elapsed_s": 8.0,
         "global_planner": {"status": "OK", "path_stamp_ns": 222,
                            "goal_id": 2.0, "global_path_id": 2.0}},
        {"event": "control_cycle", "elapsed_s": 8.2,
         "controller": {"mode": "ACTIVE", "samples": 80,
                        "local_active_path_id": 2.0,
                        "active_path_stamp_ns": 222.0,
                        "first_solve_on_active_path": True,
                        "first_safe_command_on_active_path": False}},
        {"event": "run_result", "elapsed_s": 9.0, "success": True,
         "reason": "observed local mode GOAL_REACHED"},
    ]
    write_run(tmp_path, "S08", rows, {"scenario": "S08", "seed": 7, "run": 1})
    handoffs = analyzer.collect_handoffs(tmp_path)
    assert len(handoffs) == 2
    first, second = sorted(handoffs, key=lambda row: row["local_active_path_id"])
    assert first["goal_id"] == 1.0 and first["global_path_id"] == 1.0
    assert math.isclose(first["handoff_latency_ms"], 100.0)
    assert second["goal_id"] == 2.0
    assert math.isnan(second["safe_latency_ms"])


def test_summary_analyzer_aggregates_m73_fields(tmp_path):
    analyzer = load_script("analyze_m7_results")
    rows = [
        {"event": "run_started", "elapsed_s": 0.0},
        {"event": "control_cycle", "elapsed_s": 1.0,
         "state": {"position_enu_m": [0.0, 0.0, 5.0],
                   "velocity_enu_m_s": [1.0, 0.0, 0.0]},
         "controller": {"mode": "ACTIVE", "samples": 80, "N_safe": 0,
                        "t_total_ms": 20.0, "t_scheduling_delay_ms": 3.0,
                        "t_wall_cycle_ms": 103.0,
                        "reject_stopping_distance": 80,
                        "minimum_collision_clearance_m": 2.0,
                        "minimum_stopping_clearance_m": 0.5}},
        {"event": "run_result", "elapsed_s": 2.0, "success": True,
         "reason": "observed local mode GOAL_REACHED"},
    ]
    run = write_run(tmp_path, "S05", rows, {"scenario": "S05", "seed": 7, "run": 1})
    summary = analyzer.summarize_run(run / "events.jsonl")
    assert summary["no_safe_cycles"] == 1
    assert summary["reject_stopping_distance_total"] == 80
    assert summary["max_scheduling_delay_ms"] == 3.0
    assert summary["min_stopping_clearance_m"] == 0.5

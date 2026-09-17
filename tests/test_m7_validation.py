import csv
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import yaml

from mppi_ardupilot.global_planner import AStarConfig, AStarGlobalPlanner


ROOT = Path(__file__).resolve().parents[1]


def load_script(name: str):
    path = ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_m7_scenario_matrix_is_complete_and_references_real_worlds():
    files = sorted((ROOT / "tests/scenarios").glob("s*.yaml"))
    assert len(files) == 10
    scenarios = [yaml.safe_load(path.read_text()) for path in files]
    assert [item["id"] for item in scenarios] == [f"S{i:02d}" for i in range(1, 11)]
    for scenario in scenarios:
        assert scenario["schema_version"] == 1
        assert len(scenario["goal_enu_m"]) == 3
        assert scenario["timeout_s"] > 0
        assert scenario["repetitions"] == (10 if scenario["id"] <= "S06" else 5)
        assert (ROOT / scenario["world"]).is_file()
        expected = (scenario.get("expected_terminal_modes") or
                    scenario.get("expected_global_status") or
                    scenario.get("expected_adapter_states"))
        assert expected


def test_m7_baseline_is_performance_mode_and_100ms_deadline():
    config = yaml.safe_load(
        (ROOT / "uav_navigation_bringup/config/m7_baseline.yaml").read_text())
    local = config["local_navigation"]["ros__parameters"]
    assert local["control_rate_hz"] == 10.0
    assert local["max_compute_time_ms"] == 100.0
    assert local["visualization.enabled"] is False
    assert local["visualization.max_mppi_samples"] == 0
    assert local["mppi.samples"] == 80
    assert local["mppi.path_progress_objective"] is True


def test_s02_geometry_really_creates_an_approximately_45_degree_turn():
    scenario = yaml.safe_load((ROOT / "tests/scenarios/s02_turn45.yaml").read_text())
    planner = AStarGlobalPlanner.from_sdf(
        ROOT / scenario["world"], AStarConfig(clearance_m=1.8))
    path = planner.plan([0.0, 0.0, 5.0], scenario["goal_enu_m"]).path_enu
    segments = np.diff(path[:, :2], axis=0)
    headings = np.arctan2(segments[:, 1], segments[:, 0])
    turns = np.degrees(np.diff(headings))
    turns = (turns + 180.0) % 360.0 - 180.0
    assert any(40.0 <= abs(value) <= 55.0 for value in turns)


def test_analyzer_computes_percentiles_and_safety_metrics(tmp_path):
    analyzer = load_script("analyze_m7_results")
    run = tmp_path / "s01" / "run_01_seed_7"
    run.mkdir(parents=True)
    manifest = {
        "scenario": "S01", "run": 1, "seed": 7,
        "expected_terminal_modes": ["GOAL_REACHED"],
    }
    (run / "manifest.json").write_text(json.dumps(manifest))
    rows = [{"event": "run_started", "elapsed_s": 0.0}]
    for i, compute in enumerate([10.0, 20.0, 30.0, 40.0]):
        mode = "GOAL_REACHED" if i == 3 else "ACTIVE"
        rows.append({
            "event": "control_cycle", "elapsed_s": float(i),
            "state": {"position_enu_m": [float(i), 0.0, 5.0],
                      "velocity_enu_m_s": [float(i), 0.0, 0.0]},
            "controller": {"mode": mode, "t_total_ms": compute,
                           "samples": 0 if mode == "GOAL_REACHED" else 80,
                           "cross_track_error_m": 0.1 * i,
                           "minimum_clearance_m": 2.0 - 0.1 * i,
                           "safe_samples": 8 - i, "ess": 2.0 - 0.1 * i,
                           "deadline_miss": False},
        })
    rows.append({"event": "run_result", "elapsed_s": 3.0,
                 "success": True, "reason": "observed local mode GOAL_REACHED"})
    (run / "events.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows))
    summary = analyzer.summarize_run(run / "events.jsonl")
    assert summary["success"] is True
    assert summary["path_m"] == 3.0
    assert summary["path_efficiency"] == 1.0
    assert summary["compute_p50_ms"] == 20.0
    assert math.isclose(summary["compute_p95_ms"], 29.0)
    assert summary["deadline_misses"] == 0
    assert summary["min_safe_samples"] == 6.0
    assert summary["collision"] is False
    analyzer.write_outputs(tmp_path, [summary])
    with (tmp_path / "summary.csv").open() as stream:
        output = list(csv.DictReader(stream))
    assert output[0]["scenario"] == "S01"
    assert (tmp_path / "REPORT.md").is_file()

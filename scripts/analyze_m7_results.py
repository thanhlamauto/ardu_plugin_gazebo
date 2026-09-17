#!/usr/bin/env python3
"""Aggregate M7 JSONL runs into a stable CSV and a concise Markdown report."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable


SUMMARY_FIELDS = [
    "scenario", "run", "seed", "success", "terminal_reason", "time_s",
    "performance_mode",
    "time_to_goal_s", "path_m", "path_efficiency", "min_clearance_m",
    "cte_rms_m", "cte_p95_m", "peak_xy_speed_m_s", "peak_accel_m_s2",
    "compute_p50_ms", "compute_p95_ms", "compute_p99_ms", "compute_max_ms",
    "deadline_misses", "min_safe_samples", "ess_min", "collision",
    "median_safe_samples", "ess_median", "best_feasible_cost_min",
    "best_feasible_cost_median", "no_safe_trajectory_events",
    "disconnected_events", "stale_command_events",
    "late_command_accepted", "stale_command_violations",
    "setpoint_while_disarmed", "setpoint_in_wrong_mode",
    "replan_latency_ms", "replan_to_command_ms",
]


def percentile(values: Iterable[float], percentage: float) -> float:
    data = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not data:
        return math.nan
    position = (len(data) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return data[lower]
    fraction = position - lower
    return data[lower] * (1.0 - fraction) + data[upper] * fraction


def _finite(values: Iterable[Any]) -> list[float]:
    output = []
    for value in values:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(parsed):
            output.append(parsed)
    return output


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{number}: JSON object required")
            rows.append(value)
    return rows


def summarize_run(path: Path) -> dict[str, Any]:
    rows = read_jsonl(path)
    start_indices = [index for index, row in enumerate(rows)
                     if row.get("event") == "run_started"]
    result_indices = [index for index, row in enumerate(rows)
                      if row.get("event") == "run_result"]
    start_index = start_indices[0] if start_indices else 0
    end_index = result_indices[-1] + 1 if result_indices else len(rows)
    run_rows = rows[start_index:end_index]
    cycles = [row for row in run_rows if row.get("event") == "control_cycle"]
    planner_cycles = [
        row for row in cycles
        if float(row.get("controller", {}).get("samples", 0.0) or 0.0) > 0.0
    ]
    manifest_path = path.with_name("manifest.json")
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    timestamps = _finite(row.get("elapsed_s") for row in run_rows)
    positions = [row.get("state", {}).get("position_enu_m") for row in cycles]
    positions = [p for p in positions if isinstance(p, list) and len(p) == 3]
    velocities = [row.get("state", {}).get("velocity_enu_m_s") for row in cycles]
    velocities = [v for v in velocities if isinstance(v, list) and len(v) == 3]
    cycle_times = _finite(row.get("elapsed_s") for row in cycles)

    path_length = 0.0
    for first, second in zip(positions, positions[1:]):
        path_length += math.dist(first, second)
    direct = math.dist(positions[0], positions[-1]) if len(positions) > 1 else 0.0
    efficiency = direct / path_length if path_length > 0.0 else math.nan

    speeds = [math.hypot(v[0], v[1]) for v in velocities]
    accelerations = []
    for i in range(1, min(len(velocities), len(cycle_times))):
        dt = cycle_times[i] - cycle_times[i - 1]
        if dt > 1e-6:
            accelerations.append(
                math.hypot(velocities[i][0] - velocities[i - 1][0],
                           velocities[i][1] - velocities[i - 1][1]) / dt)

    compute = _finite(row.get("controller", {}).get("t_total_ms") for row in planner_cycles)
    cte = _finite(row.get("controller", {}).get("cross_track_error_m") for row in planner_cycles)
    clearance = _finite(row.get("controller", {}).get("minimum_clearance_m") for row in planner_cycles)
    safe_samples = _finite(row.get("controller", {}).get("safe_samples") for row in planner_cycles)
    ess = _finite(row.get("controller", {}).get("ess") for row in planner_cycles)
    feasible_costs = _finite(
        row.get("controller", {}).get("best_feasible_cost") for row in planner_cycles)
    local_modes = [row.get("controller", {}).get("mode") for row in cycles]
    global_statuses = [row.get("global_planner", {}).get("status") for row in run_rows]
    adapter_states = [row.get("adapter", {}).get("adapter_state") for row in run_rows]
    expected_modes = manifest.get("expected_terminal_modes", [])
    expected_global = manifest.get("expected_global_status")
    expected_adapter = manifest.get("expected_adapter_states", [])
    success = (any(mode in expected_modes for mode in local_modes) if expected_modes else True)
    if expected_global:
        success = success and expected_global in global_statuses
    if expected_adapter:
        success = success and any(state in expected_adapter for state in adapter_states)
    result_events = [row for row in run_rows if row.get("event") == "run_result"]
    if result_events:
        success = bool(result_events[-1].get("success", success))
        terminal_reason = result_events[-1].get("reason", "")
    else:
        terminal_reason = "missing run_result"
        success = False
    goal_times = [float(row["elapsed_s"]) for row in cycles
                  if row.get("controller", {}).get("mode") == "GOAL_REACHED"]
    deadline_misses = sum(
        bool(row.get("controller", {}).get("deadline_miss")) or
        float(row.get("controller", {}).get("t_total_ms", 0.0) or 0.0) > 100.0
        for row in cycles
    )
    collision = any(
        row.get("controller", {}).get("mode") == "ACTIVE" and
        number <= 0.0
        for row in cycles
        for number in _finite([row.get("controller", {}).get("minimum_clearance_m")])
    )
    late_command_accepted = sum(
        row.get("controller", {}).get("mode") == "ACTIVE" and
        float(row.get("controller", {}).get("t_total_ms", 0.0) or 0.0) > 100.0
        for row in cycles
    )
    stale_command_violations = 0
    setpoint_while_disarmed = 0
    setpoint_in_wrong_mode = 0
    previous_sequence = None
    unsafe_adapter_states = {"STALE_COMMAND", "DISCONNECTED", "CONNECTED_NOT_READY", "FAULT"}
    for row in (item for item in run_rows if item.get("event") == "adapter"):
        sequence = row.get("mavros_setpoint_sequence")
        if (previous_sequence is not None and sequence is not None and
                row.get("adapter", {}).get("adapter_state") in unsafe_adapter_states and
                sequence > previous_sequence):
            stale_command_violations += 1
        if previous_sequence is not None and sequence is not None and sequence > previous_sequence:
            adapter = row.get("adapter", {})
            if adapter.get("armed") is False:
                setpoint_while_disarmed += 1
            if adapter.get("mode") not in (None, "GUIDED"):
                setpoint_in_wrong_mode += 1
        previous_sequence = sequence
    goals = [row for row in rows if row.get("event") == "goal_published"]
    replan_latency_ms = math.nan
    replan_to_command_ms = math.nan
    if len(goals) > 1:
        replan_time = float(goals[-1]["elapsed_s"])
        plans = [row for row in rows
                 if row.get("event") == "global_planner" and
                 float(row.get("elapsed_s", -1.0)) >= replan_time and
                 row.get("global_planner", {}).get("status") == "OK"]
        if plans:
            plan_time = float(plans[0]["elapsed_s"])
            replan_latency_ms = (plan_time - replan_time) * 1000.0
            commands = [row for row in cycles
                        if float(row.get("elapsed_s", -1.0)) >= plan_time and
                        row.get("controller", {}).get("mode") == "ACTIVE"]
            if commands:
                replan_to_command_ms = (float(commands[0]["elapsed_s"]) - plan_time) * 1000.0
    return {
        "scenario": manifest.get("scenario", path.parent.parent.name),
        "run": manifest.get("run", path.parent.name),
        "seed": manifest.get("seed", ""),
        "performance_mode": bool(manifest.get("performance_mode", True)),
        "success": success,
        "terminal_reason": terminal_reason,
        "time_s": max(timestamps, default=0.0),
        "time_to_goal_s": min(goal_times) if goal_times else math.nan,
        "path_m": path_length,
        "path_efficiency": efficiency,
        "min_clearance_m": min(clearance, default=math.nan),
        "cte_rms_m": math.sqrt(statistics.fmean(value * value for value in cte)) if cte else math.nan,
        "cte_p95_m": percentile(cte, 95),
        "peak_xy_speed_m_s": max(speeds, default=math.nan),
        "peak_accel_m_s2": max(accelerations, default=math.nan),
        "compute_p50_ms": percentile(compute, 50),
        "compute_p95_ms": percentile(compute, 95),
        "compute_p99_ms": percentile(compute, 99),
        "compute_max_ms": max(compute, default=math.nan),
        "deadline_misses": deadline_misses,
        "min_safe_samples": min(safe_samples, default=math.nan),
        "median_safe_samples": statistics.median(safe_samples) if safe_samples else math.nan,
        "ess_min": min(ess, default=math.nan),
        "ess_median": statistics.median(ess) if ess else math.nan,
        "best_feasible_cost_min": min(feasible_costs, default=math.nan),
        "best_feasible_cost_median": statistics.median(feasible_costs) if feasible_costs else math.nan,
        "no_safe_trajectory_events": sum(mode == "NO_SAFE_TRAJECTORY" for mode in local_modes),
        "disconnected_events": sum(state == "DISCONNECTED" for state in adapter_states),
        "stale_command_events": sum(state == "STALE_COMMAND" for state in adapter_states),
        "collision": collision,
        "late_command_accepted": late_command_accepted,
        "stale_command_violations": stale_command_violations,
        "setpoint_while_disarmed": setpoint_while_disarmed,
        "setpoint_in_wrong_mode": setpoint_in_wrong_mode,
        "replan_latency_ms": replan_latency_ms,
        "replan_to_command_ms": replan_to_command_ms,
    }


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return "" if math.isnan(value) else f"{value:.6g}"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def write_outputs(root: Path, summaries: list[dict[str, Any]]) -> None:
    root.mkdir(parents=True, exist_ok=True)
    with (root / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in summaries:
            writer.writerow({key: format_value(row.get(key, "")) for key in SUMMARY_FIELDS})

    by_scenario: dict[str, list[dict[str, Any]]] = {}
    for row in summaries:
        by_scenario.setdefault(str(row["scenario"]), []).append(row)
    lines = ["# M7 validation result summary", "",
             "Generated from immutable per-run `events.jsonl` and `manifest.json` files.", "",
             "| Scenario | Runs | Success | Collision | Deadline misses | Late accepted | Stale setpoint | Disarmed setpoint | Wrong-mode setpoint | Compute p99 worst (ms) | Min clearance (m) |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for scenario, rows in sorted(by_scenario.items()):
        p99 = _finite(row["compute_p99_ms"] for row in rows)
        clearance = _finite(row["min_clearance_m"] for row in rows)
        lines.append(
            f"| {scenario} | {len(rows)} | {sum(bool(r['success']) for r in rows) / len(rows):.1%} | "
            f"{sum(bool(r['collision']) for r in rows)} | "
            f"{sum(int(r['deadline_misses']) for r in rows)} | "
            f"{sum(int(r['late_command_accepted']) for r in rows)} | "
            f"{sum(int(r['stale_command_violations']) for r in rows)} | "
            f"{sum(int(r['setpoint_while_disarmed']) for r in rows)} | "
            f"{sum(int(r['setpoint_in_wrong_mode']) for r in rows)} | "
            f"{format_value(max(p99, default=math.nan))} | "
            f"{format_value(min(clearance, default=math.nan))} |"
        )
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path, nargs="?", default=Path("results/m7"))
    parser.add_argument("--include-debug", action="store_true",
                        help="include runs recorded with visualization enabled")
    args = parser.parse_args()
    paths = sorted(args.results.glob("**/events.jsonl"))
    if not paths:
        parser.error(f"no events.jsonl below {args.results}")
    summaries = [summarize_run(path) for path in paths]
    if not args.include_debug:
        summaries = [row for row in summaries if row["performance_mode"]]
    if not summaries:
        parser.error("no performance runs found (use --include-debug to inspect debug runs)")
    write_outputs(args.results, summaries)
    print(f"wrote {args.results / 'summary.csv'} and {args.results / 'REPORT.md'} ({len(summaries)} runs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""M7.3 root-cause tables from focused campaign JSONL runs.

This script consumes the per-run ``events.jsonl``/``manifest.json`` files written
by ``run_m7_scenario.py`` after the M7.3 instrumentation commit. It produces the
evidence tables needed to explain the S03 feasibility collapse, the S05 margin
loss, the S08 path handoff and the baseline planner-deadline tails.

Outputs (written into the results root):

- ``run_metrics.csv``: one row per run with turn, margin and rejection metrics;
- ``planner_timeouts.csv``: one row per ``PLANNER_TIMEOUT`` cycle with a timing
  classification;
- ``s08_handoff.csv``: goal_id/global_path_id/local active path joins;
- ``m73_summary.md``: human-readable summary tables.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable, Iterator


def number(value: Any, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite(values: Iterable[Any]) -> list[float]:
    output = []
    for value in values:
        parsed = number(value)
        if math.isfinite(parsed):
            output.append(parsed)
    return output


def percentile(values: Iterable[float], percentage: float) -> float:
    data = sorted(finite(values))
    if not data:
        return math.nan
    position = (len(data) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return data[lower]
    fraction = position - lower
    return data[lower] * (1.0 - fraction) + data[upper] * fraction


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def iter_runs(root: Path) -> Iterator[tuple[Path, list[dict[str, Any]], dict[str, Any]]]:
    for events in sorted(root.glob("**/events.jsonl")):
        manifest_path = events.with_name("manifest.json")
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        yield events.parent, read_jsonl(events), manifest


def run_cycles(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    starts = [index for index, row in enumerate(rows)
              if row.get("event") == "run_started"]
    results = [index for index, row in enumerate(rows)
               if row.get("event") == "run_result"]
    start = starts[0] if starts else 0
    end = results[-1] + 1 if results else len(rows)
    return [row for row in rows[start:end]
            if row.get("event") == "control_cycle"]


def planner_cycles(cycles: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in cycles
            if number(row.get("controller", {}).get("samples"), 0.0) > 0.0]


def rejection_totals(cycles: list[dict[str, Any]]) -> dict[str, int]:
    keys = ["reject_non_finite", "reject_swept_collision",
            "reject_static_collision", "reject_dynamic_collision",
            "reject_stopping_distance", "reject_other"]
    return {key: int(sum(finite(row.get("controller", {}).get(key)
                                for row in cycles))) for key in keys}


def proposal_min(cycles: list[dict[str, Any]], key: str) -> float:
    return min(finite(row.get("controller", {}).get(key) for row in cycles),
               default=math.nan)


def first_index(cycles: list[dict[str, Any]], predicate) -> int | None:
    for index, row in enumerate(cycles):
        if predicate(row):
            return index
    return None


def controller_value(row: dict[str, Any], key: str, default: float = math.nan) -> float:
    return number(row.get("controller", {}).get(key), default)


def extract_run_metrics(run_dir: Path, rows: list[dict[str, Any]],
                        manifest: dict[str, Any]) -> dict[str, Any]:
    cycles = run_cycles(rows)
    solved = planner_cycles(cycles)
    result = next((row for row in rows if row.get("event") == "run_result"), {})
    local_modes = [row.get("controller", {}).get("mode") for row in cycles]
    velocities = [row.get("state", {}).get("velocity_enu_m_s", []) for row in solved]
    peak_speed = max((math.hypot(number(v[0]), number(v[1]))
                      for v in velocities if len(v) == 3), default=math.nan)
    goal_times = [number(row.get("elapsed_s"))
                  for row in cycles if row.get("controller", {}).get("mode") == "GOAL_REACHED"]

    collapse = first_index(solved, lambda row: controller_value(row, "N_safe", 1.0) <= 0.0)
    turn_apex = None
    if solved:
        turn_apex = max(range(len(solved)),
                        key=lambda i: controller_value(solved[i], "path_curvature_per_m", 0.0))
    heading_index = None
    if solved:
        heading_index = max(range(len(solved)),
                            key=lambda i: abs(controller_value(solved[i], "heading_error_rad", 0.0)))

    def collapse_field(key: str, default: float = math.nan) -> float:
        if collapse is None:
            return default
        return controller_value(solved[collapse], key, default)

    rejections = rejection_totals(solved)
    collapse_rejections: dict[str, int] = {}
    if collapse is not None:
        window = solved[max(0, collapse - 2):collapse + 3]
        collapse_rejections = rejection_totals(window)
    dominant = max(collapse_rejections, key=collapse_rejections.get) \
        if collapse_rejections and max(collapse_rejections.values()) > 0 else "none"

    compute = finite(controller_value(row, "t_total_ms") for row in solved)
    scheduling = finite(controller_value(row, "t_scheduling_delay_ms") for row in solved)
    return {
        "scenario": manifest.get("scenario", run_dir.parent.name),
        "variant": manifest.get("variant") or "base",
        "run": manifest.get("run", run_dir.name),
        "seed": manifest.get("seed", ""),
        "success": bool(result.get("success", False)),
        "terminal_reason": result.get("reason", ""),
        "n_cycles": len(cycles),
        "n_planner_cycles": len(solved),
        "time_to_goal_s": min(goal_times) if goal_times else math.nan,
        "peak_xy_speed_m_s": peak_speed,
        "min_N_safe": min(finite(controller_value(row, "N_safe") for row in solved),
                          default=math.nan),
        "no_safe_cycles": sum(1 for row in solved if controller_value(row, "N_safe", 1.0) <= 0.0),
        "first_no_safe_elapsed_s": (number(solved[collapse].get("elapsed_s"))
                                    if collapse is not None else math.nan),
        "speed_at_collapse_m_s": collapse_field("speed_xy_m_s"),
        "heading_error_at_collapse_rad": collapse_field("heading_error_rad"),
        "curvature_at_collapse_per_m": collapse_field("path_curvature_per_m"),
        "stopping_distance_at_collapse_m": collapse_field("stopping_distance_m"),
        "min_collision_clearance_m": min(finite(
            controller_value(row, "minimum_collision_clearance_m") for row in solved),
            default=math.nan),
        "min_stopping_clearance_m": min(finite(
            controller_value(row, "minimum_stopping_clearance_m") for row in solved),
            default=math.nan),
        "reject_dominant_at_collapse": dominant,
        "reject_non_finite_total": rejections["reject_non_finite"],
        "reject_swept_collision_total": rejections["reject_swept_collision"],
        "reject_static_collision_total": rejections["reject_static_collision"],
        "reject_dynamic_collision_total": rejections["reject_dynamic_collision"],
        "reject_stopping_distance_total": rejections["reject_stopping_distance"],
        "reject_other_total": rejections["reject_other"],
        "gaussian_safe_at_collapse": collapse_field("gaussian_safe"),
        "reference_safe_at_collapse": collapse_field("reference_safe"),
        "braking_safe_at_collapse": collapse_field("braking_safe"),
        "recovery_safe_at_collapse": collapse_field("recovery_safe"),
        "specific_safe_at_collapse": collapse_field("specific_safe"),
        "min_gaussian_safe": proposal_min(solved, "gaussian_safe"),
        "min_reference_safe": proposal_min(solved, "reference_safe"),
        "min_braking_safe": proposal_min(solved, "braking_safe"),
        "min_recovery_safe": proposal_min(solved, "recovery_safe"),
        "min_specific_safe": proposal_min(solved, "specific_safe"),
        "apex_speed_xy_m_s": (controller_value(solved[turn_apex], "speed_xy_m_s")
                              if turn_apex is not None else math.nan),
        "apex_heading_error_rad": (controller_value(solved[turn_apex], "heading_error_rad")
                                   if turn_apex is not None else math.nan),
        "apex_curvature_per_m": (controller_value(solved[turn_apex], "path_curvature_per_m")
                                 if turn_apex is not None else math.nan),
        "max_abs_heading_error_rad": (abs(controller_value(solved[heading_index],
                                                           "heading_error_rad"))
                                      if heading_index is not None else math.nan),
        "max_scheduling_delay_ms": max(scheduling, default=math.nan),
        "max_compute_ms": max(compute, default=math.nan),
        "p99_compute_ms": percentile(compute, 99) if compute else math.nan,
        "planner_timeout_cycles": sum(mode == "PLANNER_TIMEOUT" for mode in local_modes),
        "active_path_id_max": max(finite(controller_value(row, "local_active_path_id")
                                         for row in solved), default=math.nan),
        "solves_on_active_path_max": max(finite(controller_value(row, "solves_on_active_path")
                                                for row in solved), default=math.nan),
        "run_dir": str(run_dir),
    }


TIMEOUT_FIELDS = [
    "timeout_id", "scenario", "variant", "seed", "cycle", "elapsed_s",
    "mode", "reason", "scheduling_delay_ms", "input_snapshot_ms",
    "path_reference_ms", "sampling_ms", "rollout_ms", "cost_ms", "safety_ms",
    "optimizer_update_ms", "conditioner_ms", "publish_ms", "diagnostics_ms",
    "total_ms", "wall_cycle_ms", "classification",
]


def classify_timeout(controller: dict[str, Any]) -> str:
    total = number(controller.get("t_total_ms"), 0.0)
    scheduling = number(controller.get("t_scheduling_delay_ms"), 0.0)
    compute = max(total - scheduling, 0.0)
    if scheduling >= max(20.0, 0.5 * total):
        return "A_callback_started_late"
    components = {
        "B_rollout": number(controller.get("t_rollout_ms"), 0.0),
        "C_safety": number(controller.get("t_safety_ms"), 0.0),
        "D_cost": number(controller.get("t_cost_ms"), 0.0),
        "E_io_logging": max(
            number(controller.get("t_conditioner_ms"), 0.0),
            number(controller.get("t_publish_ms"), 0.0),
            number(controller.get("t_diagnostics_ms"), 0.0),
            number(controller.get("t_input_snapshot_ms"), 0.0),
            number(controller.get("t_path_reference_ms"), 0.0),
            number(controller.get("t_sampling_ms"), 0.0),
            number(controller.get("t_optimizer_update_ms"), 0.0)),
    }
    dominant = max(components, key=components.get)
    if components[dominant] <= 0.0 or compute <= 0.0:
        return "F_other"
    return dominant


def collect_timeouts(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    timeout_id = 0
    for run_dir, raw, manifest in iter_runs(root):
        cycles = run_cycles(raw)
        for index, row in enumerate(cycles):
            controller = row.get("controller", {})
            if controller.get("mode") != "PLANNER_TIMEOUT":
                continue
            timeout_id += 1
            rows.append({
                "timeout_id": timeout_id,
                "scenario": manifest.get("scenario", run_dir.parent.name),
                "variant": manifest.get("variant") or "base",
                "seed": manifest.get("seed", ""),
                "cycle": index,
                "elapsed_s": number(row.get("elapsed_s")),
                "mode": controller.get("mode"),
                "reason": controller.get("reason"),
                "scheduling_delay_ms": number(controller.get("t_scheduling_delay_ms")),
                "input_snapshot_ms": number(controller.get("t_input_snapshot_ms")),
                "path_reference_ms": number(controller.get("t_path_reference_ms")),
                "sampling_ms": number(controller.get("t_sampling_ms")),
                "rollout_ms": number(controller.get("t_rollout_ms")),
                "cost_ms": number(controller.get("t_cost_ms")),
                "safety_ms": number(controller.get("t_safety_ms")),
                "optimizer_update_ms": number(controller.get("t_optimizer_update_ms")),
                "conditioner_ms": number(controller.get("t_conditioner_ms")),
                "publish_ms": number(controller.get("t_publish_ms")),
                "diagnostics_ms": number(controller.get("t_diagnostics_ms")),
                "total_ms": number(controller.get("t_total_ms")),
                "wall_cycle_ms": number(controller.get("t_wall_cycle_ms")),
                "classification": classify_timeout(controller),
            })
    return rows


HANDOFF_FIELDS = [
    "scenario", "variant", "seed", "run", "local_active_path_id",
    "goal_id", "global_path_id", "active_path_stamp_ns",
    "global_path_published_elapsed_s", "local_path_received_elapsed_s",
    "first_solve_elapsed_s", "first_safe_elapsed_s",
    "solve_latency_ms", "safe_latency_ms", "handoff_latency_ms",
]


def collect_handoffs(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for run_dir, raw, manifest in iter_runs(root):
        plans: dict[int, dict[str, Any]] = {}
        for row in raw:
            if row.get("event") != "global_planner":
                continue
            planner = row.get("global_planner", {})
            stamp = planner.get("path_stamp_ns")
            if stamp is None or planner.get("status") != "OK":
                continue
            plans[int(stamp)] = {
                "goal_id": planner.get("goal_id"),
                "global_path_id": planner.get("global_path_id"),
                "elapsed_s": number(row.get("elapsed_s")),
            }
        cycles = run_cycles(raw)
        seen: dict[int, dict[str, Any]] = {}
        for row in cycles:
            controller = row.get("controller", {})
            path_id = int(number(controller.get("local_active_path_id"), 0.0))
            if path_id <= 0:
                continue
            stamp = int(number(controller.get("active_path_stamp_ns"), 0.0))
            entry = seen.setdefault(path_id, {
                "scenario": manifest.get("scenario", run_dir.parent.name),
                "variant": manifest.get("variant") or "base",
                "seed": manifest.get("seed", ""),
                "run": manifest.get("run", run_dir.name),
                "local_active_path_id": path_id,
                "goal_id": plans.get(stamp, {}).get("goal_id"),
                "global_path_id": plans.get(stamp, {}).get("global_path_id"),
                "active_path_stamp_ns": stamp,
                "global_path_published_elapsed_s": plans.get(stamp, {}).get("elapsed_s"),
                "local_path_received_elapsed_s": number(row.get("elapsed_s")),
                "first_solve_elapsed_s": math.nan,
                "first_safe_elapsed_s": math.nan,
            })
            if controller.get("first_solve_on_active_path"):
                entry["first_solve_elapsed_s"] = number(row.get("elapsed_s"))
            if controller.get("first_safe_command_on_active_path"):
                entry["first_safe_elapsed_s"] = number(row.get("elapsed_s"))
        for entry in seen.values():
            if math.isfinite(entry["first_solve_elapsed_s"]):
                entry["solve_latency_ms"] = (
                    entry["first_solve_elapsed_s"] - entry["local_path_received_elapsed_s"]) * 1000.0
            else:
                entry["solve_latency_ms"] = math.nan
            if math.isfinite(entry["first_safe_elapsed_s"]):
                entry["safe_latency_ms"] = (
                    entry["first_safe_elapsed_s"] - entry["local_path_received_elapsed_s"]) * 1000.0
            else:
                entry["safe_latency_ms"] = math.nan
            if (math.isfinite(entry["first_solve_elapsed_s"]) and
                    math.isfinite(entry["global_path_published_elapsed_s"])):
                entry["handoff_latency_ms"] = (
                    entry["first_solve_elapsed_s"] -
                    entry["global_path_published_elapsed_s"]) * 1000.0
            else:
                entry["handoff_latency_ms"] = math.nan
            rows.append(entry)
    return rows


RUN_METRIC_FIELDS = [
    "scenario", "variant", "run", "seed", "success", "terminal_reason",
    "n_cycles", "n_planner_cycles", "time_to_goal_s", "peak_xy_speed_m_s",
    "min_N_safe", "no_safe_cycles", "first_no_safe_elapsed_s",
    "speed_at_collapse_m_s", "heading_error_at_collapse_rad",
    "curvature_at_collapse_per_m", "stopping_distance_at_collapse_m",
    "min_collision_clearance_m", "min_stopping_clearance_m",
    "reject_dominant_at_collapse", "reject_non_finite_total",
    "reject_swept_collision_total", "reject_static_collision_total",
    "reject_dynamic_collision_total", "reject_stopping_distance_total",
    "reject_other_total", "gaussian_safe_at_collapse",
    "reference_safe_at_collapse", "braking_safe_at_collapse",
    "recovery_safe_at_collapse", "specific_safe_at_collapse",
    "min_gaussian_safe", "min_reference_safe", "min_braking_safe",
    "min_recovery_safe", "min_specific_safe", "apex_speed_xy_m_s",
    "apex_heading_error_rad", "apex_curvature_per_m",
    "max_abs_heading_error_rad", "max_scheduling_delay_ms", "max_compute_ms",
    "p99_compute_ms", "planner_timeout_cycles", "active_path_id_max",
    "solves_on_active_path_max", "run_dir",
]


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return "" if math.isnan(value) else f"{value:.6g}"
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return ""
    return str(value)


def write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_value(row.get(key, "")) for key in fields})


def median(values: Iterable[float]) -> float:
    data = finite(values)
    return statistics.median(data) if data else math.nan


def markdown_table(headers: list[str], rows: list[list[Any]]) -> list[str]:
    lines = ["| " + " | ".join(headers) + " |",
             "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(format_value(value) for value in row) + " |")
    return lines


def build_summary(metrics: list[dict[str, Any]], timeouts: list[dict[str, Any]],
                  handoffs: list[dict[str, Any]]) -> str:
    lines = ["# M7.3 focused root-cause summary", "",
             "Generated from instrumented focused-campaign runs.", ""]
    lines.append("## Run outcomes")
    lines.append("")
    lines.extend(markdown_table(
        ["Scenario", "Runs", "Pass", "No-safe cycles",
         "Median peak speed", "Median min N_safe", "Median min collision clearance"],
        [[scenario,
          len(group),
          sum(bool(row["success"]) for row in group),
          sum(int(row["no_safe_cycles"]) for row in group),
          median(row["peak_xy_speed_m_s"] for row in group),
          median(row["min_N_safe"] for row in group),
          median(row["min_collision_clearance_m"] for row in group)]
         for scenario, group in _group(metrics, "scenario")]))
    lines.append("")
    lines.append("## Planner timeout attribution")
    lines.append("")
    if timeouts:
        counts: dict[str, int] = {}
        for row in timeouts:
            counts[row["classification"]] = counts.get(row["classification"], 0) + 1
        for classification, count in sorted(counts.items()):
            lines.append(f"- `{classification}`: {count}")
    else:
        lines.append("- no `PLANNER_TIMEOUT` cycles recorded")
    lines.append("")
    lines.append("## Path handoffs")
    lines.append("")
    lines.extend(markdown_table(
        ["Scenario", "Seed", "local id", "goal id", "global id",
         "solve latency (ms)", "safe latency (ms)", "handoff latency (ms)"],
        [[row["scenario"], row["seed"], row["local_active_path_id"],
          row["goal_id"], row["global_path_id"], row["solve_latency_ms"],
          row["safe_latency_ms"], row["handoff_latency_ms"]]
         for row in sorted(handoffs,
                           key=lambda item: (str(item["scenario"]), str(item["seed"]),
                                             int(item["local_active_path_id"])))]))
    lines.append("")
    return "\n".join(lines) + "\n"


def _group(rows: list[dict[str, Any]], key: str) -> list[tuple[Any, list[dict[str, Any]]]]:
    grouped: dict[Any, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(row.get(key), []).append(row)
    return sorted(grouped.items())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    root = args.results
    if not (root / "events.jsonl").exists() and not any(root.glob("**/events.jsonl")):
        parser.error(f"no events.jsonl below {root}")
    metrics = [extract_run_metrics(run_dir, rows, manifest)
               for run_dir, rows, manifest in iter_runs(root)]
    timeouts = collect_timeouts(root)
    handoffs = collect_handoffs(root)
    write_csv(root / "run_metrics.csv", RUN_METRIC_FIELDS, metrics)
    write_csv(root / "planner_timeouts.csv", TIMEOUT_FIELDS, timeouts)
    write_csv(root / "s08_handoff.csv", HANDOFF_FIELDS, handoffs)
    (root / "m73_summary.md").write_text(build_summary(metrics, timeouts, handoffs),
                                         encoding="utf-8")
    print(f"wrote run_metrics.csv ({len(metrics)}), planner_timeouts.csv "
          f"({len(timeouts)}), s08_handoff.csv ({len(handoffs)}) and "
          f"m73_summary.md into {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

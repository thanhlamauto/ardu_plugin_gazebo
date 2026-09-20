#!/usr/bin/env python3
"""Aggregate an M7.4 A/B stabilization campaign into comparison tables.

The campaign root contains one directory per arm (for example A, B, C, D), each
holding scenario sub-directories produced by run_m7_scenario.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_m73_root_cause import (  # noqa: E402
    RUN_METRIC_FIELDS,
    extract_run_metrics,
    finite,
    format_value,
    iter_runs,
    median,
)


def collect_arm(arm_dir: Path) -> list[dict[str, Any]]:
    rows = []
    for run_dir, raw, manifest in iter_runs(arm_dir):
        metrics = extract_run_metrics(run_dir, raw, manifest)
        metrics["arm"] = arm_dir.name
        rows.append(metrics)
    return rows


def group(rows: list[dict[str, Any]], *keys: str):
    grouped: dict[tuple, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row.get(key) for key in keys), []).append(row)
    return sorted(grouped.items(), key=lambda item: tuple(str(x) for x in item[0]))


def margin_label(row: dict[str, Any]) -> str:
    if row.get("sim_infra_failure"):
        return "sim_infra_failure"
    if not row["success"]:
        return "fail"
    return "clean_pass" if float(row["no_safe_cycles"] or 0) == 0 \
        else "functional_pass_margin_concern"


def is_controller_outcome(row: dict[str, Any]) -> bool:
    """Only valid runs count as controller outcomes; infra failures do not."""
    return margin_label(row) in ("clean_pass", "functional_pass_margin_concern")


def build_report(rows: list[dict[str, Any]]) -> str:
    lines = ["# M7.4 focused A/B stabilization summary", "",
             "Generated from instrumented runs. `A` is the frozen baseline, `B`",
             "speed shaping, `C` stopping recovery, `D` both.", ""]
    lines.append("## Per-arm / scenario")
    lines.append("")
    lines.append("| Arm | Scenario | Runs | Pass | Clean pass | Margin concern | "
                 "Sim infra | Median peak speed | Median no-safe cycles | Min collision clearance | "
                 "Min stopping clearance | Recovery entries | Median p99 compute |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for (arm, scenario), group_rows in group(rows, "arm", "scenario"):
        passes = [row for row in group_rows if is_controller_outcome(row)]
        clean = [row for row in group_rows if margin_label(row) == "clean_pass"]
        concern = [row for row in group_rows if margin_label(row) == "functional_pass_margin_concern"]
        infra = [row for row in group_rows if margin_label(row) == "sim_infra_failure"]
        lines.append(
            f"| {arm} | {scenario} | {len(group_rows)} | {len(passes)} | {len(clean)} | "
            f"{len(concern)} | {len(infra)} | "
            f"{format_value(median([float(r['peak_xy_speed_m_s'] or 'nan') for r in group_rows]))} | "
            f"{format_value(median([float(r['no_safe_cycles'] or 'nan') for r in group_rows]))} | "
            f"{format_value(min(finite([r['min_collision_clearance_m'] for r in group_rows]), default=math.nan))} | "
            f"{format_value(min(finite([r['min_stopping_clearance_m'] for r in group_rows]), default=math.nan))} | "
            f"{int(sum(float(r['stopping_recovery_entries'] or 0) for r in group_rows))} | "
            f"{format_value(median([float(r['p99_compute_ms'] or 'nan') for r in group_rows]))} |")
    lines.append("")

    lines.append("## S03 turn-entry comparison")
    lines.append("")
    lines.append("| Arm | Runs | Pass | Median speed at collapse | Median heading err | "
                 "Dominant rejection at collapse | Median apex speed |")
    lines.append("|---|---:|---:|---:|---:|---|---:|")
    for (arm,), group_rows in group([r for r in rows if r["scenario"] == "S03"], "arm"):
        counts: dict[str, int] = {}
        for row in group_rows:
            key = row["reject_dominant_at_collapse"]
            counts[key] = counts.get(key, 0) + 1
        dominant = ", ".join(f"{k}:{v}" for k, v in sorted(counts.items()))
        lines.append(
            f"| {arm} | {len(group_rows)} | "
            f"{sum(1 for r in group_rows if is_controller_outcome(r))} | "
            f"{format_value(median([float(r['speed_at_collapse_m_s'] or 'nan') for r in group_rows]))} | "
            f"{format_value(median([abs(float(r['heading_error_at_collapse_rad'] or 'nan')) for r in group_rows]))} | "
            f"{dominant} | "
            f"{format_value(median([float(r['apex_speed_xy_m_s'] or 'nan') for r in group_rows]))} |")
    lines.append("")

    lines.append("## S05 margin comparison")
    lines.append("")
    lines.append("| Arm | Runs | Pass | Clean pass | Margin concern | Min clearance | "
                 "Median no-safe cycles | Recovery entries |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for (arm,), group_rows in group([r for r in rows if r["scenario"] == "S05"], "arm"):
        passes = [row for row in group_rows if is_controller_outcome(row)]
        clean = [row for row in group_rows if margin_label(row) == "clean_pass"]
        concern = [row for row in group_rows if margin_label(row) == "functional_pass_margin_concern"]
        lines.append(
            f"| {arm} | {len(group_rows)} | {len(passes)} | {len(clean)} | {len(concern)} | "
            f"{format_value(min(finite([r['min_collision_clearance_m'] for r in group_rows]), default=math.nan))} | "
            f"{format_value(median([float(r['no_safe_cycles'] or 'nan') for r in group_rows]))} | "
            f"{int(sum(float(r['stopping_recovery_entries'] or 0) for r in group_rows))} |")
    lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    root = args.results
    arms = [path for path in sorted(root.iterdir()) if path.is_dir()]
    if not arms:
        parser.error(f"no arm directories in {root}")
    rows: list[dict[str, Any]] = []
    for arm in arms:
        rows.extend(collect_arm(arm))
    fields = ["arm"] + RUN_METRIC_FIELDS
    with (root / "m74_ab_runs.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_value(row.get(key, "")) for key in fields})
    (root / "m74_ab_report.md").write_text(build_report(rows), encoding="utf-8")
    print(f"wrote m74_ab_runs.csv ({len(rows)} runs) and m74_ab_report.md into {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

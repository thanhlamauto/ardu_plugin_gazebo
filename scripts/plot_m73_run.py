#!/usr/bin/env python3
"""Plot the M7.3 feasibility, rejection and turn-entry signals for one run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else math.nan
    except (TypeError, ValueError):
        return math.nan


REJECT_KEYS = [
    ("reject_swept_collision", "swept cloud"),
    ("reject_static_collision", "known map"),
    ("reject_stopping_distance", "stopping distance"),
    ("reject_non_finite", "non-finite"),
    ("reject_other", "other"),
]
PROPOSAL_KEYS = [
    ("gaussian_safe", "gaussian"),
    ("reference_safe", "reference"),
    ("braking_safe", "braking"),
    ("recovery_safe", "recovery"),
    ("specific_safe", "specific"),
]


def series(cycles, key):
    return [number(row.get("controller", {}).get(key)) for row in cycles]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("events", type=Path, help="path to events.jsonl")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        parser.error(f"matplotlib is required: {error}")
    rows = [json.loads(line) for line in args.events.read_text().splitlines()
            if line.strip()]
    starts = [index for index, row in enumerate(rows)
              if row.get("event") == "run_started"]
    results = [index for index, row in enumerate(rows)
               if row.get("event") == "run_result"]
    start = starts[0] if starts else 0
    end = results[-1] + 1 if results else len(rows)
    cycles = [row for row in rows[start:end]
              if row.get("event") == "control_cycle"
              and number(row.get("controller", {}).get("samples")) > 0]
    if not cycles:
        parser.error("no solved control_cycle records")
    t = [number(row.get("elapsed_s")) for row in cycles]
    figure, axes = plt.subplots(6, 1, sharex=True, figsize=(11, 14))
    axes[0].plot(t, series(cycles, "N_total"), label="N_total", linewidth=1.0)
    axes[0].plot(t, series(cycles, "N_safe"), label="N_safe", linewidth=1.4)
    axes[0].set_ylabel("samples")
    axes[0].legend(loc="upper right")
    for key, label in REJECT_KEYS:
        axes[1].plot(t, series(cycles, key), label=label, linewidth=1.0)
    axes[1].set_ylabel("rejected samples")
    axes[1].legend(loc="upper right", fontsize=8)
    for key, label in PROPOSAL_KEYS:
        axes[2].plot(t, series(cycles, key), label=label, linewidth=1.0)
    axes[2].set_ylabel("safe proposals")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[3].plot(t, series(cycles, "minimum_collision_clearance_m"),
                 label="collision", linewidth=1.2)
    axes[3].plot(t, series(cycles, "minimum_stopping_clearance_m"),
                 label="stopping", linewidth=1.2)
    axes[3].plot(t, series(cycles, "stopping_distance_m"),
                 label="required stop length", linewidth=1.0, linestyle="--")
    axes[3].set_ylabel("clearance (m)")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[4].plot(t, series(cycles, "speed_xy_m_s"), label="XY speed", linewidth=1.2)
    axes[4].plot(t, series(cycles, "heading_error_rad"), label="heading error", linewidth=1.2)
    axes[4].set_ylabel("speed (m/s), error (rad)")
    axes[4].legend(loc="upper right", fontsize=8)
    axes[5].plot(t, series(cycles, "t_scheduling_delay_ms"),
                 label="scheduling delay", linewidth=1.0)
    axes[5].plot(t, series(cycles, "t_total_ms"), label="compute total", linewidth=1.0)
    axes[5].set_ylabel("ms")
    axes[5].set_xlabel("Elapsed time (s)")
    axes[5].legend(loc="upper right", fontsize=8)
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.suptitle(args.events.parent.parent.name + " / " + args.events.parent.name)
    figure.tight_layout()
    output = args.output or args.events.with_name("m73_timeseries.png")
    figure.savefig(output, dpi=160)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

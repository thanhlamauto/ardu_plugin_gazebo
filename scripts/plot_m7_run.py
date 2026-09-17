#!/usr/bin/env python3
"""Plot the M7 narrow-passage signals from one structured run."""

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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("events", type=Path, help="path to events.jsonl")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        parser.error(f"matplotlib is required: {error}")
    rows = [json.loads(line) for line in args.events.read_text().splitlines() if line.strip()]
    cycles = [row for row in rows if row.get("event") == "control_cycle"]
    if not cycles:
        parser.error("no control_cycle records")
    t = [number(row.get("elapsed_s")) for row in cycles]
    clearance = [number(row.get("controller", {}).get("minimum_clearance_m")) for row in cycles]
    safe = [number(row.get("controller", {}).get("safe_samples")) for row in cycles]
    speed = []
    cost = [number(row.get("controller", {}).get("best_feasible_cost")) for row in cycles]
    for row in cycles:
        velocity = row.get("state", {}).get("velocity_enu_m_s", [math.nan] * 3)
        speed.append(math.hypot(number(velocity[0]), number(velocity[1])))
    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    for axis, values, label in zip(
            axes, (clearance, safe, speed, cost),
            ("Minimum clearance (m)", "N_safe", "XY speed (m/s)", "Best feasible cost")):
        axis.plot(t, values, linewidth=1.3)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time (s)")
    figure.suptitle(args.events.parent.parent.name + " / " + args.events.parent.name)
    figure.tight_layout()
    output = args.output or args.events.with_name("timeseries.png")
    figure.savefig(output, dpi=160)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Create post-run plots from MPPI JSONL diagnostics.

No synthetic values are inserted: missing channels are omitted and reported.
Multiple ``--run LABEL=path.jsonl`` arguments create the fair-comparison plot.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{number}: JSON không hợp lệ: {exc}") from exc
            if row.get("schema_version") != 1:
                raise SystemExit(f"{path}:{number}: schema_version phải bằng 1")
            rows.append(row)
    if not rows:
        raise SystemExit(f"{path}: không có record")
    return rows


def vector(rows, key, width):
    values = [row.get(key) for row in rows]
    if any(value is None or len(value) != width for value in values):
        return None
    return np.asarray(values, dtype=float)


def scalar(rows, key):
    values = [row.get(key) for row in rows]
    if any(value is None for value in values):
        return None
    return np.asarray(values, dtype=float)


def save(fig, output: Path, name: str):
    path = output / name
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    return path


def plot_run(label: str, rows: list[dict], output: Path) -> list[Path]:
    import matplotlib.pyplot as plt

    made = []
    t0 = float(rows[0]["timestamp_monotonic_s"])
    t = np.asarray([float(row["timestamp_monotonic_s"]) - t0 for row in rows])
    pos = vector(rows, "position_enu", 3)
    vel = vector(rows, "velocity_enu", 3)
    rates = vector(rows, "measured_body_rates_flu_rad_s", 3)
    thrust = scalar(rows, "thrust_normalized")
    clearance = scalar(rows, "minimum_clearance_m")
    goal = scalar(rows, "goal_dist_m")
    latency = np.asarray([float(row.get("compute_ms", np.nan)) for row in rows])

    if pos is not None:
        fig, ax = plt.subplots()
        ax.plot(pos[:, 0], pos[:, 1], label=label)
        ax.scatter(pos[0, 0], pos[0, 1], marker="o", label="start")
        ax.scatter(pos[-1, 0], pos[-1, 1], marker="x", label="end")
        ax.set(xlabel="East / ENU x [m]", ylabel="North / ENU y [m]", title="XY trajectory")
        ax.axis("equal"); ax.grid(True); ax.legend()
        made.append(save(fig, output, f"{label}_xy_trajectory.png")); plt.close(fig)

        fig, ax = plt.subplots()
        ax.plot(t, pos[:, 2])
        ax.set(xlabel="Time [s]", ylabel="Up / ENU z [m]", title="Altitude")
        ax.grid(True)
        made.append(save(fig, output, f"{label}_altitude.png")); plt.close(fig)

    if vel is not None:
        fig, ax = plt.subplots()
        for index, axis in enumerate("xyz"):
            ax.plot(t, vel[:, index], label=f"v{axis}")
        ax.set(xlabel="Time [s]", ylabel="Velocity [m/s]", title="Measured velocity (ENU)")
        ax.grid(True); ax.legend()
        made.append(save(fig, output, f"{label}_velocity.png")); plt.close(fig)

    commands = vector(rows, "nominal_control", 4)
    if rates is not None and commands is not None and rows[0].get("control_kind") == "thrust-body-rates":
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(7, 7))
        for index, axis in enumerate("pqr"):
            axes[index].plot(t, commands[:, index + 1], label=f"{axis} cmd")
            axes[index].plot(t, rates[:, index], label=f"{axis} measured")
            axes[index].grid(True); axes[index].legend()
        axes[-1].set_xlabel("Time [s]")
        fig.suptitle("Commanded vs measured body rates (FLU)")
        made.append(save(fig, output, f"{label}_body_rates.png")); plt.close(fig)

    if thrust is not None:
        fig, ax = plt.subplots(); ax.plot(t, thrust)
        ax.set(xlabel="Time [s]", ylabel="Normalized thrust", title="Sent thrust")
        ax.grid(True)
        made.append(save(fig, output, f"{label}_thrust.png")); plt.close(fig)

    if clearance is not None or goal is not None:
        series = [("Clearance [m]", clearance), ("Goal distance [m]", goal)]
        series = [(name, values) for name, values in series if values is not None]
        fig, axes = plt.subplots(len(series), 1, sharex=True, squeeze=False)
        axes = axes[:, 0]
        for ax, (name, values) in zip(axes, series):
            ax.plot(t, values); ax.set_ylabel(name); ax.grid(True)
        axes[-1].set_xlabel("Time [s]")
        made.append(save(fig, output, f"{label}_clearance_goal.png")); plt.close(fig)

    fig, ax = plt.subplots(); ax.plot(t, latency)
    deadline = float(rows[0].get("timing", {}).get("deadline_ms", np.nan))
    if np.isfinite(deadline):
        ax.axhline(deadline, color="r", linestyle="--", label="deadline")
    ax.set(xlabel="Time [s]", ylabel="Planner compute [ms]", title="Planner latency")
    ax.grid(True); ax.legend()
    made.append(save(fig, output, f"{label}_latency.png")); plt.close(fig)
    return made


def comparison(runs: dict[str, list[dict]], output: Path) -> Path | None:
    if len(runs) < 2:
        return None
    import matplotlib.pyplot as plt

    labels, final_goal, min_clearance, mean_ms, p95_ms = [], [], [], [], []
    for label, rows in runs.items():
        labels.append(label)
        final_goal.append(float(rows[-1]["goal_dist_m"]))
        clearance = scalar(rows, "minimum_clearance_m")
        min_clearance.append(float(np.nanmin(clearance)) if clearance is not None else np.nan)
        latency = np.asarray([float(row.get("compute_ms", np.nan)) for row in rows])
        mean_ms.append(float(np.nanmean(latency)))
        p95_ms.append(float(np.nanpercentile(latency, 95)))
    values = np.asarray([final_goal, min_clearance, mean_ms, p95_ms]).T
    fig, axes = plt.subplots(2, 2, figsize=(9, 7))
    for ax, index, title, unit in zip(
        axes.flat, range(4),
        ("Final goal distance", "Minimum clearance", "Compute mean", "Compute p95"),
        ("m", "m", "ms", "ms"),
    ):
        ax.bar(labels, values[:, index]); ax.set_title(title); ax.set_ylabel(unit); ax.grid(axis="y")
    return save(fig, output, "mppi_vs_pa_mppi_comparison.png")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, metavar="LABEL=JSONL")
    parser.add_argument("--output-dir", default="output/plots")
    args = parser.parse_args()
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for item in args.run:
        if "=" not in item:
            raise SystemExit("--run phải có dạng LABEL=path.jsonl")
        label, path = item.split("=", 1)
        runs[label] = load_jsonl(Path(path))
    paths = []
    for label, rows in runs.items():
        paths.extend(plot_run(label, rows, output))
    compared = comparison(runs, output)
    if compared:
        paths.append(compared)
    print("\n".join(str(path) for path in paths))


if __name__ == "__main__":
    main()

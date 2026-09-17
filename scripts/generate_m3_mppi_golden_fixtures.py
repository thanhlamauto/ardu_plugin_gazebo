#!/usr/bin/env python3
"""Generate deterministic MPPI dynamics fixtures from the Python baseline."""
import json
from pathlib import Path

import numpy as np
import torch

from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI


COMMON = dict(
    tau=0.5,
    command_alpha=0.30,
    max_accel_xy=3.0,
    max_accel_z=0.4,
    max_yaw_accel=0.6,
    vmax=10.0,
    vzmax=0.6,
    yaw_rate_max=0.6,
    response_accel_xy=3.0,
    response_jerk_xy=4.0,
)
START_STAMP_NS = 1_000_000_000


def state(position=(0, 0, 5), velocity=(0, 0, 0), yaw=0.0,
          applied=(0, 0, 0, 0), acceleration=(0, 0, 0), response=True):
    values = [*position, *velocity, yaw, *applied]
    if response:
        values.extend(acceleration)
    return np.asarray(values, dtype=np.float64)


def write_fixture(output, name, controls, *, dt=0.1, initial=None,
                  response=True, **overrides):
    params = dict(COMMON)
    params.update(overrides)
    params.update(dt=dt, response_accel_model=response)
    controls = np.asarray(controls, dtype=np.float64).reshape(-1, 4)
    if initial is None:
        initial = state(response=response)
    initial = np.asarray(initial, dtype=np.float64)
    expected_width = 14 if response else 11
    if initial.shape != (expected_width,):
        raise ValueError(f"{name}: initial state must have width {expected_width}")

    planner = QuadMPPI(MPPIConfig(
        horizon=max(2, len(controls)), samples=4, device="cpu", **params))
    current = torch.as_tensor(initial, dtype=torch.double)
    expected = [current.numpy().copy()]
    for index, control in enumerate(controls):
        current = planner._dynamics(
            current, torch.as_tensor(control, dtype=torch.double), index)
        expected.append(current.detach().numpy().copy())

    dt_ns = round(dt * 1e9)
    payload = {
        "dt": dt,
        "tau": params["tau"],
        "command_alpha": params["command_alpha"],
        "max_accel_xy": params["max_accel_xy"],
        "max_accel_z": params["max_accel_z"],
        "max_yaw_accel": params["max_yaw_accel"],
        "vmax": params["vmax"],
        "vzmax": params["vzmax"],
        "yaw_rate_max": params["yaw_rate_max"],
        "response_accel_model": response,
        "response_accel_xy": params["response_accel_xy"],
        "response_jerk_xy": params["response_jerk_xy"],
        "state_width": expected_width,
        "initial_stamp_ns": START_STAMP_NS,
        "initial_state_flat": initial.tolist(),
        "controls_flat": controls.reshape(-1).tolist(),
        "expected_states_flat": np.asarray(expected).reshape(-1).tolist(),
        "expected_times_flat": (np.arange(len(expected)) * dt).tolist(),
        "expected_stamps_flat": [START_STAMP_NS + i * dt_ns
                                  for i in range(len(expected))],
    }
    (output / f"{name}.json").write_text(json.dumps(payload, indent=2) + "\n")


def repeated(control, count):
    return [control for _ in range(count)]


def main():
    output = (Path(__file__).resolve().parents[1] / "uav_navigation_core" /
              "test" / "fixtures" / "mppi")
    output.mkdir(parents=True, exist_ok=True)

    write_fixture(output, "hover", repeated([0, 0, 0, 0], 5))
    write_fixture(output, "forward", repeated([8, 0, 0, 0], 12))
    write_fixture(output, "lateral", repeated([0, -6, 0, 0], 10))
    write_fixture(output, "climb", repeated([0, 0, 0.6, 0], 8))
    write_fixture(output, "descend", repeated([0, 0, -0.6, 0], 8),
                  initial=state(position=(0, 0, 8)))
    write_fixture(output, "yaw", repeated([0, 0, 0, 0.6], 10),
                  initial=state(yaw=3.0))
    mixed = [[4, -2, 0.5, 0.4], [4, 2, -0.4, -0.3],
             [-3, 2, 0.2, 0.6], [0, 0, 0, 0]] * 3
    write_fixture(output, "mixed_control", mixed,
                  initial=state(position=(1, -2, 4), velocity=(2, -1, 0.2),
                                yaw=-0.7, applied=(1.5, -0.5, 0.1, 0.2),
                                acceleration=(0.8, -0.3, 0.1)))
    long_controls = [[6*np.sin(i*.13), 5*np.cos(i*.09),
                      .6*np.sin(i*.07), .6*np.cos(i*.11)] for i in range(120)]
    write_fixture(output, "long_horizon", long_controls)
    write_fixture(output, "control_bounds", repeated([50, -50, 5, 5], 8),
                  response=False, initial=state(response=False), dt=0.2,
                  command_alpha=1.0, max_accel_xy=100.0,
                  max_accel_z=100.0, max_yaw_accel=100.0)
    write_fixture(output, "large_dt", [[3, 1, .4, .5], [-2, 0, 0, -.5]],
                  response=False, initial=state(response=False), dt=0.8)
    write_fixture(output, "small_dt", repeated([2, 1, .2, .3], 10), dt=0.001)
    write_fixture(output, "single_step", [[2, -1, .3, -.2]])


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Simulator health monitor for the M7 harness (M7.4b).

The Gazebo<->ArduPilot FDM stream already reports its own failures in the
Gazebo server log (the Harness launch log): "Broken ArduPilot connection",
"Duplicate input frame", "Missed N input frames", "ArduPilot controller has
reset", "Incorrect protocol magic", and the positive "Connected to ArduPilot
controller". This module combines those with /clock, odometry, and the MAVROS
adapter state so a run can be gated on a healthy simulator and labelled
SIM_INFRA_FAILURE when the platform, not the controller, lost the stream.

Pure observation logic: every call takes an explicit wall-clock time so the
monitor is unit-testable without ROS or Gazebo.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# Any of these means the plugin lost part of the FDM stream.
FDM_LOSS_MARKERS = (
    "Broken ArduPilot connection",
    "Duplicate input frame",
    "input frames",
    "ArduPilot controller has reset",
    "Incorrect protocol magic",
)
# A severe marker latches the run as SIM_INFRA_FAILURE immediately instead of
# racing the unhealthy-window timeout. The stream is gone for good, not jittery.
FDM_SEVERE_MARKERS = (
    "Broken ArduPilot connection",
    "ArduPilot controller has reset",
    "Incorrect protocol magic",
)
FDM_CONNECT_MARKER = "Connected to ArduPilot controller"
FDM_HEALTH_MARKER = "[fdm_health]"
_FDM_HEALTH_KV = re.compile(r"(\w+)=([-\d.eE+]+)")

SIM_INFRA_FAILURE = "SIM_INFRA_FAILURE"


def parse_fdm_health_text(text: str) -> dict[str, float] | None:
    """Return the last plugin fdm_health line's numeric fields, if any.

    The plugin emits one line per simulated second, for example:
    ``[fdm_health] model=iris online=1 rx_hz=250 rx_total=1 rx_age_s=0.01
    missed=0 timeouts=0 sim_s=12``.
    """
    latest = None
    for line in text.splitlines():
        if FDM_HEALTH_MARKER not in line:
            continue
        body = line.split(FDM_HEALTH_MARKER, 1)[1]
        fields = {}
        for key, value in _FDM_HEALTH_KV.findall(body):
            try:
                fields[key] = float(value)
            except ValueError:
                continue
        if fields:
            latest = fields
    return latest


@dataclass
class HealthStatus:
    ok: bool
    reason: str
    diagnostics: dict[str, Any] = field(default_factory=dict)


class SimHealthMonitor:
    def __init__(self, odom_timeout_s: float = 1.0,
                 clock_timeout_s: float = 1.0,
                 adapter_timeout_s: float = 1.0,
                 fdm_loss_grace_s: float = 3.0,
                 fdm_health_timeout_s: float = 3.0,
                 fdm_rx_age_timeout_s: float = 1.0,
                 min_fdm_rx_hz: float = 1.0,
                 min_rtf: float = 0.05, max_rtf: float = 10.0,
                 min_sim_advance_s: float = 0.05):
        self.odom_timeout_s = odom_timeout_s
        self.clock_timeout_s = clock_timeout_s
        self.adapter_timeout_s = adapter_timeout_s
        self.fdm_loss_grace_s = fdm_loss_grace_s
        self.fdm_health_timeout_s = fdm_health_timeout_s
        self.fdm_rx_age_timeout_s = fdm_rx_age_timeout_s
        self.min_fdm_rx_hz = min_fdm_rx_hz
        self.min_rtf = min_rtf
        self.max_rtf = max_rtf
        self.min_sim_advance_s = min_sim_advance_s

        self.last_odom_wall: float | None = None
        self.last_clock_wall: float | None = None
        self.last_sim_time_s: float | None = None
        self.last_sim_time_wall: float | None = None
        self.sim_time_s: float | None = None
        self.rtf: float | None = None

        self.connected = False
        self.armed = False
        self.mode = ""
        self.adapter_wall: float | None = None

        self.fdm_loss_events = 0
        self.severe_fdm_loss_events = 0
        self.severe_fdm_loss = False
        self.last_fdm_loss_wall: float | None = None
        self.fdm_connected = False
        self.fdm_rx_hz: float | None = None
        self.fdm_rx_age_s: float | None = None
        self.fdm_missed: float | None = None
        self.fdm_timeouts: float | None = None
        self.fdm_sim_s: float | None = None
        self.last_fdm_health_wall: float | None = None
        self._log_handle: Any = None
        self._log_buffer = ""

    def open_launch_log(self, path: Path) -> None:
        try:
            self._log_handle = path.open(encoding="utf-8", errors="replace")
        except OSError:
            self._log_handle = None

    def observe_clock(self, sim_time_s: float, wall_s: float) -> None:
        if not math.isfinite(sim_time_s):
            return
        self.sim_time_s = sim_time_s
        self.last_clock_wall = wall_s
        if self.last_sim_time_s is None:
            self.last_sim_time_s = sim_time_s
            self.last_sim_time_wall = wall_s
            return
        previous_wall = (self.last_sim_time_wall
                         if self.last_sim_time_wall is not None else wall_s)
        dt_wall = wall_s - previous_wall
        dt_sim = sim_time_s - self.last_sim_time_s
        if dt_wall > 0.5:
            self.rtf = dt_sim / dt_wall
            self.last_sim_time_s = sim_time_s
            self.last_sim_time_wall = wall_s

    def observe_odometry(self, wall_s: float) -> None:
        self.last_odom_wall = wall_s

    def observe_adapter(self, connected: bool, armed: bool, mode: str,
                        wall_s: float) -> None:
        self.connected = bool(connected)
        self.armed = bool(armed)
        self.mode = mode or ""
        self.adapter_wall = wall_s

    def observe_log_text(self, text: str, wall_s: float) -> None:
        for marker in FDM_LOSS_MARKERS:
            count = text.count(marker)
            if count:
                self.fdm_loss_events += count
                self.last_fdm_loss_wall = wall_s
        for marker in FDM_SEVERE_MARKERS:
            count = text.count(marker)
            if count:
                self.severe_fdm_loss_events += count
                self.severe_fdm_loss = True
        if FDM_CONNECT_MARKER in text:
            self.fdm_connected = True
        if FDM_HEALTH_MARKER in text:
            haystack = self._log_buffer[-4096:] if self._log_buffer else text
            fields = parse_fdm_health_text(haystack)
            if fields:
                self.fdm_rx_hz = fields.get("rx_hz")
                self.fdm_rx_age_s = fields.get("rx_age_s")
                self.fdm_missed = fields.get("missed")
                self.fdm_timeouts = fields.get("timeouts")
                self.fdm_sim_s = fields.get("sim_s")
                self.last_fdm_health_wall = wall_s

    def scan_launch_log(self, wall_s: float) -> None:
        if self._log_handle is None:
            return
        new = self._log_handle.read()
        if not new:
            return
        self._log_buffer += new
        self.observe_log_text(new, wall_s)

    def evaluate(self, wall_s: float) -> HealthStatus:
        adapter_age = (wall_s - self.adapter_wall
                       if self.adapter_wall is not None else None)
        diagnostics = {
            "rtf": self.rtf,
            "sim_time_s": self.sim_time_s,
            "fdm_connected": self.fdm_connected,
            "fdm_loss_events": self.fdm_loss_events,
            "severe_fdm_loss_events": self.severe_fdm_loss_events,
            "fdm_rx_hz": self.fdm_rx_hz,
            "fdm_rx_age_s": self.fdm_rx_age_s,
            "fdm_missed": self.fdm_missed,
            "fdm_timeouts": self.fdm_timeouts,
            "adapter_age_s": adapter_age,
            "connected": self.connected,
            "armed": self.armed,
            "mode": self.mode,
        }
        if self.last_clock_wall is None:
            return HealthStatus(False, "no /clock received", diagnostics)
        clock_age = wall_s - self.last_clock_wall
        if clock_age > self.clock_timeout_s:
            return HealthStatus(
                False, f"sim clock stalled ({clock_age:.2f}s since /clock)",
                diagnostics)
        if self.last_odom_wall is None:
            return HealthStatus(False, "no odometry received", diagnostics)
        if wall_s - self.last_odom_wall > self.odom_timeout_s:
            return HealthStatus(False, "odometry stale", diagnostics)
        if self.adapter_wall is None:
            return HealthStatus(False, "no adapter status received", diagnostics)
        if adapter_age is not None and adapter_age > self.adapter_timeout_s:
            return HealthStatus(
                False, f"adapter status stale ({adapter_age:.2f}s)",
                diagnostics)
        if self.severe_fdm_loss:
            return HealthStatus(
                False,
                f"severe FDM loss latched ({self.severe_fdm_loss_events} events)",
                diagnostics)
        if self.last_fdm_health_wall is not None:
            if wall_s - self.last_fdm_health_wall > self.fdm_health_timeout_s:
                return HealthStatus(False, "FDM health telemetry stale",
                                    diagnostics)
            if (self.fdm_rx_age_s is not None and
                    self.fdm_rx_age_s > self.fdm_rx_age_timeout_s):
                return HealthStatus(
                    False, f"FDM RX stale ({self.fdm_rx_age_s:.2f}s)",
                    diagnostics)
            if (self.fdm_rx_hz is not None and
                    self.fdm_rx_hz < self.min_fdm_rx_hz):
                return HealthStatus(
                    False, f"FDM RX rate {self.fdm_rx_hz:.2f} Hz",
                    diagnostics)
        if not self.connected:
            return HealthStatus(False, "MAVROS/FCU not connected", diagnostics)
        if not self.armed:
            return HealthStatus(False, "vehicle not armed", diagnostics)
        if self.mode != "GUIDED":
            return HealthStatus(False, f"mode {self.mode!r} != GUIDED",
                                diagnostics)
        # The positive connect marker is not reliably captured from the Gazebo
        # server log, so FDM health is judged by loss events. A stream that
        # never connects leaves the FCU unarmed, caught above.
        if self.last_fdm_loss_wall is not None and \
                wall_s - self.last_fdm_loss_wall <= self.fdm_loss_grace_s:
            return HealthStatus(
                False,
                f"recent FDM loss ({wall_s - self.last_fdm_loss_wall:.2f}s ago)",
                diagnostics)
        if self.rtf is not None and \
                (self.rtf < self.min_rtf or self.rtf > self.max_rtf):
            return HealthStatus(
                False, f"real-time factor {self.rtf:.3f} out of range",
                diagnostics)
        if self.sim_time_s is not None and \
                self.sim_time_s < self.min_sim_advance_s:
            return HealthStatus(
                False, f"sim time {self.sim_time_s:.3f}s not advancing",
                diagnostics)
        return HealthStatus(True, "healthy", diagnostics)


def infra_failure_due(monitor: SimHealthMonitor,
                      unhealthy_since: float | None, wall_s: float,
                      timeout_s: float) -> tuple[bool, float | None, HealthStatus]:
    """Advance the runner's mid-run infra-failure decision.

    Returns (failure, new_unhealthy_since, status). A severe FDM loss latches
    immediately, so it cannot race the sustained-unhealthy timeout.
    """
    status = monitor.evaluate(wall_s)
    if status.ok:
        return False, None, status
    if monitor.severe_fdm_loss:
        return True, unhealthy_since, status
    if unhealthy_since is None:
        return False, wall_s, status
    if wall_s - unhealthy_since >= timeout_s:
        return True, unhealthy_since, status
    return False, unhealthy_since, status


def classify_run_result(sim_infra_failure: bool, success: bool,
                        reason: str) -> str:
    """Map a finished run to PASS / FAIL / SIM_INFRA_FAILURE.

    A platform failure (lost FDM/clock/odometry) is never a controller
    outcome, so it must not be counted against the controller.
    """
    if sim_infra_failure:
        return SIM_INFRA_FAILURE
    return "PASS" if success else "FAIL"

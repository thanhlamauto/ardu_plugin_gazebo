import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(name: str):
    path = ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module  # dataclasses inspect cls.__module__
    spec.loader.exec_module(module)
    return module


def fresh(monitor, wall):
    monitor.observe_clock(wall, wall)
    monitor.observe_odometry(wall)
    monitor.observe_adapter(True, True, "GUIDED", wall)


def test_classify_run_result_never_counts_platform_failure_against_controller():
    health = load("sim_health")
    assert health.classify_run_result(True, False, "x") == "SIM_INFRA_FAILURE"
    assert health.classify_run_result(True, True, "x") == "SIM_INFRA_FAILURE"
    assert health.classify_run_result(False, True, "x") == "PASS"
    assert health.classify_run_result(False, False, "x") == "FAIL"


def test_healthy_window_then_fdm_loss_is_flagged():
    health = load("sim_health")
    monitor = health.SimHealthMonitor(fdm_loss_grace_s=3.0)
    monitor.observe_log_text(health.FDM_CONNECT_MARKER, 0.0)
    monitor.observe_clock(0.0, 0.0)
    fresh(monitor, 0.0)
    fresh(monitor, 1.0)
    assert monitor.evaluate(1.0).ok

    monitor.observe_log_text("Duplicate input frame", 1.5)
    status = monitor.evaluate(1.5)
    assert not status.ok and "FDM" in status.reason
    assert status.diagnostics["fdm_loss_events"] == 1

    fresh(monitor, 2.0)
    assert not monitor.evaluate(2.0).ok, "loss still inside the grace window"
    fresh(monitor, 5.0)
    assert monitor.evaluate(5.0).ok, "healthy again after the grace window"


def test_clock_stall_and_stale_odometry():
    health = load("sim_health")
    monitor = health.SimHealthMonitor(odom_timeout_s=1.0, clock_timeout_s=1.0)
    monitor.observe_log_text(health.FDM_CONNECT_MARKER, 0.0)
    fresh(monitor, 0.0)
    assert "clock stalled" in monitor.evaluate(2.0).reason

    fresh(monitor, 2.0)
    monitor.observe_clock(4.0, 4.0)  # clock fresh, odometry stale since t=2
    status = monitor.evaluate(4.0)
    assert not status.ok and "odometry" in status.reason


def test_rtf_window_and_loss_marker_counts():
    health = load("sim_health")
    monitor = health.SimHealthMonitor()
    fresh(monitor, 0.0)
    monitor.observe_clock(0.0, 0.0)
    fresh(monitor, 0.0)
    monitor.observe_clock(2.0, 1.0)  # 2 s sim in 1 s wall -> rtf 2.0
    assert abs(monitor.evaluate(1.0).diagnostics["rtf"] - 2.0) < 1e-9
    monitor.observe_log_text("Missed 5 input frames\n", 1.0)
    assert monitor.evaluate(1.0).diagnostics["fdm_loss_events"] == 1

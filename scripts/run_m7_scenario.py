#!/usr/bin/env python3
"""Run and record one M7 ROS 2 scenario.

ArduPilot SITL must already be running. By default this script also starts the
headless navigation launch, waits until the vehicle is armed in GUIDED and
hovering above the configured altitude, publishes the scenario goal, and
writes JSONL data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "uav_navigation_bringup/config/m7_baseline.yaml"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_health import (  # noqa: E402
    SIM_INFRA_FAILURE,
    SimHealthMonitor,
    classify_run_result,
    infra_failure_due,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def load_scenario(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as error:
        raise RuntimeError("PyYAML is required to read M7 scenarios") from error
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("scenario must be a YAML mapping")
    required = {"schema_version", "id", "name", "world", "goal_enu_m", "timeout_s", "repetitions"}
    missing = sorted(required - value.keys())
    if missing:
        raise ValueError(f"scenario is missing: {', '.join(missing)}")
    if value["schema_version"] != 1:
        raise ValueError("unsupported scenario schema_version")
    goal = value["goal_enu_m"]
    if not isinstance(goal, list) or len(goal) != 3:
        raise ValueError("goal_enu_m must contain x, y, z")
    world = (ROOT / value["world"]).resolve()
    if not world.is_file() or ROOT not in world.parents:
        raise ValueError(f"world does not exist inside repository: {world}")
    value["_world_path"] = world
    return value


def git_commit() -> str:
    environment = os.environ.copy()
    # Conda's libiconv can break Homebrew Git on macOS when ROS requires a
    # custom DYLD_LIBRARY_PATH. Git does not need the ROS runtime libraries.
    environment.pop("DYLD_LIBRARY_PATH", None)
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
        env=environment)
    return result.stdout.strip() or "unknown"


def configure_rmw_environment(environment: dict[str, str] | None = None) -> dict[str, str]:
    """Disable the Fast DDS shared-memory transport unless the caller sets it.

    On this harness the shared-memory transport intermittently fails to
    initialize ("mutex lock failed"), which kills the ros_gz parameter_bridge
    processes during startup and leaves odometry/obstacles unobservable. UDPv4
    avoids the crash; an explicit caller value is preserved.
    """
    target = os.environ if environment is None else environment
    target.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    return target


def process_ids(pattern: str) -> list[int]:
    result = subprocess.run(["pgrep", "-f", pattern], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            check=False)
    return [int(value) for value in result.stdout.split() if int(value) != os.getpid()]


def process_group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop_process_group(process_group: int, grace_s: float = 10.0) -> None:
    """Stop every launch child even when the ros2 launch leader exited first."""
    for group_signal, wait_s in ((signal.SIGINT, grace_s),
                                 (signal.SIGTERM, 3.0),
                                 (signal.SIGKILL, 1.0)):
        try:
            os.killpg(process_group, group_signal)
        except (ProcessLookupError, PermissionError):
            return
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if not process_group_exists(process_group):
                return
            time.sleep(0.1)


class M7Recorder:
    def __init__(self, node: Any, scenario: dict[str, Any], stream: Any,
                 start_immediately: bool, allow_process_faults: bool,
                 readiness_timeout_s: float,
                 start_max_speed_m_s: float | None = None,
                 start_stable_s: float = 0.0,
                 launch_log_path: Path | None = None,
                 sim_health_timeout_s: float = 3.0,
                 soak_seconds: float = 0.0):
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.node = node
        self.scenario = scenario
        self.stream = stream
        self.start_immediately = start_immediately
        self.allow_process_faults = allow_process_faults
        self.readiness_timeout_s = readiness_timeout_s
        self.start_max_speed_m_s = start_max_speed_m_s
        self.start_stable_s = start_stable_s
        self.ready_since: float | None = None
        self.created = time.monotonic()
        self.started: float | None = None
        self.done = False
        self.success = False
        self.reason = ""
        self.latest_state: dict[str, Any] = {}
        self.latest_raw: dict[str, Any] = {}
        self.latest_conditioned: dict[str, Any] = {}
        self.latest_adapter: dict[str, Any] = {}
        self.latest_global: dict[str, Any] = {}
        self.last_raw_time: float | None = None
        self.last_conditioned_time: float | None = None
        self.raw_sequence = 0
        self.conditioned_sequence = 0
        self.mavros_setpoint_sequence = 0
        self.last_mavros_setpoint_time: float | None = None
        self.replans_sent: set[int] = set()
        self.fault_started = False
        self.fault_restored = False
        self.suspended_pids: list[int] = []
        # M7.4b simulator health gate. Fault-injection variants intentionally
        # break the FCU/mode, so reliability monitoring only applies to normal
        # scenario runs.
        self.monitor_health = launch_log_path is not None and not allow_process_faults
        self.sim_failure_timeout_s = sim_health_timeout_s
        self.sim_infra_failure = False
        self.unhealthy_since: float | None = None
        self.soak_seconds = soak_seconds
        self.soak_started: float | None = None
        # The unhealthy window must outlast the classification timeout so a
        # loss event cannot fall back to healthy just as the threshold is hit.
        self.health = SimHealthMonitor(
            fdm_loss_grace_s=sim_health_timeout_s + 1.0,
            fdm_health_timeout_s=sim_health_timeout_s + 1.0)
        if self.monitor_health:
            self.health.open_launch_log(launch_log_path)
        self.goal_publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        node.create_subscription(Odometry, "/localization/odometry", self.on_odometry, sensor_qos)
        node.create_subscription(Clock, "/clock", self.on_clock, sensor_qos)
        node.create_subscription(TwistStamped, "/control/raw_velocity_command", self.on_raw, 10)
        node.create_subscription(TwistStamped, "/control/safe_velocity_command", self.on_conditioned, 10)
        node.create_subscription(TwistStamped, "/mavros/setpoint_velocity/cmd_vel",
                                 self.on_mavros_setpoint, sensor_qos)
        node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 50)
        global_qos = QoSProfile(depth=10)
        global_qos.durability = DurabilityPolicy.VOLATILE
        node.create_subscription(DiagnosticArray, "/planning/global_planner/diagnostics",
                                 self.on_global_diagnostics, global_qos)
        node.create_timer(0.1, self.tick)

    @staticmethod
    def vector(message: Any) -> dict[str, Any]:
        return {"linear_enu_m_s": [message.twist.linear.x, message.twist.linear.y,
                                    message.twist.linear.z],
                "yaw_rate_rad_s": message.twist.angular.z}

    @staticmethod
    def values(status: Any) -> dict[str, str]:
        return {item.key: item.value for item in status.values}

    @staticmethod
    def typed(values: dict[str, str]) -> dict[str, Any]:
        output: dict[str, Any] = {}
        for key, value in values.items():
            if value in ("true", "false"):
                output[key] = value == "true"
                continue
            try:
                parsed = float(value)
                output[key] = parsed if math.isfinite(parsed) else value
            except ValueError:
                output[key] = value
        return output

    def elapsed(self) -> float:
        return time.monotonic() - (self.started if self.started is not None else self.created)

    def write(self, event: str, **fields: Any) -> None:
        row = {"event": event, "wall_time_ns": time.time_ns(),
               "elapsed_s": self.elapsed(), **fields}
        self.stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
        self.stream.flush()

    def on_odometry(self, message: Any) -> None:
        p, v = message.pose.pose.position, message.twist.twist.linear
        self.latest_state = {"position_enu_m": [p.x, p.y, p.z],
                             "velocity_enu_m_s": [v.x, v.y, v.z]}
        if self.monitor_health:
            self.health.observe_odometry(time.monotonic())

    def on_clock(self, message: Any) -> None:
        if self.monitor_health:
            sim_time = message.clock.sec + message.clock.nanosec * 1e-9
            self.health.observe_clock(sim_time, time.monotonic())

    def on_raw(self, message: Any) -> None:
        self.latest_raw = self.vector(message)
        self.last_raw_time = time.monotonic()
        self.raw_sequence += 1

    def on_conditioned(self, message: Any) -> None:
        self.latest_conditioned = self.vector(message)
        self.last_conditioned_time = time.monotonic()
        self.conditioned_sequence += 1

    def on_mavros_setpoint(self, _message: Any) -> None:
        self.mavros_setpoint_sequence += 1
        self.last_mavros_setpoint_time = time.monotonic()

    def on_global_diagnostics(self, message: Any) -> None:
        for status in message.status:
            if status.name != "global_planner":
                continue
            self.latest_global = self.typed(self.values(status))
            self.latest_global["status"] = status.message.split(":", 1)[0]
            self.latest_global["message"] = status.message
            self.write("global_planner", global_planner=self.latest_global)

    def on_diagnostics(self, message: Any) -> None:
        for status in message.status:
            values = self.typed(self.values(status))
            if status.name == "autopilot_adapter":
                self.latest_adapter = values
                self.latest_adapter["message"] = status.message
                now = time.monotonic()
                if self.monitor_health:
                    self.health.observe_adapter(
                        values.get("connected"), values.get("armed"),
                        values.get("mode"), now)
                self.write(
                    "adapter", adapter=self.latest_adapter,
                    mavros_setpoint_sequence=self.mavros_setpoint_sequence,
                    mavros_setpoint_age_ms=(now - self.last_mavros_setpoint_time) * 1000.0
                    if self.last_mavros_setpoint_time else -1.0)
            elif status.name == "local_navigation":
                controller = values
                controller["message"] = status.message
                now = time.monotonic()
                self.write(
                    "control_cycle", state=self.latest_state,
                    goal_enu_m=self.current_goal(), command_raw=self.latest_raw,
                    command_conditioned=self.latest_conditioned,
                    raw_command_sequence=self.raw_sequence,
                    conditioned_command_sequence=self.conditioned_sequence,
                    raw_command_age_ms=(now - self.last_raw_time) * 1000.0 if self.last_raw_time else -1.0,
                    conditioned_command_age_ms=(now - self.last_conditioned_time) * 1000.0 if self.last_conditioned_time else -1.0,
                    controller=controller, adapter=self.latest_adapter,
                    global_planner=self.latest_global)
                self.check_expected(controller)

    def ready(self) -> bool:
        if self.start_immediately:
            return bool(self.latest_state)
        altitude = self.latest_state.get("position_enu_m", [0.0, 0.0, 0.0])[2]
        basic_ready = (
            altitude >= float(self.scenario.get("start_min_altitude_m", 4.0))
            and self.latest_adapter.get("armed") is True
            and self.latest_adapter.get("mode") == "GUIDED"
        )
        velocity = self.latest_state.get("velocity_enu_m_s", [])
        speed_ready = self.start_max_speed_m_s is None
        if self.start_max_speed_m_s is not None and len(velocity) == 3:
            speed_ready = math.sqrt(sum(float(value) ** 2 for value in velocity)) <= self.start_max_speed_m_s
        if not basic_ready or not speed_ready:
            self.ready_since = None
            return False
        if self.monitor_health:
            self.health.scan_launch_log(time.monotonic())
            health = self.health.evaluate(time.monotonic())
            if not health.ok:
                self.ready_since = None
                return False
        now = time.monotonic()
        if self.ready_since is None:
            self.ready_since = now
        return now - self.ready_since >= self.start_stable_s

    def current_goal(self) -> list[float]:
        goal = self.scenario["goal_enu_m"]
        if self.replans_sent:
            goal = self.scenario.get("replans", [])[max(self.replans_sent)]["goal_enu_m"]
        return goal

    def publish_goal(self, goal: list[float]) -> None:
        from geometry_msgs.msg import PoseStamped
        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, goal)
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)
        self.write("goal_published", goal_enu_m=goal)

    def check_expected(self, controller: dict[str, Any]) -> None:
        mode = controller.get("mode")
        if mode in self.scenario.get("expected_terminal_modes", []):
            self.finish(True, f"observed local mode {mode}")

    def apply_fault(self) -> None:
        fault = self.scenario.get("fault")
        if not fault or self.fault_started or self.elapsed() < float(fault["after_s"]):
            return
        if not self.allow_process_faults:
            self.finish(False, "process fault requires --allow-process-faults")
            return
        if fault.get("action") == "run_command":
            command = [str(value) for value in fault.get("argv", [])]
            result = subprocess.run(command, cwd=ROOT, text=True,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=10,
                                    check=False)
            self.fault_started = True
            self.fault_restored = True
            self.write("fault_injected", action="run_command", argv=command,
                       returncode=result.returncode, output=result.stdout[-2000:])
            if result.returncode != 0:
                self.finish(False, "fault command failed")
            return
        if fault.get("action") != "suspend_process":
            self.finish(False, f"unsupported fault action {fault.get('action')}")
            return
        self.suspended_pids = process_ids(str(fault["process_pattern"]))
        if not self.suspended_pids:
            self.finish(False, f"fault target not found: {fault['process_pattern']}")
            return
        for pid in self.suspended_pids:
            os.kill(pid, signal.SIGSTOP)
        self.fault_started = True
        self.write("fault_injected", action="SIGSTOP", pids=self.suspended_pids,
                   pattern=fault["process_pattern"])

    def restore_fault(self) -> None:
        fault = self.scenario.get("fault", {})
        if not self.fault_started or self.fault_restored:
            return
        if self.elapsed() < float(fault["after_s"]) + float(fault.get("restore_after_s", 0.0)):
            return
        for pid in self.suspended_pids:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        self.fault_restored = True
        self.write("fault_restored", action="SIGCONT", pids=self.suspended_pids)

    def _check_health(self, now: float) -> bool:
        """Classify the run as SIM_INFRA_FAILURE when the platform is unhealthy.

        Returns True when the run was ended by the simulator.
        """
        if not self.monitor_health:
            return False
        self.health.scan_launch_log(now)
        failure, self.unhealthy_since, status = infra_failure_due(
            self.health, self.unhealthy_since, now, self.sim_failure_timeout_s)
        if not failure:
            return False
        self.sim_infra_failure = True
        self.write("sim_infra_unhealthy", reason=status.reason,
                   severe_fdm_loss=self.health.severe_fdm_loss,
                   diagnostics=status.diagnostics)
        self.finish(False, f"simulator unhealthy: {status.reason}")
        return True

    def tick(self) -> None:
        if self.done:
            return
        if self.started is None:
            now = time.monotonic()
            if self.soak_started is not None and self._check_health(now):
                return
            if self.ready():
                if self.soak_seconds > 0.0 and self.soak_started is None:
                    self.soak_started = now
                    self.write("soak_started", duration_s=self.soak_seconds)
                if (self.soak_started is not None and
                        now - self.soak_started < self.soak_seconds):
                    return
                self.started = now
                self.publish_goal(self.scenario["goal_enu_m"])
                self.write("run_started")
            elif now - self.created > self.readiness_timeout_s:
                self.finish(
                    False,
                    f"vehicle did not become flight-ready within {self.readiness_timeout_s:g} s")
            return
        now = time.monotonic()
        if self._check_health(now):
            return
        for index, replan in enumerate(self.scenario.get("replans", [])):
            if index not in self.replans_sent and self.elapsed() >= float(replan["after_s"]):
                self.publish_goal(replan["goal_enu_m"])
                self.replans_sent.add(index)
        self.apply_fault()
        self.restore_fault()
        expected_global = self.scenario.get("expected_global_status")
        if expected_global and self.latest_global.get("status") == expected_global:
            self.finish(True, f"observed global status {expected_global}")
        expected_adapter = self.scenario.get("expected_adapter_states", [])
        if self.latest_adapter.get("adapter_state") in expected_adapter:
            self.finish(True, f"observed adapter state {self.latest_adapter['adapter_state']}")
        if self.elapsed() >= float(self.scenario["timeout_s"]):
            self.finish(False, "scenario timeout")

    def finish(self, success: bool, reason: str) -> None:
        if self.done:
            return
        self.restore_all()
        self.success, self.reason, self.done = success, reason, True
        self.write("run_result", success=success, reason=reason,
                   sim_infra_failure=self.sim_infra_failure,
                   classification=classify_run_result(
                       self.sim_infra_failure, success, reason))

    def restore_all(self) -> None:
        for pid in self.suspended_pids:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=ROOT / "results/m7")
    parser.add_argument("--run", type=int, default=1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--runtime-commit",
                        help="commit used to build runtime binaries; defaults to HEAD")
    parser.add_argument("--variant")
    parser.add_argument("--no-launch-stack", action="store_true")
    parser.add_argument("--debug-visualization", action="store_true")
    parser.add_argument("--start-immediately", action="store_true")
    parser.add_argument("--readiness-timeout", type=float, default=60.0)
    parser.add_argument("--start-max-speed", type=float, default=0.3,
                        help="wait for total vehicle speed at or below this value before goal publication")
    parser.add_argument("--start-stable", type=float, default=2.0,
                        help="seconds that all readiness conditions must remain true")
    parser.add_argument("--allow-process-faults", action="store_true")
    parser.add_argument("--sim-health-timeout", type=float, default=3.0,
                        help="seconds of sustained simulator unhealth (FDM loss, "
                             "clock stall, stale odometry) that classifies a run "
                             "as SIM_INFRA_FAILURE")
    parser.add_argument("--soak-seconds", type=float, default=0.0,
                        help="hover in place for this many seconds after the "
                             "health gate passes before publishing the goal; "
                             "used for simulator reliability soak runs")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    configure_rmw_environment()
    scenario_path = args.scenario.resolve()
    config_path = args.config.resolve()
    scenario = load_scenario(scenario_path)
    if args.readiness_timeout <= 0.0:
        parser.error("--readiness-timeout must be positive")
    if args.start_max_speed is not None and args.start_max_speed < 0.0:
        parser.error("--start-max-speed must be nonnegative")
    if args.start_stable < 0.0:
        parser.error("--start-stable must be nonnegative")
    if args.sim_health_timeout < 0.0:
        parser.error("--sim-health-timeout must be nonnegative")
    if args.soak_seconds < 0.0:
        parser.error("--soak-seconds must be nonnegative")
    variants = scenario.get("variants", {})
    variant_name = args.variant or scenario.get("default_variant")
    if variant_name:
        if variant_name not in variants:
            parser.error(f"unknown variant {variant_name}; choose from {', '.join(variants)}")
        scenario.update(variants[variant_name])
    if not config_path.is_file():
        parser.error(f"config does not exist: {config_path}")
    scenario_result_name = scenario["id"].lower()
    if variant_name:
        scenario_result_name += f"_{variant_name}"
    run_dir = args.output / scenario_result_name / f"run_{args.run:02d}_seed_{args.seed}"
    params = (ROOT / "uav_navigation_bringup/config/navigation.yaml").resolve() \
        if args.debug_visualization else config_path
    launch_command = [
        "ros2", "launch", "uav_navigation_bringup", "sim.launch.xml",
        f"params_file:={params}", f"world_file:={scenario['_world_path']}",
        f"gazebo_resource_path:={ROOT / 'models'}:{ROOT / 'worlds'}",
        f"robot_description_file:={ROOT / 'config/iris_sensor_suite.urdf'}",
        f"mppi_seed:={args.seed}", "enable_autopilot_adapter:=true",
        f"enable_gazebo_gui:={'true' if args.debug_visualization else 'false'}",
        f"enable_rviz:={'true' if args.debug_visualization else 'false'}",
    ]
    launch_command.extend(str(value) for value in scenario.get("launch_overrides", []))
    if args.dry_run:
        print(json.dumps({"scenario": scenario["id"], "run_dir": str(run_dir),
                          "launch": launch_command}, indent=2))
        return 0

    run_dir.mkdir(parents=True, exist_ok=False)
    harness_commit = git_commit()
    runtime_commit = args.runtime_commit or harness_commit
    if len(runtime_commit) < 7 or any(character not in "0123456789abcdef" for character in runtime_commit.lower()):
        parser.error("--runtime-commit must be a hexadecimal Git commit")
    manifest = {
        "schema_version": 1, "scenario": scenario["id"], "scenario_name": scenario["name"],
        "run": args.run, "seed": args.seed, "variant": variant_name,
        "git_commit": runtime_commit, "harness_commit": harness_commit,
        "scenario_file": str(scenario_path.relative_to(ROOT)),
        "scenario_sha256": sha256(scenario_path), "config_file": str(params.relative_to(ROOT)),
        "config_sha256": sha256(params), "world_file": scenario["world"],
        "world_sha256": sha256(scenario["_world_path"]),
        "performance_mode": not args.debug_visualization,
        "visualization": bool(args.debug_visualization),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "expected_terminal_modes": scenario.get("expected_terminal_modes", []),
        "expected_global_status": scenario.get("expected_global_status"),
        "expected_adapter_states": scenario.get("expected_adapter_states", []),
        "readiness": {"minimum_altitude_m": scenario.get("start_min_altitude_m", 4.0),
                      "maximum_speed_m_s": args.start_max_speed,
                      "stable_duration_s": args.start_stable,
                      "sim_health_timeout_s": args.sim_health_timeout,
                      "soak_seconds": args.soak_seconds},
        "launch_command": launch_command,
        "environment": {"platform": platform.platform(),
                        "python": sys.version.split()[0],
                        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
                        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
                        "fastdds_builtin_transports": os.environ.get(
                            "FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")},
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    launch = None
    try:
        if not args.no_launch_stack:
            launch_log = (run_dir / "launch.log").open("w", encoding="utf-8")
            launch = subprocess.Popen(launch_command, cwd=ROOT, stdout=launch_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(3.0)
            if launch.poll() is not None:
                raise RuntimeError(f"navigation launch exited early; see {run_dir / 'launch.log'}")
        import rclpy
        rclpy.init()
        node = rclpy.create_node(f"m7_recorder_{scenario['id'].lower()}_{args.run}")
        with (run_dir / "events.jsonl").open("w", encoding="utf-8") as stream:
            recorder = M7Recorder(node, scenario, stream, args.start_immediately,
                                  args.allow_process_faults,
                                  args.readiness_timeout,
                                  args.start_max_speed, args.start_stable,
                                  run_dir / "launch.log",
                                  args.sim_health_timeout,
                                  args.soak_seconds)
            if not args.start_immediately:
                speed_condition = ("" if args.start_max_speed is None else
                                   f", speed <= {args.start_max_speed:g} m/s for "
                                   f"{args.start_stable:g} s")
                print(f"waiting up to {args.readiness_timeout:g} s for odometry, "
                      f"GUIDED, armed, altitude >= 4 m{speed_condition}")
            while rclpy.ok() and not recorder.done:
                rclpy.spin_once(node, timeout_sec=0.1)
            success = recorder.success
            recorder.restore_all()
        node.destroy_node()
        rclpy.shutdown()
        print(f"{'PASS' if success else 'FAIL'} {scenario['id']}: {recorder.reason}")
        return 0 if success else 2
    finally:
        if launch is not None:
            stop_process_group(launch.pid)
            try:
                launch.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass


if __name__ == "__main__":
    raise SystemExit(main())

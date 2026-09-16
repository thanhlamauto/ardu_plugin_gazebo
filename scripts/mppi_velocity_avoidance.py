#!/usr/bin/env python3
"""MPPI local planner node cho demo ArduPilot SITL + Gazebo warehouse.

Wrapper CLI mỏng trên package ``mppi_ardupilot`` (đúng kiến trúc mentor chốt:
edge computer chạy local planner, ArduPilot GUIDED chỉ bám velocity
setpoint ``u = [vx, vy, vz, yaw_rate]`` qua SET_POSITION_TARGET_LOCAL_NED).

Pipeline (companion-side, ArduPilot OA tắt hoàn toàn)::

    goal + known static SDF --global-planner astar -> collision-free reference
    Gazebo gpu_lidar /sensor_suite/lidar/points (FLU)
      -> downsample week 3 -> BODY_FRD -> world ENU (odom pose)
    Gazebo /iris/odometry  HOẶC  MAVLink telemetry (--state-source mav)
    MPPI velocity state + applied-command memory -> u -> MAVLink mask 1479
      -> ArduPilot GUIDED

Chạy SAU khi copter đã takeoff và ở GUIDED. Ctrl+C dừng stream, copter
hold sau GUID_TIMEOUT; hạ cánh bằng ``mode land``.

Test offline không cần Gazebo/MAVLink::

    python3 scripts/mppi_velocity_avoidance.py --sim-test
"""

import argparse
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_REPO_ROOT, _SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mppi_ardupilot.mppi_local_planner_node import (  # noqa: E402
    DEFAULT_GOAL,
    load_config,
    parse_goal,
    parse_reference_path,
    run,
    run_rigid_dynamics_test,
    run_sim_test,
)
from mppi_ardupilot.mppi_controller import MPPIConfig, QuadMPPI  # noqa: E402
from mppi_ardupilot.pa_mppi_controller import (  # noqa: E402
    PAMPPIConfig,
    PerceptionAwareMPPI,
)
from mppi_ardupilot.rigid_body_pa_mppi import (  # noqa: E402
    RigidBodyPAMPPI,
    RigidBodyPAMPPIConfig,
)
from mppi_ardupilot.global_planner import AStarConfig, AStarGlobalPlanner  # noqa: E402

_DEFAULTS = MPPIConfig()
DEFAULT_MAV = "tcp:127.0.0.1:5762"
DEFAULT_LIDAR_TOPIC = "/sensor_suite/lidar/points"
DEFAULT_ODOM_TOPIC = "/iris/odometry"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="MPPI local planner -> ArduPilot GUIDED velocity + yaw_rate"
    )
    p.add_argument(
        "--planner", choices=("mppi", "pa-mppi", "rigid-pa-mppi"), default="mppi",
        help="baseline, PA-MPPI velocity v0, hoặc rigid-body PA-MPPI experimental",
    )
    p.add_argument("--goal", default=None,
                   help="tuyến 'x,y,z;x,y,z;...' (m, ENU). --state-source mav thì "
                        f"hiểu là ENU tương đối home (mặc định: {DEFAULT_GOAL})")
    p.add_argument("--global-path", default=None,
                   help="polyline global path 'x,y,z;x,y,z;...' (m, ENU), ít nhất 2 điểm; "
                        "chỉ dùng khi w_path > 0")
    p.add_argument("--global-planner", choices=("manual", "astar"), default=None,
                   help="manual: dùng --global-path/--goal; astar: tự lập path từ SDF")
    p.add_argument("--global-map-sdf", default=None,
                   help="SDF world chứa collision box/cylinder tĩnh cho A* 2.5D")
    p.add_argument("--astar-resolution", type=float, default=None,
                   help="độ phân giải lattice A* [m]")
    p.add_argument("--astar-clearance", type=float, default=None,
                   help="inflate footprint vật cản cho A* [m]")
    p.add_argument("--astar-padding", type=float, default=None,
                   help="padding search bounds quanh start/goal/obstacles [m]")
    p.add_argument("--mav", default=DEFAULT_MAV, help=f"MAVLink target (mặc định: {DEFAULT_MAV})")
    p.add_argument("--state-source", choices=("odom", "mav"), default=None,
                   help="odom: Gazebo ground truth (mặc định SITL); "
                        "mav: MAVLink LOCAL_POSITION_NED+ATTITUDE (edge computer thật)")
    p.add_argument("--topic", default=None, help="topic LiDAR GZ")
    p.add_argument("--odom-topic", default=None, help="topic odometry GZ")
    p.add_argument("--hz", type=float, default=None,
                   help="tần số vòng điều khiển, giữ >= 5 để không vượt GUID_TIMEOUT")
    p.add_argument("--config", default=None, help="file yaml ghi đè mặc định (xem mppi_ardupilot/config.yaml)")
    # Tuning MPPI: None nghĩa là "không truyền" -> lấy từ --config hoặc MPPIConfig.
    p.add_argument("--vmax", type=float, default=None, help=f"tốc ngang [m/s] (mặc định: {_DEFAULTS.vmax})")
    p.add_argument("--vzmax", type=float, default=None, help=f"tốc đứng [m/s] (mặc định: {_DEFAULTS.vzmax})")
    p.add_argument("--yaw-rate-max", type=float, default=None, help=f"yaw rate [rad/s] (mặc định: {_DEFAULTS.yaw_rate_max})")
    p.add_argument("--margin", type=float, default=None, help=f"bán kính an toàn [m] (mặc định: {_DEFAULTS.margin})")
    p.add_argument("--goal-radius", type=float, default=None, help="bán kính tới đích cuối [m]")
    p.add_argument("--wp-radius", type=float, default=None, help="bán kính chuyển waypoint [m]")
    p.add_argument("--horizon", type=int, default=None, help=f"lookahead steps (mặc định: {_DEFAULTS.horizon})")
    p.add_argument("--samples", type=int, default=None, help=f"rollout MPPI (mặc định: {_DEFAULTS.samples})")
    p.add_argument("--noise-xy", type=float, default=None, help=f"sigma tốc ngang (mặc định: {_DEFAULTS.noise_xy})")
    p.add_argument("--noise-z", type=float, default=None, help=f"sigma tốc đứng (mặc định: {_DEFAULTS.noise_z})")
    p.add_argument("--noise-yaw", type=float, default=None, help=f"sigma yaw rate (mặc định: {_DEFAULTS.noise_yaw})")
    p.add_argument("--lambda", dest="lambda_", type=float, default=None, help=f"temperature (mặc định: {_DEFAULTS.lambda_})")
    p.add_argument("--dt", type=float, default=None, help=f"bước tích phân [s] (mặc định: {_DEFAULTS.dt})")
    p.add_argument("--tau", type=float, default=None, help=f"hằng số bám tốc [s] (mặc định: {_DEFAULTS.tau})")
    p.add_argument("--device", default=None, help="torch device cpu/cuda")
    p.add_argument("--seed", type=int, default=None, help="random seed cho rollout")
    p.add_argument("--w-obstacle", type=float, default=None, help=f"trọng số obstacle (mặc định: {_DEFAULTS.w_obstacle})")
    p.add_argument("--w-yaw", type=float, default=None, help=f"trọng số bám yaw (mặc định: {_DEFAULTS.w_yaw})")
    p.add_argument("--w-path", type=float, default=None,
                   help=f"trọng số bám global path (mặc định: {_DEFAULTS.w_path}; 0=tắt)")
    p.add_argument("--path-scale-m", type=float, default=None,
                   help=f"scale chuẩn hóa lỗi path [m] (mặc định: {_DEFAULTS.path_scale_m})")
    p.add_argument("--w-reference-velocity", type=float, default=None,
                   help="trọng số bám velocity reference theo từng bước")
    p.add_argument("--reference-speed-m-s", type=float, default=None,
                   help="tốc độ dùng để time-parameterize global path [m/s]")
    p.add_argument("--cost-profile", choices=("project", "paper"), default=None,
                   help="project: objective hiện tại; paper: effort + reference position + collision indicator")
    p.add_argument("--w-collision", type=float, default=None,
                   help="paper collision indicator coefficient (mặc định: 1e6)")
    p.add_argument("--collision-radius-m", type=float, default=None,
                   help="bán kính point-cloud adapter cho paper collision set [m]")
    p.add_argument("--command-alpha", type=float, default=None,
                   help="low-pass output: 1=tắt, nhỏ hơn=mượt hơn")
    p.add_argument("--max-accel-xy", type=float, default=None,
                   help="giới hạn đổi lệnh ngang [m/s^2]")
    p.add_argument("--max-accel-z", type=float, default=None,
                   help="giới hạn đổi lệnh đứng [m/s^2]")
    p.add_argument("--max-yaw-accel", type=float, default=None,
                   help="giới hạn đổi yaw-rate [rad/s^2]")
    p.add_argument("--goal-slowdown-radius", type=float, default=None,
                   help="bán kính hút thẳng về goal [m]; 0=tắt nhánh arrival, dùng MPPI tới đích")
    p.add_argument("--goal-approach-gain", type=float, default=None,
                   help="gain đổi sai số goal thành speed cap [1/s]")
    p.add_argument("--max-points", type=int, default=None, help="obstacle tối đa mỗi scan")
    p.add_argument("--max-raw-points", type=int, default=None, help="giới hạn điểm đọc mỗi PointCloudPacked")
    p.add_argument("--sensor-offset-body-frd", nargs=3, type=float,
                   metavar=("X", "Y", "Z"), default=None,
                   help="offset gốc LiDAR so với base_link trong BODY_FRD")
    p.add_argument("--hard-brake-m", type=float, default=None, help="obstacle gần hơn -> velocity 0")
    p.add_argument("--hard-brake-release-m", type=float, default=None,
                   help="clearance để thoát recovery; phải >= hard-brake-m")
    p.add_argument("--hard-brake-delay-s", type=float, default=None,
                   help="delay dùng trong ngưỡng phanh dự đoán [s]")
    p.add_argument("--recovery-speed-m-s", type=float, default=None,
                   help="tốc độ lùi khỏi obstacle sau khi dừng [m/s]")
    p.add_argument("--stale-after-s", type=float, default=None, help="sensor cũ hơn -> velocity 0 (hold)")
    p.add_argument("--planner-timeout-ms", type=float, default=None,
                   help="compute vượt ngưỡng -> không phát control mới")
    p.add_argument("--rviz-traj-topic", default=None,
                   help="publish predicted trajectory ra nav_msgs/Path (vd: /mppi/predicted_path)")
    p.add_argument("--rviz-samples-topic", default=None,
                   help="publish các rollout tốt nhất ra MarkerArray")
    p.add_argument("--rviz-goal-topic", default=None,
                   help="nhận geometry_msgs/PoseStamped từ RViz 2D Goal Pose, vd /goal_pose")
    p.add_argument("--rviz-goal-frame", default=None,
                   help="frame bắt buộc của RViz goal (mặc định: odom; chưa tự TF transform)")
    p.add_argument("--rviz-goal-altitude", type=float, default=None,
                   help="cao độ ENU cố định cho goal click; bỏ trống để giữ cao độ hiện tại")
    p.add_argument("--rviz-top-k", type=int, default=20,
                   help="số sampled trajectory hiển thị trong RViz")
    p.add_argument("--diag-every", type=int, default=10,
                   help="in timing/cost diagnostics mỗi N chu kỳ")
    p.add_argument("--diag-jsonl", default=None,
                   help="lưu diagnostics từng chu kỳ vào JSONL")
    p.add_argument("--debug-snapshot-cycle", type=int, default=None,
                   help="lưu sample pool MPPI của đúng chu kỳ này vào NPZ (chi phí I/O lớn)")
    p.add_argument("--debug-snapshot-events", action="store_true",
                   help="lưu exact pool khi N_safe=0, ngay sau hold, và control định kỳ")
    p.add_argument("--debug-control-stride", type=int, default=25,
                   help="stride cycle cho control snapshots khi --debug-snapshot-events")
    p.add_argument("--benchmark-window", type=int, default=200,
                   help="cửa sổ rolling mean/p95/worst timing")
    p.add_argument("--no-mav", action="store_true", help="chạy MPPI + log, không gửi MAVLink (debug)")
    p.add_argument("--exit-on-goal", action="store_true",
                   help="thoát khi tới đích; mặc định stream zero để hold")
    p.add_argument("--sim-test", action="store_true", help="test planner offline, không cần Gazebo/MAVLink")
    p.add_argument(
        "--experimental-attitude-control", action="store_true",
        help="xác nhận cho phép SET_ATTITUDE_TARGET thrust/body-rate (SITL only)",
    )
    return p


_CLI_TO_CFG = {
    "vmax": "vmax", "vzmax": "vzmax", "yaw_rate_max": "yaw_rate_max",
    "margin": "margin", "horizon": "horizon", "samples": "samples",
    "noise_xy": "noise_xy", "noise_z": "noise_z", "noise_yaw": "noise_yaw",
    "lambda_": "lambda_", "dt": "dt", "tau": "tau", "device": "device",
    "w_obstacle": "w_obstacle", "w_yaw": "w_yaw", "w_path": "w_path",
    "path_scale_m": "path_scale_m", "w_reference_velocity": "w_reference_velocity",
    "reference_speed_m_s": "reference_speed_m_s", "seed": "seed",
    "cost_profile": "cost_profile", "w_collision": "w_collision",
    "collision_radius_m": "collision_radius_m",
    "command_alpha": "command_alpha", "max_accel_xy": "max_accel_xy",
    "max_accel_z": "max_accel_z", "max_yaw_accel": "max_yaw_accel",
    "goal_slowdown_radius": "goal_slowdown_radius",
    "goal_approach_gain": "goal_approach_gain",
}


_RUNTIME_DEFAULTS = {
    "goal": ("goal", DEFAULT_GOAL),
    "global_path": ("global_path", None),
    "global_planner": ("global_planner", "manual"),
    "global_map_sdf": ("global_map_sdf", None),
    "astar_resolution": ("astar_resolution", 0.5),
    "astar_clearance": ("astar_clearance", None),
    "astar_padding": ("astar_padding", 4.0),
    "state_source": ("state_source", "odom"),
    "topic": ("lidar_topic", DEFAULT_LIDAR_TOPIC),
    "odom_topic": ("odom_topic", DEFAULT_ODOM_TOPIC),
    "hz": ("hz", 10.0),
    "goal_radius": ("goal_radius", 0.25),
    "wp_radius": ("wp_radius", 2.5),
    "max_points": ("max_points", 200),
    "max_raw_points": ("max_raw_points", 25000),
    "sensor_offset_body_frd": ("sensor_offset_body_frd", (0.08, 0.0, -0.16)),
    "hard_brake_m": ("hard_brake_m", 1.0),
    "hard_brake_release_m": ("hard_brake_release_m", None),
    "hard_brake_delay_s": ("hard_brake_delay_s", 0.25),
    "recovery_speed_m_s": ("recovery_speed_m_s", 0.4),
    "stale_after_s": ("stale_after_s", 1.0),
    "planner_timeout_ms": ("planner_timeout_ms", None),
    "rviz_goal_topic": ("rviz_goal_topic", None),
    "rviz_goal_frame": ("rviz_goal_frame", "odom"),
    "rviz_goal_altitude": ("rviz_goal_altitude", None),
}


def apply_runtime_config(args, raw_cfg: dict) -> None:
    """Fill runtime options from YAML; explicit CLI values keep precedence."""
    for attr, (key, default) in _RUNTIME_DEFAULTS.items():
        if getattr(args, attr) is None:
            setattr(args, attr, raw_cfg.get(key, default))


def build_cfg(args, raw_cfg=None) -> MPPIConfig:
    if args.planner == "rigid-pa-mppi":
        cfg = RigidBodyPAMPPIConfig()
    elif args.planner == "pa-mppi":
        cfg = PAMPPIConfig()
    else:
        cfg = MPPIConfig()
    if raw_cfg is None:
        raw_cfg = load_config(args.config) if args.config else {}
    if raw_cfg:
        pa_fields = vars(RigidBodyPAMPPIConfig()).keys()
        runtime_keys = {key for key, _ in _RUNTIME_DEFAULTS.values()}
        for key, val in raw_cfg.items():
            name = "lambda_" if key == "lambda" else key
            if hasattr(cfg, name):
                setattr(cfg, name, val)
            elif name not in pa_fields and key not in runtime_keys:
                print(f"[config] bỏ qua key lạ: {key}")
    for cli, name in _CLI_TO_CFG.items():
        val = getattr(args, cli)
        if val is not None:
            setattr(cfg, name, val)
    return cfg


def main() -> None:
    if sys.platform == "darwin":
        os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
        os.environ.setdefault("OMP_NUM_THREADS", "1")
    args = build_parser().parse_args()
    raw_cfg = load_config(args.config) if args.config else {}
    apply_runtime_config(args, raw_cfg)
    if args.hz <= 0:
        raise SystemExit("[error] --hz phải lớn hơn 0")
    try:
        import torch  # noqa: F401
        from pytorch_mppi import MPPI  # noqa: F401
    except ImportError as exc:
        raise SystemExit("[error] cần torch và pytorch-mppi: pip install torch pytorch-mppi") from exc
    args.cfg = build_cfg(args, raw_cfg)
    factories = {
        "mppi": QuadMPPI,
        "pa-mppi": PerceptionAwareMPPI,
        "rigid-pa-mppi": RigidBodyPAMPPI,
    }
    args.planner_factory = factories[args.planner]
    if args.cfg.dt <= 0 or args.cfg.tau <= 0:
        raise SystemExit("[error] dt/tau phải lớn hơn 0")
    if not 0.0 < args.cfg.command_alpha <= 1.0:
        raise SystemExit("[error] command_alpha phải trong (0, 1]")
    if min(args.cfg.max_accel_xy, args.cfg.max_accel_z, args.cfg.max_yaw_accel) <= 0:
        raise SystemExit("[error] các giới hạn accel phải lớn hơn 0")
    if args.cfg.w_path < 0 or args.cfg.path_scale_m <= 0:
        raise SystemExit("[error] w_path phải >= 0 và path_scale_m phải > 0")
    if args.cfg.w_reference_velocity < 0 or args.cfg.reference_speed_m_s <= 0:
        raise SystemExit("[error] reference velocity weight phải >= 0 và speed phải > 0")
    if args.cfg.cost_profile not in ("project", "paper"):
        raise SystemExit("[error] cost_profile phải là project hoặc paper")
    if args.cfg.w_collision < 0 or args.cfg.collision_radius_m < 0:
        raise SystemExit("[error] w_collision/collision_radius_m phải >= 0")
    if (args.cfg.cost_profile == "paper" and args.hard_brake_m > 0
            and args.cfg.collision_radius_m < args.hard_brake_m):
        raise SystemExit(
            "[safety] paper collision_radius_m phải >= hard_brake_m để "
            "planner thấy collision trước safety gate"
        )
    if (args.hard_brake_release_m is not None
            and args.hard_brake_release_m < args.hard_brake_m):
        raise SystemExit("[safety] hard_brake_release_m phải >= hard_brake_m")
    if args.hard_brake_delay_s < 0 or args.recovery_speed_m_s <= 0:
        raise SystemExit("[safety] brake delay phải >=0 và recovery speed phải >0")
    if args.cfg.cost_profile == "paper" and args.planner != "mppi":
        print("[warn] cost_profile=paper chỉ áp dụng objective vanilla QuadMPPI; "
              "PA/rigid planner vẫn cộng thêm cost riêng của nhánh đó.")
    if (args.cfg.goal_slowdown_radius < 0 or args.cfg.goal_approach_gain <= 0
            or args.cfg.goal_min_speed <= 0):
        raise SystemExit("[error] goal slowdown radius phải >= 0; gain phải > 0")
    if args.diag_every <= 0 or args.benchmark_window <= 0 or args.rviz_top_k < 0:
        raise SystemExit("[error] diag/window phải > 0 và rviz-top-k phải >= 0")
    if args.planner_timeout_ms is not None and args.planner_timeout_ms <= 0:
        raise SystemExit("[error] planner-timeout-ms phải > 0")
    if args.rviz_goal_altitude is not None and not math.isfinite(args.rviz_goal_altitude):
        raise SystemExit("[error] rviz-goal-altitude phải hữu hạn")
    if args.astar_resolution <= 0 or args.astar_padding <= 0:
        raise SystemExit("[error] A* resolution/padding phải > 0")
    if args.rviz_goal_topic and args.exit_on_goal:
        raise SystemExit(
            "[error] interactive RViz không dùng --exit-on-goal; node phải còn chạy để nhận goal mới"
        )
    if args.state_source == "mav" and args.no_mav:
        print("[warn] --state-source mav cùng --no-mav: state vẫn đọc từ SITL, chỉ không gửi lệnh.")
    if args.rviz_goal_topic and args.state_source != "odom":
        raise SystemExit(
            "[safety] RViz goal hiện yêu cầu --state-source odom để goal và state "
            "dùng cùng frame; chưa hỗ trợ TF sang MAVLink local frame"
        )
    if args.planner == "rigid-pa-mppi" and not args.sim_test:
        if not args.experimental_attitude_control:
            raise SystemExit(
                "[safety] rigid-pa-mppi cần --experimental-attitude-control "
                "và chỉ được dùng trong Gazebo/SITL"
            )
        if args.state_source != "odom":
            raise SystemExit("[safety] rigid-pa-mppi hiện chỉ hỗ trợ --state-source odom")
        if args.hz < 20:
            raise SystemExit("[safety] rigid-pa-mppi cần --hz >= 20; khuyến nghị 50 Hz")
    args.goal = parse_goal(args.goal)
    args.global_planner_instance = None
    if args.global_planner == "astar":
        if args.global_path is not None:
            raise SystemExit("[error] --global-planner astar không dùng cùng --global-path")
        if not args.global_map_sdf:
            raise SystemExit("[error] --global-planner astar cần --global-map-sdf")
        if args.cfg.w_path <= 0:
            raise SystemExit("[error] A* cần w_path > 0 để MPPI bám reference")
        if args.astar_clearance is None:
            args.astar_clearance = max(
                args.cfg.collision_radius_m,
                args.hard_brake_m,
            ) + args.cfg.reference_corner_radius_m + 0.3
        if args.astar_clearance < 0:
            raise SystemExit("[error] --astar-clearance phải >= 0")
        if len(args.goal) > 1:
            print("[astar] chỉ dùng waypoint cuối làm goal; waypoint trung gian do A* tự sinh")
            args.goal = [args.goal[-1]]
        try:
            args.global_planner_instance = AStarGlobalPlanner.from_sdf(
                args.global_map_sdf,
                AStarConfig(
                    resolution_m=args.astar_resolution,
                    clearance_m=args.astar_clearance,
                    bounds_padding_m=args.astar_padding,
                ),
            )
        except (OSError, ValueError) as exc:
            raise SystemExit(f"[astar] không load được static map: {exc}") from exc
        args.global_path = None
        print(
            f"[astar] loaded {len(args.global_planner_instance.obstacles)} primitive "
            f"collisions từ {args.global_map_sdf}; resolution={args.astar_resolution:g}m "
            f"clearance={args.astar_clearance:g}m"
        )
    elif args.global_path is not None:
        args.global_path = parse_reference_path(args.global_path)
    elif args.cfg.w_path > 0 and args.rviz_goal_topic:
        args.global_path = None
        print("[path] interactive RViz: chờ click rồi tạo reference current->goal")
    elif args.cfg.w_path > 0:
        if len(args.goal) < 2:
            raise SystemExit(
                "[error] w_path > 0 cần --global-path (ít nhất 2 điểm), "
                "hoặc --goal có ít nhất 2 waypoint để dùng làm fallback"
            )
        # Convenient fallback for route experiments, but make the limitation
        # explicit: sparse mission waypoints are not a certified safe path.
        args.global_path = [point.copy() for point in args.goal]
        print("[path] --global-path chưa được truyền; dùng tuyến --goal làm "
              "reference polyline. Hãy cung cấp path dày và đã kiểm tra clearance "
              "cho thử nghiệm tránh vật cản.")
    if args.sim_test:
        if args.planner == "rigid-pa-mppi":
            run_rigid_dynamics_test(args)
        else:
            run_sim_test(args)
    else:
        run(args)


if __name__ == "__main__":
    main()

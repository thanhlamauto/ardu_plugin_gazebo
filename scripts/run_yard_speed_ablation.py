#!/usr/bin/env python3
"""Isolated real Gazebo/SITL trials, not the offline MPPI dynamics harness.

Run with the ardupilot-rviz interpreter. Stops only process groups it starts.
Requires the SITL TCP/JSON ports to be unused; refuses an existing simulation.
Each trial boots a fresh world and EEPROM. Records raw simulator odometry,
planner JSONL, console logs, exact commands, configs and dependency hashes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mppi_ardupilot.global_planner import AStarConfig, AStarGlobalPlanner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', nargs='+', type=float, default=[.4, .6, .8, 1.0])
    parser.add_argument('--seeds', nargs='+', type=int, default=[7, 17])
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'config/experiments/mppi_industrial_ablation.yaml')
    parser.add_argument('--scenario', choices=['yard', 'straight', 'yard-runup60'], default='yard',
                        help='straight: isolated 300m speed test; not yard navigation')
    parser.add_argument('--params', type=Path, help='optional isolated SITL parameter file')
    parser.add_argument('--gui', action='store_true',
                        help='open a Gazebo 3D GUI alongside each isolated server')
    parser.add_argument('--debug-snapshot-cycle', type=int, default=None,
                        help='capture full MPPI pool at this logged planner cycle (NPZ)')
    parser.add_argument('--debug-snapshot-events', action='store_true',
                        help='capture N_safe=0, post-hold, and periodic control pools')
    parser.add_argument('--debug-control-stride', type=int, default=25)
    args = parser.parse_args()
    if any(not np.isfinite(v) or not 0 < v <= 10 for v in args.speeds):
        parser.error('speeds must be finite, positive, <= 10')
    for port in (5760, 5762):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise SystemExit(f'Port {port} occupied: stop the existing SITL first')
    # Refuse even a Gazebo-only session: both would send JSON to the same SITL.
    processes = subprocess.check_output(['ps', '-axo', 'command'], text=True)
    if any('gz sim ' in line or '/bin/arducopter ' in line for line in processes.splitlines()):
        raise SystemExit('Existing Gazebo/ArduCopter detected; stop it first')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    straight = args.scenario == 'straight'
    world = ROOT / ('worlds/iris_mppi_speed_straight.sdf' if straight else
                    'worlds/iris_mppi_industrial_yard.sdf')
    runup = args.scenario == 'yard-runup60'
    if runup:
        world = ROOT/'worlds/iris_mppi_yard_runup60.sdf'
    config = args.config.resolve()
    params = ROOT / ('config/experiments/mppi_speed_straight.parm' if straight else
                     'config/mppi_velocity.parm')
    if args.params is not None:
        params = args.params.resolve()
    expected_parameters = {}
    for line in params.read_text().splitlines():
        fields = line.split('#', 1)[0].split()
        if len(fields) >= 2 and fields[0] in ('WP_SPD', 'WP_ACC'):
            expected_parameters[fields[0]] = float(fields[1])
    goal = [300, 0, 5] if straight else [32, 0, 5]
    horizontal_envelope_m = 330 if straight else 50
    ap = ROOT.parent / 'ardupilot'
    planner = AStarGlobalPlanner.from_sdf(world, AStarConfig(clearance_m=2.6))
    reference = planner.plan([0, 0, 5], goal).path_enu
    if runup:
        # Preserve original sharp turns; replace the short entry with 60m straight.
        original = AStarGlobalPlanner.from_sdf(
            ROOT/'worlds/iris_mppi_industrial_yard.sdf', AStarConfig(clearance_m=2.6))
        old_path = original.plan([0, 0, 5], [32, 0, 5]).path_enu
        reference = np.vstack(([0, 0, 5], old_path[2:] + [45.1, -1.9, 0]))
        goal = reference[-1].tolist()
        horizontal_envelope_m = 110
        if not all(planner._line_free(a[:2], b[:2], planner._active(5))
                   for a, b in zip(reference[:-1], reference[1:])):
            raise RuntimeError('Runup reference fails 2.6m clearance check')
    reference_text = ';'.join(','.join(f'{x:.6f}' for x in point) for point in reference)
    # Freeze A* result across every trial; the CLI consumes the identical path.
    metadata = {
        'speeds': args.speeds, 'seeds': args.seeds, 'start_enu': [0, 0, 5],
        'goal_enu': goal, 'reference_enu': reference.tolist(),
        'scenario': args.scenario, 'world_file': world.name,
        'gazebo_gui': args.gui,
        'reference_modification': '60m approach to translated first sharp corner' if runup else None,
        'horizontal_envelope_m': horizontal_envelope_m,
        'expected_parameters': expected_parameters,
        'reference_source': 'A* known SDF, resolution .5m, clearance 2.6m',
        'arrival_definition': 'planner reached tolerance .35m; not settled hover',
        'contact_validation': 'no physics contact sensor; geometric center clearance only',
        'speed_definition': ('reference_speed_m_s cruise request and vmax upper bound; '
                             'both are set to the requested trial speed'),
        'repo_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'ardupilot_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ap, text=True).strip(),
        'files': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                  [world, config, params, ROOT/'mppi_ardupilot/global_planner.py',
                   Path(__file__).resolve(), ROOT/'scripts/analyze_yard_speed_ablation.py',
                   ROOT/'scripts/mppi_velocity_avoidance.py', ROOT/'scripts/analyze_speed_cruise.py',
                   ROOT/'mppi_ardupilot/braking.py',
                   ROOT/'mppi_ardupilot/trajectory_validation.py',
                   ROOT/'mppi_ardupilot/known_geometry.py',
                   ROOT/'mppi_ardupilot/mavlink_interface.py',
                   ROOT/'mppi_ardupilot/mppi_controller.py', ROOT/'mppi_ardupilot/mppi_local_planner_node.py']},
    }
    (output/'manifest.json').write_text(json.dumps(metadata, indent=2))
    for p in (world, config, params):
        (output/p.name).write_bytes(p.read_bytes())
    for name in metadata['files']:
        destination = output/'source_snapshot'/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((ROOT/name).read_bytes())
    (output/'working_changes.patch').write_bytes(subprocess.check_output(['git', 'diff'], cwd=ROOT))
    os.environ['MAVLINK20'] = '1'
    os.environ['GZ_PARTITION'] = f'yard_ablation_{os.getpid()}'
    from pymavlink import mavutil
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry

    for seed in args.seeds:
        for speed in args.speeds:
            trial = output/f'v{speed:.1f}_seed{seed}'
            trial.mkdir()
            owned, streams = [], []
            connection = None
            odom_file = None
            lock = threading.Lock()
            latest = {}
            result = {'speed_reference': speed, 'seed': seed, 'status': 'SETUP_FAILED'}
            env = os.environ.copy()
            env.update(GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'build'),
                       GZ_SIM_RESOURCE_PATH=f'{ROOT}/models:{ROOT}/worlds', PYTHONUNBUFFERED='1')

            def start(command, name, cwd):
                handle = (trial/f'{name}.stdout.log').open('w')
                streams.append(handle)
                process = subprocess.Popen(command, cwd=cwd, env=env, stdout=handle,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                owned.append(process)
                (trial/f'{name}.command.json').write_text(json.dumps(command, indent=2))
                return process

            def heartbeat():
                connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                # MAVProxy normally supplies SITL RC input. Direct SITL needs
                # an explicit neutral roll/pitch/yaw and low throttle source.
                connection.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500,
                                                         65535, 65535, 65535, 65535)

            def receive(timeout=.2):
                message = connection.recv_match(blocking=True, timeout=timeout)
                if message and message.get_type() in ('STATUSTEXT', 'COMMAND_ACK', 'HEARTBEAT'):
                    with (trial/'mavlink_events.jsonl').open('a') as handle:
                        handle.write(json.dumps({'wall_s':time.monotonic(), **message.to_dict()})+'\n')
                return message

            def command(code, *values):
                connection.mav.command_long_send(1, 1, code, 0, *(list(values)+[0]*(7-len(values))))

            def odometry(message):
                nonlocal latest
                p = message.pose.position
                row = {'wall_monotonic_s': time.monotonic(),
                       'sim_time_s': message.header.stamp.sec + message.header.stamp.nsec*1e-9,
                       'position_enu': [p.x, p.y, p.z]}
                with lock:
                    latest = row
                    if odom_file is not None:
                        odom_file.write(json.dumps(row)+'\n')

            node = Node()
            node.subscribe(Odometry, '/iris/odometry', odometry)
            try:
                print(f'START {trial.name}', flush=True)
                odom_file = (trial/'ground_truth.jsonl').open('w')
                start(['gz', 'sim', '-v2', '-r', str(world), '-s'], 'gazebo', ROOT)
                if args.gui:
                    start(['gz', 'sim', '-v1', '-g'], 'gazebo_gui', ROOT)
                start([str(ap/'build/sitl/bin/arducopter'), '-w', '--model', 'JSON',
                       '--speedup', '1', '--slave', '0', '--serial1=tcp:2',
                       '--defaults', str(params), '--sim-address=127.0.0.1', '-I0',
                       '--home=-35.363262,149.165237,584,0'], 'sitl', trial)
                deadline = time.monotonic()+30
                while connection is None:
                    try:
                        connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
                    except OSError:
                        if time.monotonic() > deadline:
                            raise RuntimeError('SITL TCP startup timeout')
                        time.sleep(.5)
                if connection.wait_heartbeat(timeout=30) is None:
                    raise RuntimeError('heartbeat timeout')
                connection.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                # Wait for EKF absolute horizontal position flag, not just a timer.
                deadline = time.monotonic()+90
                ready = False
                while time.monotonic() < deadline:
                    heartbeat()
                    message = receive()
                    if message and message.get_type() == 'EKF_STATUS_REPORT' and message.flags & 16:
                        ready = True
                        break
                if not ready:
                    raise RuntimeError('EKF position gate timeout')
                for parameter, expected_value in expected_parameters.items():
                    # Verify the active parameter rather than assuming defaults loaded.
                    connection.mav.param_request_read_send(1, 1, parameter.encode(), -1)
                    deadline = time.monotonic()+10
                    while time.monotonic() < deadline:
                        heartbeat()
                        message = receive()
                        if (message and message.get_type() == 'PARAM_VALUE'
                                and message.param_id == parameter):
                            result.setdefault('parameters_readback', {})[parameter] = message.param_value
                            if parameter == 'WP_SPD':
                                result['WP_SPD_readback_m_s'] = message.param_value
                            break
                    if abs(result.get('parameters_readback', {}).get(parameter, -1)-expected_value) > .01:
                        raise RuntimeError(f'{parameter} readback must be {expected_value}')
                connection.set_mode('GUIDED')
                time.sleep(1)
                command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                deadline = time.monotonic()+30
                armed = False
                retry_at = time.monotonic()+3
                while time.monotonic() < deadline:
                    heartbeat()
                    h = receive()
                    if time.monotonic() >= retry_at:
                        connection.set_mode('GUIDED')
                        command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                        retry_at = time.monotonic()+3
                    if h and h.get_type() == 'HEARTBEAT' and h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        armed = True
                        break
                if not armed:
                    raise RuntimeError('arm gate failed')
                command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 5)
                deadline, stable_since = time.monotonic()+60, None
                while time.monotonic() < deadline:
                    heartbeat()
                    connection.recv_match(blocking=True, timeout=.1)
                    with lock:
                        sample = dict(latest)
                    p = sample.get('position_enu', [999,999,999])
                    good = (time.monotonic()-sample.get('wall_monotonic_s', 0) < 1
                            and abs(p[2]-5) < .25 and np.linalg.norm(p[:2]) < .25)
                    stable_since = (stable_since or time.monotonic()) if good else None
                    if stable_since and time.monotonic()-stable_since > 3:
                        break
                else:
                    raise RuntimeError('hover/start gate timeout')
                result['start_measured_enu'] = p
                commandline = [sys.executable, str(ROOT/'scripts/mppi_velocity_avoidance.py'),
                    '--planner', 'mppi', '--config', str(config), '--mav', 'tcp:127.0.0.1:5762',
                    '--goal', ','.join(map(str, goal)), '--global-path', reference_text,
                    '--reference-speed-m-s', str(speed), '--vmax', str(speed),
                    '--seed', str(seed),
                    '--diag-every', '20', '--diag-jsonl', str(trial/'planner.jsonl'), '--exit-on-goal']
                if args.debug_snapshot_cycle is not None:
                    commandline += ['--debug-snapshot-cycle', str(args.debug_snapshot_cycle)]
                if args.debug_snapshot_events:
                    commandline += ['--debug-snapshot-events', '--debug-control-stride',
                                    str(args.debug_control_stride)]
                result['planner_start_wall_s'] = time.monotonic()
                child = start(commandline, 'planner', ROOT)
                deadline = time.monotonic()+args.timeout
                while child.poll() is None and time.monotonic() < deadline:
                    heartbeat()
                    connection.recv_match(blocking=True, timeout=.1)
                    with lock:
                        sample = dict(latest)
                    p = np.array(sample.get('position_enu', [999,999,999]))
                    if time.monotonic()-sample.get('wall_monotonic_s', 0) > 2:
                        result['status'] = 'ABORT_STALE_ODOMETRY'
                        break
                    if (p[2] < 3.5 or p[2] > 6.5
                            or np.linalg.norm(p[:2]) > horizontal_envelope_m
                            or (straight and abs(p[1]) > 10)):
                        result['status'] = 'ABORT_FLIGHT_ENVELOPE'
                        break
                    if any(o.contains_xy(p[:2], 1.0) for o in planner._active(p[2])):
                        result['status'] = 'ABORT_GEOMETRIC_CLEARANCE_LT_1M'
                        break
                else:
                    result['status'] = 'FINISHED' if child.poll() == 0 else ('PLANNER_FAILED' if child.poll() is not None else 'TIMEOUT')
                result['planner_end_wall_s'] = time.monotonic()
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    child.wait(timeout=10)
                rows = [json.loads(line) for line in (trial/'planner.jsonl').read_text().splitlines()]
                result['reached'] = bool(rows and rows[-1].get('event') == 'reached')
                if result['status'] == 'FINISHED' and not result['reached']:
                    result['status'] = 'EXIT_WITHOUT_REACHED'
            except Exception as error:
                result['error'] = repr(error)
            finally:
                if connection:
                    connection.set_mode('LAND')
                    # GUI rendering can reduce the simulation real-time factor;
                    # allow a full descent from 5 m before tearing Gazebo down.
                    deadline = time.monotonic()+60
                    result['landed_disarmed'] = False
                    while time.monotonic() < deadline:
                        heartbeat()
                        h = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                        if h and not h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                            result['landed_disarmed'] = True
                            break
                    connection.close()
                for process in reversed(owned):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGTERM)
                            process.wait(timeout=5)
                node.unsubscribe('/iris/odometry')
                with lock:
                    if odom_file:
                        odom_file.close()
                        odom_file = None
                for handle in streams:
                    handle.close()
                (trial/'result.json').write_text(json.dumps(result, indent=2))
                print(json.dumps(result), flush=True)
                time.sleep(2)
            if result['status'] == 'SETUP_FAILED':
                raise SystemExit('Setup failed; stopping sweep instead of repeating invalid trials')


if __name__ == '__main__':
    main()

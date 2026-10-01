#!/usr/bin/env python3
"""One RGB-only obstacle-perception closed-loop Gazebo/ArduPilot trial."""
import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def signal_process(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except PermissionError:
        proc.send_signal(sig)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--world', type=Path, default=ROOT/'worlds/iris_monocular_obstacle.sdf')
    parser.add_argument('--config', type=Path, default=ROOT/'config/monocular_mppi.yaml')
    parser.add_argument('--perception-config', type=Path, help='JSON with inference CLI flags')
    parser.add_argument('--goal',type=float,nargs=3,default=[12,0,3])
    parser.add_argument('--eval-scene',type=Path,help='Evaluation-only JSON boxes; never sent to perception/planner')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--duration', type=float, default=45)
    parser.add_argument('--eval-box-center',type=float,nargs=3,default=[8,0,3],help='Evaluation only; not sent to perception/planner')
    parser.add_argument('--eval-box-size',type=float,nargs=3,default=[2,3,6],help='Evaluation only')
    parser.add_argument('--camera-blackout-at-x',type=float)
    parser.add_argument('--camera-blackout-seconds',type=float,default=5.)
    parser.add_argument('--openvins-bridge',type=Path,help='Built native OpenVINS/Gazebo bridge; required when pose_source=openvins')
    parser.add_argument('--openvins-zupt-after-motion',action='store_true',
                        help='Ablation: allow OpenVINS zero-velocity updates after the initial hover')
    parser.add_argument('--vio-only-motion-stop',action='store_true',
                        help='Run hover, ~1 m lateral translation, hover with OpenVINS only; no perception/planner')
    parser.add_argument('--vio-only-translation-m',type=float,default=1.,
                        help='Ground-truth lateral stopping distance for isolated VIO/depth probe')
    parser.add_argument('--multiframe-depth-probe',action='store_true',
                        help='Log standalone multi-frame inverse-depth landmarks during VIO-only motion-stop')
    parser.add_argument('--mentor-depth-benchmark',action='store_true',
                        help='RGB-only metric depth versus evaluation-only 3D LiDAR; no OpenVINS or planner')
    parser.add_argument('--arducopter-bin',type=Path,
                        default=Path(os.environ.get('ARDUCOPTER_BIN',ROOT.parent/'ardupilot/build/sitl/bin/arducopter')),
                        help='ArduPilot SITL executable (or set ARDUCOPTER_BIN)')
    parser.add_argument('--depth-device',choices=('cpu','mps','cuda'),
                        default='mps' if platform.system()=='Darwin' else 'cpu',
                        help='Torch device for the learned depth predictor')
    parser.add_argument('--latency-probe-speed',type=float,help='Open-loop velocity probe (m/s); collect camera/VIO latency without starting planner')
    parser.add_argument('--probe-stop-x',type=float,default=22.,help='Ground-truth x at which the open-loop latency probe begins braking')
    args = parser.parse_args()
    if not 0 < args.duration <= 300:
        parser.error('duration must be in (0,300] seconds')
    if args.latency_probe_speed is not None and not (0 < args.latency_probe_speed <= 10):
        parser.error('--latency-probe-speed must be in (0,10] m/s')
    if not 0 < args.probe_stop_x <= 60:
        parser.error('--probe-stop-x must be in (0,60] m')
    for port in (5760,5762):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',port)) == 0:
                raise SystemExit(f'SITL port {port} occupied')
    out = args.output.resolve()
    world = args.world.resolve()
    config = args.config.resolve()
    arducopter_bin = args.arducopter_bin.expanduser().resolve()
    for label, path in (('world', world), ('config', config)):
        if not path.is_file():
            parser.error(f'{label} file not found: {path}')
    if not arducopter_bin.is_file():
        parser.error(f'ArduCopter SITL executable not found: {arducopter_bin}')
    plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', str(ROOT/'build'))
    if not any(Path(entry).is_dir() for entry in plugin_path.split(os.pathsep) if entry):
        parser.error(f'Gazebo plugin directory not found: {plugin_path}')
    runtime_cfg=yaml.safe_load(config.read_text())
    min_obstacle_points=runtime_cfg.get('min_obstacle_points',0)
    if (not isinstance(min_obstacle_points,int) or isinstance(min_obstacle_points,bool)
            or not 0<=min_obstacle_points<=runtime_cfg.get('max_points',200)):
        parser.error('min_obstacle_points must be an integer in [0,max_points]')
    perception_flags = json.loads(args.perception_config.read_text()) if args.perception_config else {}
    backend=perception_flags.pop('backend','learned')
    if backend not in ('learned','geometry'):parser.error('unsupported perception backend')
    visual_mode=(backend=='geometry' and perception_flags.get('pose_source')=='visual-ground')
    openvins_mode=(backend=='geometry' and perception_flags.get('pose_source')=='openvins')
    image_pose_mode=visual_mode or openvins_mode
    if args.latency_probe_speed is not None and not openvins_mode:
        parser.error('--latency-probe-speed requires OpenVINS geometry perception')
    if args.openvins_zupt_after_motion and not openvins_mode:
        parser.error('--openvins-zupt-after-motion requires OpenVINS perception')
    if args.vio_only_motion_stop and (not openvins_mode or args.latency_probe_speed is not None):
        parser.error('--vio-only-motion-stop requires OpenVINS and cannot be combined with latency probe')
    if not 0 < args.vio_only_translation_m <= 2:
        parser.error('--vio-only-translation-m must be in (0,2] m')
    if args.multiframe_depth_probe and not args.vio_only_motion_stop:
        parser.error('--multiframe-depth-probe requires --vio-only-motion-stop')
    if args.mentor_depth_benchmark and (backend!='learned' or args.vio_only_motion_stop or
                                        args.latency_probe_speed is not None):
        parser.error('--mentor-depth-benchmark requires the learned backend without another probe')
    if args.mentor_depth_benchmark and any(perception_flags.get(key) for key in
                                           ('gt_topic','local_map','pose_source','output_topic')):
        parser.error('mentor benchmark predictor must not read GT/pose or override its isolated output topic')
    if openvins_mode and (args.openvins_bridge is None or not args.openvins_bridge.is_file()):
        parser.error('--openvins-bridge must name a built OpenVINS bridge executable')
    perception_script=ROOT/'scripts'/('monocular_geometry_gz.py' if backend=='geometry' else 'monocular_depth_gz.py')
    from mppi_ardupilot.monocular_evaluation import scene_clearance
    boxes=json.loads(args.eval_scene.read_text())["boxes"] if args.eval_scene else [dict(center=args.eval_box_center,size=args.eval_box_size)]
    scene_clearance([0,0,0],boxes)
    if not np.isfinite(args.goal).all():parser.error("goal must be finite")
    box_center=np.asarray(args.eval_box_center);box_half=np.asarray(args.eval_box_size)/2
    if np.any(box_half<=0):parser.error('evaluation box size must be positive')
    params = ROOT/runtime_cfg.get('sitl_param_file','config/mppi_velocity.parm')
    if not params.is_file() or not params.resolve().is_relative_to(ROOT):
        parser.error('sitl_param_file must name a file in the experiment snapshot')
    out.mkdir(parents=True, exist_ok=False)
    os.environ.update(MAVLINK20='1', GZ_PARTITION=f'monocular_only_{os.getpid()}')
    env = os.environ.copy()
    env.update(GZ_SIM_SYSTEM_PLUGIN_PATH=plugin_path,
        GZ_SIM_RESOURCE_PATH=f'{ROOT}/models:{ROOT}/worlds', PYTHONUNBUFFERED='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    from pymavlink import mavutil
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    files = [world, config, params, Path(__file__), perception_script]
    if args.perception_config:
        files.append(args.perception_config.resolve())
    if args.eval_scene:
        files.append(args.eval_scene.resolve())
    if args.mentor_depth_benchmark:
        # This trial does not start the planner or OpenVINS. Snapshot only its
        # runtime, scene, and offline evaluators for a focused handoff.
        files.extend(ROOT / name for name in (
            'mppi_ardupilot/monocular_depth.py',
            'mppi_ardupilot/monocular_evaluation.py',
            'mppi_ardupilot/monocular_multiframe_depth.py',
            'mppi_ardupilot/monocular_point_tracker.py',
            'mppi_ardupilot/monocular_triangulation.py',
            'mppi_ardupilot/lidar_preprocess.py',
            'scripts/log_lidar_gt_gz.py',
            'scripts/evaluate_monocular_lidar_depth.py',
            'scripts/evaluate_multiframe_ekf_lidar.py',
            'scripts/audit_mentor_depth_benchmark.py',
            'scripts/plot_mentor_depth_benchmark.py',
            'models/iris_with_monocular_camera_lidar_gt/model.sdf',
            'models/iris_with_monocular_camera_lidar_gt/model.config',
            'models/iris_with_gimbal_monocular/model.sdf',
            'models/gimbal_small_3d_monocular/model.sdf',
            'models/iris_with_standoffs/model.sdf',
            'models/gimbal_small_3d/model.sdf',
            'models/monocular_textured_box/model.sdf',
        ))
    else:
        files.extend([ROOT/'models/iris_with_monocular_camera/model.sdf',
                      ROOT/'mppi_ardupilot/monocular_depth.py',
                      ROOT/'mppi_ardupilot/camera_local_map.py'])
        files.extend([ROOT/name for name in ['mppi_ardupilot/mppi_controller.py','mppi_ardupilot/mppi_local_planner_node.py','mppi_ardupilot/camera_motion_primitives.py','mppi_ardupilot/trajectory_safety.py','mppi_ardupilot/trajectory_validation.py','mppi_ardupilot/mavlink_interface.py','mppi_ardupilot/lidar_preprocess.py','scripts/mppi_velocity_avoidance.py','models/iris_with_gimbal/model.sdf','models/iris_with_standoffs/model.sdf','models/gimbal_small_3d/model.sdf']])
        files.extend([ROOT/name for name in ['models/iris_with_gimbal_monocular/model.sdf','models/gimbal_small_3d_monocular/model.sdf'] if (ROOT/name).exists()])
        if backend=='geometry':files.extend([ROOT/'mppi_ardupilot/monocular_point_tracker.py',ROOT/'mppi_ardupilot/monocular_triangulation.py',ROOT/'models/monocular_textured_box/model.sdf'])
        files.append(ROOT/"mppi_ardupilot/monocular_evaluation.py")
        if openvins_mode:
            files.extend([ROOT/'tools/openvins_gz_bridge/main.cpp',ROOT/'tools/openvins_gz_bridge/CMakeLists.txt',
                          ROOT/'tools/openvins_gz_bridge/README.md'])
        if args.multiframe_depth_probe:
            files.extend([ROOT/'scripts/monocular_multiframe_gz.py',
                          ROOT/'scripts/analyze_monocular_multiframe_depth.py'])
        # Capture runtime helper dependencies as well as the main controller.
        for p in [*(ROOT/'mppi_ardupilot').glob('*.py'),
                  ROOT/'scripts/lidar_to_mavlink_avoidance.py',
                  ROOT/'scripts/run_monocular_speed_suite.py',
                  ROOT/'scripts/analyze_monocular_sim.py',
                  ROOT/'scripts/report_monocular_latency.py',
                  ROOT/'scripts/audit_monocular_calibration.py',
                  ROOT/'scripts/analyze_vio_motion_stop.py']:
            if p not in files:files.append(p)
    manifest = dict(perception='RGB monocular only',perception_backend=backend,
        lidar_present=args.mentor_depth_benchmark,lidar_role=('offline ground truth only' if args.mentor_depth_benchmark else None),
        depth_camera_present=False,
        known_geometry_input=image_pose_mode, localization=('ArduPilot EKF GPS/IMU flight state; RGB-only depth predictor' if args.mentor_depth_benchmark else
            'RGB ground features + IMU orientation + declared 3 m height' if visual_mode else
            'OpenVINS RGB + raw IMU + declared 3 m hover origin' if openvins_mode else
            'Gazebo odometry (not camera-only localization)'),
        openvins_bridge_sha256=hashlib.sha256(args.openvins_bridge.read_bytes()).hexdigest() if openvins_mode else None,
        openvins_zupt_after_motion=args.openvins_zupt_after_motion,
        vio_only_motion_stop=args.vio_only_motion_stop,
        vio_only_translation_m=args.vio_only_translation_m,
        multiframe_depth_probe=args.multiframe_depth_probe,
        mentor_depth_benchmark=args.mentor_depth_benchmark,
        partition=env['GZ_PARTITION'], duration_wall_s=args.duration, seed=args.seed,
        eval_box_center=args.eval_box_center,eval_box_size=args.eval_box_size,eval_boxes=boxes,goal_enu=args.goal,
        evaluation='Box surface distance from ground-truth vehicle center, 0.5 m proxy radius. Not contact telemetry.',
        files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    for p in files:
        dest=out/'source_snapshot'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(p.read_bytes())
    owned=[];streams=[];latest={};lock=threading.Lock();connection=None
    visual_latest={}
    result=dict(status='SETUP_FAILED', lidar_topic_messages=0, gt_lidar_messages=0,
        depth_topic_messages=0,
        camera_cloud_messages=0, min_obstacle_points=min_obstacle_points,
        camera_cloud_last_points=0, min_box_center_clearance_m=None, collision_proxy=False,
        eval_box_center=args.eval_box_center,eval_box_size=args.eval_box_size,eval_boxes=boxes,goal_enu=args.goal,seed=args.seed)
    blackout_resume=None;blackout_done=False;inference=None;bridge=None;lidar_logger=None
    odom_file=(out/'ground_truth.jsonl').open('w');event_file=(out/'mavlink_events.jsonl').open('w')
    telemetry_file=(out/'ardupilot_ekf_telemetry.jsonl').open('w') if args.mentor_depth_benchmark else None
    benchmark_ekf={}
    def start(cmd,name,cwd=ROOT,extra_env=None):
        handle=(out/f'{name}.log').open('w');streams.append(handle)
        proc=subprocess.Popen(cmd,cwd=cwd,env={**env,**(extra_env or {})},
                              stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        owned.append(proc);(out/f'{name}.command.json').write_text(json.dumps(cmd,indent=2));return proc
    def odom(msg):
        p=msg.pose.position
        row=dict(wall_s=time.monotonic(),sim_s=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,position_enu=[p.x,p.y,p.z],
            attitude_quaternion_wxyz=[msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        with lock:
            latest.update(row);odom_file.write(json.dumps(row)+'\n')
    def visual_odom(msg):
        p=msg.pose.position
        with lock:visual_latest.update(wall_s=time.monotonic(),position_enu=[p.x,p.y,p.z])
    def count(key):
        def cb(msg):
            with lock:result[key]+=1
        return cb
    def camera_cloud(msg):
        with lock:
            result['camera_cloud_messages']+=1
            result['camera_cloud_last_points']=int(msg.width)*int(msg.height)
            latest['camera_cloud_wall_s']=time.monotonic()
    def heartbeat():
        connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        connection.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,65535,65535,65535,65535)
    def receive(dt=.1):
        m=connection.recv_match(blocking=True,timeout=dt)
        if m is not None and telemetry_file is not None:
            kind=m.get_type()
            if kind in ('GPS_RAW_INT','GLOBAL_POSITION_INT','LOCAL_POSITION_NED','ATTITUDE','EKF_STATUS_REPORT'):
                row=dict(wall_s=time.monotonic(),kind=kind,sim_s=latest.get('sim_s'),data=m.to_dict())
                telemetry_file.write(json.dumps(row)+'\n')
                if kind=='GPS_RAW_INT':
                    result['gps_fix_type_max']=max(result.get('gps_fix_type_max',0),int(m.fix_type))
                    result['gps_fix_samples']=result.get('gps_fix_samples',0)+1
                elif kind=='LOCAL_POSITION_NED':
                    benchmark_ekf.update(wall_s=row['wall_s'],north_m=float(m.x),east_m=float(m.y),down_m=float(m.z))
                    result['ekf_local_position_samples']=result.get('ekf_local_position_samples',0)+1
        if m and m.get_type() in ('STATUSTEXT','COMMAND_ACK','HEARTBEAT'):
            event_file.write(json.dumps(dict(wall_s=time.monotonic(),data=m.to_dict()))+'\n')
        return m
    def command(code,*values):
        connection.mav.command_long_send(1,1,code,0,*(list(values)+[0]*(7-len(values))))
    node=Node();node.subscribe(Odometry,'/iris/odometry',odom)
    if image_pose_mode:node.subscribe(Odometry,'/perception/visual_odometry',visual_odom)
    node.subscribe(PointCloudPacked,'/perception/obstacles_camera',camera_cloud)
    node.subscribe(PointCloudPacked,'/sensor_suite/lidar/points',count('lidar_topic_messages'))
    if args.mentor_depth_benchmark:
        node.subscribe(PointCloudPacked,'/benchmark/lidar_ground_truth/points',count('gt_lidar_messages'))
    from gz.msgs10.image_pb2 import Image
    node.subscribe(Image,'/sensor_suite/depth',count('depth_topic_messages'))
    try:
        start(['gz','sim','-s','-r','-v3',str(world)],'gazebo')
        start([str(arducopter_bin),'-w','--model','JSON',
            '--speedup','1','--slave','0','--serial1=tcp:2','--defaults',str(params),
            '--sim-address=127.0.0.1','-I0','--home=-35.363262,149.165237,584,0'],'sitl',out)
        inference_args=[]
        for key,value in perception_flags.items():
            if isinstance(value,bool):
                if value:inference_args.append('--'+key.replace('_','-'))
            else:inference_args.extend(['--'+key.replace('_','-'),str(value)])
        inference_command=[sys.executable,str(perception_script),
            *(['--device',args.depth_device] if backend=='learned' else []),'--frames','100000','--timeout',str(args.duration+240),
            '--output-dir',str(out/'perception'),
            *(['--save-every','1','--output-topic','/benchmark/monocular_prediction'] if args.mentor_depth_benchmark else []),
            *inference_args]
        if not image_pose_mode:
            inference=start(inference_command,'inference',
                            extra_env={'KMP_DUPLICATE_LIB_OK':'TRUE'} if backend=='learned' else None)
        if args.mentor_depth_benchmark:
            lidar_logger=start([sys.executable,str(ROOT/'scripts/log_lidar_gt_gz.py'),
                                '--output-dir',str(out/'lidar_gt'),
                                '--timeout',str(args.duration+240)],'lidar_gt')
        deadline=time.monotonic()+35
        while connection is None:
            try:connection=mavutil.mavlink_connection('tcp:127.0.0.1:5760')
            except OSError:
                if time.monotonic()>deadline:raise RuntimeError('SITL startup timeout')
                time.sleep(.5)
        if connection.wait_heartbeat(timeout=30) is None:raise RuntimeError('heartbeat timeout')
        connection.mav.request_data_stream_send(1,1,mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)
        if args.mentor_depth_benchmark:
            for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                               mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                               mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                               mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
                connection.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,message_id,100000,0,0,0,0,0)
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            heartbeat();m=receive()
            if inference is not None and inference.poll() is not None:raise RuntimeError('inference exited before takeoff')
            if m and m.get_type()=='EKF_STATUS_REPORT' and m.flags&16:break
        else:raise RuntimeError('EKF position timeout')
        for parameter in ('OA_TYPE','AVOID_ENABLE','PRX1_TYPE'):
            connection.mav.param_request_read_send(1,1,parameter.encode(),-1)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                heartbeat();m=receive()
                if m and m.get_type()=='PARAM_VALUE' and m.param_id==parameter:
                    result.setdefault('parameters_readback',{})[parameter]=m.param_value
                    if m.param_value!=0:raise RuntimeError(f'{parameter} must be zero')
                    break
            else:raise RuntimeError(f'parameter readback timeout: {parameter}')
        deadline=time.monotonic()+120;retry=0
        while time.monotonic()<deadline:
            heartbeat()
            if time.monotonic()>retry:
                connection.set_mode('GUIDED');command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1);retry=time.monotonic()+3
            m=receive()
            if m and m.get_type()=='HEARTBEAT' and m.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:break
        else:raise RuntimeError('arm timeout')
        command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,3)
        deadline=time.monotonic()+60;stable=None
        while time.monotonic()<deadline:
            heartbeat();receive()
            with lock:sample=dict(latest)
            p=sample.get('position_enu',[999,999,999])
            good=time.monotonic()-sample.get('wall_s',0)<1 and abs(p[2]-3)<.3 and np.linalg.norm(p[:2])<.4
            stable=(stable or time.monotonic()) if good else None
            predictor_ready=((out/'perception/frames.jsonl').is_file() and
                             (out/'perception/frames.jsonl').stat().st_size>0)
            if stable and time.monotonic()-stable>2 and (backend=='geometry' or
                    args.mentor_depth_benchmark and predictor_ready or
                    result['camera_cloud_messages']>5):break
        else:raise RuntimeError('hover or camera readiness timeout')
        if image_pose_mode:
            if openvins_mode:
                bridge=start([str(args.openvins_bridge.resolve()),str(out/'openvins.csv'),
                              *(['--zupt-after-motion'] if args.openvins_zupt_after_motion else []),
                              *(['--debug-zupt'] if args.vio_only_motion_stop else [])],'openvins')
            if args.multiframe_depth_probe:
                start([sys.executable,str(ROOT/'scripts/monocular_multiframe_gz.py'),
                       '--output-dir',str(out/'multiframe'),
                       '--timeout',str(args.duration+240)],'multiframe')
            if not args.vio_only_motion_stop:
                inference=start(inference_command,'inference')
            calibration_start=time.monotonic()
            deadline=time.monotonic()+(25 if openvins_mode else 10)
            while time.monotonic()<deadline:
                heartbeat();receive(.1)
                with lock:ready=time.monotonic()-visual_latest.get('wall_s',0)<1
                if ready and time.monotonic()-calibration_start>=2.5:break
                if inference is not None and inference.poll() is not None:
                    raise RuntimeError('visual odometry exited before warmup')
                if bridge is not None and bridge.poll() is not None:
                    raise RuntimeError('OpenVINS bridge exited before warmup')
            else:raise RuntimeError('visual odometry not initialized at known height')
        if args.mentor_depth_benchmark:
            if result.get('gps_fix_type_max',0)<3:
                raise RuntimeError('ArduPilot GPS_RAW_INT has no 3D fix')
            if result.get('gt_lidar_messages',0)<5:
                raise RuntimeError('evaluation-only LiDAR did not publish scans')
            if not benchmark_ekf:
                raise RuntimeError('ArduPilot local EKF position unavailable')
            def benchmark_velocity_ned(north,east):
                connection.mav.set_position_target_local_ned_send(
                    0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    1479,0,0,0,north,east,0,0,0,0,0,0)
            def benchmark_sample():
                heartbeat();receive(.05)
                with lock:sample=dict(latest)
                if time.monotonic()-sample.get('wall_s',0)>1:
                    raise RuntimeError('benchmark safety truth stale')
                truth=np.asarray(sample['position_enu'])
                if abs(truth[2]-3)>.6 or truth[0]>3.5 or np.linalg.norm(truth[:2])>4:
                    raise RuntimeError('benchmark flight envelope exceeded')
                if time.monotonic()-benchmark_ekf.get('wall_s',0)>1:
                    raise RuntimeError('ArduPilot EKF state stale')
                if inference.poll() is not None or lidar_logger.poll() is not None:
                    raise RuntimeError('depth predictor or GT logger exited early')
                return sample,truth,dict(benchmark_ekf)
            phases={}
            for phase,duration in [('hover_before',4.),('move_forward',8.),('hover_after',4.)]:
                sample,truth,ekf=benchmark_sample()
                phases[phase]=dict(start_sim_s=sample['sim_s'],
                                   start_truth_enu=truth.tolist(),start_ekf_ned=ekf)
                start_wall=time.monotonic()
                start_east=ekf['east_m']
                while time.monotonic()-start_wall<duration:
                    benchmark_velocity_ned(0.,.5 if phase=='move_forward' else 0.)
                    sample,truth,ekf=benchmark_sample()
                    if phase=='move_forward' and ekf['east_m']-start_east>=2.5:
                        break
                phases[phase].update(end_sim_s=sample['sim_s'],
                                     end_truth_enu=truth.tolist(),end_ekf_ned=ekf)
            benchmark_velocity_ned(0.,0.)
            result['benchmark_phases']=phases
            result['last_position_enu']=truth.tolist()
            result['status']='DEPTH_BENCHMARK_COMPLETE'
            result['end_wall_s']=time.monotonic()
            if result['lidar_topic_messages'] or result['depth_topic_messages']:
                raise RuntimeError('unexpected non-evaluator LiDAR/depth topic')
            return
        if args.vio_only_motion_stop:
            def velocity_ned(north,east):
                connection.mav.set_position_target_local_ned_send(
                    0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    1479,0,0,0,north,east,0,0,0,0,0,0)
            def checked_sample():
                heartbeat();receive(.05)
                with lock:sample=dict(latest)
                if time.monotonic()-sample.get('wall_s',0)>1:
                    raise RuntimeError('VIO-only evaluation odometry stale')
                truth=np.asarray(sample['position_enu'])
                if abs(truth[2]-3)>.6 or np.linalg.norm(truth[:2])>3:
                    raise RuntimeError('VIO-only truth safety envelope')
                if bridge.poll() is not None:raise RuntimeError('OpenVINS bridge exited')
                return sample,truth
            phases={}
            for phase,duration in [('hover_before',6.),('move_lateral',8.),('hover_after',10.)]:
                sample,truth=checked_sample()
                phases[phase]={'start_sim_s':sample['sim_s'],
                               'start_position_enu':truth.tolist()}
                started=time.monotonic()
                while time.monotonic()-started<duration:
                    velocity_ned(.25,0.) if phase=='move_lateral' else velocity_ned(0.,0.)
                    sample,truth=checked_sample()
                    if phase=='move_lateral' and truth[1]-phases[phase]['start_position_enu'][1]>=args.vio_only_translation_m:
                        break
                phases[phase].update(end_sim_s=sample['sim_s'],end_position_enu=truth.tolist())
            velocity_ned(0.,0.)
            result['vio_motion_stop_phases']=phases
            result['last_position_enu']=truth.tolist()
            result['status']='VIO_ONLY_COMPLETE'
            result['end_wall_s']=time.monotonic()
            if result['lidar_topic_messages'] or result['depth_topic_messages']:
                raise RuntimeError('unexpected LiDAR/depth input present')
            return
        planner_gate=out/'planner.start';planner_ready=out/'planner.ready'
        planner_command=[sys.executable,str(ROOT/'scripts/mppi_velocity_avoidance.py'),
            '--config',str(config),'--goal',','.join(map(str,args.goal)),'--seed',str(args.seed),'--state-source','odom','--mav','tcp:127.0.0.1:5762',
            *(['--odom-topic','/perception/visual_odometry'] if image_pose_mode else []),
            '--diag-jsonl',str(out/'planner.jsonl'),'--diag-every','1','--exit-on-goal']
        if openvins_mode and args.latency_probe_speed is None:
            planner=start([*planner_command,'--start-gate-file',str(planner_gate),
                           '--ready-file',str(planner_ready)],'planner')
            deadline=time.monotonic()+20
            while not planner_ready.exists() and time.monotonic()<deadline:
                heartbeat();receive(.05)
                if planner.poll() is not None:raise RuntimeError('planner exited before ready')
            if not planner_ready.exists():raise RuntimeError('planner subscription readiness timeout')
        if args.latency_probe_speed is not None:
            # This is a latency experiment, not an autonomous obstacle-avoidance run.
            # Ground truth is read here only to bound the probe flight and score speed.
            result.update(status='PROBE_RUNNING',target_speed_m_s=args.latency_probe_speed,
                          planner_started=False,probe_start_wall_s=time.monotonic())
            probe_end=result['probe_start_wall_s']+args.duration
            target_x=args.probe_stop_x
            while time.monotonic()<probe_end:
                heartbeat()
                connection.mav.set_position_target_local_ned_send(
                    0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    1479,0,0,0,0,args.latency_probe_speed,0,0,0,0,0,0)
                receive(.05)
                with lock:sample=dict(latest)
                if time.monotonic()-sample.get('wall_s',0)>2:
                    raise RuntimeError('ground-truth safety odometry stale during probe')
                p=np.asarray(sample['position_enu'])
                if abs(p[2]-3)>1 or abs(p[1])>2 or p[0]>target_x+18:
                    raise RuntimeError(f'probe flight envelope exceeded at {p.tolist()}')
                if p[0]>=target_x:
                    result['probe_stop_reason']='target_x';break
                if inference.poll() is not None or bridge.poll() is not None:
                    raise RuntimeError('latency probe process exited')
            for _ in range(20):
                heartbeat()
                connection.mav.set_position_target_local_ned_send(
                    0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    1479,0,0,0,0,0,0,0,0,0,0,0)
                receive(.05)
            result['probe_end_wall_s']=time.monotonic()
            result['last_position_enu']=p.tolist()
            result['status']='PROBE_COMPLETE'
            if result['lidar_topic_messages'] or result['depth_topic_messages']:
                raise RuntimeError('unexpected LiDAR/depth input present')
            return
        if backend=='geometry':
            # Monocular triangulation needs translation; no obstacle geometry is consulted.
            with lock:
                warmup_start=np.array(visual_latest['position_enu'] if image_pose_mode else p)
                warmup_truth_start=np.array(latest.get('position_enu',p))
            deadline=time.monotonic()+15
            def warmup_velocity(v_enu):
                # Diagonal motion gives a frontal object lateral parallax.
                vx_ned,vy_ned=(v_enu,v_enu) if openvins_mode else (v_enu,0)
                connection.mav.set_position_target_local_ned_send(0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    1479,0,0,0,vx_ned,vy_ned,0,0,0,0,0,0)
            while time.monotonic()<deadline:
                heartbeat();warmup_velocity(.25);receive(.1)
                with lock:
                    sample=dict(visual_latest if image_pose_mode else latest)
                    safety_sample=dict(latest)
                    cloud_count=result['camera_cloud_messages']
                    cloud_points=result['camera_cloud_last_points']
                    cloud_recent=time.monotonic()-latest.get('camera_cloud_wall_s',0)<1
                p=np.array(sample.get('position_enu',warmup_start))
                safety_p=np.array(safety_sample.get('position_enu',warmup_truth_start))
                if openvins_mode:
                    if time.monotonic()-safety_sample.get('wall_s',0)>1:
                        warmup_velocity(0);raise RuntimeError('warmup safety odometry stale')
                    if (abs(safety_p[2]-3)>.5 or abs(safety_p[1])>2. or
                            safety_p[0]-warmup_truth_start[0]>2.4):
                        warmup_velocity(0);raise RuntimeError('warmup safety envelope')
                    if time.monotonic()-sample.get('wall_s',0)<1 and np.linalg.norm(p-safety_p)>1.:
                        warmup_velocity(0);raise RuntimeError('warmup VIO error exceeds 1 m')
                    if (safety_p[0]-warmup_truth_start[0]>.5 and
                            safety_p[1]-warmup_truth_start[1]>.5 and cloud_count>=1 and
                            cloud_points>=min_obstacle_points and cloud_recent):break
                    continue
                if abs(p[2]-warmup_start[2])>.4 or np.linalg.norm(p[:2]-warmup_start[:2])>1.:
                    warmup_velocity(0);raise RuntimeError('warmup motion envelope')
                if p[1]-warmup_start[1]>.55:break
            warmup_velocity(0);result['warmup_displacement_enu']=(p-warmup_start).tolist()
            if openvins_mode:result['warmup_truth_displacement_enu']=(safety_p-warmup_truth_start).tolist()
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                heartbeat();warmup_velocity(0);receive(.1)
                if inference.poll() is not None:raise RuntimeError('geometry perception exited')
                with lock:
                    ready=(result['camera_cloud_messages']>=(1 if openvins_mode else 6) and
                        (not min_obstacle_points or result['camera_cloud_last_points']>=min_obstacle_points
                         and time.monotonic()-latest.get('camera_cloud_wall_s',0)<1))
                    safety_sample=dict(latest)
                    visual_sample=dict(visual_latest)
                if openvins_mode:
                    if time.monotonic()-visual_sample.get('wall_s',0)>1:
                        raise RuntimeError('VIO stale while waiting for camera cloud')
                    vio_error=float(np.linalg.norm(
                        np.asarray(visual_sample['position_enu'])-
                        np.asarray(safety_sample['position_enu'])))
                    if vio_error>1.:
                        raise RuntimeError('VIO error exceeds 1 m while waiting for camera cloud')
                if ready:break
            else:
                if min_obstacle_points:
                    raise RuntimeError(f"insufficient camera evidence after warmup: {result['camera_cloud_last_points']} points, requires {min_obstacle_points}")
                raise RuntimeError('temporal geometry camera readiness timeout')
        with lock:result['camera_cloud_points_at_planner_start']=result['camera_cloud_last_points']
        result['start_position_enu']=np.asarray(p).tolist();result['planner_start_wall_s']=time.monotonic()
        if openvins_mode:planner_gate.touch()
        else:planner=start(planner_command,'planner')
        deadline=time.monotonic()+args.duration;result['status']='TIMEOUT'
        while time.monotonic()<deadline:
            heartbeat();receive(.05)
            if blackout_resume is not None and time.monotonic()>=blackout_resume:
                signal_process(inference,signal.SIGCONT);blackout_resume=None
                result['camera_blackout_end_wall_s']=time.monotonic()
            with lock:
                sample=dict(latest)
                visual_sample=dict(visual_latest)
            if time.monotonic()-sample.get('wall_s',0)>15:raise RuntimeError('simulation odometry absent for 15s')
            if time.monotonic()-sample.get('wall_s',0)>2:
                result['stale_odometry_polls']=result.get('stale_odometry_polls',0)+1
                continue
            p=np.array(sample['position_enu']);result['last_position_enu']=p.tolist()
            if openvins_mode:
                if time.monotonic()-visual_sample.get('wall_s',0)>1:
                    result['status']='VIO_STALE';break
                vio_error=float(np.linalg.norm(np.asarray(visual_sample['position_enu'])-p))
                result['max_vio_position_error_m']=max(result.get('max_vio_position_error_m',0),vio_error)
                if vio_error>1.5:
                    result['status']='LOCALIZATION_DIVERGED';break
            if args.camera_blackout_at_x is not None and not blackout_done and p[0]>=args.camera_blackout_at_x:
                signal_process(inference,signal.SIGSTOP);blackout_done=True
                result['camera_blackout_start_wall_s']=time.monotonic()
                blackout_resume=time.monotonic()+args.camera_blackout_seconds
            # Evaluation geometry is never supplied to perception/planner.
            delta=np.maximum(abs(p-box_center)-box_half,0)
            distance=float(scene_clearance(p,boxes))
            old=result['min_box_center_clearance_m'];result['min_box_center_clearance_m']=distance if old is None else min(old,distance)
            if abs(p[2]-3)>1. or np.linalg.norm(p[:2])>max(25.,np.linalg.norm(args.goal[:2])+10.):
                result['status']='FLIGHT_ENVELOPE';break
            if distance<.5:
                result.update(status='COLLISION_PROXY',collision_proxy=True);break
            if np.linalg.norm(p-np.array(args.goal))<.5:
                result['status']='GOAL_REACHED';break
            if inference.poll() is not None:raise RuntimeError('inference stopped')
            if planner.poll() is not None:
                result['status']='PLANNER_EXIT';result['planner_exit_code']=planner.returncode;break
        result['end_wall_s']=time.monotonic()
        if result['lidar_topic_messages'] or result['depth_topic_messages']:
            raise RuntimeError('unexpected LiDAR/depth input present')
    except Exception as exc:
        result['error']=str(exc)
        if result['status'] in ('TIMEOUT','PROBE_RUNNING'):result['status']='INFRA_FAILURE'
        print(str(exc),flush=True)
    finally:
        if 'planner_start_wall_s' in result:result.setdefault('end_wall_s',time.monotonic())
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        if blackout_resume is not None and inference is not None:
            signal_process(inference,signal.SIGCONT)
        # Stop velocity stream before landing; keep simulation alive for landing.
        for proc in owned[2:]:
            if proc.poll() is None:
                signal_process(proc,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:signal_process(proc,signal.SIGTERM)
        if connection:
            connection.set_mode('LAND');deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                heartbeat();m=receive(.2)
                if m and m.get_type()=='HEARTBEAT' and not m.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:break
            connection.close()
        for proc in reversed(owned):
            if proc.poll() is None:
                signal_process(proc,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:signal_process(proc,signal.SIGKILL);proc.wait()
        for topic in ('/iris/odometry','/perception/obstacles_camera','/sensor_suite/lidar/points','/sensor_suite/depth',*(['/perception/visual_odometry'] if image_pose_mode else [])):node.unsubscribe(topic)
        if args.mentor_depth_benchmark:
            node.unsubscribe('/benchmark/lidar_ground_truth/points')
        odom_file.close();event_file.close()
        if telemetry_file is not None:telemetry_file.close()
        for handle in streams:handle.close()
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    if result['status'] in ('SETUP_FAILED','INFRA_FAILURE','PLANNER_EXIT'):raise SystemExit(1)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Controlled open-loop velocity step and zero-command braking in Gazebo/SITL.

Experimental controller/telemetry timing, not a safety certification.
"""
import os,sys,time,json,signal,subprocess,threading,argparse,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--speeds',nargs='+',type=float,default=[4,6,8,10]);ap.add_argument('--repeats',type=int,default=1);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
 if args.repeats<1 or any(not 0<s<=10 for s in args.speeds):ap.error('positive repeats and speeds in (0,10] required')
 for port in (5760,5762):
  import socket
  with socket.socket() as sock:
   if sock.connect_ex(('127.0.0.1',port))==0:raise SystemExit(f'SITL port {port} occupied')
 processes=subprocess.check_output(['ps','-axo','command'],text=True)
 if any('gz sim ' in line or '/bin/arducopter ' in line for line in processes.splitlines()):raise SystemExit('existing Gazebo/SITL session')
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 world=ROOT/'worlds/iris_mppi_speed_brake_long.sdf';params=ROOT/'config/experiments/mppi_yard_high_accel.parm';apdir=ROOT.parent/'ardupilot'
 (out/'manifest.json').write_text(json.dumps({'speeds':args.speeds,'repeats':args.repeats,'world':str(world),'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest(),'params':str(params),'params_sha256':hashlib.sha256(params.read_bytes()).hexdigest(),'no_mppi':True},indent=2))
 (out/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
 os.environ['MAVLINK20']='1';os.environ['GZ_PARTITION']=f'brake_primitive_{os.getpid()}'
 from pymavlink import mavutil
 from gz.transport13 import Node
 from gz.msgs10.odometry_pb2 import Odometry
 env=os.environ.copy();env.update(GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'build'),GZ_SIM_RESOURCE_PATH=f'{ROOT}/models:{ROOT}/worlds',PYTHONUNBUFFERED='1')
 owned=[];streams=[];latest={};lock=threading.Lock();connection=None;odom_file=None;events_file=None
 def start(cmd,name,cwd):
  handle=(out/f'{name}.stdout.log').open('w');streams.append(handle)
  child=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True);owned.append(child)
  (out/f'{name}.command.json').write_text(json.dumps(cmd,indent=2));return child
 def odom(msg):
  p=msg.pose.position;row={'wall_monotonic_s':time.monotonic(),'sim_time_s':msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,'position_enu':[p.x,p.y,p.z]}
  with lock:
   latest.update(row)
   if odom_file:odom_file.write(json.dumps(row)+'\n')
 def hb():
  connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
  connection.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,65535,65535,65535,65535)
 def receive(dt=.05):
  m=connection.recv_match(blocking=True,timeout=dt)
  if m and m.get_type() in ('POSITION_TARGET_LOCAL_NED','LOCAL_POSITION_NED','STATUSTEXT','COMMAND_ACK'):
   events_file.write(json.dumps({'wall_monotonic_s':time.monotonic(),'type':m.get_type(),'data':m.to_dict()})+'\n')
  return m
 def command(code,*values):connection.mav.command_long_send(1,1,code,0,*(list(values)+[0]*(7-len(values))))
 def send(speed,kind):
  stamp=time.monotonic();connection.mav.set_position_target_local_ned_send(int(stamp*1000)&0xffffffff,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,1479,0,0,0,0,float(speed),0,0,0,0,0,0)
  with lock: sample=dict(latest)
  events_file.write(json.dumps({'wall_monotonic_s':stamp,'type':'sent_setpoint','kind':kind,'vx_enu_m_s':speed,'sim_time_latest_s':sample.get('sim_time_s')})+'\n')
  return stamp,sample
 node=Node();node.subscribe(Odometry,'/iris/odometry',odom)
 try:
  odom_file=(out/'ground_truth.jsonl').open('w');events_file=(out/'mavlink_events.jsonl').open('w')
  start(['gz','sim','-v2','-r',str(world),'-s'],'gazebo',ROOT)
  start([str(apdir/'build/sitl/bin/arducopter'),'-w','--model','JSON','--speedup','1','--slave','0','--serial1=tcp:2','--defaults',str(params),'--sim-address=127.0.0.1','-I0','--home=-35.363262,149.165237,584,0'],'sitl',out)
  deadline=time.monotonic()+30
  while connection is None:
   try:connection=mavutil.mavlink_connection('tcp:127.0.0.1:5760')
   except OSError:
    if time.monotonic()>deadline:raise RuntimeError('SITL TCP startup timeout')
    time.sleep(.5)
  if connection.wait_heartbeat(timeout=30) is None:raise RuntimeError('heartbeat timeout')
  connection.mav.request_data_stream_send(1,1,mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)
  # Request target echo where firmware supports it; this is not execution acknowledgment.
  command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,100000)
  deadline=time.monotonic()+90
  while time.monotonic()<deadline:
   hb();m=receive(.1)
   if m and m.get_type()=='EKF_STATUS_REPORT' and m.flags&16:break
  else:raise RuntimeError('EKF position gate timeout')
  connection.set_mode('GUIDED');time.sleep(1);command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
  deadline=time.monotonic()+30;retry_at=time.monotonic()+3
  while time.monotonic()<deadline:
   hb();m=receive(.1)
   if time.monotonic()>=retry_at:
    connection.set_mode('GUIDED')
    command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
    retry_at=time.monotonic()+3
   if m and m.get_type()=='HEARTBEAT' and m.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:break
  else:raise RuntimeError('arm gate failed')
  command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,5)
  deadline=time.monotonic()+60;stable=None
  while time.monotonic()<deadline:
   hb();receive(.1)
   with lock:sample=dict(latest)
   p=sample.get('position_enu',[999,999,999]);good=time.monotonic()-sample.get('wall_monotonic_s',0)<1 and abs(p[2]-5)<.25 and np.linalg.norm(p[:2])<.25
   stable=(stable or time.monotonic()) if good else None
   if stable and time.monotonic()-stable>3:break
  else:raise RuntimeError('hover gate failed')
  results=[]
  for speed in args.speeds:
   for repeat in range(args.repeats):
    start_wall=time.monotonic();last_hb=0;last_send=0;steady_since=None;last_x=None;last_v=None;start_pos=None;brake_wall=None;stop_since=None
    # Use odometry velocity from finite differences only for the online gate;
    # offline analysis uses a uniformly resampled ground-truth trajectory.
    while True:
     now=time.monotonic()
     if now-last_hb>.8:hb();last_hb=now
     if now-last_send>.09:send(speed if brake_wall is None else 0,'cruise' if brake_wall is None else 'brake');last_send=now
     receive(.025)
     with lock:sample=dict(latest)
     if not sample or now-sample.get('wall_monotonic_s',0)>.5:continue
     x,y,z=sample['position_enu'];stamp=sample['wall_monotonic_s']
     vx=(x-last_x[0])/(stamp-last_x[1]) if last_x and stamp>last_x[1]+.02 else last_v
     if vx is not None and np.isfinite(vx):last_v=vx
     last_x=(x,stamp)
     if start_pos is None:start_pos=x
     if x>3800 or not 3.5<z<6.5 or abs(y)>10:raise RuntimeError('flight envelope exceeded')
     if now-start_wall>45:raise RuntimeError('brake trial timeout')
     if brake_wall is None:
      if vx is not None and abs(vx-speed)<.3 and x-start_pos>6:
       steady_since=steady_since or now
      else:steady_since=None
      if steady_since and now-steady_since>1:
       brake_wall,brake_sample=send(0,'brake_onset');last_send=brake_wall
     else:
      if vx is not None and abs(vx)<.25:stop_since=stop_since or now
      else:stop_since=None
      if stop_since and now-stop_since>.5:
       row={'speed_target_m_s':speed,'repeat':repeat,'brake_send_wall_s':brake_wall,'brake_send_sim_latest_s':brake_sample.get('sim_time_s'),'brake_start_position_enu':brake_sample.get('position_enu'),'stop_wall_s':now,'stop_sim_latest_s':sample.get('sim_time_s'),'stop_position_enu':sample.get('position_enu')}
       results.append(row);(out/'trials.json').write_text(json.dumps(results,indent=2));print(json.dumps(row),flush=True);break
   # All repetitions at this speed use the same live vehicle; no reset to start.
 finally:
  if connection:
   try:
    connection.set_mode('LAND');deadline=time.monotonic()+35
    while time.monotonic()<deadline:
     hb();m=receive(.2)
     if m and m.get_type()=='HEARTBEAT' and not m.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:break
   finally:connection.close()
  for proc in reversed(owned):
   if proc.poll() is None:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
  node.unsubscribe('/iris/odometry')
  if odom_file:odom_file.close()
  if events_file:events_file.close()
  for f in streams:f.close()
if __name__=='__main__':main()

#!/usr/bin/env python3
"""One RGB camera + selectable pose source -> temporal occupied voxel cloud.

No model, range sensor or SDF input. Requires texture and a bounded initial
motion. Occupied-only memory is not a certificate of unknown-space coverage.
"""
import argparse,json,signal,sys,threading,time
from collections import deque
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.monocular_depth_gz import decode_image,stamp,cloud_message
from mppi_ardupilot.monocular_point_tracker import MonocularPointTracker
from mppi_ardupilot.monocular_ground_odometry import GroundVisualOdometry
from mppi_ardupilot.camera_local_map import CameraLocalMap
from mppi_ardupilot.lidar_preprocess import quat_to_rot

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--frames',type=int,default=100000)
    parser.add_argument('--timeout',type=float,default=300)
    parser.add_argument('--max-hz',type=float,default=10.)
    parser.add_argument('--map-lifetime',type=float,default=60.)
    parser.add_argument('--map-fusion-radius',type=float,default=0.)
    parser.add_argument('--map-publish-age',type=float,default=None)
    parser.add_argument('--save-every',type=int,default=50)
    parser.add_argument('--pose-source',choices=('gazebo','visual-ground','openvins'),default='gazebo')
    parser.add_argument('--initial-height-m',type=float,default=3.)
    parser.add_argument('--min-tracker-baseline-m',type=float,default=.18)
    parser.add_argument('--marker-size-m',type=float,default=None,
                        help='declared side length of visible cyan ground squares')
    args=parser.parse_args()
    if not .05<=args.min_tracker_baseline_m<=1.:
        parser.error('--min-tracker-baseline-m must be in [0.05,1]')
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    args.output_dir.mkdir(parents=True,exist_ok=True)
    node=Node();publisher=node.advertise('/perception/obstacles_camera',PointCloudPacked)
    pose_publisher=(node.advertise('/perception/visual_odometry',Odometry)
                    if args.pose_source=='visual-ground' else None)
    lock=threading.Lock();pending=[None];deferred=[None];poses=deque(maxlen=400);imus=deque(maxlen=400);sim=[None]
    pose_received={}
    def rgb_cb(msg):
        with lock:pending[0]=(msg,time.monotonic())
    def odom_cb(msg):
        with lock:
            poses.append(msg)
            if args.pose_source=='openvins':
                pose_received[stamp(msg)]=time.monotonic()
                if len(pose_received)>400:pose_received.pop(next(iter(pose_received)))
    def imu_cb(msg):
        with lock:imus.append(msg)
    def clock_cb(msg):
        with lock:sim[0]=msg.sim.sec+msg.sim.nsec*1e-9
    node.subscribe(GzImage,'/sensor_suite/rgb',rgb_cb)
    if args.pose_source=='gazebo':node.subscribe(Odometry,'/iris/odometry',odom_cb)
    elif args.pose_source=='openvins':node.subscribe(Odometry,'/perception/visual_odometry',odom_cb)
    else:node.subscribe(IMU,'/sensor_suite/imu',imu_cb)
    node.subscribe(Clock,'/clock',clock_cb)
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    grid=CameraLocalMap(lifetime=args.map_lifetime,min_hits=2,altitude=3.,half_band=.65,
                        fusion_radius=args.map_fusion_radius,publish_age=args.map_publish_age)
    tracker=None;visual=None;start=time.monotonic();last=-float('inf');records=[]
    last_cloud=None
    with (args.output_dir/'frames.jsonl').open('w') as log:
        while not stop.is_set() and len(records)<args.frames and time.monotonic()-start<args.timeout:
            if time.monotonic()-last<1/args.max_hz:time.sleep(.005);continue
            with lock:
                if args.pose_source=='openvins' and deferred[0] is not None:
                    item=deferred[0]
                else:
                    item,pending[0]=pending[0],None
                if item is None:pose=None
                elif args.pose_source in ('gazebo','openvins'):
                    pose=min(poses,key=lambda p:abs(stamp(p)-stamp(item[0]))) if poses else None
                else:
                    pose=min(imus,key=lambda p:abs(stamp(p)-stamp(item[0]))) if imus else None
                now=sim[0]
                pose_received_mono_s=pose_received.get(stamp(pose)) if pose is not None and args.pose_source=='openvins' else None
            if item is None:time.sleep(.005);continue
            msg,received=item;acquired=stamp(msg)
            pose_delta=abs(stamp(pose)-acquired) if pose else None
            if args.pose_source=='openvins' and (pose_delta is None or pose_delta>.025):
                if time.monotonic()-received<.3:
                    deferred[0]=item
                    time.sleep(.005)
                    continue
            deferred[0]=None
            last=time.monotonic();rgb=decode_image(msg)
            age=now-acquired if now is not None else None
            fresh=pose is not None and pose_delta<=.06 and age is not None and 0<=age<=1.
            record=dict(frame=len(records),source_stamp_s=acquired,pose_time_difference_s=pose_delta,simulation_age_s=age,frame_id='odom',backend='temporal-rgb-triangulation',pose_source=args.pose_source,
                        image_received_mono_s=received,pose_received_mono_s=pose_received_mono_s)
            points=np.empty((0,3),dtype='<f4')
            if fresh:
                q=pose.pose.orientation if args.pose_source in ('gazebo','openvins') else pose.orientation
                rotation=quat_to_rot(q.x,q.y,q.z,q.w)
                if args.pose_source=='visual-ground':
                    if visual is None:
                        f=msg.width/(2*np.tan(1.3962634/2))
                        k=[[f,0,(msg.width-1)/2],[0,f,(msg.height-1)/2],[0,0,1]]
                        visual=GroundVisualOdometry(k,height_m=args.initial_height_m,
                                                    marker_size_m=args.marker_size_m)
                    estimated,visual_diag=visual.observe(rgb,rotation,acquired)
                    record.update(visual_diag)
                    fresh=estimated is not None
                    if fresh:
                        body_position=estimated
                        odom=Odometry();odom.header.CopyFrom(msg.header)
                        odom.pose.position.x,odom.pose.position.y,odom.pose.position.z=map(float,estimated)
                        odom.pose.orientation.CopyFrom(q)
                        pose_publisher.publish(odom)
                else:
                    p=pose.pose.position
                    body_position=np.array([p.x,p.y,p.z])
                if fresh and abs(body_position[2]-args.initial_height_m)>.4:
                    tracker=None;grid.voxels.clear();last_cloud=None;fresh=False
                if fresh and tracker is None:
                    f=msg.width/(2*np.tan(1.3962634/2))
                    tracker=MonocularPointTracker([[f,0,(msg.width-1)/2],[0,f,(msg.height-1)/2],[0,0,1]],
                                                  min_baseline=args.min_tracker_baseline_m)
                if fresh:
                    world,diagnostic=tracker.observe(rgb,body_position,rotation)
                    record['triangulated_in_altitude_band']=int(np.count_nonzero(
                        np.isfinite(world).all(axis=1) &
                        (abs(world[:,2]-grid.altitude)<=grid.half_band)))
                    points=grid.update_world(world,acquired);record.update(diagnostic)
                    record['map_voxels_seen_once']=sum(hits==1 for _,hits in grid.voxels.values())
                    record['map_voxels_confirmed']=sum(hits>=grid.min_hits for _,hits in grid.voxels.values())
                    record['position_enu']=body_position.tolist();record['quaternion_wxyz']=[q.w,q.x,q.y,q.z]
            if fresh and len(points)>0 and record.get('triangulated_in_altitude_band',0)>0:
                last_cloud=(points.copy(),msg.header,acquired)
            # A recent cloud can be retransmitted for a late subscriber, but
            # retain its original image stamp so the planner can reject age.
            if (fresh and last_cloud is not None and
                    acquired-last_cloud[2]<=min(grid.publish_age,1.)):
                record['cloud_source_stamp_s']=last_cloud[2]
                record['published']=bool(publisher.publish(
                    cloud_message(last_cloud[0],last_cloud[1],'odom')))
            else:
                record['published']=False
            done_mono_s=time.monotonic()
            record['perception_done_mono_s']=done_mono_s
            record['cloud_published_mono_s']=done_mono_s if record['published'] else None
            record['points']=len(last_cloud[0]) if record['published'] else len(points)
            record['receive_to_publish_ms']=(done_mono_s-received)*1000
            if args.save_every and len(records)%args.save_every==0:
                Image.fromarray(rgb).save(args.output_dir/f'rgb_{len(records):06d}.png')
                np.save(args.output_dir/f'cloud_{len(records):06d}.npy',points)
            records.append(record);log.write(json.dumps(record)+'\n');log.flush()
            if len(records)%10==0:print(json.dumps(record),flush=True)
    summary=dict(backend='temporal-rgb-triangulation',pose_source=args.pose_source,frames=len(records),published_frames=sum(r['published'] for r in records),receive_to_publish_ms={f'p{p}':float(np.percentile([r['receive_to_publish_ms'] for r in records],p)) for p in [50,95,99]} if records else {},coverage=('Observed occupied voxels only. Unknown space not certified free. '
        + ('IMU attitude and declared ground-marker size/height prior provide metric scale.' if args.pose_source=='visual-ground' else
           'OpenVINS camera + raw IMU provide metric relative motion; declared hover height anchors the origin.' if args.pose_source=='openvins' else
           'Gazebo odometry provides metric scale.')))
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)

if __name__=='__main__':main()

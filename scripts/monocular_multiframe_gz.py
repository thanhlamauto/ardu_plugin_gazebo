#!/usr/bin/env python3
"""Evaluate persistent monocular landmark depth from RGB and VIO, without publishing a map."""
import argparse
import json
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mppi_ardupilot.lidar_preprocess import quat_to_rot
from mppi_ardupilot.monocular_multiframe_depth import MultiFrameLandmarkTracker, camera_observation
from scripts.monocular_depth_gz import decode_image, stamp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=120.)
    args=parser.parse_args()
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.clock_pb2 import Clock
    args.output_dir.mkdir(parents=True,exist_ok=True)
    node=Node();lock=threading.Lock();pending=[None];deferred=[None]
    poses=deque(maxlen=300);sim=[None]
    def image_cb(msg):
        with lock:pending[0]=(msg,time.monotonic())
    def pose_cb(msg):
        with lock:poses.append(msg)
    def clock_cb(msg):
        with lock:sim[0]=msg.sim.sec+msg.sim.nsec*1e-9
    node.subscribe(Image,'/sensor_suite/rgb',image_cb)
    node.subscribe(Odometry,'/perception/visual_odometry',pose_cb)
    node.subscribe(Clock,'/clock',clock_cb)
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    tracker=None;frames=0;good_frames=0;start=time.monotonic()
    with (args.output_dir/'frames.jsonl').open('w') as log:
        while not stop.is_set() and time.monotonic()-start<args.timeout:
            with lock:
                if deferred[0] is not None:item=deferred[0]
                else:item,pending[0]=pending[0],None
                pose=(min(poses,key=lambda p:abs(stamp(p)-stamp(item[0])))
                      if item is not None and poses else None)
                sim_now=sim[0]
            if item is None:
                time.sleep(.005);continue
            msg,received=item;acquired=stamp(msg)
            pose_delta=abs(stamp(pose)-acquired) if pose is not None else None
            if (pose_delta is None or pose_delta>.025) and time.monotonic()-received<.3:
                deferred[0]=item;time.sleep(.005);continue
            deferred[0]=None
            age=sim_now-acquired if sim_now is not None else None
            row=dict(frame=frames,stamp_s=acquired,pose_delta_s=pose_delta,
                     image_age_s=age,processed=False,active_tracks=0,
                     geometry_observations_added=0,confident_landmarks=0,
                     landmarks=[])
            if pose is not None and pose_delta<=.025 and age is not None and 0<=age<=1.:
                if tracker is None:
                    f=msg.width/(2*np.tan(1.3962634/2))
                    tracker=MultiFrameLandmarkTracker(
                        [[f,0,(msg.width-1)/2],[0,f,(msg.height-1)/2],[0,0,1]])
                q=pose.pose.orientation
                rotation=quat_to_rot(q.x,q.y,q.z,q.w)
                p=pose.pose.position
                landmarks,diag=tracker.observe(decode_image(msg),[p.x,p.y,p.z],rotation)
                row.update(diag)
                row['processed']=True
                if landmarks:good_frames+=1
                if diag['geometry_observations_added']:
                    row['landmarks']=[]
                    for e in landmarks:
                        pixel=tracker.tracks[e.feature_id].pixel
                        ray=camera_observation(pixel,[p.x,p.y,p.z],rotation,tracker.k)
                        row['landmarks'].append(dict(
                            id=e.feature_id,pixel=pixel.tolist(),
                            camera_center_enu=ray.center.tolist(),ray_enu=ray.ray.tolist(),
                            point_enu=e.point_enu.tolist(),
                            inverse_depth_1_m=e.inverse_depth_1_m,
                            inverse_depth_sigma_1_m=e.inverse_depth_sigma_1_m,
                            depth_sigma_m=e.depth_sigma_m,
                            observations=e.observations,baseline_m=e.baseline_m,
                            reprojection_rmse_px=e.reprojection_rmse_px))
            row['processing_ms']=(time.monotonic()-received)*1000
            log.write(json.dumps(row)+'\n');log.flush();frames+=1
            if frames%30==0:print(json.dumps({k:v for k,v in row.items() if k!='landmarks'}),flush=True)
    summary=dict(frames=frames,frames_with_confident_landmarks=good_frames,
                 backend='persistent-multiframe-inverse-depth',
                 published_to_planner=False)
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()

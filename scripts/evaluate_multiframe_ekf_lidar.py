#!/usr/bin/env python3
"""Replay monocular multi-frame geometry with ArduPilot EKF pose and LiDAR scoring."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mppi_ardupilot.monocular_multiframe_depth import MultiFrameLandmarkTracker,camera_observation
from mppi_ardupilot.monocular_triangulation import OPTICAL_TO_FLU
from mppi_ardupilot.lidar_preprocess import quat_to_rot
from scripts.evaluate_monocular_lidar_depth import error_metrics,percentiles,project_lidar_to_image


NED_TO_ENU=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
FLU_TO_FRD=np.diag([1.,-1.,-1.])


def ekf_rotation_enu(roll,pitch,yaw):
    cr,sr=np.cos(roll),np.sin(roll)
    cp,sp=np.cos(pitch),np.sin(pitch)
    cy,sy=np.cos(yaw),np.sin(yaw)
    frd_to_ned=np.array([
        [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
        [-sp,cp*sr,cp*cr],
    ])
    return NED_TO_ENU@frd_to_ned@FLU_TO_FRD


def stable_lidar_match(tree,depth,pixel,*,max_pixel_distance=6.,neighborhood=12.,max_depth_spread=.75):
    """Reject sparse-scan associations near depth discontinuities."""
    distances,indices=tree.query(pixel,k=8,distance_upper_bound=neighborhood)
    valid=np.isfinite(distances)&(indices<len(depth))
    if np.count_nonzero(valid)<3 or float(np.min(distances[valid]))>max_pixel_distance:
        return None
    ranges=depth[indices[valid]]
    if float(np.max(ranges)-np.min(ranges))>max_depth_spread:
        return None
    return float(np.median(ranges)),float(np.min(distances[valid]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    parser.add_argument('--pixel-match-radius',type=float,default=6.)
    args=parser.parse_args()
    trial=args.trial
    source=Path(__file__).read_bytes()
    snapshot=trial/'analysis_snapshot/evaluate_multiframe_ekf_lidar.py'
    snapshot.parent.mkdir(parents=True,exist_ok=True)
    snapshot.write_bytes(source)
    telemetry=[json.loads(line) for line in (trial/'ardupilot_ekf_telemetry.jsonl').open()]
    local=[item for item in telemetry if item['kind']=='LOCAL_POSITION_NED']
    attitude=[item for item in telemetry if item['kind']=='ATTITUDE']
    if len(local)<3 or len(attitude)<3:
        raise RuntimeError('not enough ArduPilot EKF position/attitude samples')
    local_t=np.array([item['data']['time_boot_ms']*.001 for item in local])
    local_ned=np.array([[item['data'][axis] for axis in ('x','y','z')] for item in local])
    att_t=np.array([item['data']['time_boot_ms']*.001 for item in attitude])
    att_rpy=np.array([[item['data'][axis] for axis in ('roll','pitch','yaw')] for item in attitude])
    att_rpy[:,2]=np.unwrap(att_rpy[:,2])
    scans=[json.loads(line) for line in (trial/'lidar_gt/scans.jsonl').open()]
    scan_t=np.array([item['stamp_s'] for item in scans])
    inference=[json.loads(line) for line in (trial/'perception/frames.jsonl').open()]
    result=json.loads((trial/'result.json').read_text())
    phases=result['benchmark_phases']
    start=phases['hover_before']['start_sim_s']
    end=phases['hover_after']['end_sim_s']
    tracker=None
    frame_rows=[]
    latest_matches={}
    all_processing=[]
    for row in inference:
        stamp=row['source_stamp_s']
        if stamp<start or stamp>end or stamp<max(local_t[0],att_t[0]) or stamp>min(local_t[-1],att_t[-1]):
            continue
        nearest_local=np.min(abs(local_t-stamp))
        nearest_att=np.min(abs(att_t-stamp))
        if nearest_local>.06 or nearest_att>.06:
            continue
        rgb_file=trial/'perception'/f"rgb_{row['frame']:06d}.png"
        if not rgb_file.exists():continue
        rgb=np.asarray(Image.open(rgb_file).convert('RGB'))
        height,width=rgb.shape[:2]
        if tracker is None:
            focal=width/(2*np.tan(1.3962634/2))
            tracker=MultiFrameLandmarkTracker([[focal,0,(width-1)/2],
                                               [0,focal,(height-1)/2],[0,0,1]])
        ned=np.array([np.interp(stamp,local_t,local_ned[:,i]) for i in range(3)])
        position=NED_TO_ENU@ned
        rpy=[np.interp(stamp,att_t,att_rpy[:,i]) for i in range(3)]
        rotation=ekf_rotation_enu(*rpy)
        begun=time.monotonic()
        landmarks,diagnostics=tracker.observe(rgb,position,rotation)
        processing_ms=(time.monotonic()-begun)*1000
        all_processing.append(processing_ms)
        matched=0
        if len(scan_t) and len(landmarks):
            index=int(np.argmin(abs(scan_t-stamp)))
            if abs(scan_t[index]-stamp)<=.06:
                with np.load(trial/'lidar_gt'/scans[index]['file']) as data:
                    uv,depth=project_lidar_to_image(data['xyz'],width,height)
                if len(depth):
                    tree=cKDTree(uv)
                    pixels=np.array([tracker.tracks[e.feature_id].pixel for e in landmarks])
                    for e,pixel in zip(landmarks,pixels):
                        association=stable_lidar_match(
                            tree,depth,pixel,max_pixel_distance=args.pixel_match_radius)
                        if association is None:continue
                        gt_depth,pixel_distance=association
                        camera=camera_observation(pixel,position,rotation,tracker.k)
                        forward=rotation@OPTICAL_TO_FLU[:,2]
                        estimate=float(np.dot(e.point_enu-camera.center,forward))
                        if estimate<=0:continue
                        matched+=1
                        latest_matches[e.feature_id]=dict(
                            feature_id=e.feature_id,stamp_s=stamp,
                            estimated_depth_m=estimate,lidar_depth_m=gt_depth,
                            pixel_distance_px=float(pixel_distance),
                            estimated_sigma_m=e.depth_sigma_m,baseline_m=e.baseline_m)
        frame_rows.append(dict(frame=row['frame'],stamp_s=stamp,
                               pose_match_error_ms=max(nearest_local,nearest_att)*1000,
                               processing_ms=processing_ms,matched_lidar_landmarks=matched,
                               **diagnostics))
    values=list(latest_matches.values())
    pred=np.array([v['estimated_depth_m'] for v in values])
    gt=np.array([v['lidar_depth_m'] for v in values])
    sigma=np.array([v['estimated_sigma_m'] for v in values])
    target=(gt>=3)&(gt<10)
    truth=[json.loads(line) for line in (trial/'ground_truth.jsonl').open()]
    truth_t=np.array([item['sim_s'] for item in truth])
    truth_p=np.array([item['position_enu'] for item in truth])
    truth_q=np.array([item['attitude_quaternion_wxyz'] for item in truth])
    position_error=[]
    orientation_error=[]
    for row in frame_rows:
        stamp=row['stamp_s']
        if not truth_t[0]<=stamp<=truth_t[-1]:continue
        ned=np.array([np.interp(stamp,local_t,local_ned[:,i]) for i in range(3)])
        ekf_p=NED_TO_ENU@ned
        gt_p=np.array([np.interp(stamp,truth_t,truth_p[:,i]) for i in range(3)])
        position_error.append(float(np.linalg.norm(ekf_p-gt_p)))
        rpy=[np.interp(stamp,att_t,att_rpy[:,i]) for i in range(3)]
        ekf_r=ekf_rotation_enu(*rpy)
        q=truth_q[int(np.argmin(abs(truth_t-stamp)))]
        gt_r=quat_to_rot(q[1],q[2],q[3],q[0])
        orientation_error.append(float(np.degrees(np.arccos(
            np.clip((np.trace(gt_r.T@ekf_r)-1)/2,-1,1)))))
    first_depth=next((r['stamp_s'] for r in frame_rows if r['confident_landmarks']>0),None)
    move_start_x=phases['move_forward']['start_truth_enu'][0]
    motion_before_depth=(float(np.interp(first_depth,truth_t,truth_p[:,0])-move_start_x)
                         if first_depth is not None else None)
    report=dict(status=result['status'],source='RGB multi-frame landmarks + ArduPilot EKF pose',
                lidar_role='offline ground truth only',
                evaluator_sha256=hashlib.sha256(source).hexdigest(),
                replayed_frames=len(frame_rows),frames_with_depth=sum(r['confident_landmarks']>0 for r in frame_rows),
                frames_with_lidar_matched_landmarks=sum(r['matched_lidar_landmarks']>0 for r in frame_rows),
                unique_lidar_matched_landmarks=len(values),
                first_depth_sim_s=first_depth,
                forward_motion_before_first_depth_m=motion_before_depth,
                processing_ms=percentiles(all_processing),
                pose_match_error_ms=percentiles([r['pose_match_error_ms'] for r in frame_rows]),
                ekf_position_error_against_truth_m=percentiles(position_error),
                ekf_orientation_error_against_truth_deg=percentiles(orientation_error),
                depth_error=error_metrics(pred,gt),
                target_3_to_10m_error=error_metrics(pred[target],gt[target]),
                fraction_absolute_error_within_reported_sigma=(
                    float(np.mean(abs(pred-gt)<=sigma)) if len(values) else None),
                lidar_association='8 nearest projected returns within 12 px, nearest <=6 px, '
                                  'at least 3 returns, axial range spread <=0.75 m',
                note='EKF uses MAVLink time_boot_ms; Gazebo truth is not passed to the tracker. '
                     'Sparse LiDAR returns near an image feature remain an approximate depth reference.')
    (trial/'multiframe_ekf_lidar_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    (trial/'multiframe_ekf_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frame_rows))
    (trial/'multiframe_ekf_landmarks.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in values))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

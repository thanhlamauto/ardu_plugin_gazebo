#!/usr/bin/env python3
"""Offline box-depth evaluation for an isolated multi-frame VIO/RGB trial.

Gazebo truth and box geometry are read here, after the trial, never by the
landmark tracker. A pixel is associated with the box using its truth-pose ray,
so selection does not depend on the depth estimate being evaluated.
"""
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mppi_ardupilot.lidar_preprocess import quat_to_rot
from mppi_ardupilot.monocular_multiframe_depth import camera_observation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    trial = args.trial
    source_bytes = Path(__file__).read_bytes()
    analysis_copy = trial/'analysis_snapshot/analyze_monocular_multiframe_depth.py'
    analysis_copy.parent.mkdir(parents=True,exist_ok=True)
    analysis_copy.write_bytes(source_bytes)
    result = json.loads((trial/'result.json').read_text())
    frames = [json.loads(line) for line in (trial/'multiframe/frames.jsonl').open()]
    truth = [json.loads(line) for line in (trial/'ground_truth.jsonl').open()]
    sdf = ET.parse(trial/'source_snapshot/models/iris_with_monocular_camera/model.sdf')
    camera = sdf.find('.//sensor[@type="camera"]/camera')
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    focal = width/(2*np.tan(float(camera.findtext('horizontal_fov'))/2))
    k = np.array([[focal,0,(width-1)/2],[0,focal,(height-1)/2],[0,0,1]])
    boxes = json.loads((trial/'manifest.json').read_text())['eval_boxes']
    if len(boxes) != 1:
        raise ValueError('this evaluator expects one box')
    box = boxes[0]
    center = np.asarray(box['center'],float)
    half = np.asarray(box['size'],float)/2
    if box.get('yaw',0) != 0:
        raise ValueError('this evaluator expects the axis-aligned pilot box')
    front_x = center[0]-half[0]
    times = np.array([r['sim_s'] for r in truth])
    positions = np.array([r['position_enu'] for r in truth])
    quaternions = np.array([r['attitude_quaternion_wxyz'] for r in truth])
    selected = {}
    counters = dict(total_logged_landmarks=0,truth_rays_hitting_box=0)
    for frame in frames:
        stamp = frame['stamp_s']
        if not frame['landmarks'] or stamp<times[0] or stamp>times[-1]:
            continue
        index = int(np.argmin(np.abs(times-stamp)))
        q = quaternions[index]
        rotation = quat_to_rot(q[1],q[2],q[3],q[0])
        position = np.array([np.interp(stamp,times,positions[:,axis]) for axis in range(3)])
        for item in frame['landmarks']:
            counters['total_logged_landmarks'] += 1
            ray = camera_observation(item['pixel'],position,rotation,k)
            if ray.ray[0] <= 0:
                continue
            distance = (front_x-ray.center[0])/ray.ray[0]
            if distance <= 0:
                continue
            hit = ray.center+distance*ray.ray
            if (abs(hit[1]-center[1])>half[1]-.1 or
                    abs(hit[2]-center[2])>half[2]-.1):
                continue
            if ray.ray[2] < 0 and -ray.center[2]/ray.ray[2] < distance:
                continue
            counters['truth_rays_hitting_box'] += 1
            selected[item['id']] = dict(item=item,hit=hit.tolist(),ray=ray.ray.tolist(),stamp_s=stamp)
    errors = np.array([np.linalg.norm(np.asarray(value['item']['point_enu'])-value['hit'])
                       for value in selected.values()])
    x_errors = np.array([abs(value['item']['point_enu'][0]-front_x)
                         for value in selected.values()])
    depth_errors = np.array([abs(np.dot(np.asarray(value['item']['point_enu'])-value['hit'],
                                         value['ray'])) for value in selected.values()])
    sigmas = np.array([value['item']['depth_sigma_m'] for value in selected.values()])
    baselines = np.array([value['item']['baseline_m'] for value in selected.values()])
    def stats(values):
        return dict(median=float(np.median(values)),p90=float(np.percentile(values,90)),
                    p95=float(np.percentile(values,95))) if len(values) else None
    phases = result.get('vio_motion_stop_phases',{})
    by_phase = {}
    for name,phase in phases.items():
        subset = [f for f in frames if phase['start_sim_s']<=f['stamp_s']<=phase['end_sim_s']]
        by_phase[name] = dict(frames=len(subset),frames_with_depth=sum(f['confident_landmarks']>0 for f in subset),
                              max_confident_landmarks=max((f['confident_landmarks'] for f in subset),default=0),
                              geometry_observations_added=sum(f['geometry_observations_added'] for f in subset))
    processed = [f for f in frames if f['processed']]
    first_depth = next((f for f in frames if f['confident_landmarks']>0),None)
    first_depth_motion_m = None
    if first_depth and 'move_lateral' in phases:
        start_y = phases['move_lateral']['start_position_enu'][1]
        first_depth_motion_m = float(np.interp(first_depth['stamp_s'],times,positions[:,1])-start_y)
    report = dict(trial_status=result['status'],frames=len(frames),
                  evaluator_sha256=hashlib.sha256(source_bytes).hexdigest(),
                  processed_frames=sum(f['processed'] for f in frames),
                  frames_with_depth=sum(f['confident_landmarks']>0 for f in frames),
                  unique_box_landmarks=len(selected),box_front_x_m=front_x,
                  point_error_m=stats(errors),front_face_x_error_m=stats(x_errors),
                  along_ray_error_m=stats(depth_errors),estimated_depth_sigma_m=stats(sigmas),
                  estimated_baseline_m=stats(baselines),
                  first_depth_sim_s=first_depth['stamp_s'] if first_depth else None,
                  lateral_motion_before_first_depth_m=first_depth_motion_m,
                  processed_image_age_ms=stats(np.array([f['image_age_s']*1000 for f in processed])),
                  processing_from_callback_ms=stats(np.array([f['processing_ms'] for f in processed])),
                  fraction_along_ray_error_within_reported_sigma=(float(np.mean(depth_errors<=sigmas))
                                                                   if len(sigmas) else None),
                  counters=counters,phases=by_phase,
                  note='Evaluator-only truth ray association; one latest estimate per feature ID. '
                       'Inverse-depth uncertainty is a pose-conditioned heuristic, not calibrated risk.')
    (trial/'multiframe_depth_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()

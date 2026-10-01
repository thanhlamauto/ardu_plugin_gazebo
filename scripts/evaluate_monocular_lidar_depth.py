#!/usr/bin/env python3
"""Offline evaluation of RGB monocular depth at co-located LiDAR returns."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def project_lidar_to_image(points, width, height, horizontal_fov=1.3962634):
    """Gazebo forward-X / left-Y / up-Z sensor frame to camera pixel + axial range."""
    xyz = np.asarray(points,float).reshape(-1,3)
    focal = width/(2*np.tan(horizontal_fov/2))
    positive = np.isfinite(xyz).all(axis=1) & (xyz[:,0]>.3) & (xyz[:,0]<25.)
    xyz = xyz[positive]
    uv = np.column_stack(((width-1)/2-focal*xyz[:,1]/xyz[:,0],
                          (height-1)/2-focal*xyz[:,2]/xyz[:,0]))
    visible = ((uv[:,0]>=0)&(uv[:,0]<=width-1)&
               (uv[:,1]>=0)&(uv[:,1]<=height-1))
    return uv[visible],xyz[visible,0]


def sample_bilinear(image, uv):
    height,width = image.shape
    u,v = uv[:,0],uv[:,1]
    x0,y0 = np.floor(u).astype(int),np.floor(v).astype(int)
    x1,y1 = np.minimum(x0+1,width-1),np.minimum(y0+1,height-1)
    wx,wy = u-x0,v-y0
    return ((1-wx)*(1-wy)*image[y0,x0]+wx*(1-wy)*image[y0,x1]+
            (1-wx)*wy*image[y1,x0]+wx*wy*image[y1,x1])


def error_metrics(predicted,reference):
    if len(predicted)==0:
        return None
    delta = predicted-reference
    return dict(samples=int(len(delta)),mae_m=float(np.mean(abs(delta))),
                median_abs_error_m=float(np.median(abs(delta))),
                rmse_m=float(np.sqrt(np.mean(delta**2))),
                bias_m=float(np.mean(delta)),
                abs_rel=float(np.mean(abs(delta)/reference)),
                delta1=float(np.mean(np.maximum(predicted/reference,reference/predicted)<1.25)),
                within_0_5_m=float(np.mean(abs(delta)<=.5)),
                within_1_m=float(np.mean(abs(delta)<=1.)))


def percentiles(values):
    return ({f'p{p}':float(np.percentile(values,p)) for p in (50,95,99)}
            if len(values) else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    args = parser.parse_args()
    trial = args.trial
    source = Path(__file__).read_bytes()
    snapshot = trial/'analysis_snapshot/evaluate_monocular_lidar_depth.py'
    snapshot.parent.mkdir(parents=True,exist_ok=True)
    snapshot.write_bytes(source)
    result = json.loads((trial/'result.json').read_text())
    predictions = [json.loads(line) for line in (trial/'perception/frames.jsonl').open()]
    scans = [json.loads(line) for line in (trial/'lidar_gt/scans.jsonl').open()]
    stamps = np.array([scan['stamp_s'] for scan in scans])
    phases = result.get('benchmark_phases',{})
    start = phases.get('hover_before',{}).get('start_sim_s',-float('inf'))
    end = phases.get('hover_after',{}).get('end_sim_s',float('inf'))
    pairs = []
    center_pairs = []
    pred_all,gt_all,uv_all = [],[],[]
    for row in predictions:
        if not start<=row['source_stamp_s']<=end or len(stamps)==0:
            continue
        idx = int(np.argmin(abs(stamps-row['source_stamp_s'])))
        time_error = abs(stamps[idx]-row['source_stamp_s'])
        if time_error>.06:
            continue
        path = trial/'perception'/f"depth_{row['frame']:06d}.npy"
        if not path.exists():
            continue
        depth = np.load(path)
        if depth.ndim!=2:
            continue
        height,width = depth.shape
        with np.load(trial/'lidar_gt'/scans[idx]['file']) as data:
            xyz = data['xyz']
        uv,ground_truth = project_lidar_to_image(xyz,width,height)
        estimated = sample_bilinear(depth,uv)
        valid = np.isfinite(estimated)&(estimated>0)
        uv,ground_truth,estimated = uv[valid],ground_truth[valid],estimated[valid]
        if len(estimated)==0:
            continue
        center = ((abs(uv[:,0]-(width-1)/2)<=25)&
                  (abs(uv[:,1]-(height-1)/2)<=25))
        if np.count_nonzero(center)>=10:
            center_pairs.append(dict(frame=row['frame'],stamp_s=row['source_stamp_s'],
                                     lidar_depth_m=float(np.median(ground_truth[center])),
                                     monocular_depth_m=float(np.median(estimated[center])),
                                     returns=int(np.count_nonzero(center))))
        pred_all.append(estimated)
        gt_all.append(ground_truth)
        uv_all.append(uv)
        pairs.append(dict(frame=row['frame'],rgb_stamp_s=row['source_stamp_s'],
                          lidar_stamp_s=stamps[idx],match_error_ms=time_error*1000,
                          matched_points=len(estimated),inference_ms=row['inference_ms'],
                          receive_to_publish_ms=row['receive_to_publish_ms'],
                          simulation_age_ms=(row['simulation_age_s']*1000
                                             if row['simulation_age_s'] is not None else None),
                          dropped_rgb=row['dropped_rgb'],
                          mae_m=float(np.mean(abs(estimated-ground_truth)))))
    if pred_all:
        predicted,reference,uv = map(np.concatenate,(pred_all,gt_all,uv_all))
        width = int(np.load(trial/'perception'/f"depth_{pairs[0]['frame']:06d}.npy").shape[1])
        central = abs(uv[:,0]-(width-1)/2)<=width*.15
    else:
        predicted=reference=np.array([]);central=np.array([],bool)
    by_range = {}
    for lo,hi in ((.3,5.),(5.,10.),(10.,20.),(20.,25.)):
        mask=(reference>=lo)&(reference<hi)
        by_range[f'{lo:g}-{hi:g}m']=error_metrics(predicted[mask],reference[mask])
    def detection(threshold):
        if len(reference)==0:return None
        gt=reference[central]<=threshold
        pred=predicted[central]<=threshold
        tp=int(np.sum(gt&pred));fp=int(np.sum(~gt&pred));fn=int(np.sum(gt&~pred))
        return dict(threshold_m=threshold,samples=int(len(gt)),true_positive=tp,
                    false_positive=fp,false_negative=fn,
                    precision=tp/(tp+fp) if tp+fp else None,
                    recall=tp/(tp+fn) if tp+fn else None)
    patch_est=np.array([row['monocular_depth_m'] for row in center_pairs])
    patch_gt=np.array([row['lidar_depth_m'] for row in center_pairs])
    patch_by_range={}
    for lo,hi in ((3.,5.),(5.,7.),(7.,10.)):
        mask=(patch_gt>=lo)&(patch_gt<hi)
        patch_by_range[f'{lo:g}-{hi:g}m']=error_metrics(patch_est[mask],patch_gt[mask])
    report=dict(status=result['status'],predictor='monocular RGB only',
                lidar_role='offline ground truth only',
                evaluator_sha256=hashlib.sha256(source).hexdigest(),
                predicted_frames=len(predictions),lidar_scans=len(scans),matched_frames=len(pairs),
                matched_points=int(len(reference)),
                synchronized_stamp_error_ms=percentiles([p['match_error_ms'] for p in pairs]),
                inference_ms=percentiles([p['inference_ms'] for p in pairs]),
                receive_to_publish_ms=percentiles([p['receive_to_publish_ms'] for p in pairs]),
                image_age_at_output_simclock_ms=percentiles([
                    p['simulation_age_ms'] for p in pairs if p['simulation_age_ms'] is not None]),
                dropped_rgb_frames=sum(p['dropped_rgb'] for p in pairs),
                all_visible_returns=error_metrics(predicted,reference),
                central_returns=error_metrics(predicted[central],reference[central]),
                center_patch_frames=len(center_pairs),
                center_patch_frame_medians=error_metrics(patch_est,patch_gt),
                center_patch_by_range=patch_by_range,
                by_range=by_range,detection_5m=detection(5.),detection_10m=detection(10.),
                pose_source='ArduPilot EKF/MAVLink flight state; depth comparison is in the co-located sensor frame',
                note='Sparse LiDAR-return evaluation only; no claim about pixels without LiDAR returns or unknown-space free coverage.')
    (trial/'lidar_depth_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    (trial/'lidar_depth_pairs.jsonl').write_text(''.join(json.dumps(pair)+'\n' for pair in pairs))
    (trial/'center_patch_depth_pairs.jsonl').write_text(''.join(json.dumps(pair)+'\n' for pair in center_pairs))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()

#!/usr/bin/env python3
"""Score an isolated OpenVINS hover–translate–hover trial against evaluator truth."""
import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    trial = args.trial
    result = json.loads((trial/'result.json').read_text())
    phases = result['vio_motion_stop_phases']
    with (trial/'openvins.csv').open(newline='') as handle:
        rows = [r for r in csv.DictReader(handle) if r['initialized']=='1']
    truth = [json.loads(line) for line in (trial/'ground_truth.jsonl').open()]
    stamps = np.array([float(r['stamp']) for r in rows])
    vio_p = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
    vio_v = np.array([[float(r[k]) for k in ('vx','vy','vz')] for r in rows])
    gyro_bias = np.array([[float(r[k]) for k in ('bgx','bgy','bgz')] for r in rows])
    accel_bias = np.array([[float(r[k]) for k in ('bax','bay','baz')] for r in rows])
    gt_t = np.array([r['sim_s'] for r in truth])
    gt_p = np.array([r['position_enu'] for r in truth])
    gt_at_vio = np.column_stack([np.interp(stamps,gt_t,gt_p[:,i]) for i in range(3)])
    error = np.linalg.norm(vio_p-gt_at_vio,axis=1)
    horizontal_error=np.linalg.norm((vio_p-gt_at_vio)[:,:2],axis=1)
    gt_q=np.array([r['attitude_quaternion_wxyz'] for r in truth])
    nearest=np.clip(np.searchsorted(gt_t,stamps),1,len(gt_t)-1)
    nearest-=np.abs(gt_t[nearest-1]-stamps)<np.abs(gt_t[nearest]-stamps)
    vio_q=np.array([[float(r[k]) for k in ('odom_qw','odom_qx','odom_qy','odom_qz')]
                    for r in rows])
    dot=np.abs(np.sum(vio_q*gt_q[nearest],axis=1))
    orientation_error_deg=np.degrees(2*np.arccos(np.clip(dot,0,1)))
    def phase_metrics(name,settle=0.):
        phase = phases[name]
        mask = (stamps>=phase['start_sim_s']+settle)&(stamps<=phase['end_sim_s'])
        indices = np.flatnonzero(mask)
        if len(indices)<2:
            return {'samples':len(indices)}
        ids=indices
        speed=np.linalg.norm(vio_v[ids],axis=1)
        return dict(samples=len(ids),start_sim_s=float(stamps[ids[0]]),
                    end_sim_s=float(stamps[ids[-1]]),
                    vio_displacement_m=float(np.linalg.norm(vio_p[ids[-1]]-vio_p[ids[0]])),
                    truth_displacement_m=float(np.linalg.norm(gt_at_vio[ids[-1]]-gt_at_vio[ids[0]])),
                    position_error_start_m=float(error[ids[0]]),
                    position_error_end_m=float(error[ids[-1]]),
                    position_error_p95_m=float(np.percentile(error[ids],95)),
                    horizontal_error_p95_m=float(np.percentile(horizontal_error[ids],95)),
                    orientation_error_p95_deg=float(np.percentile(orientation_error_deg[ids],95)),
                    vio_speed_p95_m_s=float(np.percentile(speed,95)),
                    gyro_bias_norm_p95_rad_s=float(np.percentile(np.linalg.norm(gyro_bias[ids],axis=1),95)),
                    accel_bias_norm_p95_m_s2=float(np.percentile(np.linalg.norm(accel_bias[ids],axis=1),95)),
                    active_tracks_p50=float(np.median([int(rows[i]['active_tracks']) for i in ids])),
                    msckf_updates_total=int(sum(int(rows[i]['msckf_updates']) for i in ids)))
    before=phase_metrics('hover_before',1.)
    move=phase_metrics('move_lateral')
    after=phase_metrics('hover_after',1.)
    log=re.sub(r'\x1b\[[0-9;]*m','',(trial/'openvins.log').read_text())
    accepted_chi2=[(float(a),float(b)) for a,b in re.findall(
        r'\[ZUPT\]: accepted .*?chi2 ([\d.]+) < ([\d.]+)',log)]
    passed=(all(p.get('samples',0)>=10 for p in (before,move,after)) and
            before['vio_displacement_m']<.2 and
            after['vio_displacement_m']<.2 and
            after['vio_speed_p95_m_s']<.1 and
            abs(move['vio_displacement_m']-move['truth_displacement_m'])<.2)
    report=dict(status=result['status'],passed=bool(passed),criteria=dict(
        stationary_drift_max_m=.2,post_stop_vio_speed_p95_max_m_s=.1,
        translation_displacement_error_max_m=.2),
        hover_before=before,move_lateral=move,hover_after=after,
        zupt_accepted=log.count('[ZUPT]: accepted'),
        zupt_accepted_over_printed_chi2_threshold=sum(a>b for a,b in accepted_chi2),
        zupt_rejected=log.count('[ZUPT]: rejected'),
        zupt_disparity_passed=log.count('[ZUPT]: passed disparity'),
        zupt_disparity_failed=log.count('[ZUPT]: failed disparity'),
        note='Gazebo truth is used only for the motion-stop experiment controller and offline evaluation, never by OpenVINS.')
    (trial/'vio_motion_stop_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

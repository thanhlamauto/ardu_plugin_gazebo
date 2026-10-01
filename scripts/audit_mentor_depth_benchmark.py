#!/usr/bin/env python3
"""Audit isolation and EKF/GPS evidence for a monocular/LiDAR benchmark."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    args=parser.parse_args()
    trial=args.trial
    result=json.loads((trial/'result.json').read_text())
    manifest=json.loads((trial/'manifest.json').read_text())
    inference=json.loads((trial/'inference.command.json').read_text())
    lidar=json.loads((trial/'lidar_gt.command.json').read_text())
    analysis=json.loads((trial/'lidar_depth_analysis.json').read_text())
    phases=result.get('benchmark_phases',{})
    move=phases.get('move_forward',{})
    truth_dx=(move['end_truth_enu'][0]-move['start_truth_enu'][0]) if move else None
    ekf_dx=(move['end_ekf_ned']['east_m']-move['start_ekf_ned']['east_m']) if move else None
    checks=dict(
        completed=result['status']=='DEPTH_BENCHMARK_COMPLETE',
        rgb_predictor_only=('--gt-topic' not in inference and '--local-map' not in inference and
                            '--pose-source' not in inference),
        lidar_logger_separate=('log_lidar_gt_gz.py' in ' '.join(lidar)),
        lidar_evaluator_only=(manifest.get('lidar_role')=='offline ground truth only' and
                              result['gt_lidar_messages']>0 and result['lidar_topic_messages']==0 and
                              result['depth_topic_messages']==0),
        no_openvins_or_planner=(not (trial/'openvins.command.json').exists() and
                               not (trial/'planner.command.json').exists()),
        gps_fix_3d=result.get('gps_fix_type_max',0)>=3,
        ekf_motion_matches_truth=(truth_dx is not None and ekf_dx is not None and
                                  abs(truth_dx-ekf_dx)<.2),
        synchronized_pairs=(analysis['matched_frames']>=30 and
                            analysis['synchronized_stamp_error_ms'] is not None and
                            analysis['synchronized_stamp_error_ms']['p95']<=60),
    )
    report=dict(checks=checks,all_checks_pass=all(checks.values()),
                truth_forward_motion_m=truth_dx,ekf_forward_motion_m=ekf_dx,
                depth_accuracy_gate_pass=False,
                depth_accuracy_gate_note='No acceptance threshold was declared; report raw errors, not a safety pass.')
    (trial/'mentor_benchmark_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['all_checks_pass']:
        raise SystemExit(1)


if __name__=='__main__':main()

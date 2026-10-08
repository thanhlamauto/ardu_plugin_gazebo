#!/usr/bin/env python3
"""Check that monocular research trials route only visual pose to perception/MPPI."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np
import yaml


def read_jsonl(path):
    if not path.exists():
        path = path.with_suffix(path.suffix + '.gz')
    if not path.exists():
        return []
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def audit(run_dir, trial, perception_config):
    root = run_dir / trial['name']
    result = json.loads((root / 'result.json').read_text())
    manifest = json.loads((root / 'manifest.json').read_text())
    inference = json.loads((root / 'inference.command.json').read_text())
    planner_path = root / 'planner.command.json'
    planner = json.loads(planner_path.read_text()) if planner_path.exists() else []
    frames = read_jsonl(root / 'perception/frames.jsonl')
    truth = read_jsonl(root / 'ground_truth.jsonl')
    cfg = json.loads((root / 'source_snapshot' / perception_config).read_text())
    planner_config = yaml.safe_load((root / 'source_snapshot' / trial['config']).read_text())
    source = (root / 'source_snapshot/scripts/monocular_geometry_gz.py').read_text()
    checks = {
        'visual_mode_declared': cfg.get('pose_source') == 'visual-ground'
            and cfg.get('marker_size_m') == .42,
        'visual_mode_launched': '--pose-source' in inference
            and inference[inference.index('--pose-source') + 1] == 'visual-ground',
        'rgb_imu_not_sim_pose_subscription':
            "if args.pose_source=='gazebo':node.subscribe(Odometry,'/iris/odometry',odom_cb)" in source
            and "else:node.subscribe(IMU,'/sensor_suite/imu',imu_cb)" in source,
        'planner_visual_pose_config': planner_config.get('odom_topic') == '/perception/visual_odometry',
        'planner_uses_visual_pose_if_launched':
            (bool(planner) and '--odom-topic' in planner
             and planner[planner.index('--odom-topic') + 1] == '/perception/visual_odometry')
            or (not planner and result.get('status') == 'SETUP_FAILED'),
        'perception_frames_visual': bool(frames)
            and all(row.get('pose_source') == 'visual-ground' for row in frames),
        'ground_truth_not_inference_command': not any('/iris/odometry' in part
            or 'ground_truth' in part for part in inference),
        'ground_truth_not_planner_command': not any('/iris/odometry' in part
            or 'ground_truth' in part for part in planner),
        'zero_range_messages': result.get('lidar_topic_messages') == 0
            and result.get('depth_topic_messages') == 0,
        'source_declares_visual_localization': manifest.get('localization', '').startswith('RGB ground features'),
    }
    estimate = [row for row in frames if 'position_enu' in row]
    t = np.asarray([row['sim_s'] for row in truth])
    positions = np.asarray([row['position_enu'] for row in truth]).reshape(-1, 3)
    errors = []
    if len(t) > 1:
        for row in estimate:
            stamp = row['source_stamp_s']
            if t[0] <= stamp <= t[-1]:
                reference = np.array([np.interp(stamp, t, positions[:, axis]) for axis in range(3)])
                errors.append(float(np.linalg.norm(np.asarray(row['position_enu']) - reference)))
    return {
        'name': trial['name'], 'status': result.get('status'),
        'isolation_passed': all(checks.values()), 'checks': checks,
        'estimated_pose_frames': len(estimate), 'total_frames': len(frames),
        'pose_error_vs_eval_gt_m': ({f'p{p}': float(np.percentile(errors, p))
            for p in (50, 95, 100)} if errors else {}),
        'flight_uses_autopilot_sitl': True,
        'note': 'Gazebo odometry remains evaluator-only. ArduPilot SITL still uses its own simulated navigation sensors.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    rows = [audit(args.run_dir, trial, plan['perception_config']) for trial in plan['trials']]
    report = {'run_dir': str(args.run_dir), 'trials': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

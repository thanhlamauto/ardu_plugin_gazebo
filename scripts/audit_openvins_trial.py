#!/usr/bin/env python3
"""Audit OpenVINS pose isolation and metric error against evaluator-only truth."""
import argparse
import csv
import gzip
import json
from pathlib import Path

import numpy as np


def read_jsonl(path):
    if path.exists():
        handle = path.open('rt')
    elif path.with_suffix(path.suffix + '.gz').exists():
        handle = gzip.open(path.with_suffix(path.suffix + '.gz'), 'rt')
    else:
        return []
    with handle:
        return [json.loads(line) for line in handle if line.strip()]


def audit(root):
    result = json.loads((root / 'result.json').read_text())
    manifest = json.loads((root / 'manifest.json').read_text())
    bridge_command = json.loads((root / 'openvins.command.json').read_text())
    inference_command = json.loads((root / 'inference.command.json').read_text())
    planner_path = root / 'planner.command.json'
    planner_command = json.loads(planner_path.read_text()) if planner_path.exists() else []
    bridge_source = (root / 'source_snapshot/tools/openvins_gz_bridge/main.cpp').read_text()
    perception_source = (root / 'source_snapshot/scripts/monocular_geometry_gz.py').read_text()
    samples = list(csv.DictReader((root / 'openvins.csv').open()))
    frames = read_jsonl(root / 'perception/frames.jsonl')
    truth = read_jsonl(root / 'ground_truth.jsonl')
    checks = {
        'bridge_reads_rgb_and_imu': '/sensor_suite/rgb' in bridge_source
            and '/sensor_suite/imu' in bridge_source,
        'bridge_never_reads_gazebo_pose': '/iris/odometry' not in bridge_source,
        'perception_selects_openvins': '--pose-source' in inference_command
            and inference_command[inference_command.index('--pose-source') + 1] == 'openvins',
        'perception_external_topic':
            "node.subscribe(Odometry,'/perception/visual_odometry',odom_cb)" in perception_source,
        'planner_external_topic_if_started': not planner_command or
            ('--odom-topic' in planner_command and
             planner_command[planner_command.index('--odom-topic') + 1] == '/perception/visual_odometry'),
        'no_range_sensor_messages': result.get('lidar_topic_messages') == 0
            and result.get('depth_topic_messages') == 0,
        'bridge_command_recorded': bool(bridge_command) and any(
            str(arg).endswith('openvins.csv') for arg in bridge_command),
        'zupt_ablation_recorded': ('--zupt-after-motion' in bridge_command) ==
            bool(manifest.get('openvins_zupt_after_motion', False)),
        'binary_hash_recorded': bool(manifest.get('openvins_bridge_sha256')),
    }
    initialized = [row for row in samples if row['initialized'] == '1']
    t = np.array([row['sim_s'] for row in truth], dtype=float)
    p = np.array([row['position_enu'] for row in truth], dtype=float).reshape(-1, 3)
    errors = []
    for row in initialized:
        ts = float(row['stamp'])
        if len(t) < 2 or not t[0] <= ts <= t[-1]:
            continue
        estimate = np.array([float(row[key]) for key in ('x', 'y', 'z')])
        reference = np.array([np.interp(ts, t, p[:, k]) for k in range(3)])
        errors.append(float(np.linalg.norm(estimate - reference)))
    report = {
        'status': result['status'], 'checks': checks, 'isolation_passed': all(checks.values()),
        'imu_samples': int(samples[-1]['imu_count']) if samples else 0,
        'image_samples': int(samples[-1]['image_count']) if samples else 0,
        'initialized_pose_samples': len(initialized),
        'perception_frames_with_pose': sum('position_enu' in row for row in frames),
        'camera_cloud_messages': result['camera_cloud_messages'],
        'pose_error_vs_evaluation_truth_m': {
            f'p{q}': float(np.percentile(errors, q)) for q in (50, 95, 100)
        } if errors else {},
        'note': 'Ground truth is evaluator-only. ArduPilot SITL still uses internal simulated navigation.',
    }
    (root / 'openvins_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    print(json.dumps(audit(parser.parse_args().run_dir), indent=2))

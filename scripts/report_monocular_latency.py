#!/usr/bin/env python3
"""Join OpenVINS and camera-perception clocks for a latency probe trial."""
import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path

import numpy as np


def percentiles(values):
    return ({f'p{p}': round(float(np.percentile(values, p)), 2)
             for p in (50, 95, 99)} | {'count': len(values)}) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    trial = args.trial
    result = json.loads((trial/'result.json').read_text())
    with (trial/'openvins.csv').open(newline='') as f:
        vio = list(csv.DictReader(f))
    frames = [json.loads(line) for line in (trial/'perception/frames.jsonl').read_text().splitlines()]
    truth = [json.loads(line) for line in (trial/'ground_truth.jsonl').read_text().splitlines()]
    bridge = {round(float(row['stamp']), 5): row for row in vio}
    stages = {name: [] for name in (
        'vio_queue_ms', 'vio_processing_ms', 'rgb_to_vio_publish_ms',
        'vio_transport_ms', 'rgb_to_perception_done_ms',
        'perception_rgb_to_done_ms', 'pose_to_perception_done_ms',
        'rgb_to_cloud_publish_ms')}
    matched = 0
    start = result.get('probe_start_wall_s', -float('inf'))
    end = result.get('probe_end_wall_s', float('inf'))
    probe_vio = [row for row in vio if row['image_received_mono_s']
                 and start <= float(row['image_received_mono_s']) <= end]
    probe_frames = [row for row in frames if start <= row['image_received_mono_s'] <= end]

    def add(name, first, second):
        if first is not None and second is not None:
            delta = (float(second)-float(first))*1000
            if delta >= -0.5:
                stages[name].append(max(0.0, delta))

    for row in probe_vio:
        add('vio_queue_ms', row['image_received_mono_s'], row['image_processing_start_mono_s'])
        add('vio_processing_ms', row['image_processing_start_mono_s'], row['vio_done_mono_s'])
        add('rgb_to_vio_publish_ms', row['image_received_mono_s'], row['pose_published_mono_s'] or None)
    for frame in probe_frames:
        add('perception_rgb_to_done_ms', frame['image_received_mono_s'], frame['perception_done_mono_s'])
        key = round(frame['source_stamp_s'], 5)
        row = bridge.get(key)
        if row is None or not row['pose_published_mono_s']:
            continue
        matched += 1
        add('vio_transport_ms', row['pose_published_mono_s'], frame['pose_received_mono_s'])
        add('pose_to_perception_done_ms', frame['pose_received_mono_s'], frame['perception_done_mono_s'])
        add('rgb_to_perception_done_ms', row['image_received_mono_s'], frame['perception_done_mono_s'])
        add('rgb_to_cloud_publish_ms', row['image_received_mono_s'], frame['cloud_published_mono_s'])

    gt = [row for row in truth if start <= row['wall_s'] <= end]
    speeds = []
    speed_times = []
    for a, b in zip(gt, gt[1:]):
        dt = b['wall_s']-a['wall_s']
        if .03 <= dt <= .5:
            speeds.append(float(np.linalg.norm(np.subtract(b['position_enu'][:2], a['position_enu'][:2]))/dt))
            speed_times.append((a['wall_s']+b['wall_s'])/2)
    cruise_cloud = []
    cruise_frame_count = 0
    target_speed = result.get('target_speed_m_s')
    if target_speed and speed_times:
        for frame in probe_frames:
            i = bisect_left(speed_times, frame['image_received_mono_s'])
            nearest = min((j for j in (i-1,i) if 0 <= j < len(speeds)),
                          key=lambda j: abs(speed_times[j]-frame['image_received_mono_s']))
            if abs(speed_times[nearest]-frame['image_received_mono_s']) > .15 or speeds[nearest] < .9*target_speed:
                continue
            cruise_frame_count += 1
            row = bridge.get(round(frame['source_stamp_s'], 5))
            if row and frame['cloud_published_mono_s']:
                cruise_cloud.append((frame['cloud_published_mono_s']-float(row['image_received_mono_s']))*1000)
    elapsed = gt[-1]['wall_s']-gt[0]['wall_s'] if len(gt)>1 else 0
    displacement = float(np.linalg.norm(np.subtract(gt[-1]['position_enu'][:2], gt[0]['position_enu'][:2]))) if len(gt)>1 else 0
    sim_stamps = [row['sim_s'] for row in gt]
    pose_errors = []
    for row in probe_vio:
        if row['initialized'] != '1' or not sim_stamps:
            continue
        i = bisect_left(sim_stamps, float(row['stamp']))
        candidates = gt[max(0, i-1):min(len(gt), i+1)]
        nearest = min(candidates, key=lambda sample: abs(sample['sim_s']-float(row['stamp'])))
        if abs(nearest['sim_s']-float(row['stamp'])) <= .02:
            estimate = [float(row[key]) for key in ('x','y','z')]
            pose_errors.append(float(np.linalg.norm(np.subtract(estimate,nearest['position_enu']))))
    cloud_latency = percentiles(stages['rgb_to_cloud_publish_ms'])
    cloud_times = [row['cloud_published_mono_s'] for row in probe_frames
                   if row['cloud_published_mono_s'] is not None]
    cloud_gaps_ms = [1000*(b-a) for a,b in zip(cloud_times,cloud_times[1:])]
    summary = dict(status=result['status'], target_speed_m_s=result.get('target_speed_m_s'),
        probe_wall_s=round(end-start, 2), ground_truth_samples=len(gt),
        mean_displacement_speed_m_s=round(displacement/elapsed, 2) if elapsed>0 else None,
        observed_speed_m_s=percentiles(speeds),
        pose_error_vs_ground_truth_m=percentiles(pose_errors),
        bridge_images=len(probe_vio), initialized_poses=sum(row['initialized']=='1' for row in probe_vio),
        perception_frames=len(probe_frames), matched_frames=matched,
        cloud_frames=sum(bool(row['published']) for row in probe_frames),
        cloud_gap_ms=percentiles(cloud_gaps_ms),
        max_cloud_gap_ms=round(max(cloud_gaps_ms), 2) if cloud_gaps_ms else None,
        cruise_threshold_m_s=round(.9*target_speed, 2) if target_speed else None,
        cruise_frames=cruise_frame_count, cruise_cloud_frames=len(cruise_cloud),
        cruise_rgb_to_cloud_publish_ms=percentiles(cruise_cloud),
        camera_period_ms=percentiles([1000*(float(b['stamp'])-float(a['stamp']))
                                     for a,b in zip(probe_vio,probe_vio[1:])
                                     if 0<float(b['stamp'])-float(a['stamp'])<1]),
        stages_ms={key:percentiles(values) for key,values in stages.items()},
        distance_at_target_speed_for_p95_cloud_latency_m=(round(result['target_speed_m_s']*cloud_latency['p95']/1000, 3)
            if cloud_latency and result.get('target_speed_m_s') else None),
        latency_scope='Wall monotonic from Gazebo RGB subscriber callback to VIO/perception output; capture-to-callback and planner/control excluded.',
        end_to_end_planner_latency_ms=None)
    (trial/'latency_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

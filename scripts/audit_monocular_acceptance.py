#!/usr/bin/env python3
"""Independent acceptance audit from source snapshots and raw flight telemetry."""
import argparse,gzip,hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml

def rows(path):
    if path.exists():handle=path.open()
    else:handle=gzip.open(path.with_suffix(path.suffix+'.gz'),'rt')
    with handle:return [json.loads(line) for line in handle if line.strip()]

def audit_trial(root,trial):
    result=json.loads((root/'result.json').read_text());manifest=json.loads((root/'manifest.json').read_text())
    snapshot=root/'source_snapshot';checks={}
    checks['source_hashes_match']=all(hashlib.sha256((snapshot/name).read_bytes()).hexdigest()==value for name,value in manifest['files'].items())
    config_path=next(name for name in manifest['files'] if name.endswith('monocular_mppi.yaml') or name.endswith('monocular_geometry_mppi.yaml'))
    cfg=yaml.safe_load((snapshot/config_path).read_text())
    checks['camera_cloud_only']=cfg['lidar_topic']=='/perception/obstacles_camera'
    checks['no_known_map_input']=not any(cfg.get(name) for name in ['known_obstacle_sdf','global_map_sdf','global_path']) and cfg.get('global_planner','manual')=='manual'
    checks['collision_and_stopping_guards']=bool(cfg.get('validate_final_trajectory') and cfg.get('validate_stopping_trajectory') and cfg.get('feasible_sample_weighting'))
    seen=set();sensors=[]
    def inspect_model(name):
        if name in seen:return
        seen.add(name);model=ET.parse(snapshot/'models'/name/'model.sdf')
        sensors.extend(sensor.get('type') for sensor in model.findall('.//sensor'))
        for uri in model.findall('.//include/uri'):
            assert uri.text.startswith('model://');inspect_model(uri.text[8:])
    inspect_model('iris_with_monocular_camera')
    checks['one_physical_camera_no_range']=sensors.count('camera')==1 and not any('depth' in s or 'lidar' in s for s in sensors)
    checks['zero_range_messages']=result['lidar_topic_messages']==0 and result['depth_topic_messages']==0
    checks['autopilot_range_avoidance_off']=all(result.get('parameters_readback',{}).get(k)==0 for k in ['OA_TYPE','AVOID_ENABLE','PRX1_TYPE'])
    inference=json.loads((root/'inference.command.json').read_text())
    checks['geometry_rgb_backend']=any(arg.endswith('/monocular_geometry_gz.py') for arg in inference) and not any(flag in inference for flag in ['--gt-topic','--world','--lidar-topic','--depth-topic'])
    world_path=next(name for name in manifest['files'] if name.startswith('worlds/'))
    world=ET.parse(snapshot/world_path);box=next(i for i in world.findall('.//include') if i.findtext('name')=='test_box')
    pose=np.fromstring(box.findtext('pose'),sep=' ')
    size=np.fromstring(ET.parse(snapshot/'models/monocular_textured_box/model.sdf').findtext('.//collision/geometry/box/size'),sep=' ')
    center=np.asarray(trial['eval_box_center'],float)
    checks['evaluation_matches_actual_box']=bool(np.allclose(pose[:3],center) and np.allclose(pose[3:],0) and np.allclose(size,result['eval_box_size']))
    trace=rows(root/'ground_truth.jsonl');start=result.get('planner_start_wall_s',float('inf'));end=result.get('end_wall_s',0)
    flight=[r for r in trace if start<=r['wall_s']<=end]
    positions=np.asarray([r['position_enu'] for r in flight]).reshape(-1,3)
    checks['finite_ground_truth_control_trace']=len(flight)>1 and bool(np.isfinite(positions).all())
    minimum=None;goal_error=None;max_gap=None
    if len(flight)>1:
        clearance=np.linalg.norm(np.maximum(abs(positions-center)-size/2,0),axis=1)
        minimum=float(clearance.min());goal_error=float(np.linalg.norm(positions[-1]-[12,0,3]))
        max_gap=float(np.diff([r['sim_s'] for r in flight]).max())
        checks['clearance_at_least_075m']=minimum>=.75
        checks['goal_within_05m']=result['status']=='GOAL_REACHED' and goal_error<=.5
        checks['fixed_altitude_envelope']=bool(np.all(abs(positions[:,2]-3)<=1.))
    else:checks.update(clearance_at_least_075m=False,goal_within_05m=False,fixed_altitude_envelope=False)
    planner=rows(root/'planner.jsonl') if (root/'planner.jsonl').exists() or (root/'planner.jsonl.gz').exists() else []
    checks['declared_seed_used']=bool(planner) and all(r['random_seed']==trial['seed'] for r in planner)
    blackout=None
    if trial.get('blackout'):
        pause=result.get('camera_blackout_start_wall_s');resume=result.get('camera_blackout_end_wall_s')
        observed=[r for r in planner if pause is not None and resume is not None and pause+1.2<=r['timestamp_monotonic_s']<=resume]
        holds=[r for r in observed if r['event']=='hold-stale']
        blackout=bool(holds) and all(r['event']=='hold-stale' and np.linalg.norm(r['sent_mavlink_control']['velocity_ned_m_s'])<1e-9 for r in observed)
        checks['camera_blackout_hold_zero']=blackout
    hashes=root/'compressed_log_hashes.json'
    if hashes.exists():
        checks['compressed_logs_intact']=True
        for name,digest in json.loads(hashes.read_text()).items():
            calculated=hashlib.sha256()
            with gzip.open(root/(name+'.gz'),'rb') as f:
                while chunk:=f.read(1024*1024):calculated.update(chunk)
            checks['compressed_logs_intact'] &= calculated.hexdigest()==digest
    return dict(name=trial['name'],status=result['status'],accepted=all(checks.values()),checks=checks,
        min_center_clearance_m=minimum,goal_error_m=goal_error,control_samples=len(flight),max_ground_truth_gap_sim_s=max_gap,blackout=blackout)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_root',type=Path);parser.add_argument('--plan',required=True,type=Path)
    args=parser.parse_args();plan=json.loads(args.plan.read_text());results=[];pending=[]
    for trial in plan['trials']:
        root=args.run_root/trial['name']
        if not (root/'analysis.json').exists():pending.append(trial['name']);continue
        results.append(audit_trial(root,trial))
    ordinary=[r for r in results if not next(t for t in plan['trials'] if t['name']==r['name']).get('blackout')]
    blackout=[r for r in results if next(t for t in plan['trials'] if t['name']==r['name']).get('blackout')]
    report=dict(completed=len(results),pending=pending,accepted_navigation=sum(r['accepted'] for r in ordinary),navigation_trials=len(ordinary),results=results)
    matrix={(t['layout'],t['seed']) for t in plan['trials'] if not t.get('blackout')}
    report['declared_matrix_matches']=matrix=={(layout,seed) for layout in ('centered','shifted') for seed in range(7,12)} and len([t for t in plan['trials'] if not t.get('blackout')])==10
    report['acceptance_proven']=report['declared_matrix_matches'] and not pending and len(ordinary)==10 and report['accepted_navigation']>=8 and bool(blackout) and all(r['accepted'] for r in blackout)
    (args.run_root/'independent_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()

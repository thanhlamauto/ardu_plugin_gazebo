#!/usr/bin/env python3
"""Audit speed trials from immutable sensor/source snapshots and raw telemetry."""
import argparse,gzip,hashlib,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from mppi_ardupilot.monocular_evaluation import gate_crossings

def rows(path):
    if not path.exists():path=path.with_suffix(path.suffix+'.gz')
    if not path.exists():return []
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as f:return [json.loads(x) for x in f if x.strip()]

def file_digest(path):
    digest=hashlib.sha256()
    with path.open('rb') as f:
        while data:=f.read(1048576):digest.update(data)
    return digest.hexdigest()

def audit_trial(root,trial):
    result=json.loads((root/'result.json').read_text());manifest=json.loads((root/'manifest.json').read_text());snap=root/'source_snapshot';checks={}
    checks['source_hashes']=all(file_digest(snap/name)==digest for name,digest in manifest['files'].items())
    cfg=yaml.safe_load((snap/trial['config']).read_text());scene=json.loads((snap/trial['eval_scene']).read_text());boxes=scene['boxes']
    checks['configuration_speed']=float(cfg['vmax'])==trial['speed']
    checks['camera_cloud_no_map']=cfg['lidar_topic']=='/perception/obstacles_camera' and not any(cfg.get(k) for k in ('known_obstacle_sdf','global_map_sdf','global_path')) and cfg.get('global_planner','manual')=='manual'
    checks['guards_unchanged']=all(cfg.get(k) for k in ('validate_final_trajectory','validate_stopping_trajectory','feasible_sample_weighting')) and cfg['collision_radius_m']>=1.25 and cfg['hard_brake_m']>=1.25
    sensors=[];seen=set()
    def model_sensors(name):
        if name in seen:return
        seen.add(name);xml=ET.parse(snap/'models'/name/'model.sdf')
        sensors.extend(s.get('type','') for s in xml.findall('.//sensor'))
        for uri in xml.findall('.//include/uri'):
            assert uri.text.startswith('model://');model_sensors(uri.text[8:])
    model_sensors('iris_with_monocular_camera')
    checks['one_rgb_no_range']=sensors.count('camera')==1 and not any(any(k in s for k in ('depth','lidar','ray','rgbd')) for s in sensors)
    checks['zero_range_messages']=result.get('lidar_topic_messages')==0 and result.get('depth_topic_messages')==0
    checks['autopilot_range_off']=all(result.get('parameters_readback',{}).get(k)==0 for k in ('OA_TYPE','AVOID_ENABLE','PRX1_TYPE'))
    inference=json.loads((root/'inference.command.json').read_text()) if (root/'inference.command.json').exists() else []
    forbidden=('world','eval','sdf','depth-topic','lidar-topic','gt-topic')
    checks['rgb_geometry_input']=any(x.endswith('/monocular_geometry_gz.py') for x in inference) and not any(x.startswith('--') and any(k in x for k in forbidden) for x in inference)
    command=json.loads((root/'planner.command.json').read_text()) if (root/'planner.command.json').exists() else []
    checks['planner_no_eval_map']=bool(command) and not any(x.startswith('--') and any(k in x for k in forbidden) for x in command)
    checks['planner_goal']=bool(command) and '--goal' in command and np.allclose(np.fromstring(command[command.index('--goal')+1],sep=','),trial['goal'])
    world=ET.parse(snap/trial['world']).getroot().find('world');actual=[];world_ok=True
    for item in world:
        if item.tag not in ('include','model'):continue
        name=item.get('name') or item.findtext('name')
        if name in ('iris','asphalt'):continue
        pose=np.fromstring(item.findtext('pose','0 0 0 0 0 0'),sep=' ')
        model=item
        if item.tag=='include':
            uri=item.findtext('uri');model=ET.parse(snap/'models'/uri[8:]/'model.sdf').getroot().find('model')
        collision=model.findall('.//collision')
        if len(collision)!=1 or collision[0].find('geometry/box') is None:world_ok=False;continue
        offsets=[model.findtext('link/pose','0 0 0 0 0 0'),collision[0].findtext('pose','0 0 0 0 0 0')]
        if item.tag=='include':offsets.append(model.findtext('pose','0 0 0 0 0 0'))
        for offset in offsets:
            world_ok &= bool(np.allclose(np.fromstring(offset,sep=' '),0))
        size=np.fromstring(collision[0].findtext('geometry/box/size'),sep=' ')
        actual.append(dict(name=name,center=pose[:3].tolist(),size=size.tolist(),yaw=float(pose[5])))
        world_ok &= len(pose)==6 and bool(np.allclose(pose[3:5],0))
    checks['every_actual_obstacle_evaluated']=world_ok and len(actual)==len(boxes) and {b['name'] for b in boxes}=={b['name'] for b in actual} and all(any(a['name']==b['name'] and np.allclose(a['center'],b['center']) and np.allclose(a['size'],b['size']) and np.isclose(a['yaw'],b.get('yaw',0)) for a in actual) for b in boxes) and result.get('eval_boxes')==boxes
    planner=rows(root/'planner.jsonl')
    goal_rows=[r for r in planner if r.get('active_goal_enu') is not None]
    missing_goal=[r for r in planner if r.get('active_goal_enu') is None]
    checks['seed_and_goal_used']=bool(goal_rows) and all(r['random_seed']==trial['seed'] for r in planner) and all(np.allclose(r['active_goal_enu'],trial['goal']) for r in goal_rows) and all(r['event']=='hold-stale' and np.linalg.norm(r['sent_mavlink_control']['velocity_ned_m_s'])<1e-9 for r in missing_goal)
    trace=rows(root/'ground_truth.jsonl');start=result.get('planner_start_wall_s',float('inf'));end=result.get('end_wall_s',0)
    flight=[r for r in trace if start<=r['wall_s']<=end]
    p=np.asarray([r['position_enu'] for r in flight]).reshape(-1,3);sim=np.asarray([r['sim_s'] for r in flight]);dt=np.diff(sim)
    minimum=None;error=None;gap=None;speeds={};gate=None
    checks['finite_dense_trace']=len(p)>1 and bool(np.isfinite(p).all() and np.isfinite(sim).all() and np.all(dt>=0) and dt.max()<=.1)
    if len(p)>1:
        distances=[]
        for b in actual:
            rel=p-np.asarray(b['center']);yaw=b['yaw'];c,s=np.cos(yaw),np.sin(yaw)
            local=np.column_stack((c*rel[:,0]+s*rel[:,1],-s*rel[:,0]+c*rel[:,1],rel[:,2]))
            distances.append(np.linalg.norm(np.maximum(abs(local)-np.asarray(b['size'])/2,0),axis=1))
        minimum=float(np.min(distances)) if distances else None;error=float(np.linalg.norm(p[-1]-trial['goal']));gap=float(dt.max())
        checks['clearance_075']=minimum is not None and minimum>=.75
        checks['goal_05']=result['status']=='GOAL_REACHED' and error<=.5
        checks['altitude_envelope']=bool(np.all(abs(p[:,2]-3)<=1))
        valid=dt>1e-6;velocity=np.linalg.norm(np.diff(p,axis=0)[valid,:2],axis=1)/dt[valid]
        if len(velocity):speeds={f'p{q}':float(np.percentile(velocity,q)) for q in (50,95,99)}
        if trial['scenario']=='gate':
            gate=gate_crossings(p,actual)
    else:checks.update(clearance_075=False,goal_05=False,altitude_envelope=False)
    hashes=root/'compressed_log_hashes.json'
    if hashes.exists():
        intact=True
        for name,digest in json.loads(hashes.read_text()).items():
            h=hashlib.sha256()
            with gzip.open(root/(name+'.gz'),'rb') as f:
                while data:=f.read(1048576):h.update(data)
            intact &= h.hexdigest()==digest
        checks['lossless_logs']=intact
    blackout=None
    if trial.get('blackout'):
        begin=result.get('camera_blackout_start_wall_s');finish=result.get('camera_blackout_end_wall_s')
        held=[r for r in planner if begin is not None and finish is not None and begin+1.2<=r['timestamp_monotonic_s']<=finish]
        zero=bool(held) and all(r['event']=='hold-stale' and np.linalg.norm(r['sent_mavlink_control']['velocity_ned_m_s'])<1e-9 for r in held)
        checks['blackout_injected_and_zero_command']=bool(begin is not None and finish is not None and finish-begin>=trial.get('blackout_seconds',5)-.1 and zero)
        blackout=dict(duration_wall_s=finish-begin if finish is not None and begin is not None else None,stale_zero_cycles=len(held),zero_command=zero)
    navigation=all(checks.values());task=navigation and (gate is None or gate['through_gap'])
    return dict(name=trial['name'],scenario=trial['scenario'],speed=trial['speed'],seed=trial['seed'],status=result['status'],navigation_accepted=navigation,task_accepted=task,checks=checks,clearance_m=minimum,goal_error_m=error,max_gt_gap_sim_s=gap,horizontal_speed_m_s=speeds,gate=gate,blackout=blackout)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_root',type=Path);parser.add_argument('--plan',required=True,type=Path);args=parser.parse_args()
    plan=json.loads(args.plan.read_text());results=[];pending=[]
    for trial in plan['trials']:
        root=args.run_root/trial['name']
        if not (root/'analysis.json').exists() or not (root/'compressed_log_hashes.json').exists():pending.append(trial['name']);continue
        results.append(audit_trial(root,trial))
    groups={}
    for speed in sorted({t['speed'] for t in plan['trials']}):
        r=[r for r in results if r['speed']==speed];groups[str(speed)]=dict(completed=len(r),navigation_accepted=sum(x['navigation_accepted'] for x in r),task_accepted=sum(x['task_accepted'] for x in r),declared=sum(t['speed']==speed for t in plan['trials']))
    report=dict(completed=len(results),declared=len(plan['trials']),pending=pending,by_speed=groups,results=results)
    (args.run_root/'independent_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()

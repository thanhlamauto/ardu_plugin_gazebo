#!/usr/bin/env python3
"""Run a declared monocular acceptance plan without retries or hidden exclusions."""
import argparse,gzip,hashlib,json,shutil,subprocess,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def compact_logs(root):
    preserved={}
    for p in root.rglob('*'):
        if not p.is_file() or p.suffix not in ('.jsonl','.log'):continue
        target=p.with_suffix(p.suffix+'.gz')
        digest=hashlib.sha256()
        with p.open('rb') as source,gzip.open(target,'wb',compresslevel=3) as dest:
            while data:=source.read(1024*1024):digest.update(data);dest.write(data)
        verify=hashlib.sha256()
        with gzip.open(target,'rb') as source:
            while data:=source.read(1024*1024):verify.update(data)
        assert verify.digest()==digest.digest()
        preserved[str(p.relative_to(root))]=digest.hexdigest();p.unlink()
    (root/'compressed_log_hashes.json').write_text(json.dumps(preserved,indent=2))

def read_rows(path):
    if path.exists():handle=path.open()
    else:handle=gzip.open(path.with_suffix(path.suffix+'.gz'),'rt')
    with handle:return [json.loads(line) for line in handle if line.strip()]

def run(plan_path,output):
    plan=json.loads(plan_path.read_text());output.mkdir(parents=True,exist_ok=False);attempts=[]
    required=plan.get('required_successes',8);declared=plan['trials']
    for index,trial in enumerate(declared):
        out=output/trial['name'];center=trial['eval_box_center']
        command=[sys.executable,str(ROOT/'scripts/run_monocular_sim.py'),'--output',str(out),'--world',str(ROOT/trial['world']),
            '--config',str(ROOT/plan.get('config','config/monocular_mppi.yaml')),'--perception-config',str(ROOT/plan.get('perception_config','config/monocular_perception.json')),
            '--duration',str(plan['duration']),'--seed',str(trial['seed']),'--eval-box-center',*map(str,center)]
        if trial.get('blackout'):command.extend(['--camera-blackout-at-x','2','--camera-blackout-seconds','5'])
        print('ATTEMPT',json.dumps(dict(index=index,trial=trial,command=command)),flush=True)
        completed=subprocess.run(command,cwd=ROOT,check=False)
        result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else dict(status='MISSING_RESULT')
        if (out/'result.json').exists():
            analyzed=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_monocular_sim.py'),str(out)],check=False)
            if analyzed.returncode!=0:result['analysis_error']=analyzed.returncode
        analysis=json.loads((out/'analysis.json').read_text()) if (out/'analysis.json').exists() else {}
        accepted=(result.get('status')=='GOAL_REACHED' and completed.returncode==0 and
            analysis.get('final_goal_distance_m',float('inf'))<=.5 and
            analysis.get('dense_trace_min_center_clearance_m',-1)>=.75 and not analysis.get('dense_trace_collision_proxy',True) and
            result.get('lidar_topic_messages',-1)==0 and result.get('depth_topic_messages',-1)==0)
        blackout=None
        if trial.get('blackout') and result.get('camera_blackout_start_wall_s'):
            start=result['camera_blackout_start_wall_s'];end=result.get('camera_blackout_end_wall_s',start+5)
            rows=read_rows(out/'planner.jsonl')
            holds=[r for r in rows if start+1.2<=r['timestamp_monotonic_s']<=end and r['event']=='hold-stale']
            zero=bool(holds) and all(np.linalg.norm(r['sent_mavlink_control']['velocity_ned_m_s'])<1e-9 for r in holds)
            blackout=dict(hold_stale_cycles=len(holds),zero_velocity_verified=zero,passed=zero)
        row=dict(name=trial['name'],seed=trial['seed'],layout=trial['layout'],blackout_test=bool(trial.get('blackout')),accepted=accepted,
            status=result.get('status'),returncode=completed.returncode,clearance_m=analysis.get('dense_trace_min_center_clearance_m'),
            goal_distance_m=analysis.get('final_goal_distance_m'),wall_control_s=analysis.get('control_wall_duration_s'),
            camera_p95_ms=analysis.get('during_control_receive_to_publish_ms',{}).get('p95'),blackout=blackout)
        attempts.append(row)
        (output/'attempts.json').write_text(json.dumps(attempts,indent=2));print('RESULT',json.dumps(row),flush=True)
        compact_logs(out)
    benchmark=[r for r in attempts if not r['blackout_test']];tests=[r for r in attempts if r['blackout_test']]
    success=sum(r['accepted'] for r in benchmark)
    summary=dict(declared_attempts=len(declared),benchmark_attempts=len(benchmark),accepted=success,required=required,
        success_rate=success/len(benchmark),layouts=sorted({r['layout'] for r in benchmark}),
        camera_blackout_passed=bool(tests) and all(r['accepted'] and r['blackout'] and r['blackout']['passed'] for r in tests),attempts=attempts)
    summary['acceptance_passed']=(len(benchmark)==10 and success>=required and len(summary['layouts'])>=2 and summary['camera_blackout_passed'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2));print('BATCH_RESULT',json.dumps(summary),flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.plan,args.output)

if __name__=='__main__':main()

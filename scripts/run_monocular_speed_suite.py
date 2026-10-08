"""Declared speed/scenario trials; all failures remain in results."""
import json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.run_monocular_batch import compact_logs

def run(plan_path,output):
    plan=json.loads(plan_path.read_text());output.mkdir(parents=True,exist_ok=False);attempts=[]
    for trial in plan['trials']:
        out=output/trial['name']
        command=[sys.executable,str(ROOT/'scripts/run_monocular_sim.py'),'--output',str(out),'--world',str(ROOT/trial['world']),'--config',str(ROOT/trial['config']),'--perception-config',str(ROOT/plan['perception_config']),'--eval-scene',str(ROOT/trial['eval_scene']),'--goal',*map(str,trial['goal']),'--seed',str(trial['seed']),'--duration',str(plan['duration'])]
        if trial.get('blackout'):
            command.extend(['--camera-blackout-at-x',str(trial.get('blackout_at_x',2)), '--camera-blackout-seconds',str(trial.get('blackout_seconds',5))])
        print('ATTEMPT',json.dumps(dict(trial=trial,command=command)),flush=True)
        proc=subprocess.run(command,cwd=ROOT)
        analysis={}
        if (out/'result.json').exists():
            subprocess.run([sys.executable,str(ROOT/'scripts/analyze_monocular_sim.py'),str(out)],cwd=ROOT)
            if (out/'analysis.json').exists():analysis=json.loads((out/'analysis.json').read_text())
        accepted=(proc.returncode==0 and analysis.get('status')=='GOAL_REACHED' and analysis.get('dense_trace_min_center_clearance_m',-1)>=.75 and analysis.get('final_goal_distance_m',float('inf'))<=.5 and analysis.get('lidar_topic_messages',-1)==0 and analysis.get('depth_topic_messages',-1)==0 and analysis.get('max_gt_gap_sim_s',float('inf'))<=.1)
        row=dict(trial,accepted=accepted,returncode=proc.returncode,status=analysis.get('status','MISSING_ANALYSIS'),clearance_m=analysis.get('dense_trace_min_center_clearance_m'),goal_error_m=analysis.get('final_goal_distance_m'),speed_actual=analysis.get('horizontal_speed_m_s'),control_wall_s=analysis.get('control_wall_duration_s'),planner_events=analysis.get('planner_events'))
        attempts.append(row);(output/'attempts.json').write_text(json.dumps(attempts,indent=2));print('RESULT',json.dumps(row),flush=True)
        if out.exists():compact_logs(out)
    groups={}
    for speed in sorted({r['speed'] for r in attempts}):
        values=[r for r in attempts if r['speed']==speed];groups[str(speed)]=dict(accepted=sum(r['accepted'] for r in values),attempted=len(values))
    summary=dict(attempts=attempts,by_speed=groups,declared=len(plan['trials']),completed=len(attempts))
    (output/'summary.json').write_text(json.dumps(summary,indent=2));print('SUITE_RESULT',json.dumps(summary),flush=True)

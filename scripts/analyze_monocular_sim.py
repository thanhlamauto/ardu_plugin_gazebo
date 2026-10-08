#!/usr/bin/env python3
"""Summarize the camera-only trial using recorded data, without rerunning control."""
import argparse
from collections import Counter
import json
import gzip
from pathlib import Path
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    args=parser.parse_args();root=args.run_dir
    from mppi_ardupilot.monocular_evaluation import scene_clearance
    result=json.loads((root/'result.json').read_text())
    def read(path):
        if path.exists():text=path.read_text()
        elif path.with_suffix(path.suffix+'.gz').exists():
            with gzip.open(path.with_suffix(path.suffix+'.gz'),'rt') as f:text=f.read()
        else:return []
        return [json.loads(s) for s in text.splitlines() if s]
    perception=read(root/'perception/frames.jsonl');planner=read(root/'planner.jsonl')
    start=result.get('planner_start_wall_s',float('inf'));end=result.get('end_wall_s',0)
    flight=[r for r in read(root/'ground_truth.jsonl') if start<=r['wall_s']<=end]
    summary=dict(result)
    summary['planner_events']=dict(Counter(r['event'] for r in planner))
    summary['planner_cycles']=len(planner)
    summary['cycle_deadline_misses']=sum(bool(r.get('cycle_deadline_miss')) for r in planner)
    summary['perception_frames']=len(perception)
    summary['published_frames']=sum(r['published'] for r in perception)
    active=[r for r in perception if flight and flight[0]['sim_s']<=r['source_stamp_s']<=flight[-1]['sim_s']]
    for label,rows in [('all',perception),('during_control',active)]:
        if rows:
            summary[label+'_receive_to_publish_ms']={f'p{p}':float(np.percentile([r['receive_to_publish_ms'] for r in rows],p)) for p in (50,95,99)}
    if flight:
        p=np.array([r['position_enu'] for r in flight]);t=np.array([r['wall_s']-start for r in flight])
        summary['control_sim_duration_s']=flight[-1]['sim_s']-flight[0]['sim_s']
        summary['control_wall_duration_s']=flight[-1]['wall_s']-flight[0]['wall_s']
        summary['max_forward_x_m']=float(p[:,0].max())
        summary['altitude_min_max_m']=[float(p[:,2].min()),float(p[:,2].max())]
        goal=np.asarray(result.get('goal_enu',[12,0,3]))
        summary['final_goal_distance_m']=float(np.linalg.norm(p[-1]-goal))
        box_center=np.asarray(result.get('eval_box_center',[8,0,3]));box_size=np.asarray(result.get('eval_box_size',[2,3,6]))
        boxes=result.get('eval_boxes',[dict(center=box_center.tolist(),size=box_size.tolist())])
        clearance=scene_clearance(p,boxes)
        sim=np.asarray([r['sim_s'] for r in flight]);dtime=np.diff(sim);valid=dtime>1e-6
        speeds=np.linalg.norm(np.diff(p,axis=0)[valid,:2],axis=1)/dtime[valid]
        summary['horizontal_speed_m_s']={f'p{x}':float(np.percentile(speeds,x)) for x in (50,95,99)} if len(speeds) else {}
        summary['path_length_m']=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
        summary['max_gt_gap_sim_s']=float(dtime.max()) if len(dtime) else None
        summary['dense_trace_min_center_clearance_m']=float(clearance.min())
        summary['dense_trace_collision_proxy']=bool(np.any(clearance<.5))
        summary['ground_truth_control_samples']=len(flight)
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig,ax=plt.subplots(1,2,figsize=(11,4),layout='constrained')
        ax[0].plot(p[:,0],p[:,1],label='Ground-truth flight');ax[0].scatter([p[0,0],goal[0]],[p[0,1],goal[1]],marker='x',label='Planner start / goal')
        for i,box in enumerate(boxes):
            center=np.asarray(box['center']);size=np.asarray(box['size']);ax[0].add_patch(Rectangle(tuple((center-size/2)[:2]),size[0],size[1],facecolor='tab:red',alpha=.4,label='Obstacles (evaluation only)' if i==0 else None))
        ax[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='RGB obstacle avoidance (Gazebo pose)',xlim=(-2,max(13,goal[0]+1)),ylim=(min(p[:,1].min(),min(b['center'][1]-b['size'][1]/2 for b in boxes))-1,max(p[:,1].max(),max(b['center'][1]+b['size'][1]/2 for b in boxes))+1));ax[0].set_aspect('equal');ax[0].legend(fontsize=8)
        ax[1].plot(t,p[:,0],label='X');ax[1].plot(t,p[:,1],label='Y');ax[1].plot(t,p[:,2],label='Altitude')
        ax[1].set(xlabel='Time after planner start (wall s)',ylabel='Position (m)',title=result['status']);ax[1].legend();ax[1].grid(alpha=.3)
        fig.savefig(root/'trajectory.png',dpi=150)
    (root/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()

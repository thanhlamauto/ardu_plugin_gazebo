"""Build a compact evidence report from independently audited speed trials."""
import argparse,csv,hashlib,json,shutil
from pathlib import Path
import numpy as np

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_root',type=Path)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--commit',required=True)
    args=parser.parse_args()
    audit=json.loads((args.run_root/'independent_audit.json').read_text())
    plan=json.loads(args.plan.read_text())
    assert audit['completed']==audit['declared']==len(plan['trials']) and not audit['pending']
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    for name in ('summary.json','attempts.json','independent_audit.json'):
        shutil.copy2(args.run_root/name,out/name)
    shutil.copy2(args.plan,out/'plan.json')
    records=[]
    for r in audit['results']:
        source=args.run_root/r['name']
        if not source.exists():source=args.run_root/'trials'/r['name']
        dest=out/'trials'/r['name'];dest.mkdir(parents=True,exist_ok=True)
        for name in ('result.json','analysis.json','manifest.json','trajectory.png'):
            if name=='trajectory.png' and not (source/name).exists():continue
            shutil.copy2(source/name,dest/name)
        a=json.loads((source/'analysis.json').read_text())
        wall=a.get('control_wall_duration_s');sim=a.get('control_sim_duration_s')
        rejection=r.get('rejection') or {}
        records.append(dict(name=r['name'],scenario=r['scenario'],seed=r['seed'],speed_setting=r['speed'],status=r['status'],accepted=r['task_accepted'],clearance_m=r['clearance_m'],goal_error_m=r['goal_error_m'],actual_p50=r['horizontal_speed_m_s'].get('p50'),actual_p95=r['horizontal_speed_m_s'].get('p95'),actual_p99=r['horizontal_speed_m_s'].get('p99'),wall_s=wall,sim_s=sim,rtf=sim/wall if wall and sim else None,through_gap=r['gate']['through_gap'] if r['gate'] else None,safely_rejected=rejection.get('safely_rejected'),rejection_clearance_m=rejection.get('all_trace_clearance_m')))
    with (out/'trials.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    speeds=sorted({r['speed_setting'] for r in records});scenarios=list(dict.fromkeys(r['scenario'] for r in records))
    lines=['# Kết quả mở rộng UAV monocular','',f'Immutable run `{args.run_id}`, commit `{args.commit}`.', '',f"{len(records)} lượt; mọi thất bại giữ trong mẫu số. Thành công yêu cầu tới đích, clearance ≥0,75 m, audit nguồn/cảm biến đạt; gate phải qua khe.",'','| Tình huống | '+' | '.join(f'{s:g} m/s đặt' for s in speeds)+' |','|---|'+'---|'*len(speeds)]
    for scene in scenarios:
        cells=[]
        for speed in speeds:
            group=[r for r in records if r['scenario']==scene and r['speed_setting']==speed]
            cells.append(f"{sum(r['accepted'] for r in group)}/{len(group)}")
        lines.append('| '+scene+' | '+' | '.join(cells)+' |')
    lines+=['','| Mức đặt | Đạt/toàn bộ | p95 thực trung vị (lượt đạt) | Thời gian trung vị (lượt đạt) | Clearance nhỏ nhất (lượt đạt) |','|---|---|---|---|---|']
    for speed in speeds:
        group=[r for r in records if r['speed_setting']==speed];ok=[r for r in group if r['accepted']]
        values=[f'{np.median([r[k] for r in ok]):.3f}' if ok else '—' for k in ('actual_p95','wall_s')]
        minimum=f"{min(r['clearance_m'] for r in ok):.3f}" if ok else '—'
        lines.append(f"| {speed:g} | {len(ok)}/{len(group)} | {values[0]} m/s | {values[1]} s | {minimum} m |")
    failed=[r for r in audit['results'] if not r['task_accepted']]
    if failed:
        lines+=['','Các lượt không đạt:','', '| Lượt | Status | Checks chưa đạt / gate |','|---|---|---|']
        for r in failed:
            reasons=[name for name,passed in r['checks'].items() if not passed]
            if r['gate'] and not r['gate']['through_gap']:reasons.append('outside_gate')
            lines.append(f"| {r['name']} | {r['status']} | {', '.join(reasons)} |")
    rejections=[r for r in records if r['safely_rejected']]
    if rejections:
        lines+=['',f'{len(rejections)} lượt từ chối khởi chạy vì thiếu evidence được kiểm tra an toàn trong toàn bộ takeoff/warmup/landing. Các lượt này vẫn tính thất bại tới goal. Xem cột safely_rejected/rejection_clearance_m trong CSV.']
    n_seeds=len({r['seed'] for r in records})
    lines+=['',f'Mức đặt giới hạn từng trục XY; norm vận tốc có thể cao hơn. Số liệu vận tốc lấy từ vị trí thật theo thời gian mô phỏng. Khoảng cách là tâm phương tiện tới bề mặt box, không phải telemetry tiếp xúc. {n_seeds} seed thay đổi sampling MPPI; geometry, texture và camera cố định. Tỷ lệ này chưa chứng minh độ tin cậy ngoài thực tế.','', 'Một RGB camera, không LiDAR/depth camera. Gazebo odometry cung cấp pose có scale cho triangulation và điều khiển; đây chưa phải định vị chỉ từ camera. Low-texture setup failure không chứng minh tránh được vật thể ít texture. Khi setup chưa chạy planner, các checks goal/planner thiếu evidence. Unknown space không được coi là đã chứng minh trống.','', 'Xem `trials.csv`, `independent_audit.json` và `trials/` để kiểm tra mỗi lượt.']
    (out/'REPORT_VI.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    for j,(key,label) in enumerate((('actual_p95','Actual horizontal p95 (m/s)'),('wall_s','Control wall time (s)'),('clearance_m','Center to box surface (m)'))):
        for i,speed in enumerate(speeds):
            group=[r for r in records if r['speed_setting']==speed and r['accepted']]
            axes[j].scatter(np.full(len(group),i),[r[key] for r in group],alpha=.7)
        axes[j].set_xticks(range(len(speeds)),[str(s) for s in speeds]);axes[j].set_xlabel('Per-axis speed setting (m/s)');axes[j].set_ylabel(label);axes[j].grid(alpha=.2)
    axes[2].axhline(.75,color='red',linestyle='--',label='Acceptance');axes[2].legend()
    fig.suptitle('Accepted trials only; all failures remain in the report denominator')
    fig.tight_layout();fig.savefig(out/'speed_comparison.png',dpi=180);plt.close(fig)
    for script in ('audit_monocular_speed_suite.py','report_monocular_speed_suite.py'):
        shutil.copy2(Path(__file__).parent/script,out/script)
    names=('plan.json','summary.json','independent_audit.json','trials.csv','REPORT_VI.md','audit_monocular_speed_suite.py','report_monocular_speed_suite.py')
    provenance=dict(run_id=args.run_id,commit=args.commit,input_root=str(args.run_root.resolve()),audit_note='Consumes the existing audit; the copied auditor is a code reference. This report command does not re-run the audit.',files={n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in names})
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(out/'REPORT_VI.md')

if __name__=='__main__':main()

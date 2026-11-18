"""Read-only envelope analysis; never reinterpret a short transient as steady speed."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from .vehicle import roll_pitch


def dwell_index(mask, count):
    mask = np.asarray(mask,dtype=bool)
    for k in range(max(0,len(mask)-count+1)):
        if mask[k:k+count].all():
            return k
    return None


def analyze(rows, plan, profile):
    result=[]
    for phase in plan['phases']:
        data=[r for r in rows if r['phase']==phase['name']]
        if not data:
            continue
        t=np.array([r['elapsed_s'] for r in data])
        velocities=np.array([r['after']['velocity_world'] for r in data])
        positions=np.array([r['after']['positions_world'] for r in data])
        req=np.array([[d['requested_acceleration_world'] for d in r['diagnostics']] for r in data])
        sg=np.array([[d['speed_governed_acceleration_world'] for d in r['diagnostics']] for r in data])
        gov=np.array([[d['governed_acceleration_world'] for d in r['diagnostics']] for r in data])
        acc=np.array([r['estimated_acceleration_world'] for r in data])
        force=np.array([r['forces_N'] for r in data])
        allocated=np.array([[d['allocated_thrust_N'] for d in r['diagnostics']] for r in data])
        wrench=np.array([[d['requested_wrench'] for d in r['diagnostics']] for r in data])
        achieved=np.array([[d['allocated_wrench'] for d in r['diagnostics']] for r in data])
        entry=np.array(data[0]['before']['positions_world'])
        entry_v=np.array(data[0]['before']['velocity_world'])
        agents=[]
        for i in range(4):
            speed=np.linalg.norm(velocities[:,i],axis=1)
            tilt=[np.max(np.abs(np.rad2deg(roll_pitch(r['after']['quaternion_xyzw'][i])))) for r in data]
            metrics=dict(agent=i,speed_max_m_s=float(speed.max()),
                world_velocity_min_m_s=velocities[:,i].min(0).tolist(),world_velocity_max_m_s=velocities[:,i].max(0).tolist(),
                speed_governor_fraction=float(np.mean(np.any(np.abs(req[:,i]-sg[:,i])>1e-9,axis=1))),
                acceleration_clip_fraction=float(np.mean(np.any(np.abs(sg[:,i]-gov[:,i])>1e-9,axis=1))),
                measured_minus_governed_acceleration_norm_rms_m_s2=float(np.sqrt(np.mean(np.sum((acc[:,i]-gov[:,i])**2,axis=1)))),
                measured_acceleration_norm_p95_m_s2=float(np.quantile(np.linalg.norm(acc[:,i],axis=1),.95)),
                measured_acceleration_world_min_m_s2=acc[:,i].min(0).tolist(),
                measured_acceleration_world_max_m_s2=acc[:,i].max(0).tolist(),
                allocation_force_error_norm_rms_N=float(np.sqrt(np.mean(np.sum((achieved[:,i,:3]-wrench[:,i,:3])**2,axis=1)))),
                allocation_torque_error_norm_rms_Nm=float(np.sqrt(np.mean(np.sum((achieved[:,i,3:]-wrench[:,i,3:])**2,axis=1)))),
                actuator_command_minus_allocated_rms_N=float(np.sqrt(np.mean((force[:,i]-allocated[:,i])**2))),
                allocated_at_bound_fraction=float(np.mean(np.max(np.abs(allocated[:,i]),axis=1)>=profile['test_thrust_limit_N']-1e-4)),
                max_command_thrust_N=float(np.max(np.abs(force[:,i]))),max_roll_pitch_deg=float(max(tilt)))
            if plan['case']!='track' and phase['name'] in ('positive','negative'):
                axis='xyz'.index(plan['case']); sign=1 if phase['name']=='positive' else -1
                axial=sign*velocities[:,i,axis]
                tail=axial[-50:]
                slope=float(np.polyfit(t[-len(tail):],tail,1)[0]) if len(tail)>1 else None
                steady=bool(len(data)==phase['ticks'] and len(tail)==50 and np.ptp(tail)<=plan['steady_speed_span_m_s']
                    and slope is not None and abs(slope)<=plan['steady_slope_m_s2'])
                reached=dwell_index(axial>=.9*profile['control']['speed_limit_m_s'],25)
                metrics.update(axial_tail_mean_m_s=float(tail.mean()),axial_tail_std_m_s=float(tail.std()),
                    axial_tail_span_m_s=float(np.ptp(tail)),axial_tail_slope_m_s2=slope,
                    steady_observed=steady,steady_axial_speed_m_s=float(tail.mean()) if steady else None,
                    time_to_90percent_configured_speed_s=None if reached is None else float((reached+1)*.02))
            if phase['mode']=='brake':
                stopped=dwell_index(speed<=plan['stop_speed_m_s'],25)
                end=len(data)-1 if stopped is None else stopped
                route=np.concatenate([entry[i][None],positions[:end+1,i]])
                direction=entry_v[i]/max(np.linalg.norm(entry_v[i]),1e-12)
                metrics.update(stop_observed=stopped is not None,
                    stop_threshold_m_s=plan['stop_speed_m_s'], stop_dwell_s=plan['stop_dwell_s'],
                    stop_onset_time_s=None if stopped is None else float((stopped+1)*.02),
                    stop_confirmed_time_s=None if stopped is None else float((stopped+25)*.02),
                    braking_path_to_stop_m=None if stopped is None else float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),
                    observed_phase_path_m=float(np.linalg.norm(np.diff(np.concatenate([entry[i][None],positions[:,i]]),axis=0),axis=1).sum()),
                    forward_excursion_over_brake_window_m=float(np.maximum(0,(positions[:,i]-entry[i])@direction).max()),
                    censored_window_s=len(data)*.02 if stopped is None else None)
            if phase['name']=='negative' and plan['case']!='track':
                axis='xyz'.index(plan['case'])
                reverse=dwell_index(velocities[:,i,axis]<=-.05,25)
                metrics.update(negative_velocity_observed=reverse is not None,
                    negative_velocity_onset_s=None if reverse is None else (reverse+1)*.02,
                    reversal_entry_velocity_m_s=entry_v[i].tolist())
            if phase['mode']=='waypoint':
                targets=np.array([r['targets_world'][i] for r in data])
                starts=np.array([r['segment_start_world'][i] for r in data])
                segment=targets-starts
                u=np.clip(np.sum((positions[:,i]-starts)*segment,axis=1)/np.maximum(np.sum(segment*segment,axis=1),1e-12),0,1)
                path_error=np.linalg.norm(positions[:,i]-(starts+u[:,None]*segment),axis=1)
                target_error=np.linalg.norm(positions[:,i]-targets,axis=1)
                v_ref=np.array([r['desired_velocity_world'][i] for r in data])
                metrics.update(path_error_rms_m=float(np.sqrt(np.mean(path_error**2))),path_error_max_m=float(path_error.max()),
                    endpoint_target_error_m=float(target_error[-1]),
                    velocity_tracking_error_rms_m_s=float(np.sqrt(np.mean(np.sum((velocities[:,i]-v_ref)**2,axis=1)))))
                along=np.sum((positions[:,i]-targets)*segment,axis=1)/np.maximum(np.linalg.norm(segment,axis=1),1e-12)
                metrics['overshoot_beyond_segment_endpoint_m']=float(np.maximum(along,0).max())
            agents.append(metrics)
        result.append(dict(phase=phase['name'],observed_ticks=len(data),expected_ticks=phase['ticks'],agents=agents))
    return result


def write_report(run):
    run=Path(run)
    names=['summary.json','program.json','vehicle_profile.json','scenario.json','control.jsonl']
    hashes={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in names}
    summary=json.loads((run/'summary.json').read_text())
    plan=json.loads((run/'program.json').read_text())
    profile=json.loads((run/'vehicle_profile.json').read_text())
    rows=[json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    expected=[p['name'] for p in plan['phases'] for _ in range(p['ticks'])]
    for k,r in enumerate(rows):
        if r['tick']!=k+1 or r['phase']!=expected[k] or r['clock_ns']!=r['requested_clock_ns']:
            raise ValueError('Invalid tick/phase/clock evidence')
        stamp=r['after']['stamp_ns']
        if not r['clock_ns']-2_000_000<=stamp<=r['clock_ns']:
            raise ValueError('Invalid endpoint timestamp')
        if np.max(np.abs(r['forces_N']))>profile['test_thrust_limit_N']+1e-8:
            raise ValueError('Thrust outside recorded profile')
    report=run/'report'
    report.mkdir(exist_ok=False)
    complete=bool(summary.get('engineering_acceptance_passed') and len(rows)==plan['total_ticks'])
    result=dict(schema='crk.envelope.report.v1',case=plan['case'],engineering_run_complete=complete,
        unrestricted_hardware_envelope_established=False,performance_claim=False,
        measurement_method='finite difference of filtered odometry world velocity; allocated/commanded wrench are model-derived, not force-sensor measurements',
        phases=analyze(rows,plan,profile),input_hashes=hashes)
    (report/'envelope_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    with (report/'response.csv').open('x',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['elapsed_s','phase','agent','x','y','z','vx','vy','vz',
            'requested_ax','requested_ay','requested_az','speed_governed_ax','speed_governed_ay','speed_governed_az',
            'governed_ax','governed_ay','governed_az','estimated_ax','estimated_ay','estimated_az',
            'allocation_weighted_residual','max_command_thrust_N'])
        for r in rows:
            for i,d in enumerate(r['diagnostics']):
                writer.writerow([r['elapsed_s'],r['phase'],i,*r['after']['positions_world'][i],
                    *r['after']['velocity_world'][i],*d['requested_acceleration_world'],
                    *d['speed_governed_acceleration_world'],*d['governed_acceleration_world'],
                    *r['estimated_acceleration_world'][i],d['allocation_residual'],max(abs(x) for x in r['forces_N'][i])])
    lines=['# 当前仿真模型可达能力报告', '',f'case={plan["case"]}；完整工程运行={complete}。', '',
        '本报告描述当前软件调节与物理模型共同形成的闭环包线，未测得解除调节后的硬件极限。',
        '参数、艇体、附加质量与推力上限未改变；无任务、无训练、无学习策略。', '',
        '## 观测结果', '', '| 阶段 | 艇 | 峰值速度 m/s | 观测到稳态速度 m/s | 制动时间 s | 制动距离 m | 速度调节比例 | 分配触边比例 |',
        '| --- | --- | --- | --- | --- | --- | --- | --- |']
    def fmt(v): return '未达到/不适用' if v is None else f'{v:.4f}'
    for phase in result['phases']:
        for a in phase['agents']:
            lines.append(f'| {phase["phase"]} | {a["agent"]} | {fmt(a["speed_max_m_s"])} | {fmt(a.get("steady_axial_speed_m_s"))} | {fmt(a.get("stop_onset_time_s"))} | {fmt(a.get("braking_path_to_stop_m"))} | {a["speed_governor_fraction"]:.3f} | {a["allocated_at_bound_fraction"]:.3f} |')
    lines += ['', '## 如何解释未实现的请求', '',
        '- requested→speed_governed的差异来自当前0.35 m/s速度反馈调节；speed_governed→governed的差异来自每轴加速度裁剪。',
        '- requested_wrench→allocated_wrench反映带推力边界的分配误差，分别统计力N与力矩Nm；触边不等于所有轴都不可达。',
        '- allocated_thrust→实际发布forces反映已有一阶响应与变化率限制；commanded_wrench只是安装矩阵推导值，不是实测流体作用力。',
        '- measured_acceleration−governed包含执行器、动力学/姿态耦合、控制误差与滤波有限差分误差，不能单独认定为水动力参数错误。',
        '- 零加速度阶段仍有阻力/重浮力补偿，因此不能当作零推力滑行；主动制动阶段请求为−1.2倍世界速度。',
        '- 稳态：完整阶段末1 s有50点，轴向速度极差≤0.03 m/s且线性斜率≤0.02 m/s²；不满足则无稳态结论。',
        '- 制动：三维速度≤0.05 m/s连续0.5 s；时间/路程取该保持段起点，确认时间另存。未满足时null，并记录观察窗口。',
        '- 路径误差相对相邻命令航点的有限三维线段；转弯不等于航向转艏测试，当前姿态控制仍稳定世界零航向。',
        '- 四艇相同艇型、相同激励并行，不能解释为四种硬件或独立统计重复。无接触消息不证明严格无接触。',
        '- 正反向均在各自制动后开始；记录负速度建立时间，不能称作带初速度的直接反推试验。',
        '', '详细速度、加速度、分配/电机误差、姿态、路径误差和制动窗口见envelope_metrics.json；原始记录为../control.jsonl。']
    (report/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    assert hashes=={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in names}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    write_report(parser.parse_args().run)

"""Read-only metrics and preregistered prefix comparison; never rerun a task."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from .reassignment_contract import SCHEMA, BUDGET, PREFIX_CRITERIA
from .scenario import Frame


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    return [json.loads(s) for s in path.read_text(encoding='utf-8').splitlines() if s.strip()] if path.exists() else []


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def hashes(directory):
    # Exclude generated metrics/reports; compare every raw input before/after.
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(directory.iterdir())
            if p.is_file() and p.name not in ('diagnostic_metrics.json', 'DIAGNOSTIC_REPORT.md', 'trajectory_xy.svg')
            and p.suffix in ('.json', '.jsonl')}


def values(x):
    return x['values'] if isinstance(x, dict) and 'values' in x else x


def project(point, route):
    """Nearest projection on finite polyline; earliest segment breaks ties."""
    route = np.asarray(route, dtype=float)
    if len(route) == 1:
        return 0., float(np.linalg.norm(point-route[0]))
    best, along, total = (float('inf'), 0.), 0., 0.
    for a, b in zip(route[:-1], route[1:]):
        delta = b-a
        length = float(np.linalg.norm(delta))
        fraction = float(np.clip(np.dot(point-a, delta)/length**2, 0, 1)) if length else 0.
        distance = float(np.linalg.norm(point-a-fraction*delta))
        if distance < best[0]:
            best = distance, total+fraction*length
        total += length
    return best[1], best[0]


def run_metrics(output):
    before = hashes(output)
    summary = read(output/'summary.json')
    transitions, control = rows(output/'transitions.jsonl'), rows(output/'control.jsonl')
    result = dict(schema=SCHEMA, condition=summary.get('diagnostic_condition'),
        diagnostic_valid=summary.get('diagnostic_valid', False),
        termination_reason=summary.get('termination_reason'),
        completed_task_steps=summary.get('completed_task_steps', 0),
        window='motion21..32, after post-motion20 commit; at most 2.4 s',
        limitations=['No completion/performance claim; 2.4 s may not resolve braking/arrival.',
            'Polyline projection is geometric, may be ambiguous on overlapping routes.',
            'Scans/map cell changes are not thesis search coverage.',
            'Acceleration is filtered-odometry finite difference, not independent IMU truth.',
            'Canonical allocation hashes are retained without quantization; earlier cross-platform issue is not declared resolved.'],
        agents={}, raw_input_sha256=before)
    by_step = {r['step']: r for r in transitions}
    if 20 in by_step and (output/'retained_reference.json').exists():
        ref = read(output/'retained_reference.json')
        samples = [r for r in control if 20 < r['task_step'] <= BUDGET]
        boundary = [r for r in control if r['task_step'] == 20][-1]
        result['observed_post_boundary_s'] = len(samples)*.02
        result['map_delta'] = {k: by_step[max(by_step)]['maps'][k]-by_step[20]['maps'][k]
                               for k in by_step[20]['maps']}
        for i in range(3):
            # Native and Gazebo coordinates differ by translation only in this frame.
            p0 = np.asarray(boundary['positions_world'][i])
            if Frame().yaw != 0:
                raise ValueError('v1 metrics require the original translation-only frame')
            shift = np.asarray(Frame().translation)
            fixed = np.asarray(ref['fixed_tracking_points'][i])+shift
            route = [p0, *[np.asarray(p)+shift for p in ref['remaining_paths'][str(i)]]]
            ps = np.asarray([p0, *[r['positions_world'][i] for r in samples]])
            vs = np.asarray([boundary['velocity_world'][i], *[r['velocity_world'][i] for r in samples]])
            speeds = np.linalg.norm(vs, axis=1)
            reference_speed = float(speeds[0])
            heading = vs[0]/reference_speed if reference_speed > .05 else None
            projections = [project(p, route) for p in ps]
            fixed_distance = np.linalg.norm(ps-fixed, axis=1)
            errors, current_distance, accerr, reqerr = [], [], [], []
            for r in samples:
                t = by_step[r['task_step']]
                guide = t['applied_before_motion']['guidance']['agent_assignments'][i]
                index = t['applied_before_motion']['tracker'][str(i)]['snapshot']['next_index']
                path = guide['planned_path']
                target = guide['hold_position'] if guide['hold_state'] else guide['tracking_waypoint']
                start = path[max(0, index-1)] if path else target
                p = np.asarray(r['positions_world'][i])-shift
                errors.append(project(p, [start, target])[1])
                current_distance.append(float(np.linalg.norm(p-target)))
                d = r['diagnostics'][i]
                estimate = np.asarray(r['estimated_acceleration_world'][i])
                accerr.append(float(np.linalg.norm(estimate-np.asarray(d['governed_acceleration_world']))))
                reqerr.append(float(np.linalg.norm(estimate-np.asarray(d['requested_acceleration_world']))))
            reverse = [] if heading is None else [float(k*.02) for k,v in enumerate(vs[1:],1) if np.dot(v,heading) < -.05]
            angles = [] if heading is None else [None if np.linalg.norm(v) <= .05 else
                float(np.rad2deg(np.arccos(np.clip(np.dot(v,heading)/np.linalg.norm(v), -1, 1)))) for v in vs[1:]]
            turns = [float((k+1)*.02) for k,a in enumerate(angles) if a is not None and a >= 90.]
            decel = np.diff(speeds)/.02 < -.02
            result['agents'][str(i)] = dict(fixed_reference_native=ref['fixed_tracking_points'][i],
                fixed_distance_start_m=float(fixed_distance[0]), fixed_distance_end_m=float(fixed_distance[-1]),
                fixed_distance_reduction_m=float(fixed_distance[0]-fixed_distance[-1]),
                retained_path_projected_advance_m=float(projections[-1][0]-projections[0][0]),
                retained_path_error_rms_m=float(np.sqrt(np.mean(np.square([p[1] for p in projections])))),
                cumulative_distance_m=float(np.linalg.norm(np.diff(ps,axis=0),axis=1).sum()),
                net_displacement_m=float(np.linalg.norm(ps[-1]-ps[0])),
                entry_speed_m_s=reference_speed, end_speed_m_s=float(speeds[-1]),
                deceleration_duration_s=float(decel.sum()*.02),
                first_deceleration_s=next((float((k+1)*.02) for k,d in enumerate(decel) if d), None),
                first_90deg_turn_s=turns[0] if turns else None,
                first_reverse_s=reverse[0] if reverse else None,
                reverse_duration_s=len(reverse)*.02,
                turn_angle_from_entry_deg=angles,
                current_tracking_distance_series_m=current_distance,
                applied_finite_segment_error_rms_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                acceleration_governed_error_rms_m_s2=float(np.sqrt(np.mean(np.square(accerr)))) if accerr else None,
                acceleration_requested_error_rms_m_s2=float(np.sqrt(np.mean(np.square(reqerr)))) if reqerr else None)
        result['motion_definitions'] = dict(deceleration='speed finite difference < -0.02 m/s², 20 ms samples',
            heading='entry world velocity at t20, defined only for speed >0.05 m/s',
            turn='first speed>0.05 m/s direction at least 90° from entry heading',
            reverse='velocity dot entry heading < -0.05 m/s; elapsed time relative to boundary',
            path_error='distance to active finite segment selected by tracker index BEFORE this task motion',
            fixed_reference='boundary t20 position and PRE-commit tracking point/path, identical geometric reference required in pair')
    if before != hashes(output):
        raise RuntimeError('Raw inputs changed during report')
    result['raw_inputs_unchanged'] = True
    return result


def write_run_report(output):
    value = run_metrics(output)
    write(output/'diagnostic_metrics.json', value)
    (output/'DIAGNOSTIC_REPORT.md').write_text(
        '# D2 search commit diagnostic v1\n\n'
        f"Condition: {value['condition']}; stop: {value['termination_reason']}; "
        f"diagnostic_valid={value['diagnostic_valid']}.\n\n"
        'This is an externally stopped 32-step diagnostic, not task success or a full G6 pass.\n'
        'Compare a separate baseline/defer pair with the preregistered prefix gate before interpreting differences.\n\n'
        'Motion window is steps21–32 (2.4 s); retained references are captured after motion20, before its planning commit.\n'
        'Absent turn/reverse timestamps mean not observed/undefined, not proven impossible.\n'
        'See diagnostic_metrics.json for definitions, all per-agent metrics and raw input hashes.\n', encoding='utf-8')
    trajectory_svg(output, output/'trajectory_xy.svg')
    return value


def route_signature(guidance):
    keys = ('agent_id','role','assignment_kind','assignment_id','final_waypoint','planned_path',
            'tracking_waypoint','hold_state','reachable','execution_request')
    return [[g.get(k) for k in keys]+([g['hold_position']] if g['hold_state'] else [])
            for g in guidance['agent_assignments']]


def compare(a, b):
    """Reject inadequate prefix rather than fit tolerances to a desired result."""
    sa, sb = read(a/'summary.json'), read(b/'summary.json')
    issues = []
    if (sa.get('diagnostic_condition'), sb.get('diagnostic_condition')) != ('baseline32','defer_search_reassignment32'):
        issues.append('wrong_condition_order')
    for key in ('diagnostic_schema','scenario_sha256','vehicle_profile_sha256','source_files_sha256',
                'adapter_manifest_sha256','prefix_criteria','task_dt_s','control_dt_s','physics_dt_s'):
        if key not in sa or sa.get(key) != sb.get(key):
            issues.append('metadata_'+key)
    if sa.get('prefix_criteria') != PREFIX_CRITERIA or sa.get('diagnostic_schema') != SCHEMA:
        issues.append('unknown_preregistered_contract')
    for name in ('scenario.json','resolved_config.json','role_specs.json','vehicle_profile.json'):
        if not (a/name).exists() or not (b/name).exists() or read(a/name) != read(b/name):
            issues.append('config_'+name)
    ca, cb = rows(a/'control.jsonl'), rows(b/'control.jsonl')
    ta, tb = rows(a/'transitions.jsonl'), rows(b/'transitions.jsonl')
    ia, ib = rows(a/'initial.jsonl'), rows(b/'initial.jsonl')
    maxima = dict(position_m=0., velocity_m_s=0., quaternion_deg=0., acceleration_m_s2=0.)
    hash_mismatch = []
    if len(ia)!=1 or len(ib)!=1 or len(ta)<20 or len(tb)<20 or len(ca)<200 or len(cb)<200:
        issues.append('incomplete_prefix')
    else:
        for k,(left,right) in enumerate(zip(ia+ta[:19], ib+tb[:19])):
            if route_signature(left['guidance']) != route_signature(right['guidance']):
                issues.append(f'guidance_route_step{k}')
            if left['task'] != right['task']:
                issues.append(f'task_state_step{k}')
            if left['maps'] != right['maps']:
                issues.append(f'map_counts_step{k}')
            if left['guidance']['allocation_hash'] != right['guidance']['allocation_hash']:
                hash_mismatch.append(k)
        for k,(left,right) in enumerate(zip([ia[0]]+ca[:200], [ib[0]]+cb[:200])):
            ls,rs = left['body_states'],right['body_states']
            dp = float(np.max(np.linalg.norm(np.array([s['position'] for s in ls])-np.array([s['position'] for s in rs]),axis=1)))
            # Compare WORLD velocities, not body vectors under different orientations.
            dv = float(np.max(np.linalg.norm(np.asarray(left.get('velocity_world', [[0,0,0]]*4))-
                                             np.asarray(right.get('velocity_world', [[0,0,0]]*4)),axis=1)))
            if k==0:
                # Body velocity is zero only nominally; rotate measured initial states.
                from .vehicle import rotation
                vl = [rotation(s['quaternion'])@np.asarray(s['velocity_body']) for s in ls]
                vr = [rotation(s['quaternion'])@np.asarray(s['velocity_body']) for s in rs]
                dv = float(np.max(np.linalg.norm(np.asarray(vl)-np.asarray(vr),axis=1)))
            dot = [abs(float(np.dot(l['quaternion'], r['quaternion']))) for l,r in zip(ls,rs)]
            angle = float(np.max(np.rad2deg(2*np.arccos(np.clip(dot,0,1)))))
            maxima['position_m'] = max(maxima['position_m'],dp)
            maxima['velocity_m_s'] = max(maxima['velocity_m_s'],dv)
            maxima['quaternion_deg'] = max(maxima['quaternion_deg'],angle)
            if dp > PREFIX_CRITERIA['initial_position_max_m' if k==0 else 'prefix_position_max_m']:
                issues.append(f'position_sample{k}')
            if dv > PREFIX_CRITERIA['initial_velocity_max_m_s' if k==0 else 'prefix_velocity_max_m_s']:
                issues.append(f'velocity_sample{k}')
            if angle > PREFIX_CRITERIA['quaternion_angle_max_deg']:
                issues.append(f'orientation_sample{k}')
            if k:
                for value, metadata in ((left,sa),(right,sb)):
                    if value['clock_ns']-metadata['task_epoch_ns'] != k*20_000_000:
                        issues.append(f'physical_time_sample{k}')
                if (left['task_step'],left['substep']) != (right['task_step'],right['substep']):
                    issues.append(f'control_order_sample{k}')
                la = [d['requested_acceleration_world'] for d in left['diagnostics']]
                ra = [d['requested_acceleration_world'] for d in right['diagnostics']]
                da = float(np.max(np.linalg.norm(np.asarray(la)-np.asarray(ra),axis=1)))
                maxima['acceleration_m_s2'] = max(maxima['acceleration_m_s2'],da)
                if da > PREFIX_CRITERIA['requested_acceleration_max_m_s2']:
                    issues.append(f'request_sample{k}')
        for name in ('fixed_tracking_points','remaining_paths','assignments'):
            if not (a/'retained_reference.json').exists() or not (b/'retained_reference.json').exists() or (
                    read(a/'retained_reference.json')[name] != read(b/'retained_reference.json')[name]):
                issues.append('retained_reference_'+name)
    valid_runs = sa.get('diagnostic_valid',False) and sb.get('diagnostic_valid',False)
    if not valid_runs:
        issues.append('invalid_or_incomplete_run')
    if not sb.get('search_commits_deferred',0):
        issues.append('no_search_commit_was_deferred')
    result = dict(schema=SCHEMA, prefix_criteria=PREFIX_CRITERIA, prefix_maxima=maxima,
        prefix_comparable=not issues, issues=issues, allocation_hash_mismatch_prefix_steps=hash_mismatch,
        conclusion='UNRESOLVED: compare motion metrics; short window cannot establish mission success.' if not issues else
                   'INSUFFICIENT_CONTROL: prefix or run validity failed; no causal attribution.',
        baseline=run_metrics(a), defer=run_metrics(b),
        executor_coupling='Native executor decisions remain enabled. Raw joint proposals precede the gate; '
            'mixed applied scores keep that executor fixed. Subsequent decision/event feedback can differ.')
    metric_names=('fixed_distance_reduction_m','retained_path_projected_advance_m','cumulative_distance_m',
                  'applied_finite_segment_error_rms_m','acceleration_governed_error_rms_m_s2',
                  'acceleration_requested_error_rms_m_s2','deceleration_duration_s','reverse_duration_s')
    result['differences_defer_minus_baseline']={}
    for agent,left in result['baseline']['agents'].items():
        right=result['defer']['agents'].get(agent,{})
        result['differences_defer_minus_baseline'][agent]={k:right[k]-left[k] for k in metric_names
            if left.get(k) is not None and right.get(k) is not None}
    result['difference_interpretation']='Comparable engineering contrast only; no mission-level causal conclusion.' if not issues else 'Descriptive only; inadequate control.'
    return result


def trajectory_svg(directory, target):
    control = rows(directory/'control.jsonl')
    colors = ('#2563eb','#059669','#e87906')
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="720" height="540" viewBox="0 0 720 540">',
             '<rect width="720" height="540" fill="white"/>',
             '<text x="25" y="24" font-size="16">World XY trajectory (m); colored = motion21..32</text>']
    # Full scene, equal scale; no misleading auto-zoom.
    for x in range(-10,11,5):
        parts.append(f'<text x="{360+x*22}" y="510" font-size="12">{x}</text>')
    parts.append('<rect x="140" y="50" width="440" height="440" fill="none" stroke="#aaa"/>')
    for i,color in enumerate(colors):
        for window,stroke in ((control,'#aab3bd'),([r for r in control if r['task_step']>20],color)):
            points=' '.join(f"{360+r['positions_world'][i][0]*22:.3f},{270-r['positions_world'][i][1]*22:.3f}" for r in window)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{stroke}" stroke-width="2"/>')
        parts.append(f'<text x="590" y="{80+i*25}" fill="{color}" font-size="14">auv_{i}</text>')
    parts.append('</svg>')
    target.write_text('\n'.join(parts)+'\n', encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--defer',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    root=Path(os.environ.get('CRK_OUTPUT_ROOT','/home/legion/AUV-simulation')).resolve()/'results/ch3'
    for path in (args.baseline,args.defer):
        if not path.resolve().is_relative_to(root):
            parser.error('Input runs must be in mounted host results/ch3')
    if args.output:
        destination=args.output.resolve()
        if not destination.is_relative_to(root):
            parser.error('Report output must be in mounted host results/ch3')
        destination.mkdir(exist_ok=False)
    else:
        destination=Path(tempfile.mkdtemp(prefix='d2_pair_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_',dir=root))
    destination.chmod(0o755)
    result=compare(args.baseline,args.defer)
    write(destination/'comparison.json',result)
    lines=['| Agent | Δ fixed-target progress (m) | Δ retained-path advance (m) | Δ distance (m) |',
           '| --- | ---: | ---: | ---: |']
    for agent,diff in result['differences_defer_minus_baseline'].items():
        lines.append(f"| auv_{agent} | {diff['fixed_distance_reduction_m']:.6f} | {diff['retained_path_projected_advance_m']:.6f} | {diff['cumulative_distance_m']:.6f} |")
    (destination/'REPORT.md').write_text('# D2 paired diagnostic v1\n\n'+result['conclusion']+'\n\n'
        f"prefix_comparable={result['prefix_comparable']}; issues={result['issues']}.\n\n"
        +'\n'.join(lines)+'\n\nΔ = defer minus baseline. '+result['difference_interpretation']+'\n\n'
        'Metrics and definitions: comparison.json. No full-task/G6 or learned-model claim.\n',encoding='utf-8')
    if getattr(os,'geteuid',lambda:1)()==0 and os.environ.get('CRK_OUTPUT_UID','').isdigit() and os.environ.get('CRK_OUTPUT_GID','').isdigit():
        uid,gid=int(os.environ['CRK_OUTPUT_UID']),int(os.environ['CRK_OUTPUT_GID'])
        for path in [*destination.iterdir(),destination]:
            os.chown(path,uid,gid,follow_symlinks=False)
    print(f"Pair report: {destination}; prefix_comparable={result['prefix_comparable']}")
    raise SystemExit(0 if result['prefix_comparable'] else 1)


if __name__=='__main__':
    main()

"""Reports for explicit progress_commit_v1 runs; read-only input comparison."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from .progress_commit import SCHEMA, SEARCH_CONDITIONS, FIXTURE_CONDITIONS, prefix_criteria
from .reassignment_contract import PREFIX_CRITERIA
from .reassignment_report import rows, read, route_signature, run_metrics
from .scenario import Frame
from .vehicle import rotation


def write(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def raw_hashes(path):
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(path.iterdir()) if p.is_file()
        and p.suffix in ('.json','.jsonl') and p.name not in ('progress_metrics.json','comparison.json')}


def metrics(directory):
    before=raw_hashes(directory);summary=read(directory/'summary.json')
    condition=summary['diagnostic_condition'];fixture=condition in FIXTURE_CONDITIONS
    if summary.get('diagnostic_schema')!=SCHEMA or condition not in SEARCH_CONDITIONS+FIXTURE_CONDITIONS:
        raise ValueError('Progress v1 inputs required')
    control=rows(directory/'control.jsonl');transitions=rows(directory/('cycles.jsonl' if fixture else 'transitions.jsonl'))
    events=[event for row in rows(directory/('cycles.jsonl' if fixture else 'planning.jsonl')) for event in row.get('queue_events',[])]
    committed=[e for e in events if e['kind']=='COMMITTED'];waiting=[e for e in events if e['kind']=='WAITING']
    by_step={row['cycle' if fixture else 'step']:row for row in transitions}
    fresh_matches=[]
    for event in committed:
        row=by_step.get(event['committed_step'])
        if row is None:
            fresh_matches.append(False);continue
        measured=np.asarray(row['body_states'][event['agent']]['position'])-Frame().translation
        allocation=row['program']['allocation'] if fixture else row['allocation']
        assignment=next(s for s in allocation['search_assignments'] if s['agent_id']==event['agent'])
        fresh_matches.append(bool(np.allclose(assignment['path'][0],measured,atol=1e-6,rtol=0)))
    valid_clock=all(row['task_step']==k//10+1 and row['substep']==k%10+1 and
        row['clock_ns']==row['requested_clock_ns']==row['state_ns']==summary.get('task_epoch_ns',0)+(k+1)*20_000_000
        for k,row in enumerate(control))
    complete=len(control)==320 and len(transitions)==32 and valid_clock
    result=dict(schema=SCHEMA,condition=condition,fixture=fixture,diagnostic_valid=summary.get('diagnostic_valid',False),
        complete_records=complete,termination_reason=summary.get('termination_reason'),failure_tags=summary.get('failure_tags',[]),
        committed=committed,waiting_agents=sorted({e['agent'] for e in waiting}),
        release_observed=any(e['committed_step']>e['proposed_step'] for e in committed),fresh_start_matches_measured=all(fresh_matches) if committed else None,
        raw_input_sha256=before,no_mission_success_claim=True,
        interpretation='Synthetic proposal and waypoint reference; only commit mechanics evidence.' if fixture else
                       'Original scene D2+ZERO; arrival/release may remain unobserved in 32 steps.')
    if not fixture:
        base=run_metrics(directory)
        result.update(window_metrics=base['agents'],map_delta=base.get('map_delta'),
            native_vs_adapter_diagnostics='Native result retains native policy diagnostics; delayed applied result has explicitly adapter-labelled diagnostics.')
    if before!=raw_hashes(directory):raise RuntimeError('Raw inputs changed during analysis')
    result['raw_inputs_unchanged']=True
    return result


def write_run_report(directory):
    value=metrics(directory)
    write(directory/'progress_metrics.json',value)
    text=(f"# progress_commit_v1\n\nCondition: {value['condition']}; stop: {value['termination_reason']}; "
          f"diagnostic_valid={value['diagnostic_valid']}; complete_records={value['complete_records']}.\n\n"
          f"release_observed={value['release_observed']}; fresh_start_matches_measured={value['fresh_start_matches_measured']}.\n\n"
          +value['interpretation']+'\n\nNo full G6/search or learned-model acceptance. '
          'A missing release within the 32-cycle window is unobserved, never inferred as passed.\n')
    (directory/'PROGRESS_REPORT.md').write_text(text,encoding='utf-8')
    return value


def _guide(row,fixture):return row['program']['guidance'] if fixture else row['guidance']


def compare(a,b):
    sa,sb=read(a/'summary.json'),read(b/'summary.json')
    fixture=sa.get('diagnostic_condition') in FIXTURE_CONDITIONS
    expected=FIXTURE_CONDITIONS if fixture else SEARCH_CONDITIONS
    issues=[]
    if (sa.get('diagnostic_condition'),sb.get('diagnostic_condition'))!=expected:issues.append('wrong_condition_order')
    for key in ('diagnostic_schema','scenario_sha256','vehicle_profile_sha256','source_files_sha256',
                'adapter_manifest_sha256','prefix_criteria','task_dt_s','control_dt_s','physics_dt_s'):
        if key not in sa or sa[key]!=sb.get(key):issues.append('metadata_'+key)
    if sa.get('diagnostic_schema')!=SCHEMA or sa.get('prefix_criteria')!=prefix_criteria(sa.get('diagnostic_condition')):issues.append('unknown_contract')
    for name in ('scenario.json','vehicle_profile.json')+(() if fixture else ('resolved_config.json','role_specs.json')):
        if not (a/name).exists() or not (b/name).exists() or read(a/name)!=read(b/name):issues.append('config_'+name)
    ca,cb=rows(a/'control.jsonl'),rows(b/'control.jsonl')
    ta,tb=rows(a/('cycles.jsonl' if fixture else 'transitions.jsonl')),rows(b/('cycles.jsonl' if fixture else 'transitions.jsonl'))
    ia,ib=rows(a/'initial.jsonl'),rows(b/'initial.jsonl')
    boundary=5 if fixture else 20
    maxima=dict(position_m=0.,velocity_m_s=0.,quaternion_deg=0.,request_m_s2=0.)
    hash_differences=[]
    if len(ia)!=1 or len(ib)!=1 or len(ta)<boundary or len(tb)<boundary or len(ca)<10*boundary or len(cb)<10*boundary:
        issues.append('incomplete_prefix')
    else:
        for step,(left,right) in enumerate(zip(ia+ta[:boundary-1],ib+tb[:boundary-1])):
            lg,rg=_guide(left,fixture),_guide(right,fixture)
            if route_signature(lg)!=route_signature(rg):issues.append(f'guidance_step{step}')
            if lg['allocation_hash']!=rg['allocation_hash']:hash_differences.append(step)
            if not fixture and (left['task']!=right['task'] or left['maps']!=right['maps']):issues.append(f'public_state_step{step}')
        for k,(left,right) in enumerate(zip([ia[0]]+ca[:10*boundary],[ib[0]]+cb[:10*boundary])):
            ls,rs=left['body_states'],right['body_states']
            dp=float(np.max(np.linalg.norm(np.asarray([s['position'] for s in ls])-np.asarray([s['position'] for s in rs]),axis=1)))
            vl=[rotation(s['quaternion'])@s['velocity_body'] for s in ls]
            vr=[rotation(s['quaternion'])@s['velocity_body'] for s in rs]
            dv=float(np.max(np.linalg.norm(np.asarray(vl)-vr,axis=1)))
            angle=float(np.max(np.rad2deg(2*np.arccos(np.clip([abs(np.dot(l['quaternion'],r['quaternion'])) for l,r in zip(ls,rs)],0,1)))))
            for key,value in (('position_m',dp),('velocity_m_s',dv),('quaternion_deg',angle)):maxima[key]=max(maxima[key],value)
            if dp>PREFIX_CRITERIA['initial_position_max_m' if k==0 else 'prefix_position_max_m']:issues.append(f'position_sample{k}')
            if dv>PREFIX_CRITERIA['initial_velocity_max_m_s' if k==0 else 'prefix_velocity_max_m_s']:issues.append(f'velocity_sample{k}')
            if angle>PREFIX_CRITERIA['quaternion_angle_max_deg']:issues.append(f'orientation_sample{k}')
            if k:
                def requested(d):
                    return d['requested_acceleration_world'] if 'requested_acceleration_world' in d else d['reference_acceleration_world']
                la=[requested(d) for d in left['diagnostics']];ra=[requested(d) for d in right['diagnostics']]
                delta=float(np.max(np.linalg.norm(np.asarray(la)-ra,axis=1)));maxima['request_m_s2']=max(maxima['request_m_s2'],delta)
                if delta>PREFIX_CRITERIA['requested_acceleration_max_m_s2']:issues.append(f'request_sample{k}')
                for row,summary in ((left,sa),(right,sb)):
                    if row['clock_ns']-summary['task_epoch_ns']!=k*20_000_000:issues.append(f'clock_sample{k}')
    if not fixture:
        for key in ('assignments','fixed_tracking_points','remaining_paths'):
            if not (a/'retained_reference.json').exists() or not (b/'retained_reference.json').exists() or read(a/'retained_reference.json')[key]!=read(b/'retained_reference.json')[key]:
                issues.append('retained_reference_'+key)
    am,bm=metrics(a),metrics(b)
    if not all(x['diagnostic_valid'] and x['complete_records'] for x in (am,bm)):issues.append('invalid_or_incomplete_run')
    if fixture and (not bm['release_observed'] or not bm['fresh_start_matches_measured']):issues.append('release_not_verified')
    if not fixture and sb.get('search_commits_deferred',0)==0:issues.append('intervention_not_exercised')
    result=dict(schema=SCHEMA,fixture=fixture,prefix_comparable=not issues,issues=issues,prefix_maxima=maxima,
        allocation_hash_differences_prefix=hash_differences,baseline=am,progress=bm,
        conclusion='ENGINEERING_CONTRAST_ONLY; no mission-level causal or success claim' if not issues else 'INSUFFICIENT_CONTROL; no causal attribution')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True);parser.add_argument('--progress',type=Path,required=True)
    args=parser.parse_args();root=Path(os.environ.get('CRK_OUTPUT_ROOT','/home/legion/AUV-simulation')).resolve()/'results/ch3'
    for p in (args.baseline,args.progress):
        if not p.resolve().is_relative_to(root):parser.error('Use host-mounted results/ch3 runs')
    output=Path(tempfile.mkdtemp(prefix='progress_pair_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_',dir=root));output.chmod(0o755)
    result=compare(args.baseline,args.progress);write(output/'comparison.json',result)
    (output/'REPORT.md').write_text('# Progress v1 pair\n\n'+result['conclusion']+'\n\n'+
        f"prefix_comparable={result['prefix_comparable']}; issues={result['issues']}.\n\n"
        'See comparison.json for releases, fixed-reference metrics and raw hashes.\n',encoding='utf-8')
    if getattr(os,'geteuid',lambda:1)()==0 and os.environ.get('CRK_OUTPUT_UID','').isdigit() and os.environ.get('CRK_OUTPUT_GID','').isdigit():
        for path in [*output.iterdir(),output]:os.chown(path,int(os.environ['CRK_OUTPUT_UID']),int(os.environ['CRK_OUTPUT_GID']),follow_symlinks=False)
    print(f"Progress pair: {output}; prefix_comparable={result['prefix_comparable']}")
    raise SystemExit(0 if result['prefix_comparable'] else 1)


if __name__=='__main__':main()

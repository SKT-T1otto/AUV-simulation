"""Compare independent scalar dynamics with recorded Gazebo body-x odometry."""
import argparse
import csv
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from .initial_package import file_hash
from .surge import LEVELS, PHASES, DT, reference_step, steady_speed, write_json
from .vehicle import roll_pitch


def read_run(run, level=None):
    def read(name):
        return json.loads((run/name).read_text(encoding='utf-8'))
    summary, p = read('summary.json'), read('vehicle_profile.json')
    if not summary['engineering_acceptance_passed'] or summary['termination_reason'] != 'completed':
        raise ValueError(f'Incomplete Gazebo run: {run}')
    if level is None:
        if summary.get('run_kind')!='bounded_physical_envelope' or summary.get('case')!='x':
            raise ValueError('Expected reviewed longitudinal envelope log')
        phases = [(v['name'],v['ticks']) for v in read('program.json')['phases']]
    else:
        if summary.get('run_kind')!='single_hull_surge_v1' or summary.get('level_N')!=level:
            raise ValueError('Mismatched surge trial')
        phases = PHASES
    model = ET.parse(run/'world.sdf').find(".//model[@name='auv_0']")
    axes = np.array([np.fromstring(model.findtext(f"joint[@name='thruster_{i}_joint']/axis/xyz"), sep=' ')
                     for i in range(8)])
    if axes.shape!=(8,3) or not np.allclose(np.linalg.norm(axes,axis=1),1):
        raise ValueError('Invalid SDF propulsor axes')
    expected = [(name,k+1) for name,n in phases for k in range(n)]
    epoch = summary['task_epoch_ns']
    state = summary['initial_state']
    u0 = state['velocity_body'][0][0]
    rows, boundary_u = [], [u0]
    with (run/'control.jsonl').open(encoding='utf-8') as stream:
        for i, line in enumerate(stream):
            row = json.loads(line)
            end = epoch+(i+1)*20_000_000
            after = row['after']
            if (i>=len(expected) or expected[i]!=(row['phase'],row['local_tick']) or row['tick']!=i+1 or
                    row['clock_ns']!=end or row['requested_clock_ns']!=end or row['before']!=state or
                    not state['stamp_ns']<after['stamp_ns']<=end+2_000_000 or abs(after['stamp_ns']-end)>2_000_000):
                raise ValueError(f'Clock, phase or state chain mismatch at tick {i+1}')
            motors = np.asarray(row['forces_N'][0], dtype=float)
            if motors.shape!=(8,) or not np.isfinite(motors).all() or max(abs(motors))>p['test_thrust_limit_N']+1e-8:
                raise ValueError('Invalid Newton command')
            fx = float(motors@axes[:,0])
            diag = row['diagnostics'][0]
            if not np.isclose(fx,diag['commanded_wrench'][0],atol=1e-8):
                raise ValueError('SDF surge projection disagrees with controller log')
            target = diag['requested_wrench'][0]
            if level is not None and (target!=(level if row['phase']=='drive' else 0) or
                                      row['requested_surge_N']!=target):
                raise ValueError('Unexpected axial force request')
            state = after
            v = state['velocity_body'][0]
            rows.append(dict(phase=row['phase'], command_start_s=i*DT, command_end_s=(i+1)*DT,
                        t_s=(state['stamp_ns']-epoch)*1e-9, target_N=target, published_Fx_N=fx,
                        u_gazebo_m_s=v[0], cross_speed_m_s=float(np.linalg.norm(v[1:])),
                        tilt_deg=float(max(abs(np.rad2deg(roll_pitch(state['quaternion_xyzw'][0])))))))
            boundary_u.append(reference_step(boundary_u[-1], fx, DT, p))
    if len(rows)!=len(expected) or len(rows)!=summary['completed_control_steps']:
        raise ValueError('Incomplete control history')
    # Nominal clock differs from odometry by at most one 2 ms physics tick.
    # Represent the measured starting speed at epoch; interpolate theory to actual odometry times.
    stamps = np.array([r['t_s'] for r in rows])
    if stamps[-1]>len(rows)*DT+1e-9:
        raise ValueError('Last odometry sample is outside the recorded force history')
    theory = np.interp(stamps, np.arange(len(rows)+1)*DT, boundary_u)
    residual = np.array([r['u_gazebo_m_s'] for r in rows])-theory
    if not np.isfinite(residual).all():
        raise ValueError('Non-finite velocity')
    for r, u, e in zip(rows,theory,residual):
        r.update(u_theory_m_s=float(u), gazebo_minus_theory_m_s=float(e))
    metrics = dict(run=run.name, source_sha256={n:file_hash(run/n) for n in (
        'control.jsonl','vehicle_profile.json','summary.json','world.sdf','program.json')},
        samples=len(rows), starting_u_m_s=u0, rmse_m_s=float(np.sqrt(np.mean(residual**2))),
        max_abs_error_m_s=float(max(abs(residual))), max_cross_speed_m_s=max(r['cross_speed_m_s'] for r in rows),
        max_tilt_deg=max(r['tilt_deg'] for r in rows), phases={})
    for name, count in phases:
        group = [r for r in rows if r['phase']==name]
        tail = group[-min(50,count):]
        metrics['phases'][name] = dict(final_u_m_s=group[-1]['u_gazebo_m_s'],
            tail_mean_u_m_s=float(np.mean([r['u_gazebo_m_s'] for r in tail])),
            tail_speed_span_m_s=float(np.ptp([r['u_gazebo_m_s'] for r in tail])),
            tail_slope_m_s2=float(np.polyfit([r['t_s'] for r in tail],[r['u_gazebo_m_s'] for r in tail],1)[0]),
            tail_mean_Fx_N=float(np.mean([r['published_Fx_N'] for r in tail])))
    if level is not None:
        drive = metrics['phases']['drive']
        metrics['ideal_steady_u_m_s'] = steady_speed(level,p)
        metrics['observed_plateau'] = drive['tail_speed_span_m_s']<=.01 and abs(drive['tail_slope_m_s2'])<=.01
        onset = next(r['command_start_s'] for r in rows if r['phase']=='drive')
        early = [r for r in rows if onset+.2-1e-9 <= r['t_s'] <= onset+.6+1e-9]
        for key in ('u_gazebo_m_s', 'u_theory_m_s'):
            metrics.setdefault('drive_0p2_to_0p6s_slope_m_s2', {})[key] = float(np.polyfit(
                [r['t_s'] for r in early], [r[key] for r in early], 1)[0])
        release = [r for r in rows if r['phase']=='release']
        remaining = [i for i,r in enumerate(release) if abs(r['published_Fx_N'])>.02]
        first = remaining[-1]+1 if remaining else 0
        metrics['release_below_0p02_N_after_s'] = first*DT if first<len(release) else None
        for key in ('u_gazebo_m_s','u_theory_m_s'):
            clean = release[first:]
            half = None
            if len(clean)>=25 and clean[0][key]>0:
                t_start = clean[0]['t_s']
                for a,b in zip(clean,clean[1:]):
                    threshold = .5*clean[0][key]
                    if b[key]<=threshold<a[key]:
                        half = a['t_s']-t_start+(a[key]-threshold)/(a[key]-b[key])*(b['t_s']-a['t_s'])
                        break
            metrics.setdefault('half_speed_time_after_release_s',{})[key]=half
    return rows, metrics


def plot(out, traces):
    # Always write a portable combined SVG even on a minimal ROS interpreter.
    from .far_plots import chart
    panels = []
    historical = len(traces)==1 and traces[0][0] is None
    with tempfile.TemporaryDirectory() as temp:
        for level, rows, metrics in traces:
            label = 'Archived closed-loop surge' if historical else f'{level} N open-loop surge'
            t = [0.]+[r['t_s'] for r in rows]
            gaz = [metrics['starting_u_m_s']]+[r['u_gazebo_m_s'] for r in rows]
            ref = [metrics['starting_u_m_s']]+[r['u_theory_m_s'] for r in rows]
            def steps(field):
                return [(r[k],r[field]) for r in rows for k in ('command_start_s','command_end_s')]
            boundaries = [b['command_start_s'] for a,b in zip(rows,rows[1:]) if a['phase']!=b['phase']]
            signals = [('Gazebo odometry','#146878','',list(zip(t,gaz))),
                       ('1D theory','#ac491e','stroke-dasharray="7 4"',list(zip(t,ref)))]
            inputs = [('Requested','#8a95a1','stroke-dasharray="6 3"',steps('target_N')),
                      ('Published Fx','#325487','',steps('published_Fx_N'))]
            for title, unit, lines in [(label,'Body surge (m/s)',signals),('Known input','Body surge force (N)',inputs)]:
                path = Path(temp)/'panel.svg'
                chart(path,title,'Simulation time (s)',unit,lines,boundaries=boundaries)
                panels.append(ET.parse(path).getroot())
    width, height = int(panels[0].get('width')), int(panels[0].get('height'))
    svg = ET.Element('svg', xmlns='http://www.w3.org/2000/svg', width=str(width*len(traces)),height=str(height*2))
    for i,panel in enumerate(panels):
        panel.set('x',str(width*(i//2)))
        panel.set('y',str(height*(i%2)))
        svg.append(panel)
    ET.ElementTree(svg).write(out/'surge_velocity.svg',encoding='utf-8',xml_declaration=True)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, grid = plt.subplots(3,len(traces),squeeze=False,sharex='col',figsize=(8 if historical else 11,6.7),
                             gridspec_kw=dict(height_ratios=[2,.85,1]), constrained_layout=True)
    for j,(level,rows,m) in enumerate(traces):
        ax, error, force = grid[:,j]
        t = [0.]+[r['t_s'] for r in rows]
        for key,label,color,style in [('u_gazebo_m_s','Gazebo odometry','#126575','-'),
                                      ('u_theory_m_s','1D theory','#b14a1d','--')]:
            ax.plot(t,[m['starting_u_m_s']]+[r[key] for r in rows],style,label=label,color=color,lw=2)
        edges = [r['command_start_s'] for r in rows]+[rows[-1]['command_end_s']]
        for key,label,color,style in [('target_N','Requested','#9099a0','--'),('published_Fx_N','Published','#385886','-')]:
            force.stairs([r[key] for r in rows],edges,label=label,color=color,linestyle=style,lw=1.5)
        error.plot(t,[0.]+[1000*r['gazebo_minus_theory_m_s'] for r in rows],color='#685487',lw=1.2)
        error.axhline(0,color='0.65',lw=.8)
        error.set_ylabel('Difference (mm/s)')
        error.set_title('Gazebo minus theory',loc='left',fontsize=9)
        for panel in (ax,error,force):
            panel.grid(alpha=.2)
            if panel is not error:
                panel.legend(loc='best',frameon=False,ncol=2,fontsize=9)
            panel.set_xlim(0,edges[-1])
            for a,b in zip(rows,rows[1:]):
                if a['phase']!=b['phase']:
                    panel.axvline(b['command_start_s'],color='0.65',ls=':',lw=.9)
        ax.set_title('Archived closed-loop surge | auv_0' if historical else f'{level} N step and release')
        ax.set_ylabel('Body surge velocity (m/s)')
        force.set(xlabel='Simulation time (s)',ylabel='Body surge force (N)')
    fig.savefig(out/'surge_velocity.png',dpi=190)
    plt.close(fig)


def report(out, sources):
    traces = [(level,*read_run(run,level)) for level,run in sources]
    out.mkdir(exist_ok=True,parents=True)
    with (out/'surge_velocity.csv').open('w',newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f,fieldnames=['run']+list(traces[0][1][0]))
        writer.writeheader()
        for _, rows, m in traces:
            writer.writerows(dict(run=m['run'],**r) for r in rows)
    plot(out,traces)
    write_json(out/'surge_metrics.json', [m for _,_,m in traces])


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-x', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.resolve().is_relative_to(args.archive_x.resolve()):
        parser.error('Write the report outside the original run')
    report(args.out, [(None,args.archive_x)])

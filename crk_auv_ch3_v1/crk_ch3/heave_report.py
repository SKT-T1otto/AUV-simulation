"""Reconstruct vertical thrust from SDF and compare odometry with a 1D reference."""
import csv
import json
import xml.etree.ElementTree as ET

import numpy as np

from .heave import CASES, KIND, phases, reference_step, support
from .surge import DT, write_json
from .vehicle import roll_pitch, rotation


def load(run, name):
    return json.loads((run/name).read_text(encoding='utf-8'))


def read_axes(run, profile):
    world = ET.parse(run/'world.sdf').find('world')
    models = [m for m in world.findall('model') if m.get('name').startswith('auv_')]
    if len(models) != 1 or models[0].get('name') != 'auv_0':
        raise ValueError('Expected a single experimental hull')
    hull = models[0]
    hydro = hull.find("plugin[@name='gz::sim::systems::Hydrodynamics']")
    density = world.findtext("plugin[@name='gz::sim::systems::Buoyancy']/uniform_fluid_density")
    dims = hull.findtext("link[@name='base_link']/collision[@name='displacement_hull']/geometry/box/size")
    actual = [sum(float(x.text) for x in hull.findall('link/inertial/mass')), np.prod(np.fromstring(dims,sep=' ')),
              float(density), float(hull.findtext("link[@name='base_link']/inertial/fluid_added_mass/zz")),
              -float(hydro.findtext('zW')), -float(hydro.findtext('zWabsW'))]
    expected = [profile['mass_kg'], profile['volume_m3'], profile['density_kg_m3'], profile['added_mass'][2],
                profile['linear_drag'][2], profile['quadratic_drag'][2]]
    if (not np.allclose(actual, expected, rtol=1e-9, atol=1e-12) or
            not np.allclose(np.fromstring(world.findtext('gravity'), sep=' '), [0,0,-profile['gravity_m_s2']]) or
            not np.array_equal(np.fromstring(hydro.findtext('default_current'), sep=' '), [0,0,0])):
        raise ValueError('Still-water world and theory inputs disagree')
    axes = np.array([np.fromstring(hull.findtext(f"joint[@name='thruster_{i}_joint']/axis/xyz"),sep=' ')
                     for i in range(8)])
    if axes.shape != (8,3) or not np.allclose(np.linalg.norm(axes,axis=1),1.):
        raise ValueError('Invalid thruster axes')
    return axes


def read_trial(run, case):
    summary, plan, p = [load(run, f) for f in ('summary.json','program.json','vehicle_profile.json')]
    schedule = phases(case,p)
    if (summary['run_kind'] != KIND or summary['case'] != case or plan['case'] != case or
            plan['dt_s'] != DT or plan['phases'] != [dict(name=s, ticks=n, requested_body_z_N=f) for s,n,f in schedule] or
            not all(summary[k] for k in ('engineering_acceptance_passed','linux_sdf_valid','stop_confirmed')) or
            summary['termination_reason'] != 'completed' or summary['contact_events_observed'] != 0 or
            (run/'physical_contacts.jsonl').read_text().strip()):
        raise ValueError(f'Incomplete or mismatched heave trial {case}')
    axes = read_axes(run,p)
    epoch = summary['task_epoch_ns']
    state = summary['initial_state']
    t0 = (state['stamp_ns']-epoch)*1e-9
    z0 = state['positions_world'][0][2]
    w0 = state['velocity_world'][0][2]
    initial_speed = np.linalg.norm(state['velocity_world'][0])
    if (abs(t0) > .002000001 or initial_speed > .02 or
            np.linalg.norm(np.array(state['positions_world'][0])-plan['initial_world_position']) > .02 or
            max(abs(np.rad2deg(roll_pitch(state['quaternion_xyzw'][0])))) > 1.):
        raise ValueError('Heave initial conditions out of bounds')
    expected = [(s,k+1,f) for s,n,f in schedule for k in range(n)]
    rows = []
    with (run/'control.jsonl').open(encoding='utf-8') as stream:
        for i,line in enumerate(stream):
            row = json.loads(line)
            end_ns = epoch + (i+1)*20_000_000
            after = row['after']
            if (i >= len(expected) or (row['phase'],row['local_tick'],row['requested_heave_N']) != expected[i] or
                    row['tick'] != i+1 or row['before'] != state or row['clock_ns'] != end_ns or
                    row['requested_clock_ns'] != end_ns or abs(after['stamp_ns']-end_ns) > 2_000_000 or
                    after['stamp_ns'] <= state['stamp_ns']):
                raise ValueError(f'Interrupted clock, requested force or state chain at tick {i+1}')
            motors = np.asarray(row['forces_N'], dtype=float)
            if motors.shape != (1,8) or not np.isfinite(motors).all() or abs(motors).max() > p['test_thrust_limit_N']+1e-8:
                raise ValueError('Invalid published thrust')
            body = motors[0] @ axes
            diag = row['diagnostics'][0]
            if (not np.allclose(body, diag['commanded_wrench'][:3],rtol=0,atol=1e-8) or
                    diag['requested_wrench'][:3] != [0.,0.,row['requested_heave_N']]):
                raise ValueError('SDF projection or requested force disagrees with recorded wrench')
            world_up = (rotation(state['quaternion_xyzw'][0]) @ body)[2]
            state = after
            v = state['velocity_world'][0]
            rows.append(dict(case=case, phase=row['phase'], tick=i+1, command_start_s=i*DT, command_end_s=(i+1)*DT,
                        t_s=(state['stamp_ns']-epoch)*1e-9, requested_body_z_N=row['requested_heave_N'],
                        published_body_z_N=body[2], published_world_up_N=world_up, velocity_up_gazebo_m_s=v[2],
                        depth_change_gazebo_m=z0-state['positions_world'][0][2], horizontal_speed_m_s=np.hypot(*v[:2]),
                        tilt_deg=max(abs(np.rad2deg(roll_pitch(state['quaternion_xyzw'][0]))))))
    if len(rows) != len(expected) or len(rows) != summary['completed_control_steps']:
        raise ValueError('Incomplete heave force history')

    # Reconstruct at odometry stamps even when a sample falls either side of a command boundary.
    # Startup publishes zero before epoch. No force is known beyond the final interval.
    ref = np.array([w0,0.])
    if t0 < 0:
        ref = reference_step(ref, 0., -t0, p)
    starts = []
    for r in rows:
        starts.append(ref)
        ref = reference_step(ref,r['published_world_up_N'],r['command_end_s']-max(t0,r['command_start_s']),p)
    for r in rows:
        ns = round(r['t_s']*1e9)
        if ns < 0 or ns > len(rows)*20_000_000:
            raise ValueError('Odometry outside measured force history')
        j = min(ns//20_000_000, len(rows)-1)
        w,d = reference_step(starts[j], rows[j]['published_world_up_N'],
                             r['t_s']-max(t0,rows[j]['command_start_s']),p)
        r.update(velocity_up_theory_m_s=w, depth_change_theory_m=d)

    metrics = dict(samples=len(rows), run=run.name, start_time_s=t0, start_world_z_m=z0, start_velocity_up_m_s=w0,
                   start_position_world_m=summary['initial_state']['positions_world'][0],
                   start_velocity_world_m_s=summary['initial_state']['velocity_world'][0],
                   support_N=support(p), max_tilt_deg=max(r['tilt_deg'] for r in rows),
                   max_horizontal_speed_m_s=max(r['horizontal_speed_m_s'] for r in rows), phases={})
    for key,unit in [('velocity_up','m_s'),('depth_change','m')]:
        delta = np.array([r[f'{key}_gazebo_{unit}']-r[f'{key}_theory_{unit}'] for r in rows])
        metrics[f'{key}_rmse_{unit}'] = float(np.sqrt(np.mean(delta**2)))
        metrics[f'{key}_max_abs_error_{unit}'] = float(max(abs(delta)))
    for phase,_,_ in schedule:
        group = [r for r in rows if r['phase']==phase]
        metrics['phases'][phase] = dict(final_depth_change_m=group[-1]['depth_change_gazebo_m'],
                  final_depth_theory_m=group[-1]['depth_change_theory_m'],
                  tail_mean_velocity_up_m_s=np.mean([r['velocity_up_gazebo_m_s'] for r in group[-50:]]),
                  tail_mean_thrust_up_N=np.mean([r['published_world_up_N'] for r in group[-50:]]))
    if not all(np.isfinite(v) for r in rows for v in r.values() if not isinstance(v,str)):
        raise ValueError('Non-finite measured or theoretical series')
    return rows, metrics


def report(out, sources):
    from .heave_plot import plot
    if [case for case,_ in sources] != list(CASES):
        raise ValueError('Expected three ordered trial directories for this batch')
    traces = {case:read_trial(run,case) for case,run in sources}
    with (out/'heave_response.csv').open('w',newline='',encoding='utf-8') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(traces[CASES[0]][0][0]))
        writer.writeheader()
        for rows,_ in traces.values():
            writer.writerows(rows)
    metrics = {case:m for case,(_,m) in traces.items()}
    plot(out,traces)
    write_json(out/'heave_metrics.json',metrics)
    lines = ['# Still-water heave', '', 'Velocity and thrust are positive up. Relative depth is positive down.',
             '', '| Trial | Speed RMSE (m/s) | Depth RMSE (m) | Final depth change (m) |',
             '|---|---:|---:|---:|']
    for case,(rows,m) in traces.items():
        lines.append(f"| {case} | {m['velocity_up_rmse_m_s']:.6f} | {m['depth_change_rmse_m']:.6f} | "
                     f"{rows[-1]['depth_change_gazebo_m']:+.6f} |")
    positions = np.array([m['start_position_world_m'] for m in metrics.values()])
    velocities = np.array([m['start_velocity_world_m_s'] for m in metrics.values()])
    lines.extend(['', f"Maximum pairwise initial position difference: {np.linalg.norm(positions[:,None]-positions,axis=2).max():.6f} m.",
                  f"Maximum pairwise initial velocity difference: {np.linalg.norm(velocities[:,None]-velocities,axis=2).max():.6f} m/s."])
    lines.extend(['', 'Depth change and speed use odometry in the world frame. The one-dimensional reference '
                  'uses recorded initial velocity and the world-up projection of all eight published thrust commands.',
                  'The balance row gives open-loop hover drift over 16 s. Each trial was run once.',
                  '', '![Measured and reference response](heave_response.svg)'])
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')

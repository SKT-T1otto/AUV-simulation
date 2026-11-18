"""Versioned short plant fixtures. No mission environment or vehicle integration."""
import copy
import numpy as np

SCHEMA = 'crk.envelope.v1'
DT = .02


def engineering_scene(original):
    scene = copy.deepcopy(original)
    scene.update(scenario_id='envelope_clear_tank_v1', purpose='motion_only_not_search_or_training',
                 obstacles=[], obstacle_count=0, obstacle_layout_id='none',
                 initial_agent_positions=[[6, 6, 4], [14, 6, 4], [6, 14, 4], [14, 14, 4]],
                 initial_executor_wait_point=[14, 14, 4])
    return scene


def program(case):
    if case not in ('x', 'y', 'z', 'track'):
        raise ValueError('Unknown envelope case')
    phases = [dict(name='initial_hold', ticks=50, mode='waypoint', delta=[0, 0, 0])]
    if case == 'track':
        for name, ticks, delta in [('leg_1', 300, [1.2, 0, 0]),
                                   ('leg_2', 350, [1.2, 1.2, .6]),
                                   ('leg_3', 350, [0, 1.2, -.4]),
                                   ('leg_4', 350, [0, 0, 0]),
                                   ('final_hold', 150, [0, 0, 0])]:
            phases.append(dict(name=name, ticks=ticks, mode='waypoint', delta=delta))
    else:
        axis = 'xyz'.index(case)
        amplitude = .75 if case == 'z' else 1.3
        for name, ticks, mode, sign in [('positive', 300, 'acceleration', 1),
                                         ('zero_acceleration_coast', 100, 'acceleration', 0),
                                         ('brake_positive', 200, 'brake', 0),
                                         ('negative', 300, 'acceleration', -1),
                                         ('brake_negative', 200, 'brake', 0),
                                         ('final_hold', 100, 'brake', 0)]:
            acceleration = np.zeros(3)
            acceleration[axis] = sign*amplitude
            phases.append(dict(name=name, ticks=ticks, mode=mode,
                               acceleration=acceleration.tolist()))
    return dict(schema=SCHEMA, case=case, control_dt_s=DT, phases=phases,
                total_ticks=sum(p['ticks'] for p in phases),
                stop_speed_m_s=.05, stop_dwell_s=.5,
                steady_tail_s=1., steady_speed_span_m_s=.03, steady_slope_m_s2=.02)


def reference(phase, origin, position, world_velocity, profile):
    """Engineering stimulus only; same unchanged acceleration-to-thrust plant controller."""
    c = profile['control']
    if phase['mode'] == 'acceleration':
        return np.array(phase['acceleration']), None, None
    if phase['mode'] == 'brake':
        return -c['velocity_gain']*world_velocity, None, np.zeros(3)
    target = np.array(origin)+phase['delta']
    wanted = c['position_gain']*(target-position)
    speed = np.linalg.norm(wanted)
    if speed > c['speed_limit_m_s']:
        wanted *= c['speed_limit_m_s']/speed
    return c['velocity_gain']*(wanted-world_velocity), target, wanted

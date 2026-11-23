"""Three open-loop heave trials in the same single-hull still-water world."""
import numpy as np

from .surge import DT, single_hull_assets, write_json

CASES = ('free_sink', 'hover_balance', 'bidirectional')
KIND = 'single_hull_heave_v1'


def support(profile):
    return (profile['mass_kg'] - profile['density_kg_m3'] * profile['volume_m3']) * profile['gravity_m_s2']


def phases(case, profile):
    balance = support(profile)
    schedules = {
        'free_sink': [('free', 800, 0.)],
        'hover_balance': [('balance', 800, balance)],
        'bidirectional': [('rise', 300, balance + 5.), ('balance', 200, balance),
                          ('dive', 300, balance - 5.)],
    }
    return schedules[case]


def create_trial(output, case):
    if case not in CASES:
        raise ValueError(f'Unknown heave trial {case}')
    profile, start = single_hull_assets(output)
    write_json(output/'program.json', dict(case=case, dt_s=DT, initial_world_position=start,
               phases=[dict(name=name, ticks=n, requested_body_z_N=f) for name,n,f in phases(case, profile)]))
    write_json(output/'summary.json', dict(run_kind=KIND, case=case, linux_sdf_valid=False,
               engineering_acceptance_passed=False, termination_reason='not_started'))


def reference_step(state, thrust_up, dt, profile):
    """RK4 for world-up velocity and positive-down relative depth under held force."""
    mass = profile['mass_kg'] + profile['added_mass'][2]
    linear, quadratic = profile['linear_drag'][2], profile['quadratic_drag'][2]
    net = thrust_up - support(profile)
    def rate(s):
        w = s[0]
        return np.array([(net - linear*w - quadratic*abs(w)*w) / mass, -w])
    k1 = rate(state)
    k2 = rate(state + .5*dt*k1)
    k3 = rate(state + .5*dt*k2)
    k4 = rate(state + dt*k3)
    return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

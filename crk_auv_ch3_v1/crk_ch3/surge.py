"""Single-hull open-loop surge protocol, and independent one-dimensional reference ODE."""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from .envelope_protocol import engineering_scene
from .search_world import build_search_world, search_bridge_yaml
from .vehicle import ROOT, VehicleControl, load_profile

DT = .02
PHASES = (("settle", 75), ("drive", 400), ("release", 350))
LEVELS = (5, 10)


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def create_trial(output, level):
    if level not in LEVELS:
        raise ValueError('Expected 5 or 10 N total body surge')
    _, start = single_hull_assets(output)
    write_json(output/'program.json', dict(level_N=level, phases=PHASES, dt_s=DT,
                                          initial_world_position=start))
    write_json(output/'summary.json', dict(run_kind='single_hull_surge_v1', level_N=level,
               linux_sdf_valid=False, engineering_acceptance_passed=False, termination_reason='not_started'))


def single_hull_assets(output):
    """Generate the shared still-water engineering world and one odometry bridge."""
    output.mkdir()
    profile = load_profile()
    scene = engineering_scene(json.loads((ROOT/'tests/fixtures/scene.json').read_text()))
    xml, truth = build_search_world(scene, profile)
    sdf = ET.fromstring(xml)
    world = sdf.find('world')
    buoyancy = world.find("plugin[@name='gz::sim::systems::Buoyancy']")
    for i in (1, 2, 3):
        name = f'auv_{i}'
        world.remove(world.find(f"model[@name='{name}']"))
        buoyancy.remove(next(e for e in buoyancy.findall('enable') if e.text == name))
    (output/'world.sdf').write_text(ET.tostring(sdf, encoding='unicode')+'\n', encoding='utf-8')
    (output/'bridge.yaml').write_text(search_bridge_yaml(vehicle_count=1), encoding='utf-8')
    write_json(output/'vehicle_profile.json', profile)
    return profile, truth['agents'][0]


class SurgeDrive(VehicleControl):
    """Constant body-x wrench. Only heave and angular axes receive stabilization."""
    def command(self, target_N, state, z_origin):
        p = self.profile
        # Near-level depth support. No surge-speed feedback, drag cancellation or active braking.
        support = (p['mass_kg']-p['density_kg_m3']*p['volume_m3'])*p['gravity_m_s2']
        heave = support + 5*(z_origin-state.position[2]) - 6*state.velocity_body[2]
        return self.apply_force([target_N, 0., heave], state)

    def apply_force(self, force_body, state):
        """Shared attitude stabilizer, allocator and motor dynamics for open-loop trials."""
        c = self.profile['control']
        q = np.array(state.quaternion)
        q /= np.linalg.norm(q)
        if q[3] < 0:
            q *= -1
        moment = -2*np.asarray(c['attitude_gain'])*q[:3] - np.asarray(c['rate_gain'])*state.rates_body
        wrench = np.r_[force_body, np.clip(moment, -3, 3)]
        desired = self.allocate(wrench)
        change = -np.expm1(-DT/c['motor_time_constant_s'])*(desired-self.force)
        self.force = np.clip(self.force + np.clip(change, -c['thrust_slew_N_s']*DT,
                             c['thrust_slew_N_s']*DT), -self.limit, self.limit)
        return self.force.copy(), dict(requested_wrench=wrench.tolist(),
                allocated_thrust_N=desired.tolist(), commanded_wrench=(self.B@self.force).tolist())


def reference_step(u, force, dt, profile):
    """Integrate a held body-axis command; never use the next measured velocity."""
    mass = profile['mass_kg']+profile['added_mass'][0]
    linear, quadratic = profile['linear_drag'][0], profile['quadratic_drag'][0]
    def rate(v):
        return (force-linear*v-quadratic*abs(v)*v)/mass
    k1 = rate(u)
    k2 = rate(u+.5*dt*k1)
    k3 = rate(u+.5*dt*k2)
    k4 = rate(u+dt*k3)
    return u + dt*(k1+2*k2+2*k3+k4)/6


def steady_speed(force, profile):
    linear, quadratic = profile['linear_drag'][0], profile['quadratic_drag'][0]
    return 2*force/(linear+math.sqrt(linear**2+4*quadratic*abs(force)))

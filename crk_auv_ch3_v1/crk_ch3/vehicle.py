"""G4A: documented geometry and independent control, never integrate vehicle state."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[1]


def load_profile():
    return json.loads((ROOT/'config/vehicle_g4.json').read_text(encoding='utf-8'))


def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def thrusters(profile):
    conversion = np.diag([1., -1., -1.])
    result = []
    for alpha, beta in zip(profile['horizontal_position_angles_rad'], profile['horizontal_direction_angles_rad']):
        result.append((conversion @ rot_z(alpha) @ profile['horizontal_position_frd_m'],
                       conversion @ rot_z(beta) @ np.array([1, -1, 0])/np.sqrt(2)))
    for gamma in profile['vertical_position_angles_rad']:
        result.append((conversion @ rot_z(gamma) @ profile['vertical_position_frd_m'],
                       conversion @ np.array([0., 0., -1.])))
    return result


def allocation(profile):
    geometry = thrusters(profile)
    matrix = np.column_stack([np.r_[d, np.cross(p, d)] for p, d in geometry])
    if matrix.shape != (6, 8) or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 6:
        raise ValueError('Profile does not provide six independent wrench axes')
    if any(np.linalg.norm(a[0]-b[0]) < .04 for i, a in enumerate(geometry) for b in geometry[i+1:]):
        raise ValueError('Overlapping thruster mounting centers')
    return matrix


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def roll_pitch(quaternion):
    r = rotation(quaternion)
    return np.array([np.arctan2(r[2, 1], r[2, 2]), np.arcsin(np.clip(-r[2, 0], -1, 1))])


def base_inertial(profile):
    """Correct base CG/inertia for rotor links so complete-vehicle values stay fixed."""
    geometry = thrusters(profile)
    mr = profile['rotor_link_mass_kg']
    mb = profile['mass_kg']-mr*len(geometry)
    cg = -sum((mr*p for p, _ in geometry), np.zeros(3))/mb
    def parallel(mass, p):
        return mass*(np.dot(p, p)*np.eye(3)-np.outer(p, p))
    inertia = np.diag(profile['inertia_kg_m2'])-parallel(mb, cg)
    for p, _ in geometry:
        inertia -= parallel(mr, p)+profile['rotor_link_inertia_kg_m2']*np.eye(3)
    if mb <= 0 or np.min(np.linalg.eigvalsh(inertia)) <= 0:
        raise ValueError('Invalid corrected rigid-body inertia')
    return mb, cg, inertia


class VehicleControl:
    def __init__(self, profile):
        self.profile = profile
        self.B = allocation(profile)
        self.limit = float(profile['test_thrust_limit_N'])
        self.inverse = np.linalg.pinv(self.B)
        self.W = np.diag([1., 1., 1., 4., 4., 4.])
        self.A = np.vstack([self.W @ self.B, 1e-3*np.eye(self.B.shape[1])])
        self.force = np.zeros(self.B.shape[1])

    def allocate(self, wrench):
        wrench = np.asarray(wrench, dtype=float)
        if wrench.shape != (6,) or not np.isfinite(wrench).all():
            raise ValueError('Invalid requested wrench')
        result = self.inverse @ wrench
        if np.max(np.abs(result)) <= self.limit:
            return result
        solution = lsq_linear(self.A, np.r_[self.W@wrench, np.zeros(self.B.shape[1])],
                              bounds=(-self.limit, self.limit), tol=1e-7)
        if not solution.success:
            raise RuntimeError('Thrust allocation failed')
        return solution.x

    def track(self, target, position, quaternion, body_velocity, body_rates, dt):
        if not 0 < dt <= .2 or not np.isfinite(np.r_[target, position, body_velocity, body_rates, dt]).all():
            raise ValueError('Invalid tracking inputs')
        p, c = self.profile, self.profile['control']
        r = rotation(quaternion)
        v = np.asarray(body_velocity, dtype=float)
        wanted_v = c['position_gain']*(np.asarray(target)-np.asarray(position))
        speed = np.linalg.norm(wanted_v)
        if speed > c['speed_limit_m_s']:
            wanted_v *= c['speed_limit_m_s']/speed
        acceleration = c['velocity_gain']*(wanted_v-r@v)
        acceleration = np.clip(acceleration, -c['acceleration_limit_m_s2'], c['acceleration_limit_m_s2'])
        drag = np.array(p['linear_drag'][:3])*v+np.array(p['quadratic_drag'][:3])*np.abs(v)*v
        force = (p['mass_kg']+np.array(p['added_mass'][:3]))*(r.T@acceleration)+drag
        force += r.T@np.array([0., 0., (p['mass_kg']-p['density_kg_m3']*p['volume_m3'])*p['gravity_m_s2']])
        q = np.asarray(quaternion, dtype=float)/np.linalg.norm(quaternion)
        if q[3] < 0:
            q = -q
        torque = -np.array(c['attitude_gain'])*2*q[:3]-np.array(c['rate_gain'])*np.asarray(body_rates)
        requested = np.r_[force, np.clip(torque, -3, 3)]
        desired = self.allocate(requested)
        change = (1-np.exp(-dt/c['motor_time_constant_s']))*(desired-self.force)
        change = np.clip(change, -c['thrust_slew_N_s']*dt, c['thrust_slew_N_s']*dt)
        self.force = np.clip(self.force+change, -self.limit, self.limit)
        return self.force.copy(), {'requested_wrench': requested.tolist(),
                                   'allocation_residual': float(np.linalg.norm(self.W@(self.B@desired-requested))),
                                   'reference_acceleration_world': acceleration.tolist()}

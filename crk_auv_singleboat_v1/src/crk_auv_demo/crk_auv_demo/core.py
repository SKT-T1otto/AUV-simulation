"""Frame transforms, geometry-derived allocation and waypoint control; no ROS."""
import numpy as np
from scipy.optimize import lsq_linear


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy(q):
    R = rotation(q)
    return np.array([np.arctan2(R[2, 1], R[2, 2]),
                     np.arcsin(np.clip(-R[2, 0], -1, 1)),
                     np.arctan2(R[1, 0], R[0, 0])])


def allocation_matrix(config):
    columns = []
    for thruster in config['thrusters']:
        direction = np.array(thruster['direction'], dtype=float)
        direction /= np.linalg.norm(direction)
        columns.append(np.r_[direction, np.cross(thruster['position'], direction)])
    B = np.column_stack(columns)
    if B.shape != (6, 8) or np.linalg.matrix_rank(B) != 6:
        raise ValueError('Thruster geometry must provide six independent wrench axes')
    return B


class ControlLaw:
    def __init__(self, config):
        self.config = config
        self.B = allocation_matrix(config)
        self.inverse = np.linalg.pinv(self.B)
        self.limit = float(config['thrust_limit_N'])
        self.force = np.zeros(8)
        self.W = np.diag([1., 1., 1., 4., 4., 4.])
        self.A = np.vstack([self.W @ self.B, 1e-3 * np.eye(8)])

    def allocate(self, wrench):
        force = self.inverse @ wrench
        if np.max(np.abs(force)) <= self.limit:
            return force
        result = lsq_linear(self.A, np.r_[self.W @ wrench, np.zeros(8)],
                            bounds=(-self.limit, self.limit), tol=1e-7)
        if not result.success or not np.isfinite(result.x).all():
            raise RuntimeError('Bounded thrust allocation failed')
        return result.x

    def step(self, position, quaternion, body_velocity, body_rates, target, dt):
        inputs = np.r_[position, quaternion, body_velocity, body_rates, target, dt]
        if not np.isfinite(inputs).all() or not 0 < dt <= 0.2:
            raise ValueError('Nonfinite state or invalid control interval')
        R = rotation(quaternion)
        world_velocity = R @ body_velocity
        c = self.config['control']
        desired_velocity = np.clip(c['position_gain'] * (target-position),
                                   -c['speed_limit_m_s'], c['speed_limit_m_s'])
        acceleration = np.clip(c['velocity_gain'] * (desired_velocity-world_velocity),
                               -c['accel_limit_m_s2'], c['accel_limit_m_s2'])
        # Feed forward nominal zero-current drag, not the Python environment's dynamics.
        linear = np.array(self.config['linear_drag'])
        quadratic = np.array(self.config['quadratic_drag'])
        drag = linear[:3]*body_velocity + quadratic[:3]*np.abs(body_velocity)*body_velocity
        body_force = R.T @ (self.config['mass_kg'] * acceleration) + drag
        q = np.array(quaternion, dtype=float)
        q /= np.linalg.norm(q)
        if q[3] < 0:
            q = -q
        torque = -np.array(c['attitude_gain']) * (2*q[:3]) - np.array(c['rate_gain'])*body_rates
        torque = np.clip(torque, -3., 3.)
        requested = np.r_[body_force, torque]
        desired_force = self.allocate(requested)
        # First-order motor lag plus force slew limits, applied before Gazebo.
        delta = (1-np.exp(-dt/c['motor_time_constant_s'])) * (desired_force-self.force)
        delta = np.clip(delta, -c['thrust_slew_N_s']*dt, c['thrust_slew_N_s']*dt)
        self.force = np.clip(self.force+delta, -self.limit, self.limit)
        residual = np.linalg.norm(self.W @ (self.B @ desired_force-requested))
        return self.force.copy(), float(residual)


class WaypointSequence:
    def __init__(self, waypoints, arrival_radius=0.2, speed_threshold=0.1, dwell_s=2.):
        self.points = np.asarray(waypoints, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 3 or len(self.points) < 1:
            raise ValueError('Expected a nonempty N x 3 waypoint list')
        self.index = 0
        self.accepted = 0
        self.dwell = 0.
        self.arrival_radius = arrival_radius
        self.speed_threshold = speed_threshold
        self.dwell_s = dwell_s
        self.completed = False

    @property
    def target(self):
        return self.points[self.index]

    def update(self, position, speed, dt):
        if self.completed:
            return False
        if np.linalg.norm(self.target-position) <= self.arrival_radius and speed <= self.speed_threshold:
            self.dwell += dt
        else:
            self.dwell = 0.
        if self.dwell < self.dwell_s:
            return False
        self.accepted += 1
        self.dwell = 0.
        if self.accepted == len(self.points):
            self.completed = True
        else:
            self.index += 1
        return True

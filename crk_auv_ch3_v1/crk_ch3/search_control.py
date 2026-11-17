"""G6 bounded acceleration-to-thrust controller; no waypoint prior or state integration."""
import numpy as np
from .vehicle import VehicleControl, rotation


class AccelerationControl(VehicleControl):
    def command(self, acceleration_world, quaternion, body_velocity, body_rates, dt=.02):
        a = np.array(acceleration_world, dtype=float, copy=True)
        v = np.asarray(body_velocity, dtype=float)
        w = np.asarray(body_rates, dtype=float)
        if a.shape != (3,) or v.shape != (3,) or w.shape != (3,) or not np.isfinite(np.r_[a, v, w]).all() or not 0 < dt <= .2:
            raise ValueError('Invalid acceleration control input')
        p, c = self.profile, self.profile['control']
        r = rotation(quaternion)
        requested = a.copy()
        world_v = r@v
        speed = np.linalg.norm(world_v)
        if speed > c['speed_limit_m_s']:
            direction = world_v/speed
            a -= max(0., np.dot(a, direction))*direction
            a -= c['velocity_gain']*(speed-c['speed_limit_m_s'])*direction
        a = np.clip(a, -c['acceleration_limit_m_s2'], c['acceleration_limit_m_s2'])
        drag = np.array(p['linear_drag'][:3])*v+np.array(p['quadratic_drag'][:3])*np.abs(v)*v
        force = (p['mass_kg']+np.array(p['added_mass'][:3]))*(r.T@a)+drag
        force += r.T@np.array([0., 0., (p['mass_kg']-p['density_kg_m3']*p['volume_m3'])*p['gravity_m_s2']])
        q = np.asarray(quaternion, dtype=float)/np.linalg.norm(quaternion)
        if q[3] < 0:
            q = -q
        torque = -np.array(c['attitude_gain'])*2*q[:3]-np.array(c['rate_gain'])*w
        wrench = np.r_[force, np.clip(torque, -3, 3)]
        desired = self.allocate(wrench)
        change = (1-np.exp(-dt/c['motor_time_constant_s']))*(desired-self.force)
        change = np.clip(change, -c['thrust_slew_N_s']*dt, c['thrust_slew_N_s']*dt)
        self.force = np.clip(self.force+change, -self.limit, self.limit)
        return self.force.copy(), dict(requested_acceleration_world=requested.tolist(),
            governed_acceleration_world=a.tolist(), requested_wrench=wrench.tolist(),
            allocation_residual=float(np.linalg.norm(self.W@(self.B@desired-wrench))))

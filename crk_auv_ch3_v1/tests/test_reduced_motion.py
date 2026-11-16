"""Bounded approximate 6DOF control regression, explicitly not Gazebo validation."""
import unittest
import numpy as np
from crk_ch3.motion_protocol import build_program, evaluate_phase
from crk_ch3.vehicle import VehicleControl, load_profile, roll_pitch, rotation


class ReducedMotionTests(unittest.TestCase):
    def test_four_independent_vehicles_complete_engineering_motion_program(self):
        p = load_profile()
        controls = [VehicleControl(p) for _ in range(4)]
        positions = np.array([[0., i*4., 0.] for i in range(4)])
        program = build_program(positions)
        velocity, rates = np.zeros((4, 3)), np.zeros((4, 3))
        quaternions = np.tile([0., 0., 0., 1.], (4, 1))
        effective_m = p['mass_kg']+np.array(p['added_mass'][:3])
        effective_i = np.array(p['inertia_kg_m2'])+np.array(p['added_mass'][3:])
        max_tilt = 0.
        reports = []
        for phase in program:
            starts = positions.copy()
            drift = np.zeros(4)
            for _ in range(phase.ticks):
                for i, control in enumerate(controls):
                    q, v, w = quaternions[i], velocity[i], rates[i]
                    r = rotation(q)
                    force, _ = control.track(phase.targets[i], positions[i], q, v, w, .02)
                    wrench = control.B@force
                    drag = np.array(p['linear_drag'])*np.r_[v, w]+np.array(p['quadratic_drag'])*np.abs(np.r_[v, w])*np.r_[v, w]
                    net_weight = np.array([0., 0., (p['density_kg_m3']*p['volume_m3']-p['mass_kg'])*p['gravity_m_s2']])
                    buoyancy = np.array([0., 0., p['density_kg_m3']*p['volume_m3']*p['gravity_m_s2']])
                    restoring = np.cross(p['center_of_buoyancy_flu_m'], r.T@buoyancy)
                    velocity[i] += .02*((wrench[:3]-drag[:3]+r.T@net_weight)/effective_m-np.cross(w, v))
                    rates[i] += .02*((wrench[3:]-drag[3:]+restoring-np.cross(w, effective_i*w))/effective_i)
                    qvec, qw = q[:3], q[3]
                    derivative = .5*np.r_[qw*rates[i]+np.cross(qvec, rates[i]), -np.dot(qvec, rates[i])]
                    quaternions[i] += .02*derivative
                    quaternions[i] /= np.linalg.norm(quaternions[i])
                    positions[i] += .02*(rotation(quaternions[i])@velocity[i])
                    max_tilt = max(max_tilt, np.max(np.abs(np.rad2deg(roll_pitch(quaternions[i])))))
                drift = np.maximum(drift, np.linalg.norm(positions-starts, axis=1))
            report = evaluate_phase(phase, starts, positions, drift)
            reports.append(report)
        self.assertTrue(all(r['passed'] for r in reports), reports)
        self.assertLess(max_tilt, 10.)
        self.assertLess(np.max(np.linalg.norm(velocity, axis=1)), .1)

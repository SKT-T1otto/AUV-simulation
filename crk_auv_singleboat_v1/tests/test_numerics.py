"""Offline design checks, not a replacement for Gazebo runtime acceptance."""
import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'src/crk_auv_demo'
sys.path.insert(0, str(PACKAGE))
from crk_auv_demo.core import ControlLaw, WaypointSequence, allocation_matrix, rotation

C = json.loads((PACKAGE/'config/vehicle.json').read_text())
PLAN = json.loads((PACKAGE/'config/waypoints.json').read_text())


class DesignChecks(unittest.TestCase):
    def test_frames_and_force_axes(self):
        q = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        np.testing.assert_allclose(rotation(q) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        B = allocation_matrix(C)
        for axis in range(6):
            request = np.eye(6)[axis]
            force = ControlLaw(C).allocate(request)
            np.testing.assert_allclose(B @ force, request, atol=1e-10)
        saturated = ControlLaw(C).allocate(np.array([1000, -2000, 3000, 50, 70, 100]))
        self.assertLessEqual(np.max(np.abs(saturated)), C['thrust_limit_N']+1e-10)

    def test_sdf_geometry_and_buoyancy(self):
        world = ET.parse(PACKAGE/'worlds/single_auv.sdf').getroot().find('world')
        model = world.find("model[@name='auv_0']")
        masses = sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
        np.testing.assert_allclose(masses, np.prod(C['hull_size_m'])*C['fluid_density_kg_m3'])
        self.assertEqual(len(model.findall('joint')), 8)
        for i, thruster in enumerate(C['thrusters']):
            joint = model.find(f"joint[@name='thruster_{i}_joint']")
            axis = np.fromstring(joint.findtext('axis/xyz'), sep=' ')
            np.testing.assert_allclose(axis, thruster['direction'])
            p = np.fromstring(model.find(f"link[@name='thruster_{i}']").findtext('pose'), sep=' ')[:3]
            np.testing.assert_allclose(p, thruster['position'])
        self.assertEqual(len(model.findall("plugin[@name='gz::sim::systems::Thruster']")), 8)
        # Only the displacement hull has collision volume; propellers add mass but no buoyancy.
        self.assertEqual(len(model.findall('link/collision')), 1)

    def test_dwell_requires_slow_arrival(self):
        sequence = WaypointSequence([[0, 0, 0]], dwell_s=2)
        for _ in range(150):
            sequence.update(np.zeros(3), 1., 0.02)
        self.assertFalse(sequence.completed)
        for _ in range(101):
            sequence.update(np.zeros(3), 0., 0.02)
        self.assertTrue(sequence.completed)

    def test_reduced_six_dof_closed_loop(self):
        law = ControlLaw(C)
        seq = WaypointSequence(PLAN['points'])
        p = np.array([0., 0., -2.])
        q = np.array([0.05, -0.025, 0.1, 1.])
        q /= np.linalg.norm(q)
        v = np.zeros(3)
        w = np.zeros(3)
        mass = C['mass_kg']+np.array(C['added_mass'][:3])
        x, y, z = C['hull_size_m']
        inertia = (C['mass_kg']-0.16)*np.array([y*y+z*z, x*x+z*z, x*x+y*y])/12
        for thruster in C['thrusters']:
            r = np.array(thruster['position'])
            inertia += 0.00002 + C['propeller_mass_kg']*(np.dot(r,r)-r*r)
        inertia += C['added_mass'][3:]
        dl, dq = np.array(C['linear_drag']), np.array(C['quadratic_drag'])
        completion = None
        for k in range(8000):
            seq.update(p, np.linalg.norm(v), .02)
            forces, _ = law.step(p, q, v, w, seq.target, .02)
            wrench = law.B @ forces
            for _ in range(10):
                dt = .002
                R = rotation(q)
                vdot = (wrench[:3]-dl[:3]*v-dq[:3]*abs(v)*v-np.cross(w,mass*v))/mass
                wdot = (wrench[3:]-dl[3:]*w-dq[3:]*abs(w)*w
                        -np.cross(w,inertia*w)-np.cross(v,mass*v))/inertia
                p += (R @ v)*dt
                qdot = np.r_[q[3]*w+np.cross(q[:3],w), -np.dot(q[:3],w)]*.5
                q += qdot*dt
                q /= np.linalg.norm(q)
                v += vdot*dt
                w += wdot*dt
            if seq.completed:
                if completion is None:
                    completion = k*.02
                if k*.02-completion > 10:
                    break
        self.assertTrue(seq.completed, f'position={p}, accepted={seq.accepted}')
        self.assertLess(np.linalg.norm(p-seq.target), 0.1)
        self.assertLess(np.linalg.norm(q[:3]), 0.01)
        print(f'Offline reduced-model result: accepted={seq.accepted}; '
              f'completion_s={completion:.2f}; final_error_m={np.linalg.norm(p-seq.target):.6f}')


if __name__ == '__main__':
    unittest.main()

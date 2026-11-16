import unittest
import numpy as np
from crk_ch3.vehicle import VehicleControl, allocation, base_inertial, load_profile, rotation, thrusters


class VehicleTests(unittest.TestCase):
    def setUp(self):
        self.p = load_profile()

    def test_geometry_is_full_rank_and_has_no_coincident_mounts(self):
        self.assertEqual(np.linalg.matrix_rank(allocation(self.p)), 6)
        positions = [p for p, _ in thrusters(self.p)]
        self.assertGreater(min(np.linalg.norm(a-b) for i, a in enumerate(positions) for b in positions[i+1:]), .04)
        for _, direction in thrusters(self.p):
            self.assertAlmostEqual(np.linalg.norm(direction), 1)
        self.assertEqual(thrusters(self.p)[4][1].tolist(), [0, 0, 1])

    def test_total_mass_cg_and_inertia_match_literature_after_rotor_split(self):
        mass, cg, inertia = base_inertial(self.p)
        def parallel(m, p):
            return m*(np.dot(p, p)*np.eye(3)-np.outer(p, p))
        total_inertia = inertia+parallel(mass, cg)
        weighted_cg = mass*cg
        for p, _ in thrusters(self.p):
            mr = self.p['rotor_link_mass_kg']
            weighted_cg += mr*p
            total_inertia += parallel(mr, p)+np.eye(3)*self.p['rotor_link_inertia_kg_m2']
        np.testing.assert_allclose(weighted_cg, 0, atol=1e-12)
        np.testing.assert_allclose(total_inertia, np.diag(self.p['inertia_kg_m2']), atol=1e-12)
        self.assertAlmostEqual(mass+8*self.p['rotor_link_mass_kg'], 13.5)

    def test_signed_wrench_axes_and_saturation(self):
        controller = VehicleControl(self.p)
        for axis in np.eye(6):
            for sign in (-1, 1):
                forces = controller.allocate(.1*sign*axis)
                np.testing.assert_allclose(controller.B@forces, .1*sign*axis, atol=1e-10)
        self.assertLessEqual(np.max(np.abs(controller.allocate([1000]*6))), self.p['test_thrust_limit_N']+1e-7)

    def test_controllers_are_independent_and_do_not_mutate_state(self):
        a, b = VehicleControl(self.p), VehicleControl(self.p)
        position, velocity = np.zeros(3), np.zeros(3)
        forces, _ = a.track([1, 0, 0], position, [0, 0, 0, 1], velocity, np.zeros(3), .02)
        self.assertGreater(np.linalg.norm(forces), 0)
        np.testing.assert_array_equal(b.force, 0)
        np.testing.assert_array_equal(position, 0)
        np.testing.assert_array_equal(velocity, 0)
        with self.assertRaises(ValueError):
            a.track([0]*3, position, [0]*4, velocity, [0]*3, .02)

    def test_rest_compensates_negative_buoyancy_and_frame_rotation(self):
        controller = VehicleControl(self.p)
        _, diagnostics = controller.track([0]*3, [0]*3, [0, 0, 0, 1], [0]*3, [0]*3, .02)
        self.assertAlmostEqual(diagnostics['requested_wrench'][2], .982)
        np.testing.assert_allclose(rotation([0, 0, np.sqrt(.5), np.sqrt(.5)])@[1, 0, 0], [0, 1, 0], atol=1e-12)

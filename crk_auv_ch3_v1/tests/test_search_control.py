import unittest
import numpy as np
from crk_ch3.search_control import AccelerationControl
from crk_ch3.vehicle import load_profile, rotation


class AccelerationTests(unittest.TestCase):
    def test_zero_acceleration_at_rest_only_compensates_weight(self):
        control = AccelerationControl(load_profile())
        _, d = control.command([0, 0, 0], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(d['requested_wrench'], [0, 0, .982, 0, 0, 0], atol=1e-10)
        np.testing.assert_array_equal(d['governed_acceleration_world'], [0, 0, 0])

    def test_world_acceleration_is_rotated_to_body_and_bounded(self):
        p = load_profile()
        control = AccelerationControl(p)
        q = [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]
        forces, d = control.command([.1, 0, 0], q, [0, 0, 0], [0, 0, 0])
        expected = (p['mass_kg']+np.array(p['added_mass'][:3]))*(rotation(q).T@np.array([.1, 0, 0]))
        np.testing.assert_allclose(d['requested_wrench'][:2], expected[:2], atol=1e-10)
        self.assertLessEqual(np.max(np.abs(forces)), p['test_thrust_limit_N'])
        for _ in range(100):
            forces, d = control.command([100, -100, 100], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0])
            self.assertLessEqual(np.max(np.abs(forces)), p['test_thrust_limit_N'])
            self.assertLessEqual(np.max(np.abs(d['governed_acceleration_world'])), .45)

    def test_speed_governor_brakes_and_keeps_inputs_unchanged(self):
        a = np.array([.2, .1, 0.])
        control = AccelerationControl(load_profile())
        _, d = control.command(a, [0, 0, 0, 1], [1, 0, 0], [0, 0, 0])
        self.assertLess(d['governed_acceleration_world'][0], 0)
        np.testing.assert_array_equal(a, [.2, .1, 0])
        with self.assertRaises(ValueError):
            control.command([np.nan, 0, 0], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0])

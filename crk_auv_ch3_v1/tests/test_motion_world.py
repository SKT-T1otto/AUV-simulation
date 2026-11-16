import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from crk_ch3.motion_world import WORLD, bridge_entries, build_motion_world
from crk_ch3.vehicle import load_profile, thrusters

ROOT = Path(__file__).resolve().parents[1]


class MotionWorldTests(unittest.TestCase):
    def setUp(self):
        self.p = load_profile()
        self.scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text())
        xml, self.truth = build_motion_world(self.scene, self.p)
        self.world = ET.fromstring(xml).find('world')

    def test_four_dynamic_vehicles_have_32_real_thrust_plugins(self):
        self.assertEqual(self.world.get('name'), WORLD)
        self.assertEqual(len(self.world.findall('.//joint')), 32)
        motors = [p for p in self.world.findall('.//plugin') if p.get('name').endswith('::Thruster')]
        self.assertEqual(len(motors), 32)
        for i in range(4):
            model = self.world.find(f"model[@name='auv_{i}']")
            self.assertEqual(model.findtext('static'), 'false')
            self.assertEqual(len(model.findall('joint')), 8)
            self.assertEqual([float(x) for x in model.findtext('pose').split()[:3]], list(self.truth['agents'][i]))

    def test_buoyancy_mass_and_geometry_match_the_controller(self):
        enabled = self.world.find("plugin[@name='gz::sim::systems::Buoyancy']").findall('enable')
        self.assertEqual([x.text for x in enabled], [f'auv_{i}' for i in range(4)])
        for i in range(4):
            model = self.world.find(f"model[@name='auv_{i}']")
            self.assertAlmostEqual(sum(float(link.findtext('inertial/mass')) for link in model.findall('link')), 13.5)
            size = [float(x) for x in model.findtext("link[@name='base_link']/collision/geometry/box/size").split()]
            self.assertAlmostEqual(np.prod(size), self.p['volume_m3'])
            for j, (point, direction) in enumerate(thrusters(self.p)):
                pose = model.findtext(f"link[@name='thruster_{j}']/pose")
                np.testing.assert_allclose([float(x) for x in pose.split()[:3]], point)
                axis = model.findtext(f"joint[@name='thruster_{j}_joint']/axis/xyz")
                np.testing.assert_allclose([float(x) for x in axis.split()], direction)

    def test_bridges_are_unique_and_do_not_cross_wire_agents(self):
        entries = bridge_entries()
        self.assertEqual(len(entries), 37)
        self.assertEqual(len({e[0] for e in entries}), 37)
        self.assertEqual(len({e[1] for e in entries}), 37)
        for i in range(4):
            own = [e for e in entries if e[0].startswith(f'/auv_{i}/')]
            self.assertEqual(len(own), 9)
            self.assertTrue(all(e[1].startswith(f'/model/auv_{i}/') for e in own))

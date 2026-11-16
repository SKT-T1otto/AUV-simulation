import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from crk_ch3.preview import build_preview
from crk_ch3.scenario import Frame

ROOT = Path(__file__).resolve().parents[1]


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text())

    def test_four_roles_and_exact_native_obstacles(self):
        xml, truth = build_preview(self.scene)
        world = ET.fromstring(xml).find('world')
        models = world.findall('model')
        self.assertEqual(len([m for m in models if m.get('name').startswith('auv_')]), 4)
        self.assertEqual(len([m for m in models if m.get('name').startswith('obstacle_')]), 2)
        for model in models:
            self.assertEqual(model.findtext('static'), 'true')
        self.assertEqual(len(world.findall('.//joint')), 0)
        self.assertFalse(any('Thruster' in p.get('name', '') for p in world.findall('.//plugin')))
        for i, box in enumerate(self.scene['obstacles']):
            model = world.find(f"model[@name='obstacle_{i}']")
            sizes = [float(x) for x in model.findtext('link/collision/geometry/box/size').split()]
            self.assertEqual(sizes, box['size'])
            self.assertEqual([float(x) for x in model.findtext('pose').split()[:3]], list(truth['obstacles'][i]['center']))

    def test_rotated_boxes_keep_size_and_pose_rotation(self):
        xml, truth = build_preview(self.scene, Frame(yaw=math.pi/2))
        model = ET.fromstring(xml).find("world/model[@name='obstacle_0']")
        self.assertAlmostEqual(float(model.findtext('pose').split()[-1]), math.pi/2)
        self.assertEqual([float(x) for x in model.findtext('link/collision/geometry/box/size').split()], [2, 3, 2])

    def test_target_and_role_markers_are_not_collision_bodies(self):
        xml, _ = build_preview(self.scene)
        world = ET.fromstring(xml).find('world')
        for model in world.findall('model'):
            if model.get('name').startswith('auv_') or model.get('name') in ('target_truth_marker', 'executor_wait_marker'):
                self.assertEqual(model.findall('.//collision'), [])

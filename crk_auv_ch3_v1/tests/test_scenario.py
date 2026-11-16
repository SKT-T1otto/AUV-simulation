from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from crk_ch3.scenario import Frame, project_truth_scene, select_scene, validate_scene

ROOT = Path(__file__).resolve().parents[1]


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text())

    def test_frame_round_trip_and_vector_not_translated(self):
        frame = Frame((-10, -10, -9), math.pi/2)
        for point in ([2, 3, 4], [0, 0, 0], [20, 20, 8]):
            for actual, expected in zip(frame.inverse_position(frame.position(point)), point):
                self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(frame.rotate([1, 0, 0])[1], 1)
        self.assertEqual(frame.rotate([0, 0, 0]), (0, 0, 0))
        with self.assertRaises(ValueError):
            Frame([float('nan'), 0, 0])

    def test_rigid_scene_preserves_boxes_and_native_input(self):
        before = deepcopy(self.scene)
        view = project_truth_scene(self.scene, Frame(yaw=math.pi/2))
        self.assertEqual(self.scene, before)
        self.assertEqual(len(view['agents']), 4)
        self.assertEqual(view['obstacles'][0]['size'], (2, 3, 2))
        self.assertEqual(view['obstacles'][0]['yaw'], math.pi/2)
        self.assertTrue(view['privileged_truth_not_policy_input'])
        self.assertEqual(max(p[2] for p in view['workspace_corners']), -1)
        with self.assertRaises(ValueError):
            project_truth_scene(self.scene, Frame((0, 0, 0)))

    def test_invalid_scene_and_truth_assumptions_rejected(self):
        changes = [('initial_agent_positions', [[1, 2, 3]]), ('obstacle_count', 3),
                   ('target_initial_velocity', [0, float('nan'), 0]),
                   ('target_initial_position', [1, 1, 1]), ('obstacle_knowledge_mode', 'oracle'),
                   ('target_position', [99, 0, 0]), ('map_sharing_mode', 'other')]
        for field, value in changes:
            s = deepcopy(self.scene)
            s[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_scene(s)
        s = deepcopy(self.scene)
        s['initial_agent_positions'][0] = [8, 6, 4]
        with self.assertRaises(ValueError):
            validate_scene(s)

    def test_manifest_selection_is_explicit_and_does_not_mutate(self):
        other = deepcopy(self.scene)
        other['scenario_id'] = 'other'
        manifest = {'scenarios': [self.scene, other]}
        with self.assertRaises(ValueError):
            select_scene(manifest)
        self.assertEqual(select_scene(manifest, 'other')['scenario_id'], 'other')
        with self.assertRaises(ValueError):
            select_scene(manifest, 'missing')

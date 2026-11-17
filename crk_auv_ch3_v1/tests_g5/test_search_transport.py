import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from crk_ch3.search_transport import SearchTransport, PlantFailure
from crk_ch3.search_world import build_search_world, fixture, search_bridge_yaml
from crk_ch3.mission_transport import MotionRequest
from crk_ch3.motion_protocol import BodyState
from crk_ch3.vehicle import load_profile


class SearchPlantTests(unittest.TestCase):
    def plant(self):
        states = [BodyState(40_000_000, (-8, -8+4*i, -7), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)) for i in range(4)]
        self.commands, self.records, self.calls = [], [], []
        node = SimpleNamespace(clock_ns=40_000_000, check=lambda: None,
                               publish=lambda force: self.commands.append(np.array(force)))
        def step(count):
            self.calls.append(count)
            node.clock_ns += 20_000_000
            return [BodyState(node.clock_ns, s.position, s.quaternion, s.velocity_body, s.rates_body) for s in states], node.clock_ns
        node.step = step
        plant = SearchTransport(node, states, load_profile(), record=self.records.append)
        positions = np.array([[2, 2+4*i, 2] for i in range(4)])
        request = MotionRequest.create(1, positions, np.zeros((4, 3)), np.tile([.1, 0, 0], (4, 1)), [False]*4)
        return plant, request

    def test_ten_control_batches_and_measured_feedback_not_command_integration(self):
        plant, request = self.plant()
        result = plant(request)
        self.assertEqual(self.calls, [10]*10)
        self.assertEqual(len(self.commands), 10)
        np.testing.assert_array_equal(result.positions, request.positions)
        self.assertEqual(result.stamp_ns, 200_000_000)
        self.assertEqual(len(self.records), 10)
        self.assertTrue(any(np.any(force) for force in self.commands))

    def test_clock_mismatch_and_measured_bounds_abort(self):
        plant, request = self.plant()
        plant.node.clock_ns += 2_000_000
        with self.assertRaises(PlantFailure):
            plant(request)
        with self.assertRaises(PlantFailure):
            plant.validate([BodyState(1, (15, 0, -3), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))])

    def test_sensor_geometry_and_target_has_no_collision(self):
        text, truth = build_search_world(fixture('smoke'), load_profile())
        root = ET.fromstring(text)
        self.assertEqual(len(root.findall('.//sensor')), 4)
        self.assertEqual(len(root.findall('.//joint')), 32)
        self.assertIsNone(root.find(".//model[@name='target_truth_marker']//collision"))
        self.assertIn('/auv_3/contacts', search_bridge_yaml())
        self.assertIn('ros_gz_interfaces/msg/WorldStatistics', search_bridge_yaml())
        self.assertEqual(fixture('capture')['initial_agent_positions'][3], [4.8, 2, 2])

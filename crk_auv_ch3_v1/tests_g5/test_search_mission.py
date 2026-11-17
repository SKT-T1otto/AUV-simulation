"""Execute real native task wrappers against a stationary transport stand-in, not Gazebo."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import torch
from crk_ch3.search_world import fixture
from crk_ch3.search_transport import SearchTransport
from crk_ch3.mission_runtime import ExternalMissionRuntime
from crk_ch3.motion_protocol import BodyState
from crk_ch3.scenario import Frame
from crk_ch3.vehicle import load_profile


class SearchMissionTests(unittest.TestCase):
    def test_native_discovery_handoff_capture_through_ten_step_transport(self):
        torch.set_num_threads(1)
        scene = fixture('capture')
        states = [BodyState(40_000_000, Frame().position(p), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
                  for p in scene['initial_agent_positions']]
        commands, controls = [], []
        node = SimpleNamespace(clock_ns=40_000_000, check=lambda: None, publish=lambda f: commands.append(np.array(f)))
        def step(count):
            self.assertEqual(count, 10)
            node.clock_ns += 20_000_000
            return [BodyState(node.clock_ns, s.position, s.quaternion, s.velocity_body, s.rates_body) for s in states], node.clock_ns
        node.step = step
        plant = SearchTransport(node, states, load_profile(), record=controls.append)
        runtime = ExternalMissionRuntime(scene, plant)
        try:
            with patch.object(torch, 'load', side_effect=AssertionError('No checkpoint is permitted')):
                for _ in range(8):
                    row = runtime.advance()
                    np.testing.assert_array_equal(np.asarray(row['actions']), np.zeros((4, 3)))
                    if runtime.terminal:
                        break
            self.assertTrue(runtime.env.unwrapped.mission_complete)
            self.assertEqual(runtime.found_step, 1)
            self.assertEqual(runtime.handoff_decision_step, 2)
            self.assertGreaterEqual(runtime.env.unwrapped.capture_hold_counter_max, 5)
            self.assertEqual(len(controls), 10*runtime.step)
            self.assertEqual(len(commands), len(controls))
            np.testing.assert_array_equal(plant.max_displacement, np.zeros(4))
            self.assertTrue(all(np.max(np.abs(c)) <= 15 for c in commands))
        finally:
            runtime.close()

"""Actual ROS client methods under transport stubs; no real ROS/Gazebo acceptance."""
import importlib.util
from pathlib import Path
import sys
import contextlib
import io
import json
import os
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


def client():
    # Load native/Torch modules before temporary sys.modules replacements so
    # restoring the dictionary cannot trigger a second C-extension import.
    from crk_ch3 import mission_runtime, mission_parity
    # Reuse the already tested G4 client with isolated import stand-ins.
    path = Path(__file__).resolve().parents[1]/'tests/test_motion_ros_contract.py'
    spec = importlib.util.spec_from_file_location('g6_ros_stubs', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    motion = helper.load_client()
    modules = {}
    for name in ('rclpy', 'rclpy.signals', 'ros_gz_interfaces', 'ros_gz_interfaces.msg', 'ros_gz_interfaces.srv'):
        modules[name] = ModuleType(name)
    modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO=0)
    modules['ros_gz_interfaces.msg'].Contacts = object
    modules['ros_gz_interfaces.msg'].WorldStatistics = object
    modules['ros_gz_interfaces.srv'].SetEntityPose = SimpleNamespace(Request=lambda: SimpleNamespace(
        entity=SimpleNamespace(name='', type=0), pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=0),
                                                                    orientation=SimpleNamespace(w=0))))
    modules['crk_ch3.motion_ros'] = motion
    spec = importlib.util.spec_from_file_location('crk_ch3._search_ros_test', path.parents[1]/'crk_ch3/search_ros.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class SearchRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = client()

    def test_pose_service_only_moves_privileged_target_marker(self):
        node = self.module.SearchNode.__new__(self.module.SearchNode)
        sent = []
        def call(req):
            sent.append(req)
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))
        node.marker_client = SimpleNamespace(call_async=call)
        node.wait = lambda predicate, *_: self.assertTrue(predicate())
        node.show_target([12, 13, 4])
        self.assertEqual(sent[0].entity.name, 'target_truth_marker')
        self.assertEqual(sent[0].entity.type, 2)
        self.assertEqual((sent[0].pose.position.x, sent[0].pose.position.y, sent[0].pose.position.z), (2, 3, -5))

    def test_contact_latches_a_distinct_plant_failure(self):
        node = self.module.SearchNode.__new__(self.module.SearchNode)
        node.stopping, node.error = False, None
        node.clock_ns, node.contact_events, node.contact_failure = 100, 0, None
        records = []
        node.contact_record = records.append
        node.on_contact(0, SimpleNamespace(contacts=[]))
        self.assertEqual(node.contact_events, 0)
        contact = SimpleNamespace(collision1=SimpleNamespace(name='hull', id=1),
                                  collision2=SimpleNamespace(name='rock', id=2), depths=[.001])
        node.on_contact(0, SimpleNamespace(contacts=[contact]))
        with self.assertRaises(self.module.PhysicalContact):
            node.check()
        self.assertEqual(len(records), 1)

    def test_pause_confirmation_waits_for_stats_and_rejects_epoch_drift(self):
        node = self.module.SearchNode.__new__(self.module.SearchNode)
        node.clock_ns, node.stats_sequence, node.world_status = 40_000_000, 0, None
        node.control_world = lambda: None  # Service receipt alone is insufficient.
        def wait(predicate, *_):
            self.assertFalse(predicate())
            node.on_stats(SimpleNamespace(sim_time=SimpleNamespace(sec=0, nanosec=0),
                                          paused=True, iterations=0))
            self.assertFalse(predicate())  # A queued startup pause is stale.
            node.on_stats(SimpleNamespace(sim_time=SimpleNamespace(sec=0, nanosec=node.clock_ns),
                                          paused=False, iterations=20))
            self.assertFalse(predicate())
            node.on_stats(SimpleNamespace(sim_time=SimpleNamespace(sec=0, nanosec=node.clock_ns),
                                          paused=True, iterations=20))
            self.assertTrue(predicate())
        node.wait = wait
        self.assertEqual(node.confirm_pause(), 40_000_000)
        node.clock_ns = 42_000_000
        with self.assertRaisesRegex(self.module.PlantFailure, 'expected_ns=40000000'):
            node.confirm_pause(40_000_000)

    def test_actual_entry_point_records_native_task_on_transport_standin(self):
        from crk_ch3.search_run import create_assets
        from crk_ch3.search_world import fixture
        from crk_ch3.scenario import Frame
        from crk_ch3.motion_protocol import BodyState
        module = self.module
        class FakeNode:
            def __init__(self, contact_record):
                self.clock_ns = 40_000_000
                self.marker_calls = 0
                self.world_status = dict(sim_ns=self.clock_ns, paused=True, iterations=20)
                self.contact_failure, self.contact_events, self.stopping = None, 0, False
                self.states = [BodyState(self.clock_ns, Frame().position(p), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
                               for p in fixture('capture')['initial_agent_positions']]
            def ready(self):
                return self.states
            def check(self):
                pass
            def publish(self, values):
                pass
            def show_target(self, position):
                # Reproduce a queued clock callback delivered during startup
                # marker service work, after warmup but before the first task.
                self.marker_calls += 1
                if self.marker_calls == 1:
                    self.clock_ns += 2_000_000
            def confirm_pause(self, expected_ns=None):
                if expected_ns is not None and self.clock_ns != expected_ns:
                    raise AssertionError('Epoch moved after task startup')
                self.world_status = dict(sim_ns=self.clock_ns, paused=True, iterations=self.clock_ns//2_000_000)
                return self.clock_ns
            def step(self, count):
                if count != 10:
                    raise AssertionError('Wrong batch size')
                self.clock_ns += 20_000_000
                self.states = [BodyState(self.clock_ns, s.position, s.quaternion, s.velocity_body, s.rates_body) for s in self.states]
                return self.states, self.clock_ns
            def stop_commands(self):
                pass
            def destroy_node(self):
                pass
            def get_logger(self):
                return SimpleNamespace(info=lambda *_: None)
        with tempfile.TemporaryDirectory() as directory:
            output = create_assets(Path(directory), 'capture')
            summary = json.loads((output/'summary.json').read_text())
            summary['linux_sdf_valid'] = True  # Stubbed supervisor precheck, not Linux evidence.
            (output/'summary.json').write_text(json.dumps(summary))
            with patch.dict(os.environ, CRK_OUTPUT_ROOT=directory, CRK_G6_REALTIME_VIEW='0'), \
                    patch.object(sys, 'argv', ['search_ros', '--output', str(output)]), \
                    patch.object(module, 'SearchNode', FakeNode), patch.object(module.signal, 'signal'), \
                    patch.object(module.rclpy, 'init', lambda **_: None, create=True), \
                    patch.object(module.rclpy, 'shutdown', lambda: None, create=True), \
                    patch.object(module.rclpy, 'spin_once', lambda *a, **k: None, create=True), \
                    contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as status:
                module.main()
            self.assertEqual(status.exception.code, 0, (output/'summary.json').read_text())
            result = json.loads((output/'summary.json').read_text())
            self.assertTrue(result['g6_capture_acceptance_passed'])
            trace = [json.loads(line) for line in (output/'capture_trace.jsonl').read_text().splitlines()]
            self.assertEqual([row['native_hold'] for row in trace], [0, 1, 2, 3, 4, 5])
            self.assertEqual(result['capture_evidence']['received_step'], 2)
            self.assertEqual(result['capture_evidence']['final_consecutive_steps'], 5)
            self.assertTrue(all(max(row['distance_start_m'], row['distance_end_m']) <= row['radius_m']
                                for row in trace[1:]))
            self.assertFalse(result['g6_search_acceptance_passed'])
            self.assertEqual(result['completed_task_steps'], 6)
            self.assertEqual(result['control_steps'], 60)
            self.assertEqual(result['task_epoch_ns'], 42_000_000)
            self.assertEqual(result['startup_clock_delta_ns'], 2_000_000)
            self.assertEqual(len((output/'transitions.jsonl').read_text().splitlines()), 6)
            self.assertTrue((output/'maps.jsonl').is_file())
            self.assertTrue((output/'resolved_config.json').is_file())

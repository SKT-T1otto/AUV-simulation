"""Exercise actual client methods with ROS transport stand-ins; no real simulator."""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
import numpy as np
from crk_ch3.motion_protocol import BodyState, TeamStateBuffer


def load_client():
    modules = {}
    for name in ('rclpy', 'rclpy.duration', 'rclpy.node', 'rclpy.qos', 'rclpy.signals',
                 'nav_msgs', 'nav_msgs.msg', 'rosgraph_msgs', 'rosgraph_msgs.msg',
                 'ros_gz_interfaces', 'ros_gz_interfaces.srv', 'std_msgs', 'std_msgs.msg'):
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = object
    modules['rclpy.duration'].Duration = lambda **kw: kw
    modules['rclpy.qos'].qos_profile_sensor_data = None
    modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO=0)
    modules['nav_msgs.msg'].Odometry = object
    modules['rosgraph_msgs.msg'].Clock = object
    modules['std_msgs.msg'].Float64 = lambda **kw: SimpleNamespace(**kw)
    modules['ros_gz_interfaces.srv'].ControlWorld = SimpleNamespace(
        Request=lambda: SimpleNamespace(world_control=SimpleNamespace(pause=False, multi_step=0)))
    path = Path(__file__).resolve().parents[1]/'crk_ch3/motion_ros.py'
    spec = importlib.util.spec_from_file_location('crk_ch3._motion_ros_test', path)
    result = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(result)
    return result


class RosContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_client()

    def node(self):
        node = self.module.MotionNode.__new__(self.module.MotionNode)
        node.wait = lambda predicate, *_: None if predicate() else (_ for _ in ()).throw(TimeoutError())
        return node

    def test_world_request_is_paused_and_steps_are_explicit(self):
        node = self.node()
        requests = []
        def send(request):
            requests.append(request)
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))
        node.client = SimpleNamespace(call_async=send)
        node.control_world(10)
        self.assertTrue(requests[0].world_control.pause)
        self.assertEqual(requests[0].world_control.multi_step, 10)
        node.client = SimpleNamespace(call_async=lambda _: SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=False)))
        with self.assertRaises(RuntimeError):
            node.control_world(10)

    def test_actual_step_waits_for_shared_four_state_timestamp(self):
        node = self.node()
        node.clock_ns = 40_000_000
        node.buffer = TeamStateBuffer()
        def advance(steps):
            self.assertEqual(steps, 10)
            node.clock_ns = 60_000_000
            for i in range(4):
                node.buffer.push(i, BodyState(60_000_000, (i, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)),
                                 child_frame=f'auv_{i}/base_link')
        node.control_world = advance
        states, target = node.step(10)
        self.assertEqual(target, 60_000_000)
        self.assertEqual(len(states), 4)
        node.control_world = lambda _: setattr(node, 'clock_ns', 100_000_000)
        with self.assertRaises(RuntimeError):
            node.step(10)

    def test_thrust_publication_maps_every_agent_and_motor_without_cross_wiring(self):
        node = self.node()
        received = {}
        node.publishers_ = [[SimpleNamespace(publish=lambda msg, key=(i, j): received.__setitem__(key, msg.data),
                                             wait_for_all_acked=lambda _: True) for j in range(8)] for i in range(4)]
        values = np.arange(32).reshape(4, 8)/10
        node.publish(values)
        self.assertEqual(len(received), 32)
        for key, value in received.items():
            self.assertEqual(value, values[key])
        with self.assertRaises(ValueError):
            node.publish(np.full((4, 8), np.nan))
        node.publishers_[0][0].wait_for_all_acked = lambda _: False
        with self.assertRaises(TimeoutError):
            node.publish(values)

    def test_recording_retries_only_the_initial_pause(self):
        node = self.node()
        node.client = Mock()
        node.retry_startup_pause_timeout = True
        node.publishers_ = []
        node.publish = Mock()
        node.get_logger = Mock()
        node.control_world = Mock(side_effect=[TimeoutError('startup'), None])
        node.step = Mock(return_value=('ready',0))
        self.assertEqual(node.start(),'ready')
        self.assertEqual(len(node.control_world.call_args_list),2)
        self.assertTrue(all(not call.args for call in node.control_world.call_args_list))
        node.step.assert_called_once_with(20)
        node.control_world = Mock(side_effect=TimeoutError('step timeout'))
        node.clock_ns = 0
        with self.assertRaises(TimeoutError):
            self.module.MotionNode.step(node,10)
        node.control_world.assert_called_once_with(10)

    def test_step_does_not_complete_one_physics_substep_early(self):
        node = self.node()
        node.clock_ns = 40_000_000
        node.buffer = TeamStateBuffer()
        def not_finished(_):
            node.clock_ns = 58_000_000
            for i in range(4):
                node.buffer.push(i, BodyState(58_000_000, (i, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)),
                                 child_frame=f'auv_{i}/base_link')
        node.control_world = not_finished
        with self.assertRaises(TimeoutError):
            node.step(10)

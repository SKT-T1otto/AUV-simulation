import ast
import inspect
import json
from pathlib import Path
import unittest

from crk_ch3.contracts import verify_source_lock
from crk_ch3.native_source import ROOT, SNAPSHOT, activate_native


class SourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        activate_native()

    def test_snapshot_has_exact_locked_bytes_and_keeps_g1_lock(self):
        lock = activate_native()
        self.assertEqual(verify_source_lock(lock, SNAPSHOT), 253)
        self.assertFalse(lock['identity_only_not_runtime_snapshot'])
        self.assertEqual(lock['evolution_from_g1']['changed'], [])
        self.assertEqual(lock['evolution_from_g1']['removed'], [])
        old = json.loads((ROOT/'config/source_lock.json').read_text(encoding='utf-8'))
        self.assertEqual(old['files_sha256'], lock['previous_g1_files_sha256'])

    def test_motion_adapter_keeps_exact_source_action_and_task_bookkeeping(self):
        from core.env.uav_env import _BaseUAVEnv
        from crk_ch3.mission_dynamics import ExternalDynamics
        native = inspect.getsource(_BaseUAVEnv._apply_agent_dynamics)
        port = inspect.getsource(ExternalDynamics._apply_agent_dynamics)
        before = '        flow = self._flow_at(self._agent_pos)'
        after = '        if self.task_protocol == STRICT:'
        self.assertEqual(native[:native.index(before)], port[:port.index('        # G5 replaces')])
        self.assertEqual(native[native.index(after):], port[port.index(after):])
        tree = ast.parse('class Check:\n'+port)
        self.assertFalse(any(isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Attribute)
                             and n.target.attr in ('_agent_pos', '_agent_vel') for n in ast.walk(tree)))


if __name__ == '__main__':
    unittest.main()

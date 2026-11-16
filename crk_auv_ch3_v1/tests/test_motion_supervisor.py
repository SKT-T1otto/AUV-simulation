import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from crk_ch3 import motion_run


class SupervisorTests(unittest.TestCase):
    def test_assets_are_unique_and_never_claim_motion_or_mission_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first, second = motion_run.create_assets(root), motion_run.create_assets(root)
            self.assertNotEqual(first, second)
            summary = json.loads((first/'summary.json').read_text())
            for key in ('motion_started', 'mission_executed', 'checkpoint_loaded', 'engineering_acceptance_passed'):
                self.assertFalse(summary[key])
            self.assertTrue((first/'world.sdf').is_file())
            self.assertTrue((first/'bridge.yaml').is_file())

    def run_mocked(self, root, sdf_exit, popen):
        modules = {'fcntl': SimpleNamespace(flock=lambda *args: None, LOCK_EX=1, LOCK_NB=2),
                   'ament_index_python': SimpleNamespace(),
                   'ament_index_python.packages': SimpleNamespace(get_package_prefix=lambda _: '/fake')}
        with patch.dict(sys.modules, modules), patch.dict(os.environ, {'CRK_OUTPUT_ROOT': str(root)}), \
                patch.object(sys, 'argv', ['motion_run']), \
                patch.object(motion_run.signal, 'signal'), \
                patch.object(motion_run.Path, 'is_mount', return_value=True), \
                patch.object(motion_run.os, 'geteuid', return_value=1000, create=True), \
                patch.object(motion_run.subprocess, 'run', return_value=SimpleNamespace(returncode=sdf_exit)), \
                patch.object(motion_run.subprocess, 'Popen', side_effect=popen), \
                patch.object(motion_run.os, 'killpg', create=True), contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            motion_run.main()
        summary_path = next(root.glob('results/ch3/motion_*/summary.json'))
        return error.exception.code, json.loads(summary_path.read_text())

    def test_sdf_failure_stops_before_launching_processes_and_is_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            def unexpected(*args, **kwargs):
                self.fail('No process should launch after invalid SDF')
            status, summary = self.run_mocked(Path(directory), 1, unexpected)
            self.assertEqual(status, 1)
            self.assertFalse(summary['linux_sdf_valid'])
            self.assertFalse(summary['engineering_acceptance_passed'])
            self.assertEqual(summary['termination_reason'], 'launch_or_infrastructure_failure')

    def test_zero_exit_without_acceptance_summary_is_not_success(self):
        class FakeProcess:
            def __init__(self, controller):
                self.pid = 999999
                self.controller = controller
                self.returncode = 0 if controller else None
                self.stdout = io.StringIO('probe controller exited\n')
            def poll(self):
                return self.returncode
            def wait(self, timeout=None):
                self.returncode = 0 if self.controller else -2
                return self.returncode
        with tempfile.TemporaryDirectory() as directory:
            def spawn(command, **kwargs):
                return FakeProcess('crk_ch3.motion_ros' in command)
            status, summary = self.run_mocked(Path(directory), 0, spawn)
            self.assertEqual(status, 1)
            self.assertFalse(summary['engineering_acceptance_passed'])
            self.assertIn('without a valid acceptance summary', summary['error'])

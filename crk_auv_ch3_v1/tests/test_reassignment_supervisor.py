"""Task-free explicit-entry and supervisor failure checks."""
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
from crk_ch3 import search_run
from crk_ch3.reassignment_contract import PREFIX_CRITERIA


class DiagnosticSupervisorTests(unittest.TestCase):
    def test_assets_require_explicit_condition_original_scene_and_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            default=search_run.create_assets(root,'search')
            self.assertNotIn('diagnostic_condition',json.loads((default/'summary.json').read_text()))
            output=search_run.create_assets(root,'search',diagnostic_condition='baseline32')
            meta=json.loads((output/'summary.json').read_text())
            self.assertEqual(meta['native_max_steps'],400)
            self.assertEqual(meta['diagnostic_budget'],32)
            self.assertEqual(meta['prefix_criteria'],PREFIX_CRITERIA)
            self.assertEqual((output/'world.sdf').read_bytes(),(default/'world.sdf').read_bytes())
            self.assertEqual((output/'scenario.json').read_bytes(),(default/'scenario.json').read_bytes())
            with self.assertRaises(ValueError):search_run.create_assets(root,diagnostic_condition='baseline32')
            with self.assertRaises(ValueError):search_run.create_assets(root,'search',{},diagnostic_condition='baseline32')

    def test_real_supervisor_selects_diagnostic_module_and_rejects_sdf_failure(self):
        for sdf_exit in (0,1):
            with self.subTest(sdf_exit=sdf_exit),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);commands=[]
                class Process:
                    pid=999999
                    def __init__(self,command):
                        self.controller='crk_ch3.reassignment_ros' in command
                        self.returncode=0 if self.controller else None
                        self.stdout=io.StringIO('diagnostic supervisor stand-in\n')
                    def poll(self):return self.returncode
                    def wait(self,timeout=None):self.returncode=-2
                def spawn(command,**kwargs):
                    commands.append(command)
                    if 'crk_ch3.reassignment_ros' in command:
                        output=Path(command[-1])
                        search_run.update_summary(output,engineering_acceptance_passed=True,diagnostic_valid=True,
                            termination_reason='diagnostic_budget_completed',completed_task_steps=32)
                    return Process(command)
                modules={'fcntl':SimpleNamespace(flock=lambda *_:None,LOCK_EX=1,LOCK_NB=2),
                    'ament_index_python':SimpleNamespace(),
                    'ament_index_python.packages':SimpleNamespace(get_package_prefix=lambda _:'/fake')}
                with patch.dict(sys.modules,modules),patch.dict(os.environ,CRK_OUTPUT_ROOT=tmp), \
                    patch.object(sys,'argv',['search_run','--mode','search','--diagnostic-condition','baseline32','--headless']), \
                    patch.object(search_run.signal,'signal'),patch.object(search_run.Path,'is_mount',return_value=True), \
                    patch.object(search_run.os,'geteuid',return_value=1000,create=True), \
                    patch.object(search_run.os,'killpg',create=True), \
                    patch.object(search_run.subprocess,'run',return_value=SimpleNamespace(returncode=sdf_exit)), \
                    patch.object(search_run.subprocess,'Popen',side_effect=spawn), \
                    contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as status:
                    search_run.main()
                summary=json.loads(next(root.glob('results/ch3/d2_*/summary.json')).read_text())
                if sdf_exit:
                    self.assertEqual(status.exception.code,1)
                    self.assertFalse(summary['diagnostic_valid'])
                    self.assertEqual(commands,[])
                    self.assertIn('supervisor_launch_or_infrastructure_failure',summary['failure_tags'])
                else:
                    self.assertEqual(status.exception.code,0)
                    self.assertTrue(any('crk_ch3.reassignment_ros' in c for c in commands))
                    self.assertFalse(summary['g6_search_acceptance_passed'])


if __name__=='__main__':unittest.main()

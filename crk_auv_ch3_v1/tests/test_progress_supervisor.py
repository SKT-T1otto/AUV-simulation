import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from crk_ch3 import search_run
from crk_ch3.progress_commit import CONDITIONS


class SupervisorTests(unittest.TestCase):
    def test_explicit_assets_keep_original_world_and_do_not_change_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);original=search_run.create_assets(root,'search')
            for condition in CONDITIONS:
                output=search_run.create_assets(root,'search',progress_condition=condition)
                self.assertEqual((output/'world.sdf').read_bytes(),(original/'world.sdf').read_bytes())
                self.assertEqual((output/'scenario.json').read_bytes(),(original/'scenario.json').read_bytes())
                self.assertEqual(json.loads((output/'summary.json').read_text())['diagnostic_budget'],32)
            self.assertNotIn('diagnostic_schema',json.loads((original/'summary.json').read_text()))
            with self.assertRaises(ValueError):search_run.create_assets(root,progress_condition='search_progress32')
            with self.assertRaises(ValueError):search_run.create_assets(root,'search',diagnostic_condition='baseline32',progress_condition='search_progress32')

    def test_supervisor_selects_real_entry_and_names_infrastructure_failures(self):
        for condition,module in [('fixture_progress32','crk_ch3.progress_fixture_ros'),('search_progress32','crk_ch3.reassignment_ros')]:
            for invalid_sdf in (False,True):
                with self.subTest(condition=condition,invalid_sdf=invalid_sdf),tempfile.TemporaryDirectory() as tmp:
                    commands=[]
                    class Process:
                        pid=999999
                        def __init__(self,command):
                            self.returncode=0 if module in command else None;self.stdout=io.StringIO('stub\n')
                        def poll(self):return self.returncode
                        def wait(self,timeout=None):self.returncode=-2
                    def spawn(command,**kwargs):
                        commands.append(command)
                        if module in command:search_run.update_summary(Path(command[-1]),engineering_acceptance_passed=True,
                            diagnostic_valid=True,termination_reason='diagnostic_budget_completed')
                        return Process(command)
                    modules={'fcntl':NS(flock=lambda *_:None,LOCK_EX=1,LOCK_NB=2),
                        'ament_index_python':NS(),'ament_index_python.packages':NS(get_package_prefix=lambda _:'/fake')}
                    with patch.dict(sys.modules,modules),patch.dict(os.environ,CRK_OUTPUT_ROOT=tmp), \
                        patch.object(sys,'argv',['search_run','--mode','search','--progress-condition',condition,'--headless']), \
                        patch.object(search_run.signal,'signal'),patch.object(search_run.Path,'is_mount',return_value=True), \
                        patch.object(search_run.os,'geteuid',return_value=1000,create=True),patch.object(search_run.os,'killpg',create=True), \
                        patch.object(search_run.subprocess,'run',return_value=NS(returncode=int(invalid_sdf))), \
                        patch.object(search_run.subprocess,'Popen',side_effect=spawn),contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as status:
                        search_run.main()
                    summary=json.loads(next(Path(tmp).glob('results/ch3/progress_*/summary.json')).read_text())
                    self.assertEqual(status.exception.code,int(invalid_sdf))
                    if invalid_sdf:
                        self.assertFalse(summary['diagnostic_valid']);self.assertEqual(commands,[])
                    else:self.assertTrue(any(module in c for c in commands))


if __name__=='__main__':unittest.main()

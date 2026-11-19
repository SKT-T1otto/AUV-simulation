"""Run task-free component suite with native task constructors forbidden."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from contextlib import ExitStack
from crk_ch3.mission_runtime import ExternalMissionRuntime
from crk_ch3.mission_parity import RepairedMissionRuntime


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-dir',type=Path,required=True)
    args=parser.parse_args();audit=args.audit_dir.resolve()
    plan=json.loads((audit/'budget_plan.json').read_text(encoding='utf-8'))
    if plan['local_native_task_instances_allowed']!=0 or plan['previous_budget_reset']:
        raise ValueError('This stage requires zero new task instances and no previous-budget reset')
    calls=[]
    def forbidden(*args,**kwargs):
        calls.append('forbidden_native_task_constructor_or_factory')
        raise AssertionError('progress_commit_v1 local validation forbids native task construction')
    root=Path(__file__).resolve().parents[1]
    logfile=audit/('component_tests_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.log')
    with ExitStack() as stack:
        stack.enter_context(patch.object(ExternalMissionRuntime,'__init__',forbidden))
        stack.enter_context(patch.object(RepairedMissionRuntime,'__init__',forbidden))
        stack.enter_context(patch('crk_ch3.mission_runtime._make_base_env',forbidden))
        stack.enter_context(patch('chapter3_bser.learning.base_runtime._make_base_env',forbidden))
        suite=unittest.defaultTestLoader.discover(str(root/'tests'))
        with logfile.open('x',encoding='utf-8') as stream:
            result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    value=dict(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        passed=result.wasSuccessful() and not calls,native_task_constructor_attempts=len(calls),
        native_task_instances_created=0,previous_budget_reset=False,training=False,checkpoint_loaded=False,
        evidence='component/native-class and actual-entry ROS/state-tape stand-ins; no Linux/Gazebo evidence',
        logfile=str(logfile))
    (audit/'verification.json').write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(value))
    raise SystemExit(0 if value['passed'] else 1)


if __name__=='__main__':main()

"""Bounded Windows/native-task checks, not Gazebo evidence. Reserve before init."""
import argparse
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import torch
from crk_ch3.mission_runtime import ExternalMissionRuntime
from crk_ch3.mission_transport import MotionFeedback
from crk_ch3.mission_parity import normalized
from crk_ch3.reassignment_gate import ReassignmentGate, DiagnosticInvalid
from crk_ch3.reassignment_report import compare, route_signature
from crk_ch3.search_run import create_assets
from crk_ch3.search_world import fixture
from crk_ch3.motion_protocol import BodyState
from crk_ch3.scenario import Frame


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path,value):
    path.write_text(json.dumps(normalized(value),indent=2)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-dir',type=Path,required=True)
    parser.add_argument('--input-run',type=Path,required=True)
    parser.add_argument('--phase',choices=('mechanism','entry','entry_recovery'),required=True)
    args=parser.parse_args()
    audit=args.audit_dir.resolve()
    plan=load(audit/'budget_plan.json')
    ledger_path=audit/'instance_budget.json'
    ledger=load(ledger_path) if ledger_path.exists() else dict(max_instances=8,max_steps=32,instances=[])
    trans=[json.loads(s) for s in (args.input_run/'transitions.jsonl').read_text(encoding='utf-8').splitlines()]
    control=[json.loads(s) for s in (args.input_run/'control.jsonl').read_text(encoding='utf-8').splitlines()]
    scene=load(args.input_run/'scenario.json')
    assert scene==fixture('search'), 'Never substitute the original scene'
    torch.set_num_threads(1)
    reserved=[]
    original_init,original_advance=ExternalMissionRuntime.__init__,ExternalMissionRuntime.advance
    def reserve(self,*a,**kw):
        label=reserved.pop(0)
        assert len(ledger['instances'])<plan['max_instances']
        assert label not in [r['label'] for r in ledger['instances']]
        item=dict(label=label,status='reserved_before_construction',attempted_steps=0,completed_steps=0,
                  evidence='Windows feedback stand-in; not new Gazebo evidence')
        ledger['instances'].append(item)
        write(ledger_path,ledger)
        self._diagnostic_budget_item=item
        original_init(self,*a,**kw)
        item['status']='constructed'
        write(ledger_path,ledger)
    def advance(self,*a,**kw):
        item=self._diagnostic_budget_item
        assert item['attempted_steps']<32
        item['attempted_steps']+=1
        write(ledger_path,ledger)
        try:
            result=original_advance(self,*a,**kw)
            item['completed_steps']+=1
            return result
        finally:
            write(ledger_path,ledger)
    def make(label,callback):
        reserved.append(label)
        return ExternalMissionRuntime(scene,callback)
    def feedback(request):
        k=request.task_step
        return MotionFeedback.for_request(request,trans[k-1]['transition']['motion']['measured_positions_native'],
                                         control[k*10-1]['velocity_world'])
    checks={}
    with patch.object(ExternalMissionRuntime,'__init__',reserve),patch.object(ExternalMissionRuntime,'advance',advance):
        if args.phase=='mechanism':
            reference=[]
            for label,condition in [('original_tape32',None),('baseline_gate_tape32','baseline32')]:
                runtime=make(label,feedback)
                gate=ReassignmentGate(runtime,condition,lambda *_:None) if condition else None
                actual=[]
                try:
                    for _ in range(32):
                        before=normalized(runtime.guidance)
                        result=normalized(runtime.advance())
                        if gate:gate.after_advance()
                        actual.append(dict(before=before,after=normalized(runtime.guidance),transition=result,
                            allocation=normalized(runtime.controller.inner.current_allocation)))
                finally:runtime.close()
                if condition:assert actual==reference,'baseline gate changed the native flow'
                else:reference=actual
            checks['baseline_equals_original_32_steps']=True
            runtime=None
            def arriving(request):
                positions=np.asarray(trans[request.task_step-1]['transition']['motion']['measured_positions_native']).copy()
                if request.task_step>=21:
                    positions[0]=runtime.guidance.agent_assignments[0].tracking_waypoint
                return MotionFeedback.for_request(request,positions,control[request.task_step*10-1]['velocity_world'])
            runtime=make('defer_gate_tracker32',arriving)
            records=[]
            gate=ReassignmentGate(runtime,'defer_search_reassignment32',lambda name,row:records.append(row))
            before_index=after_index=None
            try:
                for k in range(32):
                    before=normalized(runtime.guidance)
                    result=normalized(runtime.advance())
                    gate.after_advance()
                    if k<19:
                        assert before==reference[k]['before']
                        assert result==reference[k]['transition']
                        assert normalized(runtime.guidance)==reference[k]['after']
                    if k==19:
                        assert before==reference[k]['before']
                        assert result['motion']['requested_acceleration_native']==reference[k]['transition']['motion']['requested_acceleration_native']
                        before_index=runtime.bridge.path_tracker.snapshot(0).next_index
                    if k==20:after_index=runtime.bridge.path_tracker.snapshot(0).next_index
                assert after_index>before_index, (before_index,after_index)
                assert gate.deferred_commits>0
                assert records[19]['raw_policy_decision']['decision']['should_replan']
                assert records[19]['stabilization'][0]['search_commit_deferred']
                checks.update(prefix19_exact=True,motion20_request_exact=True,
                    old_route_tracker_advanced=dict(before=before_index,after=after_index),
                    search_commits_deferred=gate.deferred_commits)
                write(audit/'gate_boundary_evidence.json',dict(boundary=records[19],next_step=records[20]))
            finally:runtime.close()
            runtime=make('invalid_retained_route20',feedback)
            gate=ReassignmentGate(runtime,'defer_search_reassignment32',lambda *_:None)
            try:
                for _ in range(19):runtime.advance();gate.after_advance()
                geometry=SimpleNamespace(path_free=lambda *_:False)
                with patch('crk_ch3.reassignment_gate.PublicGeometry',return_value=geometry):
                    try:runtime.advance()
                    except DiagnosticInvalid:checks['invalid_path_stopped_after_motion20']=True
                    else:raise AssertionError('Invalid retained path was not stopped')
                assert runtime.poisoned and runtime.motion_exchange.last_request.task_step==20
            finally:runtime.close()
            write(audit/'mechanism_checks.json',checks)
        else:
            assert load(audit/'mechanism_checks.json')['baseline_equals_original_32_steps']
            helper_path=Path(__file__).resolve().parents[1]/'tests_g5/test_search_ros_contract.py'
            spec=importlib.util.spec_from_file_location('diagnostic_ros_stubs',helper_path)
            helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
            ros=helper.client()
            with patch.dict(sys.modules,{'crk_ch3.search_ros':ros}):
                from crk_ch3 import reassignment_ros as module
            module.ros=ros
            class FakeNode:
                fault=None
                def __init__(self,contact_record):
                    self.clock_ns=40_000_000
                    self.world_status=dict(sim_ns=self.clock_ns,paused=True,iterations=20)
                    self.contact_failure,self.contact_events,self.stopping=None,0,False
                    self.contact_record=contact_record
                    self.index=0
                    self.states=[BodyState(self.clock_ns,Frame().position(p),(0,0,0,1),(0,0,0),(0,0,0))
                                 for p in scene['initial_agent_positions']]
                def ready(self):
                    if self.fault=='interrupt':raise KeyboardInterrupt
                    if self.fault=='contact_startup':
                        self.contact_failure='injected startup contact'
                        self.contact_events=1
                        self.contact_record(dict(injected=True,clock_ns=self.clock_ns))
                        raise ros.PhysicalContact(self.contact_failure)
                    return self.states
                def check(self):
                    if self.fault=='contact' and self.index==1:
                        self.contact_failure='injected contact'
                        self.contact_events=1
                        self.contact_record(dict(injected=True,clock_ns=self.clock_ns))
                        raise ros.PhysicalContact(self.contact_failure)
                def publish(self,values):pass
                def show_target(self,position):pass
                def confirm_pause(self,expected_ns=None):
                    if expected_ns is not None and self.clock_ns!=expected_ns:raise RuntimeError('Clock mismatch')
                    self.world_status=dict(sim_ns=self.clock_ns,paused=True,iterations=self.clock_ns//2_000_000)
                    return self.clock_ns
                def step(self,count):
                    assert count==10
                    r=control[self.index];self.index+=1
                    self.clock_ns+=22_000_000 if self.fault=='clock' and self.index==1 else 20_000_000
                    self.states=[BodyState(self.clock_ns,tuple(p),(0,0,0,1),tuple(v),(0,0,0))
                                 for p,v in zip(r['positions_world'],r['velocity_world'])]
                    return self.states,self.clock_ns
                def stop_commands(self):pass
                def destroy_node(self):pass
                def get_logger(self):return SimpleNamespace(info=lambda *_:None)
            outputs={}
            cases=[('ros_entry_baseline32','baseline32',None),
                    ('ros_entry_defer32','defer_search_reassignment32',None),
                    ('ros_entry_clock_fault','baseline32','clock'),
                    ('ros_entry_contact_fault','baseline32','contact'),
                    ('ros_entry_interrupt_before_runtime','baseline32','interrupt')]
            if args.phase=='entry_recovery':
                cases[0]=('ros_entry_baseline32_fresh','baseline32',None)
                cases[3]=('ros_entry_contact_before_runtime','baseline32','contact_startup')
            for label,condition,fault in cases:
                output=create_assets(audit,'search',diagnostic_condition=condition)
                summary=load(output/'summary.json')
                summary.update(linux_sdf_valid=True,validation_evidence='STUB ONLY; no Linux/Gazebo')
                write(output/'summary.json',summary)
                FakeNode.fault=fault
                if fault not in ('interrupt','contact_startup'):reserved.append(label)
                with patch.dict(os.environ,CRK_OUTPUT_ROOT=str(audit),CRK_G6_REALTIME_VIEW='0'), \
                    patch.object(sys,'argv',['reassignment_ros','--output',str(output)]), \
                    patch.object(ros,'SearchNode',FakeNode),patch.object(module.signal,'signal'), \
                    patch.object(ros.rclpy,'init',lambda **_:None,create=True), \
                    patch.object(ros.rclpy,'shutdown',lambda:None,create=True), \
                    patch.object(ros.rclpy,'spin_once',lambda *a,**k:None,create=True), \
                    contextlib.redirect_stdout(io.StringIO()):
                    try:module.main()
                    except SystemExit as status:exit_code=status.code
                    else:raise AssertionError('Missing entry exit code')
                summary=load(output/'summary.json')
                checks[label]=dict(exit_code=exit_code,output=str(output),summary=summary)
                write(audit/'entry_checks.json',checks)
                assert (exit_code==0)==(fault is None),summary
                assert summary['stop_zero_command_sent']
                assert not summary['task_success'] if 'task_success' in summary else True
                assert not summary['g6_search_acceptance_passed']
                assert (output/'diagnostic_metrics.json').exists(),summary
                if fault is None:
                    assert summary['completed_task_steps']==32 and summary['control_steps']==320
                    assert summary['termination_reason']=='diagnostic_budget_completed'
                    outputs[condition]=output
                else:
                    expected={'clock':'engineering_failure','contact':'physical_contact',
                              'contact_startup':'physical_contact','interrupt':'interrupted'}[fault]
                    assert summary['termination_reason']==expected,summary
            paired=compare(outputs['baseline32'],outputs['defer_search_reassignment32'])
            assert paired['prefix_comparable'],paired['issues']
            write(audit/'standin_pair.json',paired)
            # Deliberately corrupt a derived in-memory prefix input; do not rerun a task.
            b=outputs['defer_search_reassignment32']
            path=b/'initial.jsonl';original=path.read_bytes()
            initial=json.loads(original);initial['body_states'][0]['position'][0]+=.1
            try:
                path.write_text(json.dumps(initial)+'\n',encoding='utf-8')
                assert not compare(outputs['baseline32'],b)['prefix_comparable']
            finally:path.write_bytes(original)
            checks['prefix_fault_rejected_without_new_runtime']=True
            write(audit/'entry_checks.json',checks)
    assert not reserved
    assert len(ledger['instances'])<=8 and max(i['attempted_steps'] for i in ledger['instances'])<=32
    print(json.dumps(dict(phase=args.phase,passed=True,instances=len(ledger['instances']),checks=list(checks))))


if __name__=='__main__':main()

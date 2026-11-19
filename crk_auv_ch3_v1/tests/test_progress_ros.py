"""Real ROS entry functions under prescribed state tapes; no task constructors."""
import contextlib
from dataclasses import replace,dataclass
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from crk_ch3.motion_protocol import BodyState
from crk_ch3.mission_transport import MotionRequest
from crk_ch3.progress_report import compare
from crk_ch3.scenario import Frame
from crk_ch3.search_run import create_assets
from crk_ch3.search_world import fixture
from progress_support import (Facade,search,GridGeometryView,PlanningGraphView,state)


def client(filename):
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('progress_ros_helper',root/'tests_g5/test_search_ros_contract.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    ros=helper.client()
    spec=importlib.util.spec_from_file_location('crk_ch3._progress_'+Path(filename).stem,root/'crk_ch3'/filename)
    module=importlib.util.module_from_spec(spec)
    import crk_ch3
    with patch.dict(sys.modules,{'crk_ch3.search_ros':ros}),patch.object(crk_ch3,'search_ros',ros,create=True):
        spec.loader.exec_module(module)
    module.ros=ros
    return module


class PrescribedNode:
    fault=None
    def __init__(self,contact_record):
        self.clock_ns=40_000_000;self.index=0;self.contact_record=contact_record
        self.stopping=False;self.contact_failure=None;self.contact_events=0
        self.world_status=dict(sim_ns=self.clock_ns,paused=True,iterations=20)
        self.origins=np.array([Frame().position(p) for p in fixture('search')['initial_agent_positions']])
        self.states=[BodyState(self.clock_ns,tuple(p),(0,0,0,1),(0,0,0),(0,0,0)) for p in self.origins]
    def ready(self):return self.states
    def check(self):
        if self.index==1 and self.fault=='contact':
            self.contact_failure='injected';self.contact_events=1
            self.contact_record(dict(clock_ns=self.clock_ns,injected=True));raise self.contact_type('injected')
        if self.index==1 and self.fault=='interrupt':raise KeyboardInterrupt
    def publish(self,values):pass
    def show_target(self,value):pass
    def confirm_pause(self,expected_ns=None):
        if expected_ns is not None and expected_ns!=self.clock_ns:raise RuntimeError('Injected clock mismatch')
        self.world_status=dict(sim_ns=self.clock_ns,paused=True,iterations=self.clock_ns//2_000_000)
        return self.clock_ns
    def step(self,count):
        assert count==10
        self.index+=1;self.clock_ns+=22_000_000 if self.fault=='clock' and self.index==1 else 20_000_000
        # Prescribed tape, intentionally independent of forces; not a plant model.
        self.states=[BodyState(self.clock_ns,tuple(p+[0,.005*self.index if i<3 else 0,0]),(0,0,0,1),
            (0,.25 if i<3 else 0,0),(0,0,0)) for i,p in enumerate(self.origins)]
        return self.states,self.clock_ns
    def stop_commands(self):pass
    def destroy_node(self):pass
    def get_logger(self):return NS(info=lambda *_:None)


@dataclass
class TaskFlags:
    target_found: bool=False
    executor_knows_target: bool=False
    mission_complete: bool=False


class RuntimeFacade(Facade):
    """Wiring stand-in, not ExternalMissionRuntime or a native environment."""
    def __init__(self,scene,callback):
        super().__init__();self.flags=TaskFlags();self.callback=callback;self.config=dict(max_steps=400,observation_dim=28,action_dim=3)
        self.positions=np.asarray(scene['initial_agent_positions'],dtype=float);self.observations=np.zeros((4,28))
        self.velocities=np.zeros((4,3));self.update_state()
        initial=replace(self.inner.current_allocation,search_assignments=tuple(
            search(i,tuple(p+[-1,-1,-.5]),tuple(p),'old') for i,p in enumerate(self.positions[:3])))
        self.inner.current_allocation=initial;self.inner.waypoints.commit(initial);self.compile(initial)
        module=self.env.unwrapped.map_module
        for name in ('known_free_mask','known_occupied_mask'):setattr(module,name,np.zeros(800,bool))
        module.unknown_mask=np.ones(800,bool);module.occupancy_probability=np.full(800,.5)
        module.coverage=np.zeros(800);module.belief_map=np.full(800,1/800)
        native=self.env.unwrapped;native.map_sensor_scan_count=104;native.mission_complete=False
        native.last_reward_components={};self.env.last_reward_breakdown={}
        native.target_state=NS(position=(16,16,4),to_payload=lambda:dict(position=[16,16,4]))
        native.enable_public_event_capture=lambda *a,**kw:None;native._public_event_journal=NS(reset=lambda:None)
        native.read_public_events=lambda *_:[]
        native.get_episode_result=lambda:dict(termination_reason='running',remaining_task_steps=400-self.step)
        self.env.get_public_vehicle_specs=lambda:dict(scope='FAKE entry contract only')
    def update_state(self):
        small=state(self.step)
        xyz=np.array([(x,y,z) for x in range(1,20,2) for y in range(1,20,2) for z in np.arange(.5,8,1)])
        grid=GridGeometryView((10,10,8),(0,0,0),(2,2,1),xyz)
        occ=replace(small.occupancy,occupied_mask=np.zeros(800,bool),free_mask=np.zeros(800,bool),
            unknown_mask=np.ones(800,bool),known_mask=np.zeros(800,bool),occupancy_probability=np.full(800,.5))
        graph=replace(small.planning_graph,shape=(10,10,8),cell_centers=xyz,valid_mask=np.ones(800,bool),
            component_labels=np.zeros(800,int),searcher_adjacency=((),)*800,executor_adjacency=((),)*800,endpoint_connectors=())
        agents=tuple(replace(a,position=tuple(p),velocity=tuple(v)) for a,p,v in zip(small.agents,self.positions,self.velocities))
        self.state=replace(small,grid=grid,occupancy=occ,planning_graph=graph,agents=agents)
    def advance(self):
        before=self.step;guide=self.guidance
        acceleration=np.array([np.asarray(g.tracking_waypoint)-p for g,p in zip(guide.agent_assignments,self.positions)])
        request=MotionRequest.create(before+1,self.positions,self.velocities,acceleration,[False]*4)
        feedback=self.callback(request)
        self.positions=np.array(feedback.positions);self.velocities=np.array(feedback.velocities_world)
        self.step+=1;self.update_state();self.env.unwrapped.map_module.map_revision=self.step
        self.env.unwrapped.map_sensor_scan_count+=104
        result=self.controller.step(self.state);self.compile(result.allocation)
        return dict(t=before,actions=np.zeros((4,3)),observations=self.observations.copy(),next_observations=self.observations.copy(),
            rewards=[0.]*4,dones=[False]*4,motion=dict(requested_acceleration_native=acceleration.tolist(),
            feedback_stamp_ns=feedback.stamp_ns,measured_positions_native=self.positions.tolist(),task_positions_native=self.positions.tolist()))
    def close(self):pass


class EntryTests(unittest.TestCase):
    def run_entry(self,module,root,condition,fault=None):
        output=create_assets(root,'search',progress_condition=condition)
        meta=json.loads((output/'summary.json').read_text());meta.update(linux_sdf_valid=True,validation_evidence='ROS/state-tape stand-in ONLY; no Linux or Gazebo')
        (output/'summary.json').write_text(json.dumps(meta))
        PrescribedNode.fault=fault;PrescribedNode.contact_type=module.ros.PhysicalContact
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ,CRK_OUTPUT_ROOT=str(root),CRK_G6_REALTIME_VIEW='0'))
            stack.enter_context(patch.object(sys,'argv',['entry','--output',str(output)]))
            stack.enter_context(patch.object(module.ros,'SearchNode',PrescribedNode))
            stack.enter_context(patch.object(module.signal,'signal'))
            for name,func in [('init',lambda **kw:None),('shutdown',lambda:None),('spin_once',lambda *a,**kw:None)]:
                stack.enter_context(patch.object(module.ros.rclpy,name,func,create=True))
            if hasattr(module,'ExternalMissionRuntime'):stack.enter_context(patch.object(module,'ExternalMissionRuntime',RuntimeFacade))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaises(SystemExit) as status:module.main()
        value=json.loads((output/'summary.json').read_text())
        self.assertEqual(status.exception.code,0 if fault is None else 1,value)
        self.assertTrue(value['stop_zero_command_sent']);self.assertFalse(value['task_success'])
        self.assertFalse(value['g6_search_acceptance_passed']);self.assertTrue((output/'progress_metrics.json').exists(),value)
        return output,value

    def test_actual_fixture_entries_produce_wait_release_fresh_start_and_pair(self):
        module=client('progress_fixture_ros.py')
        with tempfile.TemporaryDirectory() as tmp:
            a,ma=self.run_entry(module,Path(tmp),'fixture_baseline32')
            b,mb=self.run_entry(module,Path(tmp),'fixture_progress32')
            self.assertEqual(mb['delayed_search_commits'],3);self.assertTrue(all(k>5 for k in mb['adapter_commit_steps']))
            self.assertEqual(mb['native_task_instances'],0);self.assertEqual(mb['control_steps'],320)
            result=compare(a,b);self.assertTrue(result['prefix_comparable'],result['issues'])
            self.assertTrue(result['progress']['fresh_start_matches_measured'])

    def test_fixture_contact_clock_interrupt_stop_and_keep_partial_logs(self):
        module=client('progress_fixture_ros.py')
        with tempfile.TemporaryDirectory() as tmp:
            for fault in ('contact','clock','interrupt'):
                with self.subTest(fault=fault):
                    output,value=self.run_entry(module,Path(tmp),'fixture_progress32',fault)
                    self.assertFalse(value['diagnostic_valid']);self.assertTrue(value['failure_tags'])
                    self.assertTrue((output/'control.jsonl').exists())

    def test_actual_search_entry_wiring_uses_progress_gate_without_task_construction(self):
        module=client('reassignment_ros.py')
        with tempfile.TemporaryDirectory() as tmp:
            a,ma=self.run_entry(module,Path(tmp),'search_baseline32')
            b,mb=self.run_entry(module,Path(tmp),'search_progress32')
            self.assertEqual(mb['completed_task_steps'],32);self.assertEqual(mb['control_steps'],320)
            self.assertGreater(mb['pending_before_stop'],0);self.assertEqual(mb['delayed_search_commits'],0)
            self.assertTrue((b/'progress_stop.jsonl').exists())
            result=compare(a,b);self.assertTrue(result['prefix_comparable'],result['issues'])
            self.assertFalse(result['progress']['release_observed'])


if __name__=='__main__':unittest.main()

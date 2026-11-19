from dataclasses import replace
import unittest
from unittest.mock import patch
from crk_ch3.progress_gate import ProgressCommitGate,fresh_route
from crk_ch3.reassignment_gate import DiagnosticInvalid
from crk_ch3.progress_commit import RouteUnavailable
from chapter3_bser.online.types import OnlineStepDiagnostics
from progress_support import Facade,rebuild,search,state


class GateTests(unittest.TestCase):
    def test_defer_then_release_without_new_native_event_preserves_executor_and_cooldown(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *a:f.records.append(a),rebuild=rebuild)
        f.position(20,(1.1,1.,.5));result=f.controller.step(f.state);f.compile(result.allocation);gate.after_advance()
        self.assertEqual(len(gate.queue.pending),3)
        self.assertEqual([s.candidate_id for s in result.allocation.search_assignments],['old']*3)
        executor=result.allocation.executor_assignment
        f.position(21,(2.5,1.,.5));result=f.controller.step(f.state);f.compile(result.allocation);gate.after_advance()
        self.assertFalse(gate.queue.pending);self.assertEqual(gate.queue.committed,3)
        self.assertEqual(result.decision_reason,'PROGRESS_DELAYED_COMMIT')
        self.assertEqual([s.path[0] for s in result.allocation.search_assignments],[(2.5,1.,.5)]*3)
        self.assertEqual(result.allocation.executor_assignment,executor)
        self.assertEqual(f.marked,[20]);self.assertEqual(f.inner.replan_steps,[20]);self.assertEqual(f.policy_calls,[20])
        self.assertEqual(f.inner.waypoints.last_switch_step,{0:21,1:21,2:21})
        self.assertEqual(f.inner.waypoints.current_assignment,f.inner.current_allocation)
        self.assertTrue(all(not flag for flag in f.score_calls))
        self.assertEqual(gate.native_result.decision_reason,'NO_REPLAN_EVENT')

    def test_baseline_prefix_before20_unchanged(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        for k in range(1,20):
            f.position(k,(1.1,1.,.5));result=f.controller.step(f.state);f.compile(result.allocation);gate.after_advance()
            self.assertEqual(result.decision_reason,'NO_REPLAN_EVENT');self.assertFalse(gate.queue.pending)

    def test_native_discovery_runs_and_cancels_pending_before_diagnostic_stop(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        f.position(20,(1.1,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        f.flags.target_found=True;f.position(21,(2.5,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        self.assertFalse(gate.queue.pending);self.assertEqual(r.decision_reason,'NO_REPLAN_EVENT')
        with self.assertRaises(DiagnosticInvalid):gate.after_advance()

    def test_stop_discards_and_records_pending(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *a:f.records.append(a),rebuild=rebuild)
        f.position(20,(1.1,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        gate.on_stop('interrupted');self.assertFalse(gate.queue.pending)
        self.assertEqual(f.records[-1][0],'progress_stop.jsonl')

    def test_real_native_path_query_uses_current_start_without_refreshing_provider(self):
        f=Facade();f.position(21,(2.5,1.,.5));provider_before=vars(f.provider).copy();old_state=f.state
        cached=search(goal=(5.,1.,.5),start=(1.1,1.,.5),name='pending')
        with patch('crk_ch3.progress_gate.extract_planning_state',return_value=state(21,(2.5,1.,.5))):
            route,evidence=fresh_route(f,cached,(2.5,1.,.5))
        self.assertEqual(route.path[0],(2.5,1.,.5));self.assertEqual(route.path[-1],cached.waypoint)
        self.assertEqual(f.provider.__dict__,provider_before);self.assertIs(f.state,old_state)
        self.assertEqual(evidence['original_proposal_start'],(1.1,1.,.5))

    def test_only_arrived_searcher_releases(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        f.position(20,(1.1,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        f.position(21,(1.1,1.,.5))
        f.state=replace(f.state,agents=tuple(replace(a,position=(2.5,1.,.5)) if a.agent_id==0 else a for a in f.state.agents))
        r=f.controller.step(f.state);f.compile(r.allocation);gate.after_advance()
        self.assertEqual(set(gate.queue.pending),{1,2})
        self.assertEqual([s.candidate_id for s in r.allocation.search_assignments],['new','old','old'])
        self.assertEqual(f.inner.waypoints.last_switch_step,{0:21})

    def test_immediate_arrival_is_not_falsely_a_delayed_release(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        f.position(20,(2.5,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation);gate.after_advance()
        committed=[e for e in gate.queue.events if e['kind']=='COMMITTED']
        self.assertEqual(len(committed),3)
        self.assertTrue(all(e['proposed_step']==e['committed_step']==20 for e in committed))
        self.assertEqual(f.marked,[20]);self.assertEqual(f.inner.waypoints.last_switch_step,{0:20,1:20,2:20})

    def test_unavailable_native_connector_does_not_force_provider_refresh(self):
        f=Facade();f.position(21,(2.5,1.,.5));before=f.state
        with patch('crk_ch3.progress_gate.extract_planning_state',return_value=state(21,(1.1,1.,.5))):
            with self.assertRaisesRegex(RouteUnavailable,'no_start_connector'):
                fresh_route(f,search(goal=(5.,1.,.5)),(2.5,1.,.5))
        self.assertIs(f.state,before);self.assertEqual(f.provider._last_refresh_reason,'INCREMENTAL')

    def test_applied_diagnostics_label_adapter_while_native_diagnostics_are_preserved(self):
        f=Facade();native_step=f.controller.step
        native_diag=OnlineStepDiagnostics(21,'native',(),False,'none',.1,.1,0.,False,'','NO_REPLAN_EVENT',(),False,(),(),(),'native_new')
        def step(state,context=None):
            result=native_step(state,context)
            return replace(result,diagnostics=native_diag) if f.step==21 else result
        f.controller.step=step
        gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        f.position(20,(1.1,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        f.position(21,(2.5,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation)
        self.assertIs(gate.native_result.diagnostics,native_diag)
        self.assertTrue(r.diagnostics.accepted);self.assertFalse(r.diagnostics.optimizer_invoked)
        self.assertEqual(r.diagnostics.allocation_scope,'adapter_search_commit')
        self.assertEqual(r.diagnostics.affected_agent_ids,(0,1,2))
        self.assertEqual(f.marked,[20])

    def test_original_tracker_advances_normally_without_pending_intent(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        before=f.bridge.path_tracker.snapshot(0).next_index
        f.position(10,(2.5,1.,.5));r=f.controller.step(f.state);f.compile(r.allocation);gate.after_advance()
        self.assertGreater(f.bridge.path_tracker.snapshot(0).next_index,before)
        self.assertFalse(gate.queue.pending)

    def test_invalid_old_path_leaves_native_controller_enabled_then_invalidates_diagnostic(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        f.position(20,(1.1,1.,.5))
        with patch('crk_ch3.progress_gate.PublicGeometry') as geometry:
            geometry.return_value.path_free.return_value=False
            result=f.controller.step(f.state)
        self.assertEqual(f.marked,[20]);self.assertEqual(result.allocation.executor_assignment.source,'native_new')
        f.compile(result.allocation)
        with self.assertRaises(DiagnosticInvalid):gate.after_advance()
        self.assertFalse(gate.queue.pending)

    def test_native_hold_without_pending_is_also_inapplicable(self):
        f=Facade();gate=ProgressCommitGate(f,'search_progress32',lambda *_:None,rebuild=rebuild)
        guides=list(f.guidance.agent_assignments);guides[0]=replace(guides[0],hold_state=True,reachable=False)
        f.guidance=replace(f.guidance,agent_assignments=tuple(guides))
        with self.assertRaises(DiagnosticInvalid):gate.after_advance()


if __name__=='__main__':unittest.main()

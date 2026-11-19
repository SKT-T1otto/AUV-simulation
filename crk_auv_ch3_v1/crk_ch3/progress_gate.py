"""Opt-in recoverable search commit adapter around the immutable D2 controller."""
from dataclasses import replace
import numpy as np
from .reassignment_gate import ReassignmentGate, DiagnosticInvalid
from .progress_commit import PendingSearchCommits, RouteUnavailable, signature
from .mission_parity import normalized
from .native_source import activate_native
activate_native()
from core.mapping.planning_state import extract_planning_state
from core.mapping.travel_cost_service import TravelCostService
from chapter3_bser.safety.public_geometry import PublicGeometry, filter_planning_state


def fresh_route(runtime, assignment, position):
    # A separate READ-ONLY public route view. Never assign this to provider/state.
    public = filter_planning_state(extract_planning_state(runtime.env), runtime.planning_views.clearance)
    geometry = PublicGeometry(runtime.planning_views.live(), runtime.planning_views.clearance)
    agent = next(a for a in public.agents if a.agent_id == assignment.agent_id)
    route = TravelCostService(public).query(position, assignment.waypoint, agent)
    if not route.reachable or not geometry.path_free(route.path_points):
        raise RouteUnavailable(route.failure_reason or 'latest_public_geometry_rejected')
    rebuilt = replace(assignment, path=tuple(tuple(float(v) for v in p) for p in route.path_points),
        path_cell_indices=tuple(int(v) for v in route.path_cell_indices),
        planning_cost=float(route.planning_cost), physical_travel_time=float(route.physical_travel_time),
        failure_reason=None)
    return rebuilt, dict(graph_sha256=public.planning_graph.graph_sha256,
        route_map_revision=public.map_revision, decision_map_revision=runtime.state.map_revision,
        scope='read_only_live_public_route_view; native_decision_cache_unchanged',
        start=position, original_proposal_start=assignment.path[0] if assignment.path else None)


class ProgressCommitGate(ReassignmentGate):
    def __init__(self, runtime, condition, record, *, rebuild=None):
        if condition != 'search_progress32':
            raise ValueError('Explicit search_progress32 required')
        super().__init__(runtime, 'baseline32', record)
        self.condition = condition
        self.queue = PendingSearchCommits(runtime.bridge.path_tracker.threshold)
        self.rebuild = rebuild or (lambda assignment, position: fresh_route(runtime, assignment, position))
        self.prepared = ()
        self.event_cursor = 0
        self.native_result = None
        self.adapter_commit_steps = []
        self.native_commit_steps = []
        self.invalidated_reason = None

    @property
    def active(self):
        return self.runtime.step >= 20

    def _search_phase(self):
        task = self.runtime.env.get_task_state()
        return not (task.target_found or task.executor_knows_target or task.mission_complete or self.runtime.terminal)

    def _geometry(self, allocation):
        public = self.runtime.env.get_agent_state()
        geometry = PublicGeometry(self.runtime.planning_views.live(), self.runtime.planning_views.clearance)
        safe = {}
        for item in allocation.search_assignments:
            guide = self.runtime.guidance.agent_assignments[item.agent_id]
            snap = self.runtime.bridge.path_tracker.snapshot(item.agent_id)
            safe[item.agent_id] = bool(not item.failure_reason and guide.reachable and not guide.hold_state
                and snap.current_target is not None and geometry.path_free((public.positions[item.agent_id], *snap.remaining_path_points)))
        if self.active and not all(safe.values()):
            self.invalidated_reason = 'old_search_route_invalid_native_safety_has_priority'
        return public.positions, safe

    def _score(self, allocation, prepared):
        values = {x.assignment.agent_id: x.assignment for x in prepared}
        mixed = replace(allocation, search_assignments=tuple(values.get(x.agent_id, x) for x in allocation.search_assignments))
        scored = self.inner.allocator.rescore(self.runtime.state, mixed, finalize=False)
        if scored.executor_assignment != allocation.executor_assignment:
            raise DiagnosticInvalid('Mixed scoring changed native executor choice')
        return scored

    def stabilize(self, previous, proposed, **kwargs):
        raw = self._stabilize(previous, proposed, **kwargs)
        applied = raw
        if self.active and self._search_phase():
            effective = self.runtime.bridge._apply_overrides(previous)
            positions, safe = self._geometry(effective)
            old = {s.agent_id: s for s in effective.search_assignments}
            affected = set(kwargs['affected_agent_ids']) & {0, 1, 2}
            retained = set(self.queue.pending)
            for agent in tuple(self.queue.pending):
                if not safe.get(agent, False):
                    self.queue.drop(agent, self.runtime.step, 'native_safety_priority')
            for item in raw.search_assignments:
                if item.agent_id in affected and safe.get(item.agent_id, False):
                    self.queue.offer(item, old[item.agent_id],
                        self.runtime.guidance.agent_assignments[item.agent_id].tracking_waypoint,
                        step=self.runtime.step, proposal_hash=raw.allocation_sha256)
                    retained.add(item.agent_id)
            # Keep old routes even if a just-offered intent fails rebuilding.
            base = replace(raw, search_assignments=tuple(old[s.agent_id] if s.agent_id in retained
                and safe.get(s.agent_id, False) else s for s in raw.search_assignments))
            self.prepared = self.queue.prepare(old, positions, safe, self.rebuild, step=self.runtime.step)
            applied = self._score(base, self.prepared)
        self.proposals.append(normalized(dict(raw_stabilized_proposal=raw, applied_proposal=applied,
            search_commit_deferred=any(signature(a) != signature(b) for a,b in zip(raw.search_assignments, applied.search_assignments)),
            reason='progress_arrival_gate' if self.active else 'native_commit',
            score_scope='native proposal and actually mixed allocation scored separately')))
        self.deferred_commits += int(self.proposals[-1]['search_commit_deferred'])
        return applied

    def _acknowledge(self, result):
        installed = {s.agent_id: s for s in self.inner.current_allocation.search_assignments}
        if any(signature(installed[x.assignment.agent_id]) != signature(x.assignment) for x in self.prepared):
            raise DiagnosticInvalid('Native commit did not install prepared search routes')
        self.queue.acknowledge(self.prepared, step=self.runtime.step)
        if self.prepared:
            self.adapter_commit_steps.append(self.runtime.step)
        self.prepared = ()

    def step(self, state, context=None):
        self.policy_decision, self.proposals, self.raw_allocator = None, [], []
        self.prepared = ()
        old = self.inner.current_allocation
        if self.runtime.step == 20:
            effective = self.runtime.bridge._apply_overrides(old)
            self.anchor_reference = normalized(dict(step=20,
                positions_native=self.runtime.env.get_agent_state().positions,
                assignments=effective.search_assignments,
                fixed_tracking_points=[self.runtime.guidance.agent_assignments[i].tracking_waypoint for i in range(3)],
                remaining_paths={str(i): self.runtime.bridge.path_tracker.snapshot(i).remaining_path_points for i in range(3)}))
        phase = self._search_phase()
        if not phase:
            self.queue.clear(self.runtime.step, 'discovery_handoff_or_terminal_native_priority')
        result = self._step(state, context)  # Native detector/proposal/executor always run.
        self.native_result = result
        if result.replanned:
            self.native_commit_steps.append(self.runtime.step)
        if self.prepared:
            self._acknowledge(result)
        if phase and self.active and not self.proposals and self.queue.pending:
            effective = self.runtime.bridge._apply_overrides(result.allocation)
            positions, safe = self._geometry(effective)
            self.prepared = self.queue.prepare({s.agent_id:s for s in effective.search_assignments},
                positions, safe, self.rebuild, step=self.runtime.step)
            if self.prepared:
                # Preflight all routes/scoring first. Update real switch timestamps once.
                applied = self._score(result.allocation, self.prepared)
                updates = self.inner.waypoints.updates(result.allocation, applied,
                    reason='PROGRESS_DELAYED_COMMIT', step=self.runtime.step)
                self.inner.current_allocation = applied
                self.inner.waypoints.commit(applied)
                # Native event cooldown/replan counters are NOT marked a second time.
                diagnostics = result.diagnostics
                if diagnostics is not None:
                    diagnostics = replace(diagnostics, mechanism_version='progress_commit_v1',
                        allocation_scope='adapter_search_commit', optimizer_invoked=False,
                        old_objective=result.allocation.objective_value, proposed_objective=applied.objective_value,
                        objective_gain=applied.objective_value-result.allocation.objective_value,
                        accepted=True, accept_reason='PROGRESS_DELAYED_COMMIT', reject_reason='',
                        affected_agent_ids=tuple(x.assignment.agent_id for x in self.prepared),
                        old_waypoints=tuple((s.agent_id,s.waypoint) for s in result.allocation.search_assignments),
                        proposed_waypoints=tuple((s.agent_id,s.waypoint) for s in applied.search_assignments),
                        switch_distance_by_agent=tuple((x.agent_id,float(np.linalg.norm(np.asarray(x.new_waypoint)-x.old_waypoint)))
                            for x in updates if x.old_waypoint is not None))
                result = replace(result, allocation=applied, waypoint_updates=updates,
                    replanned=True, decision_reason='PROGRESS_DELAYED_COMMIT', diagnostics=diagnostics)
                self._acknowledge(result)
        self.last = normalized(dict(step=self.runtime.step, condition=self.condition, boundary_active=self.active,
            old_installed_allocation=old, actual_installed_allocation=self.inner.current_allocation,
            installed_allocation_sha256=self.inner.current_allocation.allocation_sha256,
            raw_allocator_proposals=self.raw_allocator, raw_policy_decision=self.policy_decision,
            stabilization=self.proposals, native_controller_result=self.native_result,
            applied_controller_result=result, diagnostics_scope='native result preserves native diagnostics; delayed applied diagnostics explicitly labelled adapter_search_commit',
            full_refresh=self.runtime.provider.last_snapshot_was_full_refresh,
            refresh_reason=self.runtime.provider._last_refresh_reason,
            graph_sha256=self.runtime.state.planning_graph.graph_sha256,
            planning_map_revision=self.runtime.state.map_revision,
            map_revision=int(self.runtime.env.unwrapped.map_module.map_revision),
            pending=self.queue.pending, queue_events=self.queue.events[self.event_cursor:]))
        self.event_cursor = len(self.queue.events)
        self.record('planning.jsonl', self.last)
        return result

    def after_advance(self):
        if not self._search_phase():
            self.queue.clear(self.runtime.step, 'native_phase_transition')
            raise DiagnosticInvalid('Search-stage diagnostic ended on discovery/handoff/terminal')
        if any(g.hold_state or not g.reachable for g in self.runtime.guidance.agent_assignments[:3]):
            self.queue.clear(self.runtime.step, 'native_search_hold_or_unreachable')
            raise DiagnosticInvalid('Native search hold/unreachable makes this diagnostic inapplicable')
        if self.invalidated_reason:
            self.queue.clear(self.runtime.step, self.invalidated_reason)
            raise DiagnosticInvalid(self.invalidated_reason)
        # Native safety reroutes/holds remain effective; stale pending intents die.
        for agent, item in tuple(self.queue.pending.items()):
            guide = self.runtime.guidance.agent_assignments[agent]
            effective = self.runtime.bridge._apply_overrides(self.inner.current_allocation)
            current = next(s for s in effective.search_assignments if s.agent_id == agent)
            if guide.hold_state or not guide.reachable or signature(current) != item.installed_signature:
                self.queue.drop(agent, self.runtime.step, 'compiled_safety_changed_installed_route')
                self.invalidated_reason = 'native_safety_changed_retained_route'
        if self.invalidated_reason:
            self.queue.clear(self.runtime.step, self.invalidated_reason)
            raise DiagnosticInvalid(self.invalidated_reason)
        nav = self.runtime.env.unwrapped._nav_targets.detach().cpu().numpy()
        for i, guide in enumerate(self.runtime.guidance.agent_assignments):
            expected = guide.hold_position if guide.hold_state else guide.tracking_waypoint
            if not np.allclose(nav[i], expected, atol=1e-6, rtol=0):
                raise DiagnosticInvalid('Guidance and actual navigation target differ')

    def on_stop(self, reason):
        self.queue.clear(self.runtime.step, 'run_stopped:'+reason)
        self.record('progress_stop.jsonl', normalized(dict(step=self.runtime.step, reason=reason,
            pending=self.queue.pending, queue_events=self.queue.events[self.event_cursor:],
            adapter_commit_steps=self.adapter_commit_steps, native_commit_steps=self.native_commit_steps)))

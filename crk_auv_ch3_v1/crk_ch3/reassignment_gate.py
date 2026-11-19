"""Explicit diagnostic-only search assignment gate; never patch the source pin.

The hook is on WaypointManager.stabilize, after the normal policy decision but
BEFORE updates(), commit(), guidance and the next observation. Executor choice
is retained from the native stabilized proposal; mixed scores are recomputed
with that executor fixed. Tracker indices are never restored or frozen.
"""
import copy
from dataclasses import replace
import numpy as np
from .mission_parity import normalized
from .native_source import activate_native
from .reassignment_contract import CONDITIONS, SCHEMA, BUDGET, BOUNDARY
activate_native()
from chapter3_bser.safety.public_geometry import PublicGeometry



class DiagnosticInvalid(RuntimeError):
    pass


def search_signature(allocation):
    return tuple((s.agent_id, s.candidate_id, s.waypoint, s.path, s.failure_reason)
                 for s in allocation.search_assignments)


class ReassignmentGate:
    def __init__(self, runtime, condition, record):
        if condition not in CONDITIONS:
            raise ValueError('Unknown diagnostic condition')
        self.runtime, self.condition, self.record = runtime, condition, record
        self.anchor = None
        self.anchor_reference = None
        self.deferred_commits = 0
        self.last = None
        self.policy_decision = None
        self.proposals = []
        self.raw_allocator = []
        self.inner = runtime.controller.inner
        self._step = runtime.controller.step
        self._stabilize = self.inner.waypoints.stabilize
        self._decide = self.inner.policy.decide
        runtime.controller.step = self.step
        self.inner.waypoints.stabilize = self.stabilize
        self.inner.policy.decide = self.decide
        for name in ('allocate', 'allocate_partial'):
            original = getattr(self.inner.allocator, name)
            def capture(*args, _original=original, _name=name, **kwargs):
                value = _original(*args, **kwargs)
                self.raw_allocator.append(normalized(dict(method=_name, result=value)))
                return value
            setattr(self.inner.allocator, name, capture)

    @property
    def active(self):
        return self.condition == CONDITIONS[1] and self.runtime.step >= BOUNDARY

    def trackers(self):
        tracker = self.runtime.bridge.path_tracker
        return {str(i): normalized(dict(snapshot=tracker.snapshot(i),
                    signature=tracker._state[i].signature if i in tracker._state else None)) for i in range(3)}

    def applicable(self):
        task = self.runtime.env.get_task_state()
        native = self.runtime.env.unwrapped
        if task.target_found or task.executor_knows_target or task.mission_complete or self.runtime.terminal:
            raise DiagnosticInvalid('Discovery/handoff/terminal state makes the diagnostic inapplicable')
        if native.last_collision_records:
            raise DiagnosticInvalid('Native task collision')

    def validate_retained(self):
        """Latest public geometry and remaining tracker path; no hidden obstacle truth."""
        self.applicable()
        geometry = PublicGeometry(self.runtime.planning_views.live(), self.runtime.planning_views.clearance)
        public = self.runtime.env.get_agent_state()
        if len(self.anchor.search_assignments) != 3:
            raise DiagnosticInvalid('Missing retained search assignment')
        for s in self.anchor.search_assignments:
            snap = self.runtime.bridge.path_tracker.snapshot(s.agent_id)
            if s.failure_reason or snap.current_target is None:
                raise DiagnosticInvalid('Retained search route is unreachable/uninitialized')
            if not geometry.path_free((public.positions[s.agent_id], *snap.remaining_path_points)):
                raise DiagnosticInvalid('Retained path failed latest public-geometry validation')

    def decide(self, events, new_objective, old_objective, step):
        events = tuple(events)
        result = self._decide(events, new_objective, old_objective, step)
        self.policy_decision = normalized(dict(actionable_events=events,
            primary_event=self.inner.policy.primary_event(events),
            raw_old_objective=old_objective, raw_proposed_objective=new_objective, decision=result))
        return result

    def stabilize(self, previous, proposed, **kwargs):
        raw = self._stabilize(previous, proposed, **kwargs)
        applied = raw
        deferred = False
        if self.active:
            self.validate_retained()
            mixed = replace(raw, search_assignments=self.anchor.search_assignments)
            # finalize=False prevents another standby selection. Keep native executor.
            applied = self.inner.allocator.rescore(self.runtime.state, mixed, finalize=False)
            if applied.executor_assignment != raw.executor_assignment:
                raise DiagnosticInvalid('Diagnostic unexpectedly modified executor assignment')
            deferred = search_signature(raw) != search_signature(applied)
            self.deferred_commits += int(deferred)
        self.proposals.append(normalized(dict(raw_stabilized_proposal=raw,
            applied_proposal=applied, search_commit_deferred=deferred,
            reason='retain_pre20_search_only' if deferred else 'native_commit',
            score_scope='raw policy scores and recomputed mixed applied scores are separate')))
        return applied

    def step(self, state, context=None):
        self.applicable()
        self.policy_decision, self.proposals, self.raw_allocator = None, [], []
        old = self.inner.current_allocation
        if self.runtime.step == BOUNDARY:
            self.anchor = copy.deepcopy(self.runtime.bridge._apply_overrides(old))
            assignments = self.runtime.guidance.agent_assignments
            for s in self.anchor.search_assignments:
                item = assignments[s.agent_id]
                if item.hold_state or not item.reachable or tuple(item.planned_path) != s.path:
                    raise DiagnosticInvalid('Installed pre20 guidance differs from retained effective path')
            public = self.runtime.env.get_agent_state()
            self.anchor_reference = normalized(dict(step=BOUNDARY,
                positions_native=public.positions, velocities_world=public.velocities,
                assignments=self.anchor.search_assignments,
                fixed_tracking_points=[assignments[i].tracking_waypoint for i in range(3)],
                remaining_paths={str(i): self.runtime.bridge.path_tracker.snapshot(i).remaining_path_points for i in range(3)}))
        if self.active:
            self.validate_retained()
        before = self.trackers()
        result = self._step(state, context)
        if self.active and search_signature(self.inner.current_allocation) != search_signature(self.anchor):
            raise DiagnosticInvalid('Search commit bypassed the stabilization gate')
        self.last = normalized(dict(step=self.runtime.step, condition=self.condition,
            boundary_active=self.active, old_installed_allocation=old,
            actual_installed_allocation=self.inner.current_allocation,
            installed_allocation_sha256=self.inner.current_allocation.allocation_sha256,
            raw_allocator_proposals=self.raw_allocator,
            raw_policy_decision=self.policy_decision, stabilization=self.proposals,
            applied_controller_result=result, tracker_before_compile=before,
            full_refresh=self.runtime.provider.last_snapshot_was_full_refresh,
            refresh_reason=self.runtime.provider._last_refresh_reason,
            graph_sha256=self.runtime.state.planning_graph.graph_sha256,
            planning_map_revision=self.runtime.state.map_revision,
            map_revision=int(self.runtime.env.unwrapped.map_module.map_revision)))
        self.record('planning.jsonl', self.last)
        return result

    def after_advance(self):
        self.applicable()
        if self.active:
            self.validate_retained()
            effective = self.runtime.bridge._apply_overrides(self.inner.current_allocation)
            if search_signature(effective) != search_signature(self.anchor):
                raise DiagnosticInvalid('Safety reroute changed retained route; stop rather than bypass safety')
            for s in self.anchor.search_assignments:
                item = self.runtime.guidance.agent_assignments[s.agent_id]
                if item.hold_state or not item.reachable or tuple(item.planned_path) != s.path:
                    raise DiagnosticInvalid('Compiled retained guidance is unsafe or inconsistent')
        nav = self.runtime.env.unwrapped._nav_targets.detach().cpu().numpy()
        for i,item in enumerate(self.runtime.guidance.agent_assignments):
            expected = item.hold_position if item.hold_state else item.tracking_waypoint
            if not np.allclose(nav[i], expected, atol=1e-6, rtol=0):
                raise DiagnosticInvalid('Guidance and applied navigation target differ')

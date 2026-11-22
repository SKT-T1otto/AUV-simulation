"""Post-decision adapter for a single pinned SafetyBridge and a fixed D2 search solution."""

import copy
from dataclasses import replace
from .initial_candidates import InitialCandidateRuntime
from .standby_geometry import CONDITIONS, nearest, radii
from chapter3_bser.d2.solver import best_standby
from chapter3_bser.objective import (
    build_objective_context,
    evaluate_objective,
    response_diagnostics,
)
from chapter3_bser.safety.safe_guidance import SafetyBridge


def effective_routes(guidance, tracker, positions):
    routes = {}
    for item in guidance.agent_assignments[:3]:
        i = item.agent_id
        tail = () if item.hold_state else tracker.snapshot(i).remaining_path_points
        if not item.hold_state and (not tail or tail[0] != item.tracking_waypoint):
            raise ValueError('Cannot recover effective search tracker')
        routes[i] = [tuple(map(float, positions[i])), *tail]
    if set(routes) != {0, 1, 2}:
        raise ValueError('Incomplete searcher state')
    return routes


class StandbyRuntime(InitialCandidateRuntime):
    def __init__(self, scene, callback, condition, boxes):
        self.radius = radii(boxes)
        super().__init__(scene, callback, condition)
        if type(self.bridge) is not SafetyBridge:
            raise ValueError('Unreviewed guidance bridge')
        self.boundary = dict(step=0, reason='initialize_unchanged')
        self.controller_step = self.controller.step
        self.controller.step = self.decide

    def preview(self, result, context):
        # SafetyBridge can repair a path. Run its exact compile with isolated tracker,
        # mutable safety state and planning-view caches; never advance the live tracker.
        owner = copy.copy(self)
        owner.safe_counts = self.safe_counts.copy()
        owner.safe_pending = self.safe_pending.copy()
        owner.planning_views = copy.copy(self.planning_views)
        owner.planning_views.runtime = owner
        bridge = copy.deepcopy(self.bridge, {id(self): owner})
        guidance = bridge.compile_guidance(
            result.allocation, self.state, context, decision_reason=result.decision_reason
        )
        return effective_routes(
            guidance, bridge.path_tracker, self.motion_exchange.last_feedback.positions
        )

    def decide(self, state, context):
        result = self.controller_step(state, context)
        raw = result.allocation
        row = self.boundary = dict(
            step=int(state.step),
            task_s=state.step * 0.2,
            decision=result.decision_reason,
            raw_sha256=raw.allocation_sha256,
            installed_sha256=raw.allocation_sha256,
        )
        if (
            state.target_found
            or context.target_found
            or context.mission_complete
            or not result.replanned
        ):
            row['reason'] = 'native_phase_or_hold'
            return result
        allocator = self.controller.inner.allocator
        routes = self.preview(result, context)
        row.update(routes=routes, raw=raw)
        movable = allocator.standby_may_change
        pool = allocator.last_standbys if movable else (allocator._frozen_standby_candidate(raw),)
        if not pool:
            raise ValueError('Native candidate pool unavailable')
        selected = tuple(allocator._frozen_search_candidate(a) for a in raw.search_assignments)
        score_state = self.state
        scores = build_objective_context(score_state, selected, pool, allocator.config)
        eligible = []
        row['candidates'] = []
        row['native_pool_movable'] = movable
        for candidate in pool:
            gaps = {i: nearest(candidate.waypoint, path) for i, path in routes.items()}
            ok = all(
                distance > self.radius[3] + self.radius[i] for i, (distance, _) in gaps.items()
            )
            row['candidates'].append(
                dict(
                    id=candidate.candidate_id,
                    point=candidate.waypoint,
                    center_distance_and_segment=gaps,
                    required_center_m=[r + self.radius[3] for r in self.radius[:3]],
                    eligible=ok,
                    reason='clear' if ok else 'inside_search_corridor',
                    score=evaluate_objective(selected, candidate, scores),
                )
            )
            if ok:
                eligible.append(candidate)
        original = best_standby(selected, pool, scores)
        if tuple(original.waypoint) != tuple(raw.executor_assignment.target_region):
            raise ValueError('Fixed-search standby rescore differs from original commit')
        if self.condition == CONDITIONS[0]:
            row['reason'] = 'original_unfiltered'
        elif not eligible:
            row['reason'] = 'clearance_no_feasible_candidate'
        else:
            new = best_standby(selected, eligible, scores)
            if new is not original:
                executor = allocator.execution.assign_standby(score_state, new)
                allocation = replace(
                    raw,
                    executor_assignment=executor,
                    response_time=response_diagnostics(
                        selected, new, scores
                    ).conditional_reachable_response_time,
                )
                self.controller.inner.current_allocation = allocation
                row.update(
                    reason='standby_replaced',
                    installed_sha256=allocation.allocation_sha256,
                    installed_point=new.waypoint,
                    installed_joint_score=evaluate_objective(selected, new, scores),
                )
                return replace(result, allocation=allocation)
            row['reason'] = 'original_eligible'
        return result

    def installed_state(self):
        data = super().installed_state()
        if not self.terminal and self.boundary.get('routes') is not None:
            actual = effective_routes(
                self.guidance,
                self.bridge.path_tracker,
                self.motion_exchange.last_feedback.positions,
            )
            if actual != self.boundary['routes']:
                raise RuntimeError('Actual search routes differ from clearance check')
        data['tracker']['3'] = self.bridge.path_tracker.snapshot(3)
        return data

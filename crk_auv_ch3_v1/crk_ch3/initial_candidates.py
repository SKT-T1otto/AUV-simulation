"""Instance-local step-zero admission. Native D2 owns the solve, commit, bridge and later planning."""

from contextlib import ExitStack
import numpy as np
from .mission_runtime import ExternalMissionRuntime
from .mission_parity import normalized
from .initial_contract import CONDITIONS, check_admission
from chapter3_bser.candidate_generator import _ordered_pool, _separated
from chapter3_bser.d2.controller import D2SearchController
from chapter3_bser.safety.planning_refresh import start_status
from chapter3_bser.types import SearchCandidate
from core.mapping.planning_state import planning_state_sha256
from core.mapping.travel_cost_service import TravelCostService


def quadrant(point):
    return 2 * int(point[0] >= 10) + int(point[1] >= 10)


def quadrant_pool(state, ids, pool, config):
    if config['k_search_exact'] != 4:
        raise ValueError('The four-quadrant contrast requires four search slots')
    centers = state.grid.cell_centers
    service = TravelCostService(state)
    candidates = []
    unreachable = 0
    reasons = []
    for agent_id in sorted(ids):
        if start_status(state, agent_id) != 'ready':
            raise ValueError(f'Searcher {agent_id} is ineligible at initialization')
        agent = state.agents[agent_id]
        admitted = []

        def visit(index, source):
            nonlocal unreachable
            if index in admitted or not _separated(
                index, admitted, centers, config['minimum_separation']
            ):
                return False
            route = service.query(agent.position, centers[index], agent)
            if not route.reachable:
                unreachable += 1
                return False
            if route.physical_travel_time > config['maximum_physical_travel_time']:
                reasons.append(f'agent_{agent_id}:PATH_BUDGET_EXCEEDED:{index}')
                return False
            candidates.append(
                SearchCandidate(
                    agent_id=agent_id,
                    candidate_id=f's{agent_id}_{len(admitted):02d}_{index:04d}',
                    waypoint=tuple(map(float, centers[index])),
                    path_points=route.path_points,
                    path_cell_indices=route.path_cell_indices,
                    path_length=route.path_length,
                    planning_cost=route.planning_cost,
                    physical_travel_time=route.physical_travel_time,
                    source=source,
                )
            )
            admitted.append(index)
            return True

        for area in range(4):
            for index, source in pool:
                if quadrant(centers[index]) == area and visit(index, source):
                    break
        for index, source in pool:
            if len(admitted) == 4:
                break
            visit(index, source)
        if len(admitted) != 4:
            reasons.append(f'agent_{agent_id}:ONLY_{len(admitted)}_OF_4')
    return tuple(sorted(candidates, key=lambda c: c.key)), unreachable, tuple(reasons)


def candidate_row(c):
    return dict(
        id=c.candidate_id,
        source=c.source,
        waypoint=list(c.waypoint),
        path=c.path_points.tolist(),
        cell_indices=c.path_cell_indices.tolist(),
        length_m=c.path_length,
        planning_cost=c.planning_cost,
        native_travel_s=c.physical_travel_time,
    )


class InitialCandidateRuntime(ExternalMissionRuntime):
    def __init__(self, scene, callback, condition):
        if condition not in CONDITIONS:
            raise ValueError('Unknown initial candidate condition')
        self.condition = condition
        self.initialize_calls = self.special_pool_calls = 0
        self.admission = None
        super().__init__(scene, callback)
        env = self.env.unwrapped
        self.admission['navigation_parameters'] = dict(
            eps=float(env.eps),
            kv_xy=env.prior_kv_xy,
            kv_z=env.prior_kv_z,
            slow_xy=env.prior_slow_radius_xy,
            slow_z=env.prior_slow_radius_z,
            v_xy=env._v_xy_max.tolist(),
            v_z=env._v_z_max.tolist(),
            a_xy=env._a_xy_max.tolist(),
            a_z=env._a_z_max.tolist(),
        )

    @property
    def controller(self):
        return self._controller

    @controller.setter
    def controller(self, value):
        # The pinned constructor assigns the assembled controller immediately before
        # initialize(). Intercept this instance only, without a second reset or solve.
        self._controller = value
        if isinstance(value, D2SearchController):
            original = value.initialize

            def initialize(state, context):
                del value.initialize
                return self._initialize(original, state, context)

            value.initialize = initialize

    def _initialize(self, original, state, context):
        allocator = self.controller.inner.allocator
        original_pool_method = allocator.search_pool.__func__.__qualname__
        if self.initialize_calls or state.step != 0 or state.target_found or allocator.arm != 'D2':
            raise ValueError('Step-zero D2 initialization required')
        self.initialize_calls += 1
        safe = self.planning_views.search(state)
        pool = _ordered_pool(safe)
        ranks = {index: rank for rank, (index, _) in enumerate(pool, 1)}
        captured = {}

        def observe(seen, candidates, standbys, **info):
            if seen is not state or captured or info['scope'] != 'D_proposal':
                raise RuntimeError('Unexpected candidate proposal scope')
            captured.update(search=candidates, standbys=standbys)

        def select(seen, ids):
            if seen is not state or self.special_pool_calls or tuple(ids) != state.searcher_ids:
                raise RuntimeError('Quadrant admission escaped its initial pool')
            self.special_pool_calls += 1
            result = quadrant_pool(safe, ids, pool, allocator.config['candidate_generation'])
            self.safe_counts['search_candidate_unreachable_queries'] += result[1]
            self.safe_counts['candidate_shortage_events'] += sum(':ONLY_' in r for r in result[2])
            return result

        with ExitStack() as scope:
            for key, fn in [('_audit_candidate_generation_observer', observe)] + (
                [('search_pool', select)] if self.condition == CONDITIONS[1] else []
            ):
                if key in vars(allocator):
                    raise RuntimeError('Unexpected preinstalled allocator hook')
                setattr(allocator, key, fn)
                scope.callback(delattr, allocator, key)
            result = original(state, context)
        installed = result.allocation
        if installed is not self.controller.inner.current_allocation:
            raise RuntimeError('Controller did not install its initial solve')
        scores = allocator.audit[-1]
        if scores['trigger'] != 'INITIALIZE' or scores['step'] != 0:
            raise RuntimeError('Admission did not occur on the INITIALIZE proposal')
        selected = {a.candidate_id for a in installed.search_assignments}
        candidates = []
        for c in captured['search']:
            cell = int(c.candidate_id.rsplit('_', 1)[1])
            if not np.array_equal(safe.grid.cell_centers[cell], c.waypoint):
                raise RuntimeError('Candidate ID and public cell disagree')
            candidates.append(
                dict(
                    candidate_row(c),
                    agent=c.agent_id,
                    pool_rank=ranks[cell],
                    cell=cell,
                    selected=c.candidate_id in selected,
                )
            )
        self.admission = dict(
            condition=self.condition,
            initialize_calls=self.initialize_calls,
            special_pool_calls=self.special_pool_calls,
            native_pool_restored='search_pool' not in vars(allocator),
            pool_count=len(pool),
            original_pool_method=original_pool_method,
            fingerprints=dict(
                graph=state.planning_graph.graph_sha256,
                safe_graph=safe.planning_graph.graph_sha256,
                public_state=planning_state_sha256(state),
            ),
            candidates=candidates,
            standbys=[candidate_row(y) for y in captured['standbys']],
            scores=dict(scores),
            executor=normalized(installed.executor_assignment),
            allocation_sha256=installed.allocation_sha256,
        )
        self.admission['reference_check'] = check_admission(self.admission, self.condition)
        return result

    def request_evidence(self, request):
        env = self.env.unwrapped
        return dict(
            step=request.task_step,
            nav_targets=env._nav_targets.tolist(),
            positions=request.positions.tolist(),
            velocities=request.velocities_world.tolist(),
            blocked=request.blocked,
            prior_enabled=env.use_residual_prior,
            prior_strength=env._prior_strength.tolist(),
            prior=env._last_prior_acc.tolist(),
            residual=env._last_residual_acc.tolist(),
            requested=request.acceleration_world.tolist(),
        )

    def installed_state(self):
        allocation = self.controller.inner.current_allocation
        tracker = self.bridge.path_tracker
        nav = self.env.unwrapped._nav_targets.detach().cpu().numpy()
        guide = self.guidance
        if not self.terminal:
            for i, g in enumerate(guide.agent_assignments):
                target = g.hold_position if g.hold_state else g.tracking_waypoint
                if not np.allclose(nav[i], target, atol=1e-6, rtol=0):
                    raise RuntimeError('Applied navigation target differs from installed guidance')
            if self.special_pool_calls != int(self.condition == CONDITIONS[1]) or 'search_pool' in vars(
                self.controller.inner.allocator
            ):
                raise RuntimeError('Initial admission still active after initialization')
        return dict(
            guidance=guide,
            allocation=allocation,
            allocation_sha256=allocation.allocation_sha256,
            planner_proposals=[
                row for row in self.controller.inner.allocator.audit if row['step'] == self.step
            ],
            pool_method=self.controller.inner.allocator.search_pool.__func__.__qualname__,
            tracker={str(i): tracker.snapshot(i) for i in range(3)},
            navigation_targets=nav.tolist(),
            safety_status=self.bridge.last_safety_status,
            task=self.env.get_task_state(),
            special_pool_calls=self.special_pool_calls,
        )

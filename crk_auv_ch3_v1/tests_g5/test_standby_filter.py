import unittest
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
from crk_ch3.standby_runtime import StandbyRuntime, effective_routes
from crk_ch3.standby_geometry import CONDITIONS
from chapter3_bser.controllers.path_tracker import PathTracker


@dataclass(frozen=True)
class Result:
    allocation: object
    replanned: bool = True
    decision_reason: str = 'native_accept'


class Filter(unittest.TestCase):
    def test_hold_found_native_no_check(self):
        raw = NS(allocation_sha256='old')
        for found, accepted in [(True, True), (False, False)]:
            result = Result(raw, accepted)
            runtime = NS(controller_step=lambda *_: result)
            returned = StandbyRuntime.decide(
                runtime,
                NS(step=5, target_found=found),
                NS(target_found=found, mission_complete=False),
            )
            self.assertIs(returned, result)
            self.assertEqual(runtime.boundary['reason'], 'native_phase_or_hold')

    def test_actual_tracker_progress_and_hold(self):
        tracker = PathTracker()
        guides = []
        positions = [(1, 0, 0)] * 3
        path = [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
        for i in range(3):
            tracker.tracking_target(i, path[0], path, path[-1])
            target = tracker.tracking_target(i, path[1], path, path[-1])
            guides.append(NS(hold_state=i == 2, agent_id=i, tracking_waypoint=target))
        routes = effective_routes(NS(agent_assignments=guides), tracker, positions)
        self.assertEqual(routes[0], [(1, 0, 0), (2, 0, 0)])
        self.assertEqual(routes[2], [(1, 0, 0)])

    def test_all_ineligible_retains_raw_and_equality_is_rejected(self):
        candidate = NS(waypoint=(0, 0, 1), candidate_id='existing')
        raw = NS(
            allocation_sha256='old',
            executor_assignment=NS(target_region=candidate.waypoint),
            search_assignments=(),
        )
        result = Result(raw)
        state = NS(step=40, target_found=False)
        allocator = NS(standby_may_change=True, last_standbys=(candidate,), config={})
        runtime = NS(
            controller_step=lambda *_: result,
            condition=CONDITIONS[1],
            state=state,
            radius=[0.5] * 4,
            controller=NS(inner=NS(allocator=allocator)),
        )
        with patch('crk_ch3.standby_runtime.best_standby', return_value=candidate), patch(
            'crk_ch3.standby_runtime.build_objective_context'
        ), patch('crk_ch3.standby_runtime.evaluate_objective', return_value=0.5):
            for z, reason in [(0, 'clearance_no_feasible_candidate'), (-1, 'original_eligible')]:
                runtime.preview = lambda *_: {i: [(0, 0, z)] for i in range(3)}
                self.assertIs(
                    StandbyRuntime.decide(
                        runtime, state, NS(target_found=False, mission_complete=False)
                    ),
                    result,
                )
                self.assertEqual(runtime.boundary['reason'], reason)

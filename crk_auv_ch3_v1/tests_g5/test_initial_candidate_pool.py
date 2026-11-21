"""Candidate admission in a public graph stub. No task instance or target truth."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from crk_ch3.initial_candidates import quadrant, quadrant_pool


class CandidatePoolTests(unittest.TestCase):
    def test_centrelines(self):
        self.assertEqual(
            [quadrant(p) for p in [(9, 9, 0), (9, 10, 0), (10, 9, 0), (10, 10, 0)]], [0, 1, 2, 3]
        )

    def test_original_order_spacing_role_specific_route_budget_and_fallback(self):
        points = np.array(
            [[1, 1, 1], [1.1, 1, 1], [2, 1, 1], [3, 1, 1], [10, 1, 1], [10, 11, 1]], dtype=float
        )
        state = SimpleNamespace(
            grid=SimpleNamespace(cell_centers=points),
            agents=[SimpleNamespace(position=(0, 0, 0), idx=i) for i in range(2)],
        )

        def query(start, goal, agent):
            index = int(np.flatnonzero(np.all(points == goal, axis=1))[0])
            return SimpleNamespace(
                reachable=not (index == 4 and agent.idx == 0),
                physical_travel_time=150 if index == 0 and agent.idx == 1 else 5.0,
                path_points=np.array([start, goal]),
                path_cell_indices=np.array([index]),
                path_length=4.0,
                planning_cost=6.0,
            )

        service = SimpleNamespace(query=query)
        config = dict(k_search_exact=4, minimum_separation=0.45, maximum_physical_travel_time=120.0)
        pool = [(i, 'public') for i in range(6)]
        with patch('crk_ch3.initial_candidates.TravelCostService', return_value=service), patch(
            'crk_ch3.initial_candidates.start_status', return_value='ready'
        ):
            found, unreachable, reasons = quadrant_pool(state, (0, 1), pool, config)
        cells = [
            [int(c.candidate_id.rsplit('_', 1)[1]) for c in found if c.agent_id == i] for i in (0, 1)
        ]
        self.assertEqual(cells, [[0, 5, 2, 3], [1, 4, 5, 2]])
        self.assertGreaterEqual(unreachable, 1)
        self.assertTrue(any('PATH_BUDGET_EXCEEDED' in reason for reason in reasons))

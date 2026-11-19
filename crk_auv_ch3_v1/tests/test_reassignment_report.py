"""Task-free geometry, preregistration and supervisor opt-in checks."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from crk_ch3.reassignment_contract import BUDGET, BOUNDARY, PREFIX_CRITERIA
from crk_ch3.reassignment_report import project, run_metrics, route_signature


class ReassignmentReportTests(unittest.TestCase):
    def test_finite_polyline_projection_separates_progress_and_error(self):
        route=[(0,0,0),(1,0,0),(1,2,0)]
        along,error=project(np.array([1.,1.,.2]),route)
        self.assertAlmostEqual(along,2.)
        self.assertAlmostEqual(error,.2)
        along,error=project(np.array([3.,2.,0.]),route)
        self.assertAlmostEqual(along,3.)
        self.assertAlmostEqual(error,2.)

    def test_budget_is_external_and_post_boundary_window_is_2point4(self):
        self.assertEqual(BUDGET,32)
        self.assertEqual(BOUNDARY,20)
        self.assertAlmostEqual((BUDGET-BOUNDARY)*.2,2.4)
        self.assertEqual(PREFIX_CRITERIA['motion_prefix_steps'],20)
        self.assertEqual(PREFIX_CRITERIA['guidance_prefix_steps'],19)

    def test_partial_run_generates_report_without_claiming_validity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            data=dict(diagnostic_condition='baseline32',termination_reason='interrupted',
                      diagnostic_valid=False,completed_task_steps=0)
            (root/'summary.json').write_text(json.dumps(data),encoding='utf-8')
            result=run_metrics(root)
            self.assertFalse(result['diagnostic_valid'])
            self.assertTrue(result['raw_inputs_unchanged'])
            self.assertEqual(result['agents'],{})

    def test_route_comparison_does_not_round_or_use_allocation_hash(self):
        g=dict(agent_id=0,hold_state=False,tracking_waypoint=[1.,2.,3.],planned_path=[])
        left=dict(agent_assignments=[g],allocation_hash='a')
        right=dict(agent_assignments=[dict(g)],allocation_hash='b')
        self.assertEqual(route_signature(left),route_signature(right))
        right['agent_assignments'][0]['tracking_waypoint']=[1.+1e-12,2.,3.]
        self.assertNotEqual(route_signature(left),route_signature(right))


if __name__=='__main__':unittest.main()

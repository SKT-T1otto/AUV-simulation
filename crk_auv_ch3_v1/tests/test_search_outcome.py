import unittest
from crk_ch3.search_outcome import acceptance
from crk_ch3.capture_evidence import capture_review


class SearchOutcomeTests(unittest.TestCase):
    def test_capture_trace_rejects_gaps_and_broken_holds(self):
        summary = dict(completed_task_steps=6, found_step=1, handoff_decision_step=2,
                       task_outcome={'termination_reason': 'success'})
        rows = [dict(step=k, feedback_elapsed_ns=k*200_000_000, target_found=True, finder_id=0,
                     executor_received=k>=2, distance_start_m=.7, distance_end_m=.7, radius_m=.8,
                     required_steps=5, native_hold=k-1, mission_complete=k==6) for k in range(1, 7)]
        self.assertTrue(capture_review(rows, summary)['verified'])
        for index, changes, tag in [
            (1, {'executor_received': False}, 'one_step_handoff_unverified'),
            (2, {'distance_start_m': .81}, 'terminal_hold_unverified'),
            (2, {'distance_end_m': .81}, 'measured_and_native_hold_disagree'),
            (3, {'feedback_elapsed_ns': 0}, 'capture_clock_or_step_gap')]:
            with self.subTest(tag=tag):
                altered = rows.copy()
                altered[index] = {**rows[index], **changes}
                review = capture_review(altered, summary)
                self.assertFalse(review['verified'])
                self.assertIn(tag, review['failure_tags'])
        self.assertFalse(capture_review([], summary)['verified'])
        self.assertFalse(capture_review(rows[:-1], summary)['verified'])

    def healthy(self):
        return dict(mode='search', termination_reason='task_terminal', linux_sdf_valid=True,
            motion_started=True, map_executed=True, map_sensor_scan_count=104,
            completed_task_steps=400, control_steps=4000, stop_zero_command_sent=True,
            contact_events_observed=0, task_success=False, task_outcome={'termination_reason': 'timeout'})

    def test_healthy_timeout_is_not_search_success(self):
        row = self.healthy()
        result = acceptance(row)
        self.assertTrue(result['engineering_acceptance_passed'])
        self.assertFalse(result['g6_search_acceptance_passed'])
        row['task_outcome'] = {'termination_reason': 'obstacle_collision'}
        self.assertFalse(acceptance(row)['g6_search_acceptance_passed'])

    def test_capture_fixture_cannot_be_labeled_full_search(self):
        row = self.healthy()
        row.update(mode='capture', task_success=True, task_outcome={'termination_reason': 'success'},
                   found_step=1, handoff_decision_step=2, capture_hold_max=5)
        self.assertFalse(acceptance(row)['g6_capture_acceptance_passed'])
        row['capture_evidence'] = {'verified': True}
        self.assertTrue(acceptance(row)['g6_capture_acceptance_passed'])
        self.assertFalse(acceptance(row)['g6_search_acceptance_passed'])
        row.update(mode='search', search_displacement_before_found_m=.3)
        self.assertTrue(acceptance(row)['g6_search_acceptance_passed'])
        for key, value in [('contact_events_observed', 1), ('control_steps', 3999),
                           ('stop_zero_command_sent', False), ('linux_sdf_valid', False)]:
            broken = {**row, key: value}
            self.assertFalse(acceptance(broken)['engineering_acceptance_passed'])
            self.assertFalse(acceptance(broken)['g6_search_acceptance_passed'])

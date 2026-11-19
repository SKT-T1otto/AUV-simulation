from dataclasses import replace
from pathlib import Path
import json
import tempfile
import unittest
from crk_ch3.progress_fixture import FixtureProgram
from crk_ch3.search_run import create_assets
from crk_ch3.progress_report import metrics


class FixtureTests(unittest.TestCase):
    def test_wait_then_release_rebuilds_start_and_native_tracker_continues(self):
        origins=[[2,2,2],[2,6,3],[2,10,4],[2,14,2]]
        p=FixtureProgram(origins,'fixture_progress32')
        for k in range(1,6):p.advance(k,origins)
        self.assertEqual(len(p.queue.pending),3)
        positions=[[2,2.8,2],[2,6.8,3],[2,10.8,4],[2,14,2]]
        p.advance(6,positions)
        self.assertFalse(p.queue.pending);self.assertEqual(p.queue.committed,3)
        self.assertEqual([s.path[0] for s in p.allocation.search_assignments],[tuple(v) for v in positions[:3]])
        self.assertEqual(p.commit_steps,[6]);self.assertEqual([p.bridge.path_tracker.snapshot(i).next_index for i in range(3)],[1,1,1])

    def test_baseline_installs_at5_and_no_continuation_or_unsafe_old_route(self):
        origins=[[2,2,2],[2,6,3],[2,10,4],[2,14,2]]
        p=FixtureProgram(origins,'fixture_baseline32')
        for k in range(1,6):p.advance(k,origins)
        self.assertEqual(p.commit_steps,[5]);self.assertFalse(p.queue.pending)
        with self.assertRaises(ValueError):p.advance(5,origins)
        with self.assertRaises(RuntimeError):p.advance(6,[[9,2,2],*origins[1:]])

    def test_fixture_uses_fixed_nominal_references_without_resetting_measured_pose(self):
        origins=[[2,2,2],[2,6,3],[2,10,4],[2,14,2]]
        actual=[[2,2,1.99998],[2,6,2.99998],[2,10,3.99998],[2,14,1.99998]]
        p=FixtureProgram(origins,'fixture_progress32',actual)
        self.assertEqual(p.guidance.agent_assignments[0].planned_path[0],(2.,2.,2.))
        self.assertEqual(p.positions[0,2],1.99998)

    def test_progress_assets_and_partial_failure_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=create_assets(Path(tmp),'search',progress_condition='fixture_progress32')
            meta=json.loads((output/'summary.json').read_text())
            self.assertTrue(meta['synthetic_proposal_fixture']);self.assertFalse(meta['mission_executed'])
            self.assertEqual(meta['boundary_step'],5)
            value=metrics(output)
            self.assertFalse(value['complete_records']);self.assertFalse(value['release_observed'])
            self.assertFalse(value['diagnostic_valid'])


if __name__=='__main__':unittest.main()

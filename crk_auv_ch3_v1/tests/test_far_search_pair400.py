"""Full-horizon software contracts; these tests do not instantiate a native mission."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from crk_ch3.far_metrics import common_samples, crossed_cells, spatial_window
from crk_ch3.far_mission import swept_distance, terminal_steps
from crk_ch3.far_pair import detection
from crk_ch3.initial_batch import run_batch
from crk_ch3.initial_contract import FULL_CONDITIONS, FULL_PREFIX, admission_rule
from crk_ch3.initial_package import file_hash


def timeout_summary():
    return dict(
        termination_reason='task_terminal',
        completed_task_steps=400,
        control_steps=4000,
        actual_simulated_duration_s=80.0,
        task_success=False,
        task_outcome=dict(
            termination_reason='timeout',
            terminal_step=400,
            terminated=True,
            truncated=False,
            success=False,
        ),
    )


class FarPairTests(unittest.TestCase):
    def test_native_terminal_required_even_on_last_step(self):
        self.assertEqual(terminal_steps(timeout_summary()), 400)
        for update in (
            dict(termination_reason='diagnostic_budget_completed'),
            dict(control_steps=3999),
            dict(actual_simulated_duration_s=79.8),
            dict(completed_task_steps=32),
        ):
            with self.assertRaises(ValueError):
                terminal_steps(dict(timeout_summary(), **update))
        s = timeout_summary()
        s['task_outcome']['terminated'] = False
        with self.assertRaises(ValueError):
            terminal_steps(s)

    def test_registered_initial_rules_and_detection_censoring(self):
        self.assertEqual(
            [admission_rule(c) for c in FULL_CONDITIONS], ['initial_original32', 'initial_quadrant32']
        )

        def mission(t):
            return dict(found_s=t, observed_task_s=80.0)

        for a, b, label, delta in [
            (None, None, 'neither_found', None),
            (10, None, 'only_A_found', None),
            (None, 20, 'only_B_found', None),
            (18, 14, 'both_found', -4),
        ]:
            d = detection(mission(a), mission(b))
            self.assertEqual((d['result'], d['B_minus_A_s']), (label, delta))

    def test_time_weighted_spread_grid_and_vertical(self):
        samples = [(i * 20_000_000, [(9 + i / 5, 0, 0), (11, 0, 0), (1, 0, 2)]) for i in range(11)]
        g = spatial_window(samples, 1, (10, 10), (20, 20))
        self.assertAlmostEqual(g['mean_m']['Dxy'], 20 / 3)
        self.assertGreater(g['mean_m']['D3d'], g['mean_m']['Dxy'])
        self.assertAlmostEqual(g['right_boat_s'], 0.3)
        self.assertAlmostEqual(g['right_fraction'], 0.5)
        self.assertEqual(g['per_agent'][0]['first_observed_right_s'], 0.1)
        self.assertEqual((g['union_cells'], g['overlapping_cells']), (3, 1))
        self.assertEqual(crossed_cells((1, 1), (5, 5), (10, 10), (20, 20)), {(0, 0), (1, 1), (2, 2)})
        other = list(samples)
        other[4] = (78_000_000, samples[4][1])
        (av, bv), match = common_samples(samples, other, 1)
        self.assertEqual(match['matched_slots'], 10)
        self.assertIsNone(bv[4])
        self.assertAlmostEqual(spatial_window(av, 1, (10, 10), (20, 20))['integrated_s'], 0.16)

    def test_swept_distance_is_contemporaneous(self):
        d, t = swept_distance((0, 0, 0), (2, 0, 0), (2, 0, 0), (0, 0, 0))
        self.assertEqual((d, t), (0, 0.5))
        self.assertEqual(swept_distance((0, 0, 0), (2, 0, 0), (2, 0, 0), (4, 0, 0)), (2, 0))

    def test_timeout_starts_b_failures_pack_and_skip_as_needed(self):
        # Driver simulates case validation; real Gazebo runs supply the raw JSONL streams.
        for stop, expected in [('none', 0), ('A', 1), ('B', 1), ('cancel', 130), ('clock', 1)]:
            with self.subTest(stop=stop), tempfile.TemporaryDirectory() as tmp:
                batch = Path(tmp) / (FULL_PREFIX + 'test')
                batch.mkdir()
                calls = []

                class Driver:
                    environment = 'software test'

                    def prepare(self, batch, meta):
                        meta.update(git_sha='test', adapter_manifest_sha256='test')

                    def run(self, batch, condition):
                        calls.append(condition)
                        case = batch / condition
                        case.mkdir()
                        s = timeout_summary()
                        if stop == ('A' if len(calls) == 1 else 'B'):
                            s['termination_reason'] = 'physical_contact'
                        (case / 'summary.json').write_text(json.dumps(s))
                        if stop == 'cancel':
                            (batch / 'CANCEL').write_text('SIGINT')
                        return int(s['termination_reason'] != 'task_terminal')

                def read_case(directory, condition):
                    s = json.loads((directory / 'summary.json').read_text())
                    self.assertEqual(terminal_steps(s), 400)
                    if stop == 'clock':
                        raise ValueError('Control clock mismatch')
                    return dict(summary=dict(s, adapter_manifest_sha256='test'))

                with patch('crk_ch3.initial_batch.load_case', side_effect=read_case), patch(
                    'crk_ch3.far_pair.compare_full', return_value=dict(valid=True)
                ), redirect_stdout(io.StringIO()) as stream:
                    rc = run_batch(batch, Driver(), full=True)
                self.assertEqual(rc, expected)
                self.assertEqual(
                    calls, list(FULL_CONDITIONS) if stop in ('none', 'B') else [FULL_CONDITIONS[0]]
                )
                archive = batch.with_suffix('.zip')
                with zipfile.ZipFile(archive) as z:
                    self.assertIsNone(z.testzip())
                    for f in (
                        'batch.log',
                        'batch.json',
                        'comparison.json',
                        'REPORT.md',
                        'SHA256SUMS',
                        'agent_metrics.json',
                        'timeseries.csv',
                        'tracks.svg',
                        'auv2_z.svg',
                        'dispersion.svg',
                    ):
                        self.assertIn(f, z.namelist())
                self.assertIn('ZIP_SHA256=' + file_hash(archive), stream.getvalue())


if __name__ == '__main__':
    unittest.main()

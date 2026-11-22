import math
import unittest
import tempfile
import zipfile
import contextlib
import io
import json
from pathlib import Path
from unittest.mock import patch
from crk_ch3.initial_batch import run_batch
from crk_ch3.standby_geometry import CONDITIONS, PREFIX
from crk_ch3.standby_geometry import nearest, world_box, hull_gap, radii, REFERENCE_NS
from crk_ch3.standby_compare import crossing


class Batch(unittest.TestCase):
    def test_observed_contact_continues_and_failure_stops(self):
        for stop in ('physical_contact', 'engineering_failure'):
            with self.subTest(stop=stop), tempfile.TemporaryDirectory() as tmp:
                batch = Path(tmp) / (PREFIX + 'test')
                batch.mkdir()
                calls = []

                class Engine:
                    environment = 'synthetic'

                    def prepare(self, directory, meta):
                        meta.update(git_sha='test', adapter_manifest_sha256='stub')

                    def run(self, directory, condition):
                        calls.append(condition)
                        dest = directory / condition
                        dest.mkdir()
                        (dest / 'summary.json').write_text(
                            json.dumps(dict(termination_reason=stop))
                        )
                        return 1

                # This test isolates scheduling; the production reader verifies contact evidence.
                with patch(
                    'crk_ch3.standby_records.load_standby',
                    return_value=dict(summary=dict(adapter_manifest_sha256='stub')),
                ), patch('crk_ch3.standby_compare.compare', return_value=dict(valid=True)), patch(
                    'crk_ch3.standby_compare.report',
                    side_effect=lambda d, m, r: (d / 'batch.json').write_text(json.dumps(m)),
                ), contextlib.redirect_stdout(
                    io.StringIO()
                ):
                    rc = run_batch(batch, Engine(), standby=True, full=True)
                self.assertEqual(
                    calls, list(CONDITIONS) if stop == 'physical_contact' else [CONDITIONS[0]]
                )
                self.assertEqual(rc, int(stop == 'engineering_failure'))
                with zipfile.ZipFile(batch.with_suffix('.zip')) as z:
                    self.assertIsNone(z.testzip())
                    self.assertIn('SHA256SUMS', z.namelist())


class Geometry(unittest.TestCase):
    def test_finite_segments_repeats_and_vertical(self):
        route = [(0, 0, 0), (0, 0, 0), (2, 0, 0)]
        self.assertEqual(nearest((1, 0, 2), route), (2, 1))
        self.assertEqual(nearest((-1, 0, 0), route), (1, 0))
        self.assertEqual(nearest((2, 0, 1), [(2, 0, 0)]), (1, 0))
        with self.assertRaises(ValueError):
            nearest((0, 0, 0), [])

    def test_rotated_boxes_and_offset(self):
        hull = ((0, 0, 0.1), (1, 0.5, 0.2))
        body = dict(position=[0, 0, 0], quaternion=[0, 0, 0, 1])
        a = world_box(body, hull)
        b = world_box(
            dict(body, position=[2, 0, 0], quaternion=[0, 0, math.sqrt(0.5), math.sqrt(0.5)]), hull
        )
        self.assertAlmostEqual(hull_gap(a, b), 0.5)
        self.assertAlmostEqual(hull_gap(a, a), 0)
        self.assertAlmostEqual(radii([hull])[0], math.hypot(1, 0.5, 0.3))

    def test_crossing_requires_strict_later_pose_and_monitor(self):
        rows = [dict(ns=0, monitor_online=True), dict(ns=REFERENCE_NS, monitor_online=True)]
        case = dict(samples=rows, contacts=[])
        self.assertEqual(crossing(case)['status'], 'crossing_not_observed')
        rows.append(dict(ns=REFERENCE_NS + 12_000_000, monitor_online=True))
        self.assertEqual(crossing(case)['status'], 'crossed_no_contact_observed')
        self.assertEqual(
            crossing(case)['strict_verdict'], 'insufficient_continuous_sensor_evidence'
        )
        case['summary'] = dict(
            task_epoch_ns=10_000_000,
            sensor_source_ranges_ns=[[[0, 22_000_000_000]] for _ in range(4)],
        )
        self.assertEqual(crossing(case)['strict_verdict'], 'crossed_without_contact')
        rows[1]['monitor_online'] = False
        self.assertEqual(crossing(case)['status'], 'insufficient_monitor_coverage')
        case['contacts'] = [dict(ns=REFERENCE_NS, pair=[0, 2])]
        self.assertEqual(crossing(case)['status'], 'contact_observed')
        self.assertEqual(crossing(case, [1, 3])['status'], 'insufficient_monitor_coverage')

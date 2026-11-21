"""No native task instances. Synthetic geometry exercises timing and package failure paths."""

import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from crk_ch3.initial_batch import DockerCases, export_zip, run_batch
from crk_ch3.initial_compare import compare_agent, replacement, unit, sub
from crk_ch3.initial_contract import COMMON, CONDITIONS, NEXT
from crk_ch3.initial_package import file_hash
from crk_ch3.scenario import Frame


def synthetic(k, agent=2, moving=True, edge=4):
    first = COMMON[agent]
    second = NEXT[k][agent]
    axis = unit(sub(NEXT[1][agent], NEXT[0][agent]))

    def body(sample):
        amplitude = max(0, sample - 10) * 0.002 if k and moving else 0
        p = [a + amplitude * b for a, b in zip(Frame().position(first), axis)]
        return dict(
            position=p, quaternion=[0, 0, 0, 1], velocity_body=[0, 0, 0], stamp_ns=sample * 20_000_000
        )

    def installed(step):
        changed = step >= edge
        goal = first if step == 0 else second
        path = [first, second] if not changed else [first, [9, 9, 1]]
        g = dict(
            assignment_kind='search',
            assignment_id=str(step) if not changed else 'new',
            tracking_waypoint=goal,
            planned_path=path,
            final_waypoint=path[-1],
            hold_state=False,
            reachable=True,
        )
        t = dict(next_index=int(step > 0), remaining_path_points=list(path[int(step > 0) :]))
        return dict(guidance=dict(agent_assignments=[g] * 4), tracker={str(agent): t})

    case = dict(condition=CONDITIONS[k], start=[], end=[], controls=[])
    for step in range(1, 33):
        for name, index in [('start', step - 1), ('end', step)]:
            case[name].append(
                dict(step=step, installed=installed(index), body_states=[body(index * 10)] * 4)
            )
        for sample in range(10 * (step - 1) + 1, 10 * step + 1):
            cmd = [k * 0.3 if step >= 2 else 0, 0, 0]
            d = dict(requested_acceleration_world=cmd, governed_acceleration_world=cmd)
            case['controls'].append(
                dict(
                    diagnostics=[d] * 4,
                    body_states=[body(sample)] * 4,
                    positions_world=[body(sample)['position']] * 4,
                )
            )
    return case


def analyze(b):
    return compare_agent(
        synthetic(0), b, 2, [0, 0], [], dict(position_m=0, velocity_m_s=0, attitude_deg=0)
    )


class InitialPairTests(unittest.TestCase):
    def test_detached_monitor_error_requests_cleanup_before_archiving(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = Path(tmp)
            cond = CONDITIONS[0]

            def acknowledge(_seconds):
                self.assertTrue((batch / 'CANCEL').read_text().startswith('fault:'))
                (batch / (cond + '.exit')).write_text('130')

            with patch('crk_ch3.initial_batch.os.getuid', create=True, return_value=1000), patch(
                'crk_ch3.initial_batch.os.getgid', create=True, return_value=1000
            ), patch('crk_ch3.initial_batch.command'), patch.object(
                DockerCases, 'wait_for_runner', side_effect=RuntimeError('probe failed')
            ), patch(
                'crk_ch3.initial_batch.time.sleep', side_effect=acknowledge
            ):
                with self.assertRaisesRegex(RuntimeError, 'runner exit after cancellation: 130'):
                    DockerCases().run(batch, cond)

    def test_true_motion_time_and_signed_vertical(self):
        r = analyze(synthetic(1, edge=3))
        self.assertEqual(r['window_end_step'], 3)
        self.assertEqual(r['classification'], 'directional_displacement')
        self.assertEqual(r['axis_B_minus_A'], [0, 0, -1])
        self.assertEqual(r['first_different_pre_motion_target']['step'], 2)
        self.assertEqual(r['events']['requested']['command_time_s'], 0.2)
        e = r['events']['directional']
        self.assertLess(e['first']['delta_z_m'], 0)
        self.assertAlmostEqual(e['confirmed']['t_s'] - e['first']['t_s'], 0.04)
        self.assertEqual(len(r['series']), 30)
        self.assertIsNotNone(r['routes'][CONDITIONS[1]]['remaining_at_replacement_m'])
        self.assertEqual(
            replacement(synthetic(1), 2), 4
        )  # Candidate ID changes alone do not replace geometry.

    def test_unresolved_and_independent_prefix_checks(self):
        b = synthetic(1, moving=False)
        self.assertEqual(analyze(b)['classification'], 'request_unresolved')
        b['controls'][0]['body_states'][2]['stamp_ns'] += 2_000_000
        self.assertIn('odometry_timestamps', analyze(b)['failures'])
        b = synthetic(1, moving=False)
        b['controls'][0]['diagnostics'][2]['requested_acceleration_world'] = [0.2, 0, 0]
        self.assertEqual(analyze(b)['classification'], 'insufficient')

    def test_allocation_only(self):
        b = synthetic(1, moving=False)
        for row in b['controls']:
            for d in row['diagnostics']:
                for key in d:
                    d[key] = [0, 0, 0]
        self.assertEqual(analyze(b)['classification'], 'allocation_only')

    def test_package_after_failure_or_cancel_skips_b(self):
        for interrupted in (False, True):
            with self.subTest(interrupted=interrupted), tempfile.TemporaryDirectory() as tmp:
                parent = Path(tmp)
                batch = parent / 'g6_initial_candidates_pair_test'
                batch.mkdir()
                calls = []

                class Driver:
                    environment = 'Windows/Linux software stand-in'

                    def prepare(self, batch, meta):
                        meta['git_sha'] = 'test'

                    def run(self, batch, condition):
                        calls.append(condition)
                        (batch / 'trace.log').write_text('early evidence')
                        if interrupted:
                            (batch / 'CANCEL').write_text('SIGINT')
                        return 130 if interrupted else 1

                with redirect_stdout(io.StringIO()) as stdout:
                    rc = run_batch(batch, Driver())
                self.assertEqual(rc, 130 if interrupted else 1)
                self.assertEqual(calls, [CONDITIONS[0]])
                archive = next(parent.glob('*.zip'))
                self.assertIn(file_hash(archive), stdout.getvalue())
                with zipfile.ZipFile(archive) as z:
                    self.assertIsNone(z.testzip())
                    self.assertIn('trace.log', z.namelist())
                    self.assertIn('REPORT.md', z.namelist())
                    self.assertEqual(
                        json.loads(z.read('batch.json'))['status'],
                        'interrupted' if interrupted else 'failed',
                    )

    def test_export_preserves_bytes_and_rejects_wrong_existing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            source = p / 'result.zip'
            source.write_bytes(b'result')
            share = p / 'share'
            share.mkdir()
            with patch('crk_ch3.initial_batch.command', return_value=str(share)):
                saved = export_zip(source, share, file_hash(source))
                self.assertEqual(saved.read_bytes(), source.read_bytes())
                saved.write_bytes(b'other')
                with self.assertRaises(FileExistsError):
                    export_zip(source, share, file_hash(source))


if __name__ == '__main__':
    unittest.main()

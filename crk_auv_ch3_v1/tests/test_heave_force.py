"""Desktop fixtures for vertical force, timing, plots, batch failure and ZIP delivery."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.integrate import solve_ivp

from crk_ch3 import heave, heave_batch, heave_plot, heave_report, surge_batch
from crk_ch3.motion_run import update_summary
from crk_ch3.vehicle import load_profile
from test_surge_force import PlantStub, load_trial


def sample(states):
    state, = states
    return dict(stamp_ns=state.stamp_ns, positions_world=[list(state.position)],
                velocity_world=[list(state.velocity_body)], quaternion_xyzw=[list(state.quaternion)],
                velocity_body=[list(state.velocity_body)], rates_body=[list(state.rates_body)])


def fake_run(root,case,headless,module,marker):
    assert headless and marker and module=='crk_ch3.surge_ros'
    summary = heave_report.load(case,'summary.json')
    summary['completed_control_steps'] = 0
    plant = PlantStub(heave_report.load(case,'program.json')['initial_world_position'])
    collector = load_trial()
    collector.state_data = sample
    with (case/'control.jsonl').open('x') as stream:
        collector.collect(plant,case,summary,stream)
    (case/'physical_contacts.jsonl').touch()
    summary.update(linux_sdf_valid=True,stop_confirmed=True,engineering_acceptance_passed=True,
                   contact_events_observed=0,termination_reason='completed')
    update_summary(case,**summary)
    return 0


class HeaveTests(unittest.TestCase):
    def test_reference_and_signs(self):
        p = load_profile()
        balance = heave.support(p)
        self.assertAlmostEqual(balance,.982)
        np.testing.assert_allclose(heave.reference_step(np.zeros(2),balance,1,p),0)
        for target in [0.,balance,balance+5.,balance-5.]:
            for initial in [-.08,0.,.11]:
                state = np.array([initial,0.])
                for _ in range(250):
                    state = heave.reference_step(state,target,.02,p)
                ref = solve_ivp(lambda _,s: [(target-.982-33*s[0]-190*abs(s[0])*s[0])/32.18,-s[0]],
                                (0,5),[initial,0],rtol=1e-11,atol=1e-13)
                np.testing.assert_allclose(state,ref.y[:,-1],rtol=0,atol=5e-8)

    def test_full_collection_report_and_archive(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(surge_batch,'supervise',side_effect=fake_run), \
                patch.object(surge_batch.os,'geteuid',return_value=1000,create=True):
            root = Path(tmp)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(heave_batch.run(root),0)
            archive, = (root/'results/ch3').glob('*.zip')
            batch = archive.with_suffix('')
            with zipfile.ZipFile(archive) as z:
                self.assertIsNone(z.testzip())
                for name in ('heave_response.svg','heave_response.csv','heave_metrics.json','REPORT.md','SHA256SUMS'):
                    self.assertIn(name,z.namelist())
                self.assertEqual(len(z.read('heave_response.csv').splitlines()),2401)
            data = heave_report.load(batch,'batch.json')
            traces = {}
            for condition in heave.CASES:
                case = batch/data['trials'][condition]
                traces[condition] = heave_report.read_trial(case,condition)
            free, hover, bidir = [traces[k][0] for k in heave.CASES]
            self.assertGreater(free[-1]['depth_change_theory_m'],.3)
            self.assertLess(abs(hover[-1]['depth_change_theory_m']),.01)
            self.assertGreater(bidir[299]['velocity_up_theory_m_s'],.08)
            self.assertLess(bidir[-1]['velocity_up_theory_m_s'],-.08)
            # The fixture's static odometry cannot reseed the predicted depth or velocity.
            self.assertEqual(free[-1]['depth_change_gazebo_m'],0)
            self.assertLess(max(abs(r['published_world_up_N']) for r in free),1e-12)
            heave_plot.plot_svg(batch,traces)
            self.assertEqual(len(ET.parse(batch/'heave_response.svg').getroot()),9)
            # Permit non-grid odometry within the recorded input history.
            case = batch/data['trials']['bidirectional']
            path = case/'control.jsonl'
            original = path.read_text().splitlines()
            rows = [json.loads(s) for s in original]
            for i in (298,299,300):
                rows[i]['after']['stamp_ns'] -= 2_000_000
                rows[i+1]['before']['stamp_ns'] -= 2_000_000
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            shifted,_ = heave_report.read_trial(case,'bidirectional')
            self.assertLess(shifted[299]['t_s'],6.)
            # At this interval start the body z axis is tilted; depth comes from world height.
            angle = .1
            q = [np.sin(angle/2),0.,0.,np.cos(angle/2)]
            rows[299]['after']['quaternion_xyzw'] = [q]
            rows[300]['before']['quaternion_xyzw'] = [q]
            rows[300]['after']['positions_world'][0][2] -= .03
            rows[301]['before']['positions_world'][0][2] -= .03
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            tilted,_ = heave_report.read_trial(case,'bidirectional')
            self.assertAlmostEqual(tilted[300]['published_world_up_N'],
                                   tilted[300]['published_body_z_N']*np.cos(angle))
            self.assertAlmostEqual(tilted[300]['depth_change_gazebo_m'],.03)
            rows[300]['forces_N'][0][4] += .1
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            with self.assertRaisesRegex(ValueError,'projection'):
                heave_report.read_trial(case,'bidirectional')

    def test_second_trial_failure_packages_first_and_skips_third(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(surge_batch.os,'geteuid',return_value=1000,create=True):
            calls = iter((fake_run, lambda *args: 1))
            with patch.object(surge_batch,'supervise',side_effect=lambda *args:next(calls)(*args)) as supervisor, \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(heave_batch.run(Path(tmp)),1)
            self.assertEqual(supervisor.call_count,2)
            archive, = (Path(tmp)/'results/ch3').glob('*.zip')
            with zipfile.ZipFile(archive) as z:
                data = json.loads(z.read('batch.json'))
                self.assertEqual(data['status'],'partial')
                self.assertIn('hover_balance',data['error'])
                self.assertIn(data['trials']['free_sink']+'/control.jsonl',z.namelist())
                self.assertNotIn('heave_response.svg',z.namelist())

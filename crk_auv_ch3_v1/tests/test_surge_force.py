"""Numerical, SDF, logging and package regressions; no Gazebo or native task instances."""
import contextlib
import importlib.util
import io
import json
import sys
import tempfile
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

from crk_ch3 import surge, surge_batch, surge_report
from crk_ch3.motion_protocol import BodyState, TeamStateBuffer
from crk_ch3.motion_run import update_summary
from crk_ch3.vehicle import load_profile


def load_trial():
    # Exercise the real collection loop without importing a mission or starting ROS.
    mods = {n:ModuleType(n) for n in ['rclpy','rclpy.signals','crk_ch3.search_ros','crk_ch3.envelope_ros',
                                        'crk_ch3.search_transport']}
    mods['rclpy.signals'].SignalHandlerOptions = object
    mods['crk_ch3.search_ros'].SearchNode = object
    mods['crk_ch3.envelope_ros'].state_data = lambda states: dict(stamp_ns=states[0].stamp_ns,
            velocity_body=[states[0].velocity_body], quaternion_xyzw=[states[0].quaternion])
    mods['crk_ch3.search_transport'].SearchTransport = lambda *args,**kwargs: SimpleNamespace(
        validate=lambda s:None, max_tilt=0.)
    path = Path(surge.__file__).with_name('surge_ros.py')
    spec = importlib.util.spec_from_file_location('crk_ch3._test_surge_ros',path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,mods):
        spec.loader.exec_module(module)
    return module


class PlantStub:
    def __init__(self, pos, drift=False):
        self.pos, self.drift, self.clock_ns = pos, drift, 40_000_000

    def ready(self):
        return [BodyState(self.clock_ns, self.pos,(0,0,0,1),(0,0,0),(0,0,0))]

    def check(self):
        pass

    def confirm_pause(self, expected=None):
        assert expected is None or expected==self.clock_ns
        return self.clock_ns

    def publish(self, values):
        assert np.array(values).shape==(1,8)

    def step(self, n):
        assert n==10
        self.clock_ns += 22_000_000 if self.drift else 20_000_000
        return self.ready(), self.clock_ns


class SurgeTests(unittest.TestCase):
    def test_independent_drag_reference_and_signed_free_decay(self):
        p = load_profile()
        for force in (5,10):
            eq = surge.steady_speed(force,p)
            self.assertAlmostEqual(13.7*eq+141*eq*eq,force)
            self.assertAlmostEqual(surge.reference_step(eq,force,.02,p),eq)
        for sign in (-1,1):
            u = sign*.2
            for _ in range(50):
                u = surge.reference_step(u,0,.02,p)
            factor = np.exp(-13.7/(13.5+6.36))
            exact = sign*.2*factor/(1+141*.2/13.7*(1-factor))
            self.assertAlmostEqual(u,exact,places=8)

    def test_single_model_topics_and_original_barrier(self):
        with tempfile.TemporaryDirectory() as d:
            trial = Path(d)/'trial'
            surge.create_trial(trial,5)
            w = ET.parse(trial/'world.sdf').find('world')
            self.assertEqual([m.get('name') for m in w.findall('model') if m.get('name').startswith('auv_')],['auv_0'])
            self.assertEqual([e.text for e in w.findall(".//plugin[@name='gz::sim::systems::Buoyancy']/enable")],['auv_0'])
            yaml = (trial/'bridge.yaml').read_text()
            self.assertNotIn('auv_1',yaml)
            self.assertEqual(yaml.count('direction: ROS_TO_GZ'),8)
            self.assertIn('/auv_0/contacts',yaml)
            self.assertEqual(len(TeamStateBuffer().by_agent),4)
            self.assertEqual(len(TeamStateBuffer(1).by_agent),1)

    def trial(self,root,case,headless,module,marker):
        info = json.loads((case/'program.json').read_text())
        summary = dict(level_N=info['level_N'],completed_control_steps=0)
        with (case/'control.jsonl').open('w') as stream:
            load_trial().collect(PlantStub(info['initial_world_position']),case,summary,stream)
        update_summary(case,**summary,termination_reason='completed',engineering_acceptance_passed=True)
        return 0

    def test_batch_report_with_real_collect_and_failing_clock(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            with patch.object(surge_batch,'supervise',side_effect=self.trial), \
                 patch.object(surge_batch.os,'geteuid',return_value=1000,create=True),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(surge_batch.run(root),0)
            archive, = (root/'results/ch3').glob('*.zip')
            with zipfile.ZipFile(archive) as z:
                self.assertIsNone(z.testzip())
                self.assertIn('surge_velocity.svg',z.namelist())
                self.assertEqual(len(json.loads(z.read('surge_metrics.json'))),2)
            batch = archive.with_suffix('')
            case = next(batch.glob('*_5N'))
            rows, m = surge_report.read_run(case,5)
            self.assertLess(m['release_below_0p02_N_after_s'],1)
            self.assertGreater(rows[-1]['t_s'],16)
            # Stand-in body velocity stays zero; theory must run independently.
            self.assertGreater(max(r['u_theory_m_s'] for r in rows),.13)
            plan = json.loads((case/'program.json').read_text())
            with self.assertRaisesRegex(RuntimeError,'clock'):
                load_trial().collect(PlantStub(plan['initial_world_position'],True),case,
                                     dict(level_N=5,completed_control_steps=0),io.StringIO())
            with (case/'control.jsonl').open('a') as stream:
                stream.write('{}\n')
            with self.assertRaises((ValueError,KeyError)):
                surge_report.read_run(case,5)

    def test_failed_first_run_preserves_partial_zip(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(surge_batch,'supervise',return_value=1) as call, \
                 patch.object(surge_batch.os,'geteuid',return_value=1000,create=True),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(surge_batch.run(Path(d)),1)
            call.assert_called_once()
            archive, = (Path(d)/'results/ch3').glob('*.zip')
            with zipfile.ZipFile(archive) as z:
                self.assertEqual(json.loads(z.read('batch.json'))['status'],'partial')
                self.assertNotIn('surge_velocity.svg',z.namelist())

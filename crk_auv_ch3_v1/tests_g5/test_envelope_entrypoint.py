"""Exercise real entry point and report writing with an explicit stationary ROS stub."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
from crk_ch3 import mission_runtime, mission_parity
from crk_ch3.motion_run import create_assets
from crk_ch3.motion_protocol import BodyState


class EntryTests(unittest.TestCase):
    def run_entrypoint(self, drift=False):
        with tempfile.TemporaryDirectory() as directory:
            out=create_assets(Path(directory),'x')
            summary=json.loads((out/'summary.json').read_text())
            summary['linux_sdf_valid']=True  # Stand-in, never Linux acceptance.
            (out/'summary.json').write_text(json.dumps(summary))
            origins=json.loads((out/'truth_scene.json').read_text())['agents']
            class FakeNode:
                def __init__(self, _):
                    self.clock_ns=40_000_000
                    self.contact_events=0;self.contact_failure=None
                    self.states=[BodyState(self.clock_ns,p,[0,0,0,1],[0,0,0],[0,0,0]) for p in origins]
                def ready(self): return self.states
                def confirm_pause(self, expected_ns=None):
                    if expected_ns is not None: assert self.clock_ns==expected_ns
                    return self.clock_ns
                def check(self): pass
                def publish(self, values): assert np.asarray(values).shape==(4,8)
                def step(self,count):
                    assert count==10
                    self.clock_ns+=22_000_000 if drift else 20_000_000
                    self.states=[BodyState(self.clock_ns,s.position,s.quaternion,s.velocity_body,s.rates_body) for s in self.states]
                    return self.states,self.clock_ns
                def stop_commands(self): pass
                def destroy_node(self): pass
                def get_logger(self): return SimpleNamespace(info=lambda *_:None)
            import numpy as np
            rclpy=ModuleType('rclpy')
            rclpy.init=lambda **_:None;rclpy.shutdown=lambda:None;rclpy.spin_once=lambda *a,**k:None
            signals=ModuleType('rclpy.signals');signals.SignalHandlerOptions=SimpleNamespace(NO=0)
            search=ModuleType('crk_ch3.search_ros');search.SearchNode=FakeNode
            path=Path(__file__).resolve().parents[1]/'crk_ch3/envelope_ros.py'
            spec=importlib.util.spec_from_file_location('crk_ch3._envelope_test',path)
            module=importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules,{'rclpy':rclpy,'rclpy.signals':signals,'crk_ch3.search_ros':search}):
                spec.loader.exec_module(module)
            with patch.dict(os.environ,CRK_OUTPUT_ROOT=directory,CRK_G4_REALTIME_VIEW='0'), \
                    patch.object(sys,'argv',['envelope_ros','--output',str(out)]), \
                    patch.object(module.Path,'is_mount',return_value=True),patch.object(module.signal,'signal'), \
                    contextlib.redirect_stdout(io.StringIO()),self.assertRaises(SystemExit) as status:
                module.main()
            self.assertEqual(status.exception.code,1 if drift else 0,(out/'summary.json').read_text())
            summary=json.loads((out/'summary.json').read_text())
            self.assertEqual(summary['completed_control_steps'],1 if drift else 1250)
            result=json.loads((out/'report/envelope_metrics.json').read_text())
            self.assertEqual(result['engineering_run_complete'],not drift)
            self.assertFalse(result['unrestricted_hardware_envelope_established'])
            self.assertTrue((out/'report/REPORT.md').is_file())
            self.assertEqual(len((out/'control.jsonl').read_text().splitlines()),0 if drift else 1250)
            self.assertTrue(summary['stop_zero_command_sent'])
            if drift:
                self.assertEqual(summary['termination_reason'],'engineering_failure')
                self.assertIn('clock drift',summary['error'])

    def test_recording_and_report_and_stop_on_standin(self):
        self.run_entrypoint()

    def test_clock_drift_stops_and_preserves_partial_failure(self):
        self.run_entrypoint(drift=True)

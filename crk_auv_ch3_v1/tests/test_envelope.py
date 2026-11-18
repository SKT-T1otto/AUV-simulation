import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from crk_ch3.envelope_protocol import engineering_scene, program, reference
from crk_ch3.envelope_report import analyze, dwell_index
from crk_ch3.motion_run import create_assets
from crk_ch3.search_control import AccelerationControl
from crk_ch3.vehicle import load_profile


class EnvelopeTests(unittest.TestCase):
    def test_bounded_clear_scene_preserves_input_and_profile(self):
        for case, ticks in [('x',1250),('y',1250),('z',1250),('track',1550)]:
            self.assertEqual(program(case)['total_ticks'],ticks)
        with tempfile.TemporaryDirectory() as directory:
            out=create_assets(Path(directory),'x')
            self.assertEqual(json.loads((out/'vehicle_profile.json').read_text()),load_profile())
            scene=json.loads((out/'scenario.json').read_text())
            self.assertEqual(scene['obstacle_count'],0)
            self.assertEqual(engineering_scene(copy.deepcopy(scene)),scene)
            text=(out/'world.sdf').read_text()
            self.assertEqual(text.count('<fluid_added_mass>'),4)
            self.assertNotIn('xDotU',text)
            self.assertEqual(text.count('name="hull_contact"'),4)
            summary=json.loads((out/'summary.json').read_text())
            self.assertFalse(summary['engineering_acceptance_passed'])
            self.assertFalse(summary['mission_executed'])

    def test_zero_request_is_distinct_from_active_braking(self):
        profile=load_profile()
        coast=program('x')['phases'][2];brake=program('x')['phases'][3]
        v=np.array([.3,0,0])
        a,_,_=reference(coast,np.zeros(3),np.zeros(3),v,profile)
        b,_,_=reference(brake,np.zeros(3),np.zeros(3),v,profile)
        np.testing.assert_array_equal(a,[0,0,0])
        np.testing.assert_allclose(b,[-.36,0,0])

    def test_diagnostics_separate_governor_allocation_and_actuator(self):
        ctrl=AccelerationControl(load_profile())
        forces,d=ctrl.command([1.3,1.3,.75],[0,0,0,1],[.5,0,0],[0,0,0])
        self.assertNotEqual(d['requested_acceleration_world'],d['speed_governed_acceleration_world'])
        self.assertNotEqual(d['speed_governed_acceleration_world'],d['governed_acceleration_world'])
        np.testing.assert_allclose(d['allocated_wrench'],ctrl.B@d['allocated_thrust_N'])
        np.testing.assert_allclose(d['commanded_wrench'],ctrl.B@forces)
        self.assertGreater(np.linalg.norm(np.array(d['allocated_thrust_N'])-forces),0)

    def test_stop_dwell_and_censored_braking_have_distinct_labels(self):
        self.assertIsNone(dwell_index([True]*24,25))
        self.assertEqual(dwell_index([False]*3+[True]*25,25),3)
        profile=load_profile(); plan=program('x'); phase=plan['phases'][3]
        rows=[]
        for k in range(30):
            ctrl=AccelerationControl(profile)
            force,d=ctrl.command([0,0,0],[0,0,0,1],[0,0,0],[0,0,0])
            rows.append(dict(phase=phase['name'],elapsed_s=(k+1)*.02,
                before=dict(positions_world=[[0,0,0]]*4,velocity_world=[[.3,0,0]]*4),
                after=dict(positions_world=[[(k+1)*.006,0,0]]*4,
                           velocity_world=[[.3,0,0]]*4,quaternion_xyzw=[[0,0,0,1]]*4),
                diagnostics=[d]*4,forces_N=[force.tolist()]*4,estimated_acceleration_world=[[0,0,0]]*4))
        result=analyze(rows,plan,profile)[0]['agents'][0]
        self.assertFalse(result['stop_observed'])
        self.assertIsNone(result['braking_path_to_stop_m'])
        self.assertEqual(result['censored_window_s'],.6)
        for r in rows:
            r['after']['velocity_world']=[[0,0,0]]*4
        result=analyze(rows,plan,profile)[0]['agents'][0]
        self.assertTrue(result['stop_observed'])
        self.assertEqual(result['stop_confirmed_time_s'],.5)


if __name__=='__main__':
    unittest.main()

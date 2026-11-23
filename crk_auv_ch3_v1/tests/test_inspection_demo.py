import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET
import numpy as np

from crk_ch3.inspection_demo import SurveyTransects, outcome, phase, presentation_world
from crk_ch3.mission_transport import MotionRequest
from crk_ch3.search_world import fixture, build_search_world
from crk_ch3.vehicle import load_profile


class InspectionDemoTests(unittest.TestCase):
    def test_motion_uses_stationary_delivery_only_after_receipt(self):
        starts = fixture('demo')['initial_agent_positions']
        survey = SurveyTransects(starts)
        env = Mock()
        env.get_task_state.return_value = SimpleNamespace(executor_knows_target=False)
        request = MotionRequest.create(1, starts, np.zeros((4,3)), np.ones((4,3)), [False]*4)
        applied, evidence = survey.apply(request, env)
        env.get_public_delivered_target.assert_not_called()
        self.assertEqual(applied.token,request.token)
        self.assertEqual(applied.acceleration_world[3].tolist(),[0.,0.,0.])
        self.assertTrue(np.all(applied.acceleration_world[:3,0] > 0))
        self.assertFalse(evidence['stationary_site_received'])
        env.get_task_state.return_value.executor_knows_target = True
        env.get_public_delivered_target.return_value = None
        with self.assertRaises(ValueError):
            survey.apply(request,env)
        env.get_public_delivered_target.return_value = dict(payload=dict(position=[5.6,6.,3.],velocity=[0.,0.,0.]))
        request = MotionRequest.create(2,starts,np.zeros((4,3)),np.ones((4,3)),[True,True,True,False])
        applied, evidence = survey.apply(request, env)
        self.assertTrue(evidence['stationary_site_received'])
        self.assertTrue(np.all(applied.acceleration_world[:3] == 0))
        self.assertLess(applied.acceleration_world[3,1],0)
        self.assertEqual(evidence['original_d2_request_m_s2'],request.acceleration_world.tolist())

    def test_scene_only_adds_visuals_to_original_plant(self):
        scene = fixture('demo')
        self.assertEqual(scene['target_initial_velocity'], [0,0,0])
        original, _ = build_search_world(scene, load_profile())
        before, after = ET.fromstring(original), ET.fromstring(presentation_world(original))
        for root in (before,after):
            for link in root.findall('.//link'):
                for visual in link.findall('visual'):
                    link.remove(visual)
            ET.indent(root)
        self.assertEqual(ET.tostring(before),ET.tostring(after))
        self.assertEqual(fixture('search')['target_initial_position'],[16,16,4])

    def test_display_requires_observed_discovery_handoff_travel_and_hold(self):
        sample = dict(mission_complete=False, native_hold=0, executor_received=False, target_found=False,
                      finder_id=1,executor_end=[0.,0.,0.],distance_end_m=2.,radius_m=.8)
        self.assertIn('搜索',phase(sample))
        sample.update(target_found=True)
        self.assertIn('auv_1',phase(sample))
        sample.update(executor_received=True)
        self.assertIn('交接',phase(sample))
        final = dict(sample,native_hold=5,mission_complete=True,executor_end=[1.5,0.,0.],distance_end_m=.5)
        self.assertIn('完成',phase(final))
        summary = dict(engineering_acceptance_passed=True, capture_evidence=dict(verified=True),
                       found_step=10,demo_search_displacements_m=[.3,.3,.3])
        self.assertTrue(outcome([sample,final],summary)['verified'])
        for key,value in [('engineering_acceptance_passed',False), ('capture_evidence',dict(verified=False)),
                          ('found_step',1), ('demo_search_displacements_m',[.3,.3,.01])]:
            self.assertFalse(outcome([sample,final],dict(summary,**{key:value}))['verified'])
        self.assertFalse(outcome([final],summary)['verified'])

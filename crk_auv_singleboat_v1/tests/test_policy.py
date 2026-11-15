"""Contracts, causal reply matching and decimated actor/actuator integration."""
import copy
import importlib.util
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]/'src/crk_auv_demo'
sys.path.insert(0, str(PACKAGE))
from crk_auv_demo.policy import ObservationAdapter, ResidualActionMapper, PolicyPipeline, TaskContext
from crk_auv_demo.actors import verify_metadata
from crk_auv_demo.core import ControlLaw

CONFIG = json.loads((PACKAGE/'config/policy_demo.json').read_text())
VEHICLE = json.loads((PACKAGE/'config/vehicle.json').read_text())


class PolicyContracts(unittest.TestCase):
    def test_observation_layout_and_frames(self):
        c = copy.deepcopy(CONFIG)
        c['world_to_policy_rotation'] = [[0,-1,0],[1,0,0],[0,0,1]]
        adapter = ObservationAdapter(c)
        context = TaskContext(target_known=True, known_target_world=[2,1,-2],
                              nearest_obstacle_m=3, waypoint_progress=.25, hold_progress=.5)
        obs = adapter.observe([1,1,-2], [0,0,0,1], [1,0,0], [1,3,-2], context)
        np.testing.assert_allclose(obs[:3], [4,6,3])
        np.testing.assert_allclose(obs[3:6], [0,1,0])
        np.testing.assert_allclose(obs[6:9], [-2,0,0])
        np.testing.assert_allclose(obs[9:12], [-1,0,0])
        np.testing.assert_allclose(obs[12:15], [0,1,0])
        np.testing.assert_allclose(obs[15:22], [.2,1,0,.3,.25,0,.5], atol=1e-6)
        np.testing.assert_array_equal(obs[22:], [0,1,0,0,0,1])
        self.assertEqual(obs.dtype, np.float32)
        with self.assertRaises(ValueError):
            adapter.observe([0,0,0], [0,0,0,1], [0,0,0], [0,0,0],
                            TaskContext(target_known=True))

    def test_original_prior_formula_and_residual_clip(self):
        c = json.loads((PACKAGE/'config/policy_reference_search_balanced.json').read_text())
        obs = np.zeros(28)
        obs[6:9] = [3,4,2]
        obs[3:6] = [.2,-.1,.1]
        out = ResidualActionMapper(c).compose(obs, [3,-2,.5])
        # Desired XY is [1.32,1.76], speed norm 2.2; prior XY clips to [1,1].
        np.testing.assert_allclose(out['prior_acc_policy'], [1,1,.7])
        np.testing.assert_allclose(out['residual_acc_policy'], [.45,-.45,.1575])
        np.testing.assert_allclose(out['acceleration_policy'], [1,.55,.7])
        disabled = ResidualActionMapper(c).compose(obs, [1,1,1], enabled=False)
        np.testing.assert_array_equal(disabled['acceleration_policy'], np.zeros(3))
        for invalid in ([1,2], [float('nan'),0,0], [[1,2,3]]):
            with self.assertRaises(ValueError):
                ResidualActionMapper(c).compose(obs, invalid)

    def test_policy_rate_and_external_matching(self):
        pipeline = PolicyPipeline(CONFIG, 'external')
        state = ([0,0,-2], [0,0,0,1], [0,0,0], [1,0,-2], TaskContext())
        packet = pipeline.update(1., *state)
        response = {'schema': 'crk.residual3.v1', 'seq': packet['seq'],
                    'policy_contract_sha256': packet['policy_contract_sha256'],
                    'observation_sim_s': packet['sim_s'], 'action': [.1,0,0]}
        self.assertFalse(pipeline.accept_external({**response, 'seq': 0}, 1.01))
        self.assertTrue(pipeline.accept_external(response, 1.02))
        held = pipeline.acceleration_world.copy()
        self.assertFalse(pipeline.accept_external(response, 1.03))
        self.assertIsNone(pipeline.update(1.1, *state))
        np.testing.assert_array_equal(held, pipeline.acceleration_world)
        self.assertEqual(pipeline.update(1.2, *state)['seq'], 2)
        with self.assertRaises(TimeoutError):
            pipeline.update(1.36, *state)

    def test_reject_response_timestamp_and_frame_reflection(self):
        pipeline = PolicyPipeline(CONFIG, 'external')
        packet = pipeline.update(1., [0,0,-2], [0,0,0,1], [0,0,0], [0,0,-2], TaskContext())
        with self.assertRaises(ValueError):
            pipeline.accept_external({'schema':'crk.residual3.v1','seq':1,
                                      'policy_contract_sha256':'wrong-contract',
                                      'observation_sim_s':1.,'action':[0,0,0]}, 1.02)
        with self.assertRaises(ValueError):
            pipeline.accept_external({'schema':'crk.residual3.v1','seq':1,
                                      'policy_contract_sha256':packet['policy_contract_sha256'],
                                      'observation_sim_s':.9,'action':[0,0,0]}, 1.02)
        c = copy.deepcopy(CONFIG)
        c['world_to_policy_rotation'] = [[1,0,0],[0,1,0],[0,0,-1]]
        with self.assertRaises(ValueError):
            ObservationAdapter(c)

    def test_actor_changes_physical_command_without_double_prior(self):
        c = copy.deepcopy(CONFIG)
        c['use_residual_prior'] = False
        pipeline = PolicyPipeline(c, 'torchscript', actor=lambda obs: [.5,0,0])
        # Mock inference exercises the same path as a TorchScript actor, at a distant waypoint.
        pipeline.update(0., [0,0,-2], [0,0,0,1], [0,0,0], [100,100,-2], TaskContext())
        np.testing.assert_allclose(pipeline.acceleration_world, [.135,0,0])
        actuator = ControlLaw(VEHICLE)
        force, _ = actuator.execute_acceleration(pipeline.acceleration_world,
                                                 [0,0,0,1], np.zeros(3), np.zeros(3), .02)
        wrench = actuator.B @ force
        self.assertGreater(wrench[0], 0)
        np.testing.assert_allclose(wrench[1:], np.zeros(5), atol=1e-10)
        c['world_to_policy_rotation'] = [[0,-1,0],[1,0,0],[0,0,1]]
        rotated = PolicyPipeline(c, 'torchscript', actor=lambda obs: [.5,0,0])
        rotated.update(0., [0,0,-2], [0,0,0,1], [0,0,0], [100,100,-2], TaskContext())
        np.testing.assert_allclose(rotated.acceleration_world, [0,-.135,0])

    def test_export_metadata_refuses_wrong_weights_or_team_policy(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model, config, meta = root/'model.pt', root/'config.json', root/'meta.json'
            model.write_bytes(b'not-loaded-for-metadata-test')
            config.write_text(json.dumps(CONFIG), encoding='utf-8')
            metadata = {'schema':'crk.actor_export.v1', 'observation_dim':28, 'action_dim':3,
                        'output_semantics':'normalized_acceleration_residual',
                        'requires_team_context':False, 'recurrent':False,
                        'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
                        'policy_config_sha256':hashlib.sha256(config.read_bytes()).hexdigest()}
            meta.write_text(json.dumps(metadata), encoding='utf-8')
            verify_metadata(model, meta, config)
            metadata['requires_team_context'] = True
            meta.write_text(json.dumps(metadata), encoding='utf-8')
            with self.assertRaises(ValueError):
                verify_metadata(model, meta, config)
            metadata['requires_team_context'] = False
            metadata['model_sha256'] = '0'*64
            meta.write_text(json.dumps(metadata), encoding='utf-8')
            with self.assertRaises(ValueError):
                verify_metadata(model, meta, config)

    @unittest.skipUnless(importlib.util.find_spec('torch'), 'Optional PyTorch is not installed')
    def test_real_torchscript_inference(self):
        import torch
        from crk_auv_demo.actors import TorchScriptActor
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model_path, policy_path, meta = root/'actor.pt', root/'policy.json', root/'actor.json'
            actor = torch.nn.Linear(28, 3).eval()
            with torch.no_grad():
                actor.weight.zero_()
                actor.bias.copy_(torch.tensor([.1,-.2,.3]))
            torch.jit.trace(actor, torch.zeros(1,28)).save(str(model_path))
            policy_path.write_text(json.dumps(CONFIG), encoding='utf-8')
            metadata = {'schema':'crk.actor_export.v1', 'observation_dim':28, 'action_dim':3,
                        'output_semantics':'normalized_acceleration_residual',
                        'requires_team_context':False, 'recurrent':False,
                        'model_sha256':hashlib.sha256(model_path.read_bytes()).hexdigest(),
                        'policy_config_sha256':hashlib.sha256(policy_path.read_bytes()).hexdigest()}
            meta.write_text(json.dumps(metadata), encoding='utf-8')
            loaded = TorchScriptActor(model_path, meta, policy_path)
            np.testing.assert_allclose(loaded(np.zeros(28, dtype=np.float32)), [.1,-.2,.3], atol=1e-7)


if __name__ == '__main__':
    unittest.main()

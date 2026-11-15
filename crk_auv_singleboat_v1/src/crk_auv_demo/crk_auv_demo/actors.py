"""Optional stateless TorchScript actor, with an explicit export contract."""
import hashlib
import json
from pathlib import Path
import numpy as np
from crk_auv_demo.policy import vector3


def verify_metadata(model_path, metadata_path, policy_path):
    metadata = json.loads(Path(metadata_path).read_text(encoding='utf-8'))
    expected = {'schema': 'crk.actor_export.v1', 'observation_dim': 28, 'action_dim': 3,
                'output_semantics': 'normalized_acceleration_residual',
                'requires_team_context': False, 'recurrent': False}
    for key, value in expected.items():
        if type(metadata.get(key)) is not type(value) or metadata[key] != value:
            raise ValueError('Unsupported actor metadata: '+key)
    for key, path in [('model_sha256', model_path), ('policy_config_sha256', policy_path)]:
        digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        if metadata.get(key) != digest:
            raise ValueError('Actor metadata digest mismatch: '+key)
    return metadata


class TorchScriptActor:
    def __init__(self, model_path, metadata_path, policy_path):
        self.metadata = verify_metadata(model_path, metadata_path, policy_path)
        # PyTorch is deliberately optional: zero/external modes do not import it.
        import torch
        self.torch = torch
        self.model = torch.jit.load(str(model_path), map_location='cpu').eval()

    def __call__(self, observation):
        if np.asarray(observation).shape != (28,):
            raise ValueError('TorchScript actor requires a 28D observation')
        torch = self.torch
        with torch.inference_mode():
            tensor = torch.tensor(observation, dtype=torch.float32).reshape(1, 28)
            result = self.model(tensor)
        if not isinstance(result, torch.Tensor) or tuple(result.shape) != (1, 3):
            raise ValueError('Exported actor must return one [1,3] tensor')
        return vector3(result.detach().cpu().numpy()[0])

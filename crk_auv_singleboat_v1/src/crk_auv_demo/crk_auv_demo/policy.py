"""28D observation / 3D residual contract and simulation-time policy scheduling.

Independent of ROS and PyTorch. No dynamics integration and no implicit team state.
"""
from dataclasses import dataclass
import hashlib
import json
import numpy as np
from crk_auv_demo.core import rotation


def vector3(value):
    out = np.asarray(value, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError('Expected finite vector with shape (3,)')
    return out


@dataclass
class TaskContext:
    # Explicit single-boat test context, not inferred search/communication events.
    target_known: bool = False
    known_target_world: object = None
    nearest_obstacle_m: float = 10.0
    waypoint_progress: float = 0.0
    finished: bool = False
    hold_progress: float = 0.0
    enabled: bool = True


class ObservationAdapter:
    def __init__(self, config):
        self.config = config
        self.R = np.asarray(config['world_to_policy_rotation'], dtype=float)
        self.offset = vector3(config['world_to_policy_translation_m'])
        if (self.R.shape != (3, 3) or not np.isfinite(self.R).all()
                or not np.allclose(self.R.T @ self.R, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(self.R), 1., atol=1e-7)
                or not np.allclose(self.R @ [0, 0, 1], [0, 0, 1], atol=1e-7)):
            raise ValueError('Frame transform must be a proper rotation preserving vertical up')
        if type(config['role_index']) is not int or not 0 <= config['role_index'] < 4:
            raise ValueError('role_index must be in [0,3]')

    def observe(self, position_world, quaternion, body_velocity, target_world, context):
        c = self.config
        position = self.R @ vector3(position_world) + self.offset
        velocity = self.R @ rotation(quaternion) @ vector3(body_velocity)
        target = self.R @ vector3(target_world) + self.offset
        relative = target-position
        distance = max(float(np.linalg.norm(relative)), 1e-6)
        direction = relative/distance
        speed = max(float(np.linalg.norm(velocity)), 1e-6)
        known_delta = np.zeros(3)
        if context.target_known:
            if context.known_target_world is None:
                raise ValueError('Known target flag requires a delivered target position')
            known_delta = self.R @ vector3(context.known_target_world) + self.offset-position
        context_values = [context.nearest_obstacle_m, context.waypoint_progress, context.hold_progress]
        if (not np.isfinite(context_values).all() or context.nearest_obstacle_m < 0
                or not 0 <= context.waypoint_progress <= 1 or not 0 <= context.hold_progress <= 1):
            raise ValueError('Invalid task context')
        role = np.eye(4)[c['role_index']]
        obs = np.r_[position, velocity, relative, direction, known_delta,
                    np.clip(distance/10., 0, 1), np.clip(speed/(c['v_xy_max']+1e-6), 0, 1),
                    np.tanh(np.dot(velocity, direction)/(c['v_xy_max']+1e-6)),
                    np.clip(context.nearest_obstacle_m/10., 0, 1), context.waypoint_progress,
                    float(context.finished), context.hold_progress, role,
                    [0., 1.] if context.target_known else [1., 0.]]
        if obs.shape != (28,) or not np.isfinite(obs).all():
            raise ValueError('Invalid observation contract')
        return obs.astype(np.float32)


class ResidualActionMapper:
    def __init__(self, config):
        self.config = config
        for key in ('v_xy_max', 'v_z_max', 'a_xy_max', 'a_z_max', 'prior_kv_xy',
                    'prior_kv_z', 'prior_slow_radius_xy', 'prior_slow_radius_z'):
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('Invalid policy parameter: '+key)
        for key in ('prior_strength', 'residual_scale'):
            if not np.isfinite(config[key]) or config[key] < 0:
                raise ValueError('Invalid policy parameter: '+key)

    def compose(self, obs, action, enabled=True):
        """Original reference prior/scale/axis clipping; no actuator-side speed clipping."""
        c = self.config
        obs = np.asarray(obs, dtype=float)
        if obs.shape != (28,) or not np.isfinite(obs).all():
            raise ValueError('Expected finite 28D observation')
        action = np.clip(vector3(action), -1., 1.)
        relative, velocity = obs[6:9], obs[3:6]
        dist_xy = max(float(np.linalg.norm(relative[:2])), 1e-6)
        desired_xy = relative[:2]/dist_xy*c['v_xy_max']*np.clip(dist_xy/c['prior_slow_radius_xy'], 0, 1)
        desired_z = np.clip(relative[2]/c['prior_slow_radius_z'], -1, 1)*c['v_z_max']
        maximum = np.array([c['a_xy_max'], c['a_xy_max'], c['a_z_max']])
        prior = np.clip(np.r_[c['prior_kv_xy']*(desired_xy-velocity[:2]),
                              c['prior_kv_z']*(desired_z-velocity[2])], -maximum, maximum)
        prior = c['prior_strength']*prior if c['use_residual_prior'] else np.zeros(3)
        residual = c['residual_scale']*maximum*action
        acceleration = np.clip(prior+residual, -maximum, maximum)
        if not enabled:
            prior[:], residual[:], acceleration[:] = 0., 0., 0.
        return {'action_norm': action, 'prior_acc_policy': prior,
                'residual_acc_policy': residual, 'acceleration_policy': acceleration}


class PolicyPipeline:
    """Sample at policy frequency, hold acceleration between decisions.

    External replies must identify the exact observation sequence and timestamp.
    Missing/stale inference stops the test; it never silently substitutes a policy.
    """
    def __init__(self, config, mode='zero', actor=None):
        if mode not in ('zero', 'external', 'torchscript'):
            raise ValueError('Unknown policy mode')
        self.adapter = ObservationAdapter(config)
        self.mapper = ResidualActionMapper(config)
        self.config, self.mode, self.actor = config, mode, actor
        self.contract_sha256 = hashlib.sha256(json.dumps(
            config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if mode == 'torchscript' and actor is None:
            raise ValueError('TorchScript mode requires an actor')
        self.period = float(config['decision_period_s'])
        self.timeout = float(config['action_timeout_s'])
        if not 0 < self.timeout < self.period:
            raise ValueError('Action timeout must be positive and shorter than decision period')
        self.last_decision = None
        self.sequence = 0
        self.pending = None
        self.acceleration_world = np.zeros(3)
        self.last_diagnostics = None
        self.accepted_actions = 0
        self.rejected_actions = 0

    def _apply(self, action, obs, enabled):
        diagnostics = self.mapper.compose(obs, action, enabled)
        self.acceleration_world = self.adapter.R.T @ diagnostics['acceleration_policy']
        self.last_diagnostics = {k: v.tolist() for k, v in diagnostics.items()}
        self.accepted_actions += 1

    def update(self, sim_s, position, quaternion, velocity, target, context):
        if not np.isfinite(sim_s):
            raise ValueError('Invalid simulation time')
        if self.pending is not None and sim_s-self.pending['sim_s'] > self.timeout+1e-9:
            raise TimeoutError('External action timed out')
        if self.last_decision is not None:
            if sim_s < self.last_decision:
                raise ValueError('Simulation time reset; restart the policy pipeline')
            if sim_s-self.last_decision < self.period-1e-9:
                return None
        obs = self.adapter.observe(position, quaternion, velocity, target, context)
        self.sequence += 1
        self.last_decision = float(sim_s)
        packet = {'schema': 'crk.observation28.v1', 'seq': self.sequence, 'sim_s': float(sim_s),
                  'frame': 'policy', 'profile_id': self.config['profile_id'],
                  'policy_contract_sha256': self.contract_sha256,
                  'role_index': self.config['role_index'], 'observation': obs.tolist(),
                  'enabled': context.enabled, 'singleboat_test_context': True}
        if self.mode == 'external':
            self.pending = {'seq': self.sequence, 'sim_s': float(sim_s),
                            'observation': obs, 'enabled': context.enabled}
        else:
            action = np.zeros(3) if self.mode == 'zero' else self.actor(obs.copy())
            self._apply(action, obs, context.enabled)
        return packet

    def accept_external(self, packet, now_sim_s):
        if self.mode != 'external':
            raise ValueError('External actions are disabled')
        if not isinstance(packet, dict):
            raise ValueError('Action envelope must be a JSON object')
        pending = self.pending
        if (pending is None or packet.get('seq') != pending['seq']
                or type(packet.get('seq')) is not int):
            self.rejected_actions += 1
            return False
        if (packet.get('schema') != 'crk.residual3.v1' or
                packet.get('policy_contract_sha256') != self.contract_sha256 or
                not np.isfinite(packet.get('observation_sim_s', float('nan'))) or
                abs(packet['observation_sim_s']-pending['sim_s']) > 1e-8):
            raise ValueError('Action schema/timestamp mismatch')
        age = now_sim_s-pending['sim_s']
        if not np.isfinite(age) or not -1e-8 <= age <= self.timeout+1e-9:
            raise TimeoutError('External action arrived outside its simulation-time window')
        self._apply(packet['action'], pending['observation'], pending['enabled'])
        self.pending = None
        return True

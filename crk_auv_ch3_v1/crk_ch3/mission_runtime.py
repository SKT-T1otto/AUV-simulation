"""G5 mission runtime: original D2 maps/events/rewards with external motion."""
import copy
import json
from types import MethodType
import numpy as np
import torch

from .native_source import ROOT, activate_native
activate_native()
from chapter3_bser.learning.base_runtime import _make_base_env
from chapter3_bser.learning.config import validate
from chapter3_bser.learning.runtime import RepairedMissionRuntime
from .mission_dynamics import ExternalDynamics
from .mission_transport import MotionExchange
from .scenario import validate_scene


def reference_config():
    # Source b2 is the shared repaired task/reward/gating configuration. ZERO
    # means zero normalized residual, not a B2 actor and not zero total thrust.
    config = json.loads((ROOT/'native_g5/configs/b2.json').read_text(encoding='utf-8'))
    if config.get('map_experiment') is not None:
        raise ValueError('G5 pin expects original D2; research map variants require their own gate')
    return validate(config)


class ZeroResidualPolicy:
    def step(self, observations, explore=False):
        if explore or np.asarray(observations).shape != (4, 28):
            raise ValueError('G5 ZERO requires four 28D observations and deterministic inference')
        return [torch.zeros((1, 3)) for _ in range(4)]


class ExternalMissionRuntime(RepairedMissionRuntime):
    def __init__(self, scenario, motion_callback, *, seed=None):
        activate_native()
        self.motion_exchange = MotionExchange(motion_callback)
        self.poisoned = False
        scene = validate_scene(scenario)
        if type(scene.get('planner_seed')) is not int or scene['planner_seed'] < 0:
            raise ValueError('Native mission reset requires an explicit nonnegative planner_seed')
        super().__init__(reference_config(), copy.deepcopy(scene),
                         seed=scene['scenario_seed'] if seed is None else int(seed))

    def _make_environment(self, config):
        base = _make_base_env(config)
        native = base.unwrapped
        if native.task_protocol != 'collision_terminal_v1' or native.dt != .2:
            raise ValueError('G5 requires the pinned strict 0.2 s mission contract')
        native._crk_motion_exchange = self.motion_exchange
        native._apply_agent_dynamics = MethodType(ExternalDynamics._apply_agent_dynamics, native)
        return base

    def advance(self, model=None, *, explore=False):
        if self.poisoned or self.motion_exchange.failed:
            raise RuntimeError('Mission transaction failed; construct a new runtime before continuing')
        if explore:
            raise ValueError('G5 does not enable exploration or training')
        if model is not None:
            raise ValueError('G5 enables ZERO only; learned policies wait for G7')
        try:
            result = super().advance(ZeroResidualPolicy(), explore=False)
            exchange = self.motion_exchange
            raw = exchange.last_feedback.positions
            task = self.env.unwrapped._agent_pos.detach().cpu().numpy().copy()
            result['motion'] = dict(task_step=exchange.last_request.task_step,
                                    requested_acceleration_native=exchange.last_request.acceleration_world.tolist(),
                                    measured_positions_native=raw.tolist(), task_positions_native=task.tolist(),
                                    task_projection_distance_m=np.linalg.norm(task-raw, axis=1).tolist(),
                                    feedback_stamp_ns=exchange.last_feedback.stamp_ns)
            return result
        except Exception:
            self.poisoned = True
            raise

"""G5 bounded reference recording and replay, NOT a Gazebo motion experiment."""
import copy
from dataclasses import fields, is_dataclass
from enum import Enum
import json
import math
import random
from types import MethodType
import numpy as np
import torch

from .contracts import canonical_hash
from .native_source import ROOT, activate_native
activate_native()
from chapter3_bser.learning.runtime import RepairedMissionRuntime
from chapter3_bser.learning.base_runtime import _make_base_env
from .mission_runtime import ExternalMissionRuntime, ZeroResidualPolicy, reference_config
from .mission_dynamics import ExternalDynamics
from .mission_transport import MotionExchange, MotionFeedback


def normalized(value):
    if torch.is_tensor(value):
        return {'dtype': str(value.dtype), 'shape': list(value.shape),
                'values': normalized(value.detach().cpu().tolist())}
    if isinstance(value, np.ndarray):
        return {'dtype': str(value.dtype), 'shape': list(value.shape), 'values': normalized(value.tolist())}
    if isinstance(value, np.generic):
        return normalized(value.item())
    if isinstance(value, Enum):
        return normalized(value.value)
    if is_dataclass(value):
        return {f.name: normalized(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, dict):
        return {str(k): normalized(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [normalized(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_reference_diagnostic': str(value)}
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if hasattr(value, 'to_payload'):
        return normalized(value.to_payload())
    raise TypeError('Unreviewed comparison value: '+str(type(value)))


def scalar_and_tensor_state(obj):
    return {key: value for key, value in vars(obj).items() if not key.startswith('_crk_')
            and (torch.is_tensor(value) or isinstance(value, (np.ndarray, np.generic, str, bool, int, float))
                 or value is None)}


def snapshot(env, observations, *, guidance=None, reward=None):
    native = env.unwrapped
    return normalized(dict(observations=observations, guidance=guidance, reward=reward,
        agent=env.get_agent_state(), task=env.get_task_state(), execution=env.get_search_execution_state(),
        native_state=scalar_and_tensor_state(native), map_state=scalar_and_tensor_state(native.map_module),
        target=native.target_state.to_payload(), handoff=native.fixed_reliable_handoff.state_dict(),
        delivered_target=native.executor_delivered_target_state,
        outcome=native.episode_outcome, collisions=native.last_collision_records,
        public_events=native.read_public_events()))


def assert_same(reference, replay, label):
    if reference != replay:
        keys = sorted(set(reference) | set(replay))
        changed = [key for key in keys if reference.get(key) != replay.get(key)]
        raise AssertionError(f'{label}: source/replay mismatch in {changed}')
    return canonical_hash(reference)


def rng_state():
    return random.getstate(), np.random.get_state(), torch.get_rng_state().clone()


def restore_rng(state):
    random.setstate(state[0])
    np.random.set_state(state[1])
    torch.set_rng_state(state[2])


def capture_motion(native, tape, path=None):
    original = native._apply_agent_dynamics
    def instrument(env, actions):
        start = env._agent_pos.detach().cpu().numpy().copy()
        original(actions)
        endpoint = env._agent_pos.detach().cpu().numpy().copy()
        velocity = env._agent_vel.detach().cpu().numpy().copy()
        # Strict native collision handling projects onto first contact. Recover
        # the recorded pre-projection candidate so collision tau is replayed too.
        for hit in env.last_collision_records:
            i = hit['agent_id']
            endpoint[i] = hit['candidate_position']
            velocity[i] = (endpoint[i]-start[i])/env.dt
        tape.append(dict(step=int(env.step_count), positions=endpoint.tolist(), velocities=velocity.tolist(),
                         acceleration=env._agent_acc.detach().cpu().numpy().tolist()))
        if path is not None:
            with path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(tape[-1])+'\n')
    native._apply_agent_dynamics = MethodType(instrument, native)


def enable_journal(native):
    native.enable_public_event_capture('g5_parity', capacity=256)
    native._public_event_journal.reset()


def engineering_scenes():
    original = json.loads((ROOT/'tests/fixtures/scene.json').read_text(encoding='utf-8'))
    original.update(planner_seed=2729, purpose='g5_bounded_parity_fixture_not_performance_scenario')
    mapping = copy.deepcopy(original)
    mapping.update(scenario_id='g5_mapping_reflection', target_position=[19.7, 16, 4],
                   target_initial_position=[19.7, 16, 4], target_initial_velocity=[1, 0, 0])
    events = copy.deepcopy(original)
    events.update(scenario_id='g5_discovery_handoff_capture', initial_executor_wait_point=[4.8, 2, 2],
                  target_position=[4.1, 2, 2], target_initial_position=[4.1, 2, 2], target_initial_velocity=[.1, 0, 0])
    events['initial_agent_positions'][3] = [4.8, 2, 2]
    collision = copy.deepcopy(original)
    collision['scenario_id'] = 'g5_collision_priority'
    return mapping, events, collision


def replay_callback(tape, request):
    record = tape[-1]
    if record['step'] != request.task_step:
        raise AssertionError('Reference motion record belongs to another step')
    np.testing.assert_allclose(request.acceleration_world, record['acceleration'], atol=0, rtol=0)
    return MotionFeedback.for_request(request, record['positions'], record['velocities'])


def full_runtime_pair(scene, steps, directory, *, probes=False):
    print(f"G5 full-runtime parity: {scene['scenario_id']} (at most {steps} steps)", flush=True)
    (directory/'scenario.json').write_text(json.dumps(scene, indent=2)+'\n', encoding='utf-8')
    tape = []
    reference = RepairedMissionRuntime(reference_config(), scene, seed=scene['scenario_seed'])
    reference_rng = rng_state()
    port = None
    try:
        port = ExternalMissionRuntime(scene, lambda request: replay_callback(tape, request))
        port_rng = rng_state()
        for runtime in (reference, port):
            enable_journal(runtime.env.unwrapped)
        capture_motion(reference.env.unwrapped, tape, directory/'recorded_motion.jsonl')
        assert_same(snapshot(reference.env, reference.observations, guidance=reference.guidance),
                    snapshot(port.env, port.observations, guidance=port.guidance), 'reset')
        rows = []
        for index in range(steps):
            restore_rng(reference_rng)
            expected = reference.advance(ZeroResidualPolicy(), explore=False)
            reference_rng = rng_state()
            restore_rng(port_rng)
            actual = port.advance()
            port_rng = rng_state()
            motion = actual.pop('motion')
            assert_same(normalized(expected), normalized(actual), f'full transition {index+1}')
            digest = assert_same(snapshot(reference.env, reference.observations, guidance=reference.guidance,
                                          reward=reference.env.last_reward_breakdown),
                                 snapshot(port.env, port.observations, guidance=port.guidance,
                                          reward=port.env.last_reward_breakdown), f'full state {index+1}')
            assert_same(normalized(reference_rng), normalized(port_rng), f'RNG {index+1}')
            rows.append(dict(task_step=port.step, identical=True, compared_state_sha256=digest,
                             target_found=port.env.get_task_state().target_found,
                             executor_knows_target=port.env.get_task_state().executor_knows_target,
                             terminal=port.terminal, motion=motion))
            with (directory/'comparisons.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(rows[-1])+'\n')
            if (index+1) % 4 == 0 or port.terminal:
                print(f'  identical step={port.step}, found={reference.found_step}, terminal={port.terminal}', flush=True)
            if port.terminal:
                break
        assert_same(normalized(reference.summary()), normalized(port.summary()), 'episode diagnostics')
        result = dict(case=scene['scenario_id'], paired_steps=len(rows), parity_passed=True,
                      found_step=port.found_step, handoff_event_step=port.handoff_event_step,
                      handoff_decision_step=port.handoff_decision_step,
                      capture_hold_max=port.env.unwrapped.capture_hold_counter_max,
                      native_termination=port.env.unwrapped.episode_outcome.termination_reason)
        (directory/'case_summary.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        if scene['scenario_id'] == 'g5_discovery_handoff_capture':
            if not (result['found_step'] is not None and result['handoff_decision_step'] is not None
                    and result['native_termination'] == 'success' and result['capture_hold_max'] >= 5):
                raise AssertionError('Event fixture failed to exercise discovery, handoff and five-step capture')
        else:
            assert not port.env.get_task_state().target_found
            assert port.env.unwrapped.target_state.velocity[0] < 0, 'Reflection fixture did not reflect target'
        if probes:
            # A second, deliberately stationary plant record must remain
            # stationary: no old drag/flow/position integrator may run secretly.
            before = port.env.unwrapped._agent_pos.detach().cpu().numpy().copy()
            port.motion_exchange.callback = lambda r: MotionFeedback.for_request(r, r.positions, np.zeros((4, 3)))
            port.advance()
            np.testing.assert_array_equal(before, port.env.unwrapped._agent_pos.detach().cpu().numpy())
            result['stationary_external_motion_did_not_integrate'] = True
            port.motion_exchange.callback = lambda _: None
            try:
                port.advance()
            except ValueError:
                pass
            else:
                raise AssertionError('Invalid feedback did not fail')
            try:
                port.advance()
            except RuntimeError:
                result['failed_transaction_cannot_retry'] = True
            else:
                raise AssertionError('Poisoned mission was reused')
        (directory/'case_summary.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        return result
    finally:
        reference.close()
        if port is not None:
            port.close()


def core_pair(scene, *, collision=False):
    """Two focused environment pairs; fixtures only, no change to runtime config."""
    from chapter3_bser.learning.base_runtime import seed_innovations
    seed_innovations(scene['scenario_seed'])
    reference = _make_base_env(reference_config())
    reference.reset(scene)
    reference_rng = rng_state()
    seed_innovations(scene['scenario_seed'])
    port = _make_base_env(reference_config())
    port.reset(scene)
    port_rng = rng_state()
    tape = []
    exchange = MotionExchange(lambda request: replay_callback(tape, request))
    port.unwrapped._crk_motion_exchange = exchange
    port.unwrapped._apply_agent_dynamics = MethodType(ExternalDynamics._apply_agent_dynamics, port.unwrapped)
    try:
        for env in (reference, port):
            enable_journal(env.unwrapped)
            if collision:
                # An explicit recorded-state fixture approaching a wall.
                env.unwrapped._agent_pos[0] = torch.tensor([6.95, 6., 4.])
                env.unwrapped._agent_vel[0] = torch.tensor([2.8, 0., 0.])
            else:
                env.unwrapped.max_steps = 2  # Bounded timeout fixture, not production horizon.
        capture_motion(reference.unwrapped, tape)
        rows = []
        for index in range(1 if collision else 2):
            # Exercise nonzero, clipped normalized residuals with source prior.
            actions = torch.tensor([[1.2, -.6, .4], [-.4, .2, -.8], [.3, -.1, .7], [-.5, .6, .2]])
            restore_rng(reference_rng)
            expected = reference.step(actions)
            reference_rng = rng_state()
            restore_rng(port_rng)
            actual = port.step(actions)
            port_rng = rng_state()
            assert_same(normalized(expected), normalized(actual), 'core transition')
            assert_same(snapshot(reference, expected[0]), snapshot(port, actual[0]), 'core map/events/outcome')
            assert_same(normalized(reference_rng), normalized(port_rng), 'core RNG')
            rows.append(index+1)
        reason = port.unwrapped.episode_outcome.termination_reason
        assert reason == ('obstacle_collision' if collision else 'timeout'), reason
        if collision:
            assert port.unwrapped.target_state.sample_step == reference.unwrapped.target_state.sample_step == 0
            assert port.unwrapped.map_module.map_update_count == reference.unwrapped.map_module.map_update_count
            np.testing.assert_array_equal(actual[1], np.full(4, -2.))
        return dict(case='collision_priority_nonzero_action' if collision else 'timeout_nonzero_action',
                    paired_steps=len(rows), parity_passed=True, native_termination=reason,
                    bounded_fixture_override='injected pre-contact pose/velocity on both core fixtures' if collision else 'max_steps=2 on both core fixtures')
    finally:
        reference.close()
        port.close()


def run_suite(output, on_case=None):
    """Eight native/replay episodes, each <=32 steps; no learner/checkpoint."""
    activate_native()
    previous_rng, threads = rng_state(), torch.get_num_threads()
    torch.set_num_threads(1)
    try:
        mapping, events, collision = engineering_scenes()
        cases = []
        def record(result):
            cases.append(result)
            if on_case is not None:
                on_case(cases)
        # Verify the low-level boundary before building on full task wrappers.
        print('G5 core parity: action composition, strict collision and timeout', flush=True)
        record(core_pair(collision, collision=True))
        record(core_pair(mapping))
        for scene, steps, probes in ((mapping, 16, True), (events, 32, False)):
            directory = output/scene['scenario_id']
            directory.mkdir()
            record(full_runtime_pair(scene, steps, directory, probes=probes))
        return cases
    finally:
        restore_rng(previous_rng)
        torch.set_num_threads(threads)

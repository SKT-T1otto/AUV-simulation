"""Read back one physical run and verify timing, installed guidance and applied controls."""

from contextlib import ExitStack
import json
import math
from pathlib import Path

from .contracts import canonical_hash
from .initial_contract import (
    BUDGET,
    FULL_CONDITIONS,
    FULL_SCHEMA,
    HORIZON,
    admission_rule,
    COMMON,
    CONDITIONS,
    FIXTURE,
    SCHEMA,
    CONFIG_SHA,
    ROLES_SHA,
    check_admission,
)
from .scenario import Frame

CONTROL_NS = 20_000_000
TASK_NS = 10 * CONTROL_NS


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def lines(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def write(path, value):
    Path(path).write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8'
    )


def require(test, message):
    if not test:
        raise ValueError(message)


def vector_close(a, b, tolerance=1e-6):
    return len(a) == len(b) == 3 and math.dist(a, b) <= tolerance


def target(installed, agent):
    return installed['guidance']['agent_assignments'][agent]


def signature(installed, agent):
    guide = target(installed, agent)
    return (
        guide['planned_path'],
        guide['final_waypoint'],
        guide['assignment_kind'],
        guide['hold_state'],
        guide['reachable'],
    )


def body_clock(bodies, clock):
    require(
        len(bodies) == 4
        and len({b['stamp_ns'] for b in bodies}) == 1
        and 0 <= clock - bodies[0]['stamp_ns'] <= 2_000_000,
        'Four-body time barrier mismatch',
    )


def validate_installed(data, condition):
    require(
        data['special_pool_calls'] == int(admission_rule(condition) == CONDITIONS[1]),
        'Unexpected admission call count',
    )
    guide = data['guidance']
    require(
        guide['allocation_hash'] == data['allocation_sha256'],
        'Controller/guidance allocation hash mismatch',
    )
    for i, g in enumerate(guide['agent_assignments']):
        nav = g['hold_position'] if g['hold_state'] else g['tracking_waypoint']
        require(vector_close(data['navigation_targets'][i], nav), 'Installed navigation target mismatch')
        if i < 3 and g['assignment_kind'] == 'search' and not g['hold_state'] and g['reachable']:
            t = data['tracker'][str(i)]
            require(
                vector_close(t['current_target'], g['tracking_waypoint']), 'Applied tracker mismatch'
            )


def check_motion_input(sample, installed, motion, params):
    require(
        sample['nav_targets'] == installed['navigation_targets'] and sample['prior_enabled'],
        'Navigation targets or prior at the native motion boundary differ',
    )

    def clamp(x, limit):
        return max(-limit, min(limit, x))

    for i, (p, v, goal) in enumerate(
        zip(sample['positions'], sample['velocities'], sample['nav_targets'])
    ):
        dx, dy, dz = [g - x for g, x in zip(goal, p)]
        dist = max(math.hypot(dx, dy), params['eps'])
        xy = params['v_xy'][i] * min(1.0, dist / params['slow_xy']) / dist
        desired = [dx * xy, dy * xy, clamp(dz / params['slow_z'], 1.0) * params['v_z'][i]]
        gains = [params['kv_xy'], params['kv_xy'], params['kv_z']]
        limits = [params['a_xy'][i], params['a_xy'][i], params['a_z'][i]]
        prior = [
            sample['prior_strength'][i] * clamp(g * (d - old), limit)
            for d, old, g, limit in zip(desired, v, gains, limits)
        ]
        applied = (
            [0.0, 0.0, 0.0]
            if sample['blocked'][i]
            else [clamp(x, limit) for x, limit in zip(prior, limits)]
        )
        require(
            vector_close(prior, sample['prior'][i], 3e-5)
            and sample['residual'][i] == [0, 0, 0]
            and vector_close(sample['requested'][i], applied, 3e-5)
            and sample['requested'][i] == motion['requested_acceleration_native'][i],
            'Reconstructed ZERO navigation prior disagrees with the physical request',
        )


def validate_initial(initial, a):
    require(
        initial['allocation_sha256'] == a['allocation_sha256'],
        'Initial commit differs from native solve',
    )
    for i, c in enumerate(c for c in a['candidates'] if c['selected']):
        g = target(initial, i)
        assignment = initial['allocation']['search_assignments'][i]
        require(
            c['agent'] == i
            and c['id'] == g['assignment_id'] == assignment['candidate_id']
            and g['planned_path'] == c['path'] == assignment['path']
            and g['final_waypoint'] == c['waypoint']
            and tuple(g['tracking_waypoint']) == COMMON[i],
            'Initial search route/first waypoint differs',
        )
    require(initial['allocation']['executor_assignment'] == a['executor'], 'Executor commit mismatch')


def validate_step(
    step, epoch, condition, a, pre, post, previous, transition, controls, request, terminal
):
    require(
        pre['step'] == post['step'] == transition['step'] == step
        and pre['clock_ns'] == epoch + (step - 1) * TASK_NS
        and post['clock_ns'] == epoch + step * TASK_NS,
        'Task clock mismatch',
    )
    require(
        pre['installed']['guidance']['step'] == step - 1
        and post['installed']['guidance']['step'] == (step - 1 if terminal else step)
        and post['installed']['task'] == transition['task'],
        'Guidance or installed task belongs to a different step',
    )
    if previous is not None:
        require(
            pre['installed'] == previous['installed'] and pre['body_states'] == previous['body_states'],
            'Post-step guidance or physical state misaligned with next motion',
        )
    body_clock(pre['body_states'], pre['clock_ns'])
    body_clock(post['body_states'], post['clock_ns'])
    validate_installed(pre['installed'], condition)
    if terminal:
        require(
            pre['installed']['guidance'] == post['installed']['guidance']
            and pre['installed']['allocation_sha256'] == post['installed']['allocation_sha256'],
            'Unexpected terminal planning commit',
        )
    else:
        validate_installed(post['installed'], condition)
    require(
        all(row['installed']['pool_method'] == a['original_pool_method'] for row in (pre, post)),
        'Special selection remained installed after INITIALIZE',
    )
    require(
        post['installed']['guidance'] == transition['guidance'],
        'Transition guidance is not the saved post-step value',
    )
    tr = transition['transition']
    require(
        all(v == 0 for row in tr['actions']['values'] for v in row),
        'Residual action differs from ZERO',
    )
    motion = tr['motion']
    require(request['step'] == step, 'Native request step mismatch')
    check_motion_input(request, pre['installed'], motion, a['navigation_parameters'])
    require(
        motion['task_step'] == step
        and motion['feedback_stamp_ns'] == post['body_states'][0]['stamp_ns'] - epoch,
        'Native feedback step/time mismatch',
    )
    for sub, row in enumerate(controls, 1):
        clock = pre['clock_ns'] + CONTROL_NS * sub
        require(
            row['task_step'] == step
            and row['substep'] == sub
            and row['clock_ns'] == row['requested_clock_ns'] == clock,
            'Control clock mismatch',
        )
        bodies = row['body_states']
        body_clock(bodies, clock)
        require(
            row['state_ns'] == bodies[0]['stamp_ns']
            and row['positions_world'] == [b['position'] for b in bodies],
            'Odometry row mismatch',
        )
        require(len(row['diagnostics']) == 4, 'Missing control channel')
        for i, diag in enumerate(row['diagnostics']):
            require(
                (diag['mode'] == 'finished_agent_hold') == request['blocked'][i],
                'Applied controller mode differs from native blocked mask',
            )
            if diag['mode'] == 'source_acceleration':
                require(
                    vector_close(
                        diag['requested_acceleration_world'],
                        Frame().rotate(motion['requested_acceleration_native'][i]),
                    ),
                    'Motion request differs from acceleration sent to controller',
                )
            else:
                require(diag['mode'] == 'finished_agent_hold', 'Unknown controller mode')
    require(
        bodies == post['body_states']
        and all(
            vector_close(Frame().inverse_position(b['position']), p)
            for b, p in zip(bodies, motion['measured_positions_native'])
        ),
        'Post-step physical feedback mismatch',
    )


def load_case(directory, condition):
    directory = Path(directory)
    s = read(directory / 'summary.json')
    full = condition in FULL_CONDITIONS
    expected = dict(
        FIXTURE,
        initial_candidate_condition=condition,
        native_max_steps=HORIZON,
        engineering_acceptance_passed=True,
        linux_sdf_valid=True,
        stop_zero_command_sent=True,
        contact_events_observed=0,
        checkpoint_loaded=False,
        training_executed=False,
    )
    if full:
        from .far_mission import terminal_steps, mission_evidence
        from .search_outcome import acceptance

        expected.update(mode='search', run_kind='far_search_pair400_gazebo', pair_schema=FULL_SCHEMA)
        require(
            all(s.get(k) == v for k, v in acceptance(s).items()), 'Full G6 acceptance labels disagree'
        )
        n = terminal_steps(s)
        require('diagnostic_budget' not in s, 'External diagnostic stop on a formal-length mission')
    else:
        expected.update(
            diagnostic_schema=SCHEMA,
            diagnostic_budget=BUDGET,
            diagnostic_valid=True,
            termination_reason='diagnostic_budget_completed',
            completed_task_steps=BUDGET,
            control_steps=BUDGET * 10,
            g6_search_acceptance_passed=False,
            g6_capture_acceptance_passed=False,
            task_success=False,
        )
        n = BUDGET
        require(
            not (s['task_outcome']['terminated'] or s['task_outcome']['truncated']),
            'Short diagnostic expected an ongoing mission',
        )
    require(
        all(s.get(k) == v for k, v in expected.items()),
        'Invalid case summary: ' + str(s.get('termination_reason')) + ' ' + str(s.get('error', '')),
    )
    require(
        s['supervisor_cleanup']['verified']
        and s['batch_id'] == directory.parent.name
        and not s.get('stop_error')
        and s['map_sensor_scan_count'] > 0,
        'Unclean stop or missing scans',
    )
    scene = read(directory / 'scenario.json')
    require(
        canonical_hash(scene) == FIXTURE['scenario_sha256']
        and canonical_hash(read(directory / 'vehicle_profile.json'))
        == FIXTURE['vehicle_profile_sha256'],
        'Scene/profile identity mismatch',
    )
    config = read(directory / 'resolved_config.json')
    roles = read(directory / 'role_specs.json')
    require(
        canonical_hash(config) == CONFIG_SHA and canonical_hash(roles) == ROLES_SHA,
        'Native config or roles differ',
    )
    a = read(directory / 'admission.json')
    reference = check_admission(a, condition)
    require(reference['valid'], 'Invalid step-zero public inputs, admission, routes or scores')
    epoch = s['task_epoch_ns']
    require(
        s['startup_world_status']['paused']
        and epoch == s['startup_world_status']['sim_ns']
        and 0 <= epoch - s['warmup_clock_ns'] <= 2_000_000,
        'Invalid paused task epoch',
    )
    case = dict(
        summary=s,
        condition=condition,
        admission=a,
        reference=reference,
        start=[],
        end=[],
        controls=[],
        config_sha=canonical_hash(config),
        roles_sha=canonical_hash(roles),
    )
    keep = n
    replaced = set()
    if full:
        radii = s['native_detect_radii_m']
        require(
            len(radii) == 3
            and all(abs(r - spec['sensor_range'] - 0.1) < 1e-6 for r, spec in zip(radii, roles)),
            'Effective sensor radii disagree with pinned roles',
        )
        case['mission'] = mission_evidence(directory, s, scene, radii)
        case['samples'] = []
        keep = case['mission']['search_end_step']
    names = ('step_start', 'step_end', 'transitions', 'motion_input', 'control')
    previous = None
    frame = Frame()
    with ExitStack() as stack:
        streams = {
            key: stack.enter_context((directory / (key + '.jsonl')).open(encoding='utf-8'))
            for key in names
        }

        def take(key):
            line = streams[key].readline()
            require(bool(line), 'Incomplete record count: ' + key)
            return json.loads(line)

        for step in range(1, n + 1):
            pre, post, transition, request = [take(k) for k in names[:4]]
            controls = [take('control') for _ in range(10)]
            if step == 1:
                validate_initial(pre['installed'], a)
                original_routes = [signature(pre['installed'], i) for i in range(3)]
            validate_step(
                step,
                epoch,
                condition,
                a,
                pre,
                post,
                previous,
                transition,
                controls,
                request,
                terminal=full and step == n,
            )
            if step <= keep:
                if full:
                    # Keep only the fields used by the initial-route comparison. A boat can
                    # remain on its initial route beyond step 32 in a full-length mission.
                    for key, boundary in (('start', pre), ('end', post)):
                        installed = boundary['installed']
                        case[key].append(
                            dict(
                                step=step,
                                body_states=boundary['body_states'],
                                installed=dict(
                                    guidance=installed['guidance'], tracker=installed['tracker']
                                ),
                            )
                        )
                    for row in controls:
                        diagnostics = [
                            {
                                k: d[k]
                                for k in ('requested_acceleration_world', 'governed_acceleration_world')
                                if k in d
                            }
                            for d in row['diagnostics'][:3]
                        ]
                        case['controls'].append(
                            dict(
                                body_states=row['body_states'],
                                diagnostics=diagnostics,
                                positions_world=row['positions_world'],
                            )
                        )
                    replaced.update(
                        i for i in range(3) if signature(post['installed'], i) != original_routes[i]
                    )
                    if len(replaced) == 3:
                        keep = step
                else:
                    case['start'].append(pre)
                    case['end'].append(post)
                    case['controls'].extend(controls)
            if full:
                bodies = [r['body_states'] for r in controls]
                if step == 1:
                    bodies.insert(0, pre['body_states'])
                for states in bodies:
                    case['samples'].append(
                        (
                            states[0]['stamp_ns'] - epoch,
                            [frame.inverse_position(b['position']) for b in states[:3]],
                        )
                    )
            previous = post
        for name, stream in streams.items():
            require(not stream.readline(), 'Extra record: ' + name)
    return case

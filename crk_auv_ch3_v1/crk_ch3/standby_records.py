"""Validate native completion or a guarded physical endpoint, including a partial last step."""

from contextlib import ExitStack
from itertools import combinations
import json
import math
from .contracts import canonical_hash
from .initial_contract import FIXTURE, CONFIG_SHA, ROLES_SHA, check_admission
from .initial_records import (
    read,
    lines,
    require,
    body_clock,
    validate_step,
    validate_initial,
    validate_installed,
    CONTROL_NS,
    TASK_NS,
)
from .far_mission import terminal_steps
from .search_outcome import acceptance
from .scenario import Frame
from .standby_geometry import hulls, hull_gap, world_box, CONDITIONS, SCHEMA

PAIRS = tuple(combinations(range(4), 2))


def contact_evidence(directory, epoch, summary):
    path = directory / 'physical_contacts.jsonl'
    messages = lines(path) if path.exists() else []
    require(len(messages) == summary['contact_events_observed'], 'Contact journal count mismatch')
    observed = []
    for msg in messages:
        ns = msg['received_clock_ns'] - epoch
        require(ns >= 0 and msg['contacts'], 'Invalid contact message')
        for c in msg['contacts']:
            names = [c['collision1'], c['collision2']]
            models = {k.split('::')[0] for k in names}
            pair = sorted(i for i in range(4) if f'auv_{i}' in models)
            source = msg.get('source_ns')
            observed.append(
                dict(
                    ns=ns,
                    source_ns=source - epoch if source is not None else None,
                    pair=pair,
                    collision=names,
                )
            )
    return sorted(observed, key=lambda c: c['ns'])


def sample(bodies, epoch, boxes, installed, counts, diagnostics):
    ns = bodies[0]['stamp_ns'] - epoch
    poses = [world_box(b, h) for b, h in zip(bodies, boxes)]
    gaps = [hull_gap(poses[i], poses[j]) for i, j in PAIRS]
    centers = [math.dist(bodies[i]['position'], bodies[j]['position']) for i, j in PAIRS]
    guide = installed['guidance']['agent_assignments']
    goal = guide[3]['final_waypoint']
    positions = [Frame().inverse_position(b['position']) for b in bodies]
    return dict(
        ns=ns,
        bodies=bodies,
        positions=positions,
        gaps=gaps,
        centers=centers,
        standby=goal,
        distance_to_standby=math.dist(positions[3], goal),
        monitor_online=len(counts) == 4 and all(n > 0 for n in counts),
        targets=[g['tracking_waypoint'] for g in guide],
        channels=[
            (d.get('requested_acceleration_world'), d.get('governed_acceleration_world'))
            for d in diagnostics
        ],
    )


def load_standby(directory, condition, rc):
    s = read(directory / 'summary.json')
    expected = dict(
        FIXTURE,
        pair_schema=SCHEMA,
        initial_candidate_condition=condition,
        native_max_steps=400,
        mode='search',
        checkpoint_loaded=False,
        training_executed=False,
        linux_sdf_valid=True,
        stop_zero_command_sent=True,
        batch_id=directory.parent.name,
    )
    require(
        all(s.get(k) == v for k, v in expected.items()),
        'Unexpected condition, source or task identity',
    )
    require(
        s['supervisor_cleanup']['verified']
        and not s.get('stop_error')
        and not s.get('failure_tags')
        and not (directory / 'error_traceback.txt').exists(),
        'Cleanup or concurrent engineering failure',
    )
    for file, sha in [
        ('scenario', FIXTURE['scenario_sha256']),
        ('vehicle_profile', FIXTURE['vehicle_profile_sha256']),
        ('resolved_config', CONFIG_SHA),
        ('role_specs', ROLES_SHA),
    ]:
        require(
            canonical_hash(read(directory / (file + '.json'))) == sha, 'Input mismatch: ' + file
        )
    epoch = s['task_epoch_ns']
    require(
        s['startup_world_status']['paused']
        and epoch == s['startup_world_status']['sim_ns']
        and 0 <= epoch - s['warmup_clock_ns'] <= 2_000_000,
        'Invalid paused epoch',
    )
    contacts = contact_evidence(directory, epoch, s)
    n = s['completed_task_steps']
    guarded = rc == 1 and s['termination_reason'] == 'physical_contact'
    if guarded:
        require(
            s.get('reason_before_contact_drain') in ('physical_contact', 'task_terminal')
            and contacts
            and all(len(c['pair']) == 2 for c in contacts)
            and 0 <= n <= 400
            and not s['engineering_acceptance_passed']
            and s['supervisor_cleanup']['exit_codes'][-1] == 1,
            'Unverified inter-vehicle contact endpoint',
        )
    else:
        require(
            rc == 0 and not contacts and acceptance(s)['engineering_acceptance_passed'],
            'Unexpected stop',
        )
        terminal_steps(s)
    admission = read(directory / 'admission.json')
    require(check_admission(admission, condition)['valid'], 'Invalid initialization')
    boxes = hulls(directory / 'world.sdf')
    samples, boundaries = [], []
    prev = None
    checks = lines(directory / 'standby_checks.jsonl') if n else []
    require(len(checks) == n, 'Wrong clearance record count')
    with ExitStack() as stack:

        def stream(name):
            path = directory / (name + '.jsonl')
            if not path.exists():
                return iter(())
            return map(json.loads, stack.enter_context(path.open(encoding='utf8')))

        starts, ends, trs, requests, controls = map(
            stream, ('step_start', 'step_end', 'transitions', 'motion_input', 'control')
        )

        def take(rows):
            row = next(rows, None)
            require(row is not None, 'Missing raw record')
            return row

        def initial(pre):
            validate_initial(pre['installed'], admission)
            samples.append(
                sample(
                    pre['body_states'],
                    epoch,
                    boxes,
                    pre['installed'],
                    s['epoch_contact_publishers'],
                    [],
                )
            )

        for step in range(1, n + 1):
            pre, post, tr, req = [take(rows) for rows in (starts, ends, trs, requests)]
            block = [take(controls) for _ in range(10)]
            terminal = not guarded and step == n
            validate_step(
                step, epoch, condition, admission, pre, post, prev, tr, block, req, terminal
            )
            if step == 1:
                initial(pre)
            for row in block:
                samples.append(
                    sample(
                        row['body_states'],
                        epoch,
                        boxes,
                        pre['installed'],
                        row['contact_publishers'],
                        row['diagnostics'],
                    )
                )
            record = checks[step - 1]
            require(
                record['clock_ns'] == post['clock_ns'] and record['terminal'] == terminal,
                'Clearance log clock or phase mismatch',
            )
            if not terminal:
                boundary = record['boundary']
                actual = post['installed']
                require(
                    boundary['step'] == step
                    and boundary['installed_sha256'] == actual['allocation_sha256'],
                    'Decision/installation mismatch',
                )
                if 'raw' in boundary:
                    require(
                        boundary['raw']['search_assignments']
                        == actual['allocation']['search_assignments'],
                        'Search assignments changed during clearance selection',
                    )
                if boundary['reason'] == 'standby_replaced':
                    require(
                        condition == CONDITIONS[1]
                        and boundary['installed_point']
                        == actual['allocation']['executor_assignment']['target_region']
                        and boundary['installed_point']
                        == actual['guidance']['agent_assignments'][3]['final_waypoint'],
                        'Replacement differs from installed executor assignment',
                    )
                boundaries.append(boundary)
            prev = post
        tail = list(controls)
        extra_starts, extra_requests = list(starts), list(requests)
        require(
            not list(ends) and not list(trs) and s['control_steps'] == 10 * n + len(tail),
            'Incomplete raw step',
        )
        require(
            len(extra_starts) <= int(guarded)
            and len(extra_requests) <= len(extra_starts)
            and len(tail) <= 10
            and (not tail or extra_requests),
            'Unexpected trailing records',
        )
        if extra_starts:
            pre = extra_starts[0]
            require(
                pre['step'] == n + 1 and pre['clock_ns'] == epoch + n * TASK_NS,
                'Wrong contact-step clock',
            )
            body_clock(pre['body_states'], pre['clock_ns'])
            validate_installed(pre['installed'], condition)
            if prev:
                require(
                    pre['installed'] == prev['installed']
                    and pre['body_states'] == prev['body_states'],
                    'Contact-step input differs',
                )
            else:
                initial(pre)
            if extra_requests:
                require(
                    extra_requests[0]['step'] == n + 1
                    and extra_requests[0]['nav_targets'] == pre['installed']['navigation_targets'],
                    'Partial motion input mismatch',
                )
            for j, row in enumerate(tail, 1):
                stamp = epoch + n * TASK_NS + j * CONTROL_NS
                require(
                    row['task_step'] == n + 1
                    and row['substep'] == j
                    and row['clock_ns'] == row['requested_clock_ns'] == stamp,
                    'Partial control clock mismatch',
                )
                body_clock(row['body_states'], stamp)
                samples.append(
                    sample(
                        row['body_states'],
                        epoch,
                        boxes,
                        pre['installed'],
                        row['contact_publishers'],
                        row['diagnostics'],
                    )
                )
    require(
        samples
        and all(
            0 < b['ns'] - a['ns'] <= CONTROL_NS + 2_000_000 for a, b in zip(samples, samples[1:])
        ),
        'Missing synchronous poses',
    )
    if guarded:
        require(
            0 <= contacts[0]['ns'] <= (n + 1) * TASK_NS
            and contacts[0]['ns'] >= max(0, (n - 1) * TASK_NS),
            'Contact timestamp inconsistent with final physical step',
        )
    return dict(
        summary=s,
        samples=samples,
        boundaries=boundaries,
        contacts=contacts,
        admission=admission,
        mission=dict(
            terminal=s['termination_reason'],
            outcome=s['task_outcome'],
            found_step=s['found_step'],
            handoff_step=s['handoff_decision_step'],
            success=s['task_success'],
        ),
    )

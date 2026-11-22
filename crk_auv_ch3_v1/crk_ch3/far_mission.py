"""Read native terminal, discovery and handoff evidence. Host-side, after physics has stopped."""

import json
import math

from .capture_evidence import capture_review
from .initial_contract import HORIZON
from .initial_records import read, require

DT = 0.2


def records(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def terminal_steps(summary):
    n = summary['completed_task_steps']
    outcome = summary['task_outcome']
    require(type(n) is int and 1 <= n <= HORIZON, 'Invalid native duration')
    require(
        summary['termination_reason'] == 'task_terminal'
        and (outcome['terminated'] or outcome['truncated'])
        and outcome['terminal_step'] == n,
        'Missing native terminal evidence',
    )
    reason = outcome['termination_reason']
    require(reason in ('success', 'timeout', 'obstacle_collision'), 'Unknown pinned native outcome')
    require(reason != 'timeout' or n == HORIZON, 'Early timeout')
    require(
        summary['task_success'] == outcome['success'] == (reason == 'success'), 'Success labels disagree'
    )
    require(
        summary['control_steps'] == 10 * n
        and math.isclose(summary['actual_simulated_duration_s'], n * DT, abs_tol=1e-8),
        'Native duration or control count mismatch',
    )
    return n


def swept_distance(a0, a1, t0, t1):
    # Same bounded relative segment and EPS as native core.env.target_motion.
    r = [a - t for a, t in zip(a0, t0)]
    v = [(b - a) - (u - t) for a, b, t, u in zip(a0, a1, t0, t1)]
    squared = sum(x * x for x in v)
    tau = 0.0 if squared <= 1e-9 else min(1.0, max(0.0, -sum(x * y for x, y in zip(r, v)) / squared))
    return math.hypot(*(x + tau * y for x, y in zip(r, v))), tau


def native_events(path):
    steps, scans = {}, []
    for sequence, event in enumerate(records(path), 1):
        require(event['sequence'] == sequence, 'Public event sequence gap')
        step = event['physical_step']
        if event['kind'] == 'step':
            require(step not in steps, 'Duplicate detector step')
            steps[step] = event['payload']
        elif event['kind'] == 'scan':
            distances = event['payload']['distances']
            require(
                len(distances) == 4 and all(len(d) == 26 for d in distances),
                'Incomplete neighbor26 scan',
            )
            scans.append(step)
    return steps, scans


def mission_evidence(directory, summary, scene, radii, *, capture_trace=True):
    n = terminal_steps(summary)
    detector, scans = native_events(directory / 'events.jsonl')
    require(set(detector) == set(range(1, n + 1)), 'Incomplete native detector events')
    require(
        scans == [s for s in range(1, n + 1) if detector[s]['target_transition_completed']]
        and summary['map_sensor_scan_count'] == (1 + len(scans)) * 4 * 26,
        'Native ray counter, reset scan or scan sequence mismatch',
    )
    trace = list(records(directory / 'capture_trace.jsonl')) if capture_trace else []
    if capture_trace:
        require(len(trace) == n, 'Incomplete per-step capture trace')
    target = scene['target_initial_position']
    found = finder = received = None
    closest = [None] * 3
    reason = summary['task_outcome']['termination_reason']
    last = 0
    for j, row in enumerate(records(directory / 'transitions.jsonl'), 1):
        last = j
        t, outcome, event = row['task'], row['outcome'], detector[j]
        final = j == n
        require(
            row['step'] == t['step'] == j
            and event['agent_end'] == row['transition']['motion']['task_positions_native']
            and (outcome['terminated'] or outcome['truncated']) == final
            and row['transition']['dones'] == [final] * 4
            and (
                outcome == summary['task_outcome']
                if final
                else outcome['termination_reason'] == 'running'
            ),
            'Native transition/terminal sequence mismatch',
        )
        require(
            event['found_before'] == (found is not None) and event['found_after'] == t['target_found'],
            'Discovery event/transition mismatch',
        )
        end = row['target_truth_audit_only']['position']
        if capture_trace:
            c = trace[j - 1]
            require(
                c['step'] == j
                and c['target_end'] == end
                and math.dist(c['target_start'], target) < 1e-5
                and c['target_found'] == t['target_found']
                and c['finder_id'] == t['finder_id']
                and c['executor_received'] == t['executor_knows_target']
                and c['mission_complete'] == t['mission_complete'],
                'Capture trace and native state disagree',
            )
            target = c['target_start']
        if found is None and event['discovery_check_completed']:
            hits = []
            for i, radius in enumerate(radii):
                dist, tau = swept_distance(event['agent_start'][i], event['agent_end'][i], target, end)
                if closest[i] is None or dist < closest[i]['distance_m']:
                    closest[i] = dict(
                        step=j,
                        fraction=tau,
                        distance_m=dist,
                        radius_m=radius,
                        clearance_m=dist - radius,
                        closest_time_s=(j - 1 + tau) * DT,
                    )
                if dist <= radius:
                    hits.append((dist, i))
            require(bool(hits) == t['target_found'], 'Swept detector disagrees with first discovery')
            if hits:
                found, finder = j, min(hits)[1]
        require(
            t['target_found'] == (found is not None)
            and t['finder_id'] == (-1 if finder is None else finder),
            'Finder or retained discovery mismatch',
        )
        if t['executor_knows_target'] and received is None:
            received = j
        require(t['executor_knows_target'] == (received is not None), 'Nonmonotonic handoff')
        target = end
    require(last == n and found == summary['found_step'], 'Discovery step/record count mismatch')
    require(
        received is None or (found is not None and received == found + 1),
        'Native handoff timing mismatch',
    )
    require(received is not None or found is None or found == n, 'Expected native handoff missing')
    decision = summary['handoff_decision_step']
    require(decision == (received if received != n else None), 'Handoff decision step mismatch')
    verified_capture = capture_review(trace, summary) if capture_trace and reason == 'success' else None
    if verified_capture is not None:
        require(
            summary.get('capture_evidence') == verified_capture,
            'Capture trace and recorded review disagree',
        )
    maps = {}
    for m in records(directory / 'maps.jsonl'):
        mask = m['fields']['unknown_mask']
        shape = mask['shape']
        values = mask['values']
        unknown = sum(sum(sum(z) for z in y) for y in values)
        maps[m['step']] = unknown / math.prod(shape)
    require(1 in maps and n in maps and shape == [10, 10, 8], 'Missing terminal or initial map grid')
    extent = read(directory / 'truth_scene.json')['space_size']
    search_end = found if found is not None else n
    return dict(
        terminal_reason=reason,
        terminal_step=n,
        observed_task_s=n * DT,
        task_success=summary['task_success'],
        search_acceptance=summary['g6_search_acceptance_passed'],
        found_step=found,
        finder_id=finder,
        found_s=None if found is None else found * DT,
        search_end_step=search_end,
        search_duration_s=search_end * DT,
        handoff_received_step=received,
        handoff_decision_step=decision,
        completion_step=t['completion_step'],
        capture_hold_max=summary['capture_hold_max'],
        capture=verified_capture,
        closest_swept_by_searcher=closest,
        native_sensor_ray_count=summary['map_sensor_scan_count'],
        scan_events_total=len(scans),
        search_phase_scan_events=sum(s <= search_end for s in scans),
        final_map_unknown_fraction=maps[n],
        grid_shape_xy=shape[:2],
        space_xy=extent[:2],
    )

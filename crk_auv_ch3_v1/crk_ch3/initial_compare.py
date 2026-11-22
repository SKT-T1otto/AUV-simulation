"""Paired physical effects on each original route. All times are relative to the confirmed epoch."""

import math
from .initial_contract import COMMON, CRITERIA as C, NEXT
from .initial_records import CONTROL_NS, signature, target
from .scenario import Frame


def sub(b, a):
    return [y - x for y, x in zip(b, a)]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def unit(v):
    length = math.hypot(*v)
    return [x / length for x in v] if length else [0.0, 0.0, 0.0]


def angle(a, b):
    cosine = abs(dot(a, b)) / (math.hypot(*a) * math.hypot(*b))
    return math.degrees(2 * math.acos(min(1.0, cosine)))


def world_velocity(body):
    q = body['quaternion']
    x, y, z, w = [v / math.hypot(*q) for v in q]
    vx, vy, vz = body['velocity_body']
    return [
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
    ]


def replacement(case, i):
    original = signature(case['start'][0]['installed'], i)
    return next((r['step'] for r in case['end'] if signature(r['installed'], i) != original), None)


def route_metrics(case, i, limit):
    # Project only within the currently commanded leg, with no credit for a tracker index change.
    start = case['start'][0]
    initial = Frame().inverse_position(start['body_states'][i]['position'])
    original = start['installed']['tracker'][str(i)]
    route = [initial, *original['remaining_path_points']]
    swum = 0.0
    for j in range(limit):
        position = Frame().inverse_position(case['start'][j]['body_states'][i]['position'])
        for row in case['controls'][10 * j : 10 * j + 10]:
            nxt = Frame().inverse_position(row['positions_world'][i])
            swum += math.dist(position, nxt)
            position = nxt
    # Each block is a contiguous tracker index. Never project onto a later crossing or credit threshold skips.
    net = 0.0
    j = 0
    while j < limit:
        k = j + 1
        t = case['start'][j]['installed']['tracker'][str(i)]['next_index']
        while k < limit and case['start'][k]['installed']['tracker'][str(i)]['next_index'] == t:
            k += 1
        idx = min(len(route) - 1, 1 + t - original['next_index'])
        left, right = route[idx - 1 : idx + 1]
        length = math.dist(left, right)
        axis = unit(sub(right, left))

        def coord(body):
            return min(
                length, max(0.0, dot(sub(Frame().inverse_position(body['position']), left), axis))
            )

        net += max(
            0.0, coord(case['end'][k - 1]['body_states'][i]) - coord(case['start'][j]['body_states'][i])
        )
        j = k
    remaining = case['start'][limit - 1]['installed']['tracker'][str(i)]['remaining_path_points']
    endpoint = Frame().inverse_position(case['end'][limit - 1]['body_states'][i]['position'])
    points = [endpoint, *remaining]
    return dict(
        through_step=limit,
        initial_length_m=sum(map(math.dist, route, route[1:])),
        along_route_net_m=net,
        measured_path_m=swum,
        remaining_on_pre_motion_route_m=sum(map(math.dist, points, points[1:])),
    )


def geometry(row_a, row_b):
    return dict(
        position_m=math.dist(row_a['position'], row_b['position']),
        velocity_m_s=math.dist(world_velocity(row_a), world_velocity(row_b)),
        attitude_deg=angle(row_a['quaternion'], row_b['quaternion']),
    )


def pair(a, b):
    shared = []
    for k in ('roles_sha', 'config_sha'):
        if a[k] != b[k]:
            shared.append(k)
    for k in (
        'adapter_manifest_sha256',
        'scenario_sha256',
        'vehicle_profile_sha256',
        'source_files_sha256',
        'batch_id',
    ):
        if a['summary'][k] != b['summary'][k]:
            shared.append(k)
    first = [c['start'][0]['body_states'] for c in (a, b)]
    epochs = [c['summary']['task_epoch_ns'] for c in (a, b)]
    initial = [geometry(x, y) for x, y in zip(*first)]
    if first[0][0]['stamp_ns'] - epochs[0] != first[1][0]['stamp_ns'] - epochs[1]:
        shared.append('initial_odometry_time')
    if any(
        v['position_m'] > C['initial_position_m']
        or v['velocity_m_s'] > C['initial_velocity_m_s']
        or v['attitude_deg'] > C['attitude_deg']
        for v in initial
    ):
        shared.append('initial_body_states')
    agents = [compare_agent(a, b, i, epochs, shared, initial[i]) for i in range(3)]
    return dict(
        criteria=C,
        initial_state_differences=initial,
        shared_failures=shared,
        agents=agents,
        reference_checks=[c['reference'] for c in (a, b)],
        valid=not shared and all(v['classification'] != 'insufficient' for v in agents),
    )


def compare_agent(a, b, i, epochs, shared, initial):
    cases = (a, b)
    change = [replacement(c, i) for c in cases]
    limit = min([len(c['start']) for c in cases] + [v for v in change if v is not None])
    axis = unit(sub(NEXT[1][i], NEXT[0][i]))
    starts = [c['start'][0]['body_states'][i]['position'] for c in cases]
    noise = abs(dot(sub(starts[1], starts[0]), axis))
    failures = list(shared)
    max_common = {**initial, 'requested_m_s2': 0.0, 'governed_m_s2': 0.0}
    first_target = on_branch = None
    events = {}
    series = []
    streak = []

    def save(key, data):
        if key not in events:
            events[key] = data

    for j in range(limit):
        guides = [target(c['start'][j]['installed'], i) for c in cases]
        goal = [g['tracking_waypoint'] for g in guides]
        if goal[0] != goal[1] and first_target is None:
            first_target = dict(step=j + 1, command_time_s=j * 0.2, A=goal[0], B=goal[1])
        if on_branch is None and all(tuple(g) == NEXT[k][i] for k, g in enumerate(goal)):
            on_branch = j + 1
        for k in range(j * 10, (j + 1) * 10):
            rows = [c['controls'][k] for c in cases]
            ds = [r['diagnostics'][i] for r in rows]
            command_s = k * CONTROL_NS / 1e9
            command = dict(step=j + 1, substep=k % 10 + 1, command_time_s=command_s)
            for key, field in [
                ('requested', 'requested_acceleration_world'),
                ('governed', 'governed_acceleration_world'),
            ]:
                if any(field not in d for d in ds):
                    if 'controller_mode' not in failures:
                        failures.append('controller_mode')
                    continue
                va, vb = [d[field] for d in ds]
                gap = math.dist(va, vb)
                event = dict(**command, A=va, B=vb, norm_m_s2=gap)
                if gap > 1e-9:
                    save(key + '_numeric', event)
                if gap > C['acceleration_m_s2']:
                    save(key, event)
                if first_target is None:
                    max_common[key + '_m_s2'] = max(max_common[key + '_m_s2'], gap)
            bodies = [r['body_states'][i] for r in rows]
            ts = [s['stamp_ns'] - epoch for s, epoch in zip(bodies, epochs)]
            if ts[0] != ts[1]:
                if 'odometry_timestamps' not in failures:
                    failures.append('odometry_timestamps')
                streak.clear()
                continue
            delta = sub(bodies[1]['position'], bodies[0]['position'])
            proj = dot(delta, axis)
            sample = dict(
                step=j + 1,
                substep=k % 10 + 1,
                t_s=ts[0] / 1e9,
                delta_xyz_m=delta,
                separation_m=math.hypot(*delta),
                projection_m=proj,
                delta_z_m=delta[2],
            )
            series.append(sample)
            if first_target is None:
                noise = max(noise, abs(proj))
                for key, v in geometry(*bodies).items():
                    max_common[key] = max(max_common[key], v)
            threshold = max(C['minimum_projection_m'], noise)
            if (
                first_target
                and 'requested' in events
                and sample['t_s'] > events['requested']['command_time_s']
            ):
                if sample['separation_m'] > threshold:
                    save('separation', sample)
                if (
                    on_branch
                    and 'governed' in events
                    and sample['t_s'] > events['governed']['command_time_s']
                    and proj > threshold
                ):
                    streak.append(sample)
                    if len(streak) == C['consecutive_samples']:
                        save('directional', dict(first=streak[0], confirmed=sample))
                else:
                    streak.clear()
    limits = dict(
        position_m=C['common_position_m'],
        velocity_m_s=C['common_velocity_m_s'],
        attitude_deg=C['attitude_deg'],
        requested_m_s2=C['acceleration_m_s2'],
        governed_m_s2=C['acceleration_m_s2'],
    )
    failures.extend('common_' + k for k, v in max_common.items() if v > limits[k])
    # One vehicle may leave the common point a step earlier. Its first distinct target must still be its registered branch.
    if first_target and any(
        tuple(first_target[key]) not in (COMMON[i], NEXT[k][i]) for k, key in enumerate(('A', 'B'))
    ):
        failures.append('unexpected_first_waypoint')
    label = 'allocation_only'
    if 'requested' in events:
        label = 'request_unresolved'
    if 'directional' in events:
        label = 'directional_displacement'
    if failures:
        label = 'insufficient'
    routes = {}
    for c, swap in zip(cases, change):
        metrics = route_metrics(c, i, swap or len(c['start']))
        routes[c['condition']] = dict(
            common_window=route_metrics(c, i, limit),
            own_initial_route=metrics,
            first_replacement_after_step=swap,
            remaining_at_replacement_m=metrics['remaining_on_pre_motion_route_m'] if swap else None,
        )
        if swap:
            routes[c['condition']]['replacement'] = {
                name: target(c[name][swap - 1]['installed'], i) for name in ('start', 'end')
            }
            routes[c['condition']]['native_decision'] = c['end'][swap - 1]['installed']['guidance'].get(
                'decision_reason'
            )
    return dict(
        agent='auv_' + str(i),
        classification=label,
        failures=failures,
        window_end_step=limit,
        first_different_pre_motion_target=first_target,
        both_registered_branches_step=on_branch,
        common_max=max_common,
        projection_threshold_m=max(C['minimum_projection_m'], noise),
        axis_B_minus_A=axis,
        events=events,
        series=series,
        routes=routes,
    )

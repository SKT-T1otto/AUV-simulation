"""Ubuntu host analysis of a single explicit pair. All times use each case's paused epoch."""

import csv
import math
from .initial_compare import angle, world_velocity
from .initial_contract import CRITERIA as C
from .initial_records import write
from .standby_geometry import CONDITIONS, REFERENCE_NS
from .standby_records import PAIRS


def window(samples, end):
    rows = [r for r in samples if r['ns'] <= end]
    minima = {}
    for k, (i, j) in enumerate(PAIRS):
        row = min(rows, key=lambda r: r['gaps'][k])
        minima[f'{i}-{j}'] = dict(
            ns=row['ns'],
            gap_lower_bound_m=row['gaps'][k],
            center_m_at_minimum=row['centers'][k],
            minimum_center_m=min(r['centers'][k] for r in rows),
            executor_distance_to_standby_m=row['distance_to_standby'],
            executor_within_tracker_threshold=row['distance_to_standby'] < 0.75,
        )
    key = min(minima, key=lambda name: minima[name]['gap_lower_bound_m'])
    return dict(
        first_ns=rows[0]['ns'],
        last_ns=rows[-1]['ns'],
        count=len(rows),
        pairs=minima,
        team_minimum_pair=key,
        team_minimum=minima[key],
    )


def crossing(case, pair=None):
    samples, contacts = case['samples'], case['contacts']
    later = next((r for r in samples if r['ns'] > REFERENCE_NS), None)
    observed = next((c for c in contacts if pair is None or c['pair'] == pair), None)
    # Contact messages use receive-clock timestamps, not interpolated poses.
    if observed and (not later or observed['ns'] <= later['ns']):
        status = 'contact_observed'
    elif later is None:
        status = 'crossing_not_observed'
    elif all(r['monitor_online'] for r in samples if r['ns'] <= later['ns']):
        status = 'crossed_no_contact_observed'
    else:
        status = 'insufficient_monitor_coverage'
    strict = status
    if status == 'crossed_no_contact_observed':
        summary = case.get('summary', {})
        epoch = summary.get('task_epoch_ns', 0)
        spans = summary.get('sensor_source_ranges_ns', [[] for _ in range(4)])
        ids = pair if pair is not None else range(4)
        covered = all(
            any(lo <= epoch and hi >= epoch + later['ns'] for lo, hi in spans[i]) for i in ids
        )
        strict = 'crossed_without_contact' if covered else 'insufficient_continuous_sensor_evidence'
        for c in contacts:
            if (
                (pair is None or c['pair'] == pair)
                and c.get('source_ns') is not None
                and 0 <= c['source_ns'] <= later['ns']
            ):
                strict = 'contact_observed'
                break
    return dict(
        status=status,
        strict_verdict=strict,
        first_later_ns=later['ns'] if later else None,
        first_contact=observed,
        coverage_basis='timestamped 500 Hz hull sensor ticks from epoch through first later pose',
    )


def prefix(a, b, end):
    failures = set()
    maximum = dict(position_m=0.0, velocity_m_s=0.0, attitude_deg=0.0, acceleration_m_s2=0.0)
    count = 0
    for x, y in zip(a, b):
        if max(x['ns'], y['ns']) > end:
            break
        count += 1
        if x['ns'] != y['ns']:
            failures.add('relative_odometry_clock')
        for i, (u, v) in enumerate(zip(x['bodies'], y['bodies'])):
            values = [
                math.dist(u['position'], v['position']),
                math.dist(world_velocity(u), world_velocity(v)),
                angle(u['quaternion'], v['quaternion']),
            ]
            limits = [
                C['initial_position_m'] if count == 1 else C['common_position_m'],
                C['initial_velocity_m_s'] if count == 1 else C['common_velocity_m_s'],
                C['attitude_deg'],
            ]
            for key, value, limit in zip(('position_m','velocity_m_s','attitude_deg'), values, limits):
                maximum[key] = max(maximum[key], value)
                if value > limit:
                    failures.add(f'auv_{i}/{key}')
            if x['targets'][i] != y['targets'][i]:
                failures.add(f'auv_{i}/tracking')
        for left, right in zip(x['channels'], y['channels']):
            for u, v in zip(left, right):
                if u is None or v is None:
                    if u != v:
                        failures.add('controller_mode')
                else:
                    diff = math.dist(u, v)
                    maximum['acceleration_m_s2'] = max(maximum['acceleration_m_s2'], diff)
                    if diff > C['acceleration_m_s2']:
                        failures.add('control_acceleration')
    return dict(samples=count, through_ns=end, maxima=maximum, failures=sorted(failures))


def compare(a, b):
    end = min(a['samples'][-1]['ns'], b['samples'][-1]['ns'])
    changed = next((r for r in b['boundaries'] if r['reason'] == 'standby_replaced'), None)
    comparison = prefix(
        a['samples'], b['samples'], min(end, changed['step'] * 200_000_000 if changed else end)
    )
    # Compare the two native decisions at the first treatment boundary as well.
    # Include discrete routes, not floating objective hashes, in the equivalence test.
    cutoff = comparison['through_ns']
    def native_decisions(case):
        rows = []
        for record in case['boundaries']:
            if record['step'] * 200_000_000 > cutoff:
                break
            raw = record.get('raw')
            signature = None
            if raw is not None:
                signature = (
                    [(x['candidate_id'], x['waypoint'], x['path']) for x in raw['search_assignments']],
                    raw['executor_assignment']['target_region'], raw['executor_assignment']['path'],
                )
            rows.append((record['step'], record['decision'], signature))
        return rows
    if native_decisions(a) != native_decisions(b):
        comparison['failures'].append('native_decisions_or_installed_routes_before_treatment')
    runs = {}
    for condition, case in zip(CONDITIONS, (a, b)):
        rows = case['samples']
        last = rows[-1]['ns']
        runs[condition] = dict(
            mission=case['mission'],
            init_hash=case['admission']['allocation_sha256'],
            common=window(rows, end),
            through_reference=window(rows, min(last, REFERENCE_NS)),
            full=window(rows, last),
            main_crossing=crossing(case, [1, 3]),
            team_crossing=crossing(case),
            first_contact=next(iter(case['contacts']), None),
            min_sample_interval_ns=min(
                (y['ns'] - x['ns'] for x, y in zip(rows, rows[1:])), default=None
            ),
            max_sample_interval_ns=max(
                (y['ns'] - x['ns'] for x, y in zip(rows, rows[1:])), default=None
            ),
            disconnected_slots=sum(not r['monitor_online'] for r in rows),
        )
    boundary = next((r for r in a['boundaries'] if r['step'] == 20), {})
    raw = boundary.get('raw', {}).get('executor_assignment', {})
    conflict = next((c for c in boundary.get('candidates', []) if c['point'] == [3, 11, 2.5]), {})
    first = runs[CONDITIONS[0]]['main_crossing']['first_contact']
    history = dict(
        step20_endpoint=raw.get('target_region'),
        step20_conflict_reproduced=raw.get('target_region') == [3, 11, 2.5]
        and conflict.get('center_distance_and_segment', {}).get('1', [None])[0] == 0,
        historical_contact_ns=REFERENCE_NS,
        contact_pair_reproduced=first is not None,
        contact_time_delta_ns=first['ns'] - REFERENCE_NS if first else None,
    )
    if runs[CONDITIONS[0]]['init_hash'] != runs[CONDITIONS[1]]['init_hash']:
        comparison['failures'].append('initial_allocation_hash')
    first_rejection = next(
        (
            r
            for r in b['boundaries']
            if r['reason'] in ('standby_replaced', 'clearance_no_feasible_candidate')
        ),
        None,
    )
    delta = (
        runs[CONDITIONS[1]]['common']['pairs']['1-3']['gap_lower_bound_m']
        - runs[CONDITIONS[0]]['common']['pairs']['1-3']['gap_lower_bound_m']
    )
    return dict(
        valid=not comparison['failures'],
        prefix=comparison,
        runs=runs,
        history=history,
        common_pair13_lower_bound_change_m=delta,
        first_disqualified_original=first_rejection
        and dict(step=first_rejection['step'], reason=first_rejection['reason']),
        first_replacement=changed
        and dict(
            step=changed['step'],
            point=changed['installed_point'],
            observed_in_next_motion=any(
                r['ns'] > changed['step'] * 200_000_000
                and r['standby'] == changed['installed_point']
                for r in b['samples']
                if r['ns'] <= (changed['step'] + 1) * 200_000_000
            ),
        ),
        traces=[a['samples'], b['samples']],
    )


def figure(traces):
    out = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="820" height="435" viewBox="0 0 820 435">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]
    for k, rows in enumerate(traces):
        x0 = 40 + 400 * k
        out.append(f'<text x="{x0}" y="24">{"AB"[k]} · measured XY (m), standby ×</text>')

        def xy(point):
            return x0 + 16 * point[0], 380 - 16 * point[1]

        for m in range(0, 21, 5):
            x, y = xy((m, m))
            out.extend(
                [
                    f'<path d="M{x} 60 V380 M{x0} {y} h320" stroke="#ddd"/>',
                    f'<text x="{x}" y="400">{m}</text><text x="{x0-18}" y="{y}">{m}</text>',
                ]
            )
        for i, color in enumerate(('#4a7e71', '#2369a2', '#b38435', '#a33f50')):
            points = ' '.join('%.2f,%.2f' % xy(r['positions'][i]) for r in rows)
            out.append(
                f'<polyline points="{points}" stroke="{color}" fill="none" stroke-width="2"/>'
            )
            out.append(f'<text x="{x0+i*80}" y="423" fill="{color}">auv_{i}</text>')
        for goal in sorted({tuple(r['standby']) for r in rows}):
            x, y = xy(goal)
            out.append(f'<path d="M{x-4} {y-4} l8 8 m-8 0 l8 -8" stroke="black"/>')
    return '\n'.join(out + ['</svg>'])


def report(batch, meta, result):
    data = dict(
        result or dict(valid=False),
        batch_status=meta['status'],
        git_sha=meta['git_sha'],
        reason=meta['reason'],
    )
    traces = data.pop('traces', [])
    write(batch / 'comparison.json', data)
    write(batch / 'batch.json', meta)
    with (batch / 'hull_gaps.csv').open('w', encoding='utf8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                'condition',
                'relative_odom_ns',
                'pair',
                'sat_lower_bound_m',
                'center_m',
                'standby_distance_m',
            ]
        )
        for cond, rows in zip(CONDITIONS, traces):
            for row in rows:
                for k, (i, j) in enumerate(PAIRS):
                    writer.writerow(
                        [
                            cond,
                            row['ns'],
                            f'{i}-{j}',
                            row['gaps'][k],
                            row['centers'][k],
                            row['distance_to_standby'],
                        ]
                    )
    (batch / 'tracks.svg').write_text(figure(traces), encoding='utf8')
    md = [
        '# G6 待命点净空对照',
        '',
        meta['status'] + ' ' + meta['reason'],
        '',
        '20 ms 同步里程计。船壳指标为 SDF 有向盒分离轴间隙下界。越界时间采用相对任务 epoch 的实测时钟。',
        '接触订阅连接按每个控制周期核验。事件传感器没有空消息心跳，越界结果描述在线监测期间收到的消息。',
        '',
    ]
    for cond, run in data.get('runs', {}).items():
        md.extend(
            [
                '## ' + cond,
                '',
                f"主艇对越界结果 {run['main_crossing']}。全队 {run['team_crossing']}。",
                '',
                '|窗口|主艇对最小下界 m|时刻 s|执行艇在待命阈值内|全队最小艇对|全队最小下界 m|',
                '|---|---:|---:|---|---|---:|',
            ]
        )
        for name in ('common', 'through_reference', 'full'):
            win = run[name]
            main = win['pairs']['1-3']
            md.append(
                f"|{name}|{main['gap_lower_bound_m']:.4f}|{main['ns']/1e9:.3f}|{main['executor_within_tracker_threshold']}|{win['team_minimum_pair']}|{win['team_minimum']['gap_lower_bound_m']:.4f}|"
            )
        md.extend(
            [
                '',
                f"原生结果 {run['mission']['outcome'].get('termination_reason')}，停止原因 {run['mission']['terminal']}。",
                '',
            ]
        )
    if result:
        md.extend(
            [
                f"共同窗口内主艇对最小船壳间隙下界 B−A 为 {data['common_pair13_lower_bound_change_m']:.4f} m。",
                f"原待命点首次失格 {data['first_disqualified_original']}。",
                f"首次实际替换 {data['first_replacement']}。",
                f"与历史 B 核对 {data['history']}。",
                f"共同前缀 {data['prefix']}。",
                '',
                '![艇位与待命点](tracks.svg)',
            ]
        )
    (batch / 'REPORT.md').write_text('\n'.join(md) + '\n', encoding='utf8')


if __name__ == '__main__':
    from .initial_batch import main

    main(full=True, standby=True)

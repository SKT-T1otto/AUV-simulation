"""Compare two native-duration far searches. This module is also the Ubuntu host entry."""

import csv
from itertools import zip_longest

from .initial_compare import pair, dot, sub, unit
from .initial_contract import FULL_CONDITIONS, FULL_SCHEMA, NEXT
from .initial_records import write
from .far_metrics import FIELDS, geometry, common_samples, spatial_window

PRIOR_PROJECTIONS = (0.4563948803975234, 0.5614611090997114, 0.46777722364467067)
AXES = [unit(sub(b, a)) for a, b in zip(*NEXT)]
COORDS = tuple(f'auv{i}_{axis}_m' for i in range(3) for axis in 'xyz')
CSV_FIELDS = (
    'condition',
    'slot',
    'odom_ns',
    't_s',
    'search_phase',
    'common_search',
    'stamp_paired',
    *COORDS,
    *FIELDS,
    'B_minus_A_auv2_z_m',
)


def detection(a, b):
    ta, tb = a['found_s'], b['found_s']
    if ta is None or tb is None:
        label = (
            'neither_found'
            if ta is None and tb is None
            else ('only_B_found' if ta is None else 'only_A_found')
        )
        return dict(
            result=label,
            B_minus_A_s=None,
            A_observed_s=a['observed_task_s'],
            B_observed_s=b['observed_task_s'],
        )
    return dict(
        result='both_found',
        B_minus_A_s=tb - ta,
        A_observed_s=a['observed_task_s'],
        B_observed_s=b['observed_task_s'],
    )


def prefix_reference(a, b, prefix, common_steps):
    cross = []
    for i, agent in enumerate(prefix['agents']):
        first = agent['first_different_pre_motion_target']
        row = dict(
            agent=agent['agent'],
            prior_first_step=14,
            observed_first=first,
            first_step_delta=None if first is None else first['step'] - 14,
            prior_step20_projection_m=PRIOR_PROJECTIONS[i],
        )
        for field in ('requested', 'governed'):
            event = agent['events'].get(field)
            row[field] = event
            row[field + '_time_minus_prior_s'] = (
                None if event is None else event['command_time_s'] - 2.60
            )
        row.update(step20_projection_m=None, projection_minus_prior_m=None, step20_delta_z_m=None)
        if common_steps >= 20:
            sa, sb = a['samples'][200], b['samples'][200]
            if sa[0] == sb[0]:
                delta = sub(sb[1][i], sa[1][i])
                projection = dot(delta, AXES[i])
                row.update(
                    step20_odom_s=sa[0] / 1e9,
                    step20_projection_m=projection,
                    projection_minus_prior_m=projection - PRIOR_PROJECTIONS[i],
                    step20_delta_z_m=delta[2],
                )
        cross.append(row)
    return cross


def timeseries(cases, steps):
    rows = []
    for slot, samples in enumerate(zip_longest(*(c['samples'] for c in cases))):
        a, b = samples
        paired = a is not None and b is not None and a[0] == b[0]
        dz = b[1][2][2] - a[1][2][2] if paired else None
        for case, sample in zip(cases, samples):
            if sample is None:
                continue
            ns, points = sample
            row = dict(
                condition=case['condition'],
                slot=slot,
                odom_ns=ns,
                t_s=ns / 1e9,
                search_phase=slot <= 10 * case['mission']['search_end_step'],
                common_search=0 <= ns and slot <= 10 * steps,
                stamp_paired=paired,
                B_minus_A_auv2_z_m=dz,
            )
            row.update(zip(COORDS, [v for point in points for v in point]))
            row.update(geometry(points))
            rows.append(row)
    return rows


def compare_full(a, b):
    cases = (a, b)
    missions = [c['mission'] for c in cases]
    steps = min(m['search_end_step'] for m in missions)
    views, alignment = common_samples(a['samples'], b['samples'], steps)
    prefix = pair(a, b)
    failures = [
        *prefix['shared_failures'],
        *[ag['agent'] + '/' + tag for ag in prefix['agents'] for tag in ag['failures']],
    ]
    if alignment['unmatched']:
        failures.append('common_search_odometry_timestamps')
    common, own = {}, {}
    for c, samples in zip(cases, views):
        m = c['mission']
        shape = m['grid_shape_xy']
        extent = m['space_xy']
        common[c['condition']] = spatial_window(samples, steps, shape, extent)
        own[c['condition']] = spatial_window(c['samples'], m['search_end_step'], shape, extent)
    av, bv = [common[c['condition']]['mean_m']['Dxy'] for c in cases]
    diff = None if av is None or bv is None else bv - av
    return dict(
        schema=FULL_SCHEMA,
        valid=not failures,
        comparability_failures=failures,
        common_end_step=steps,
        common_end_s=steps * 0.2,
        alignment=alignment,
        missions=dict(zip(FULL_CONDITIONS, missions)),
        common=common,
        own_search=own,
        Dxy_B_minus_A_m=diff,
        detection=detection(*missions),
        prefix=prefix,
        prior32_reference=prefix_reference(a, b, prefix, steps),
        time_rows=timeseries(cases, steps),
    )


def number(v):
    return '—' if v is None else f'{v:.4f}'


def report_full(batch, meta, result):
    from .far_plots import plot_search

    data = (
        dict(result)
        if result is not None
        else dict(schema=FULL_SCHEMA, valid=False, comparability_failures=[meta['reason']])
    )
    data.update(batch_status=meta['status'], git_sha=meta['git_sha'], reason=meta['reason'])
    rows = data.pop('time_rows', [])
    with (batch / 'timeseries.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    write(batch / 'comparison.json', data)
    write(batch / 'batch.json', meta)
    write(
        batch / 'agent_metrics.json',
        {
            window: {c: s['per_agent'] for c, s in data.get(window, {}).items()}
            for window in ('common', 'own_search')
        },
    )
    plot_search(batch, rows, data)
    md = [
        '# G6 远目标步零候选完整任务对照',
        '',
        f"批次 `{batch.name}`，状态 `{meta['status']}`。运行环境 {meta['environment']}。Git `{meta['git_sha']}`。",
        '',
    ]
    if data.get('common'):
        effect = number(data['Dxy_B_minus_A_m'])
        md.append(
            f"共同搜索窗口截至第 {data['common_end_step']} 步（{data['common_end_s']:.2f} s）。"
            f"组内平均水平间距的时间均值 B−A 为 {effect} m。"
        )
        d = data['detection']
        label = {
            'neither_found': '两组均未发现',
            'only_A_found': '仅 A 发现',
            'only_B_found': '仅 B 发现',
            'both_found': '两组均发现',
        }[d['result']]
        finding = f"首次发现结果为{label}。"
        if d['B_minus_A_s'] is not None:
            finding += f"首次发现时间 B−A 为 {number(d['B_minus_A_s'])} s。"
        md += [finding, '']
    if not data['valid']:
        md += ['**对照不充分。** ' + meta['reason'], ', '.join(data['comparability_failures']), '']
    md += [
        '|条件|运行或跳过原因|原生终止|实测任务秒数|首次发现 步 / 秒 / 艇|收到交接步|捕获完成步|',
        '|---|---|---|---:|---|---:|---:|',
    ]
    for c in FULL_CONDITIONS:
        e = meta['cases'].get(c, {})
        m = data.get('missions', {}).get(c, e.get('mission', {}))
        found = '未发现' if m and m['found_step'] is None else '—'
        if m.get('finder_id') is not None:
            found = f"{m['found_step']} / {m['found_s']:.2f} / auv_{m['finder_id']}"
        md.append(
            f"|{c}|{e.get('status','跳过')}|{m.get('terminal_reason',e.get('termination_reason','—'))}|"
            f"{number(m.get('observed_task_s'))}|{found}|{m.get('handoff_received_step') or '—'}|{m.get('completion_step') or '—'}|"
        )
    if rows:
        md += [
            '',
            '|窗口|条件|搜索时长 s|Dxy m|三维均距 m|右半区艇秒 / 占比|轨迹格并集 / 重叠格|',
            '|---|---|---:|---:|---:|---|---|',
        ]
        for group, title in [('common', '共同'), ('own_search', '本组')]:
            for c, s in data[group].items():
                md.append(
                    f"|{title}|{c}|{s['nominal_s']:.2f}|{number(s['mean_m']['Dxy'])}|{number(s['mean_m']['D3d'])}|"
                    f"{s['right_boat_s']:.3f} / {number(s['right_fraction'])}|{s['union_cells']} / {s['overlapping_cells']}|"
                )
        md += [
            '',
            '|条件|三对水平间距均值 01 / 02 / 12（m）|实测最小水平间距 m|三对三维间距均值 01 / 02 / 12（m）|',
            '|---|---|---:|---|',
        ]
        for c, s in data['common'].items():
            av = s['mean_m']
            xy = ' / '.join(number(av['d' + p + '_xy']) for p in ('01', '02', '12'))
            xyz = ' / '.join(number(av['d' + p + '_3d']) for p in ('01', '02', '12'))
            md.append(f"|{c}|{xy}|{number(s['minimum_observed_pair_xy_m'])}|{xyz}|")
        md += [
            '',
            '|条件|搜索艇|最大 x（m）|首次实测右侧（s）|右侧艇秒|经过格数|自身搜索期距探测半径最近余量（m）|',
            '|---|---|---:|---:|---:|---:|---:|',
        ]
        for c, s in data['common'].items():
            mission = data['missions'][c]
            for p, nearest in zip(s['per_agent'], mission['closest_swept_by_searcher']):
                margin = nearest['clearance_m'] if nearest else None
                md.append(
                    f"|{c}|{p['agent']}|{number(p['max_x_m'])}|{number(p['first_observed_right_s'])}|"
                    f"{number(p['right_boat_s'])}|{p['visited_xy_cells']}|{number(margin)}|"
                )
        for c, m in data['missions'].items():
            md += [
                '',
                f"{c} 原生扫描射线计数为 {m['native_sensor_ray_count']}，扫描事件 {m['scan_events_total']} 条，"
                f"终止占据图未知体素占比为 {m['final_map_unknown_fraction']:.4f}。",
            ]
            if m['capture'] is not None:
                check = m['capture']
                md.append(
                    f"实测捕获核验为 {check['verified']}，末段连续保持 {check['final_consecutive_steps']} 步。"
                    f"核验标签 {check['failure_tags']}。"
                )
        md += [
            '',
            '|艇|首个不同运动前跟踪点所在步|较上轮步差|第 20 步有向差 m|较上轮差 m|',
            '|---|---:|---:|---:|---:|',
        ]
        for r in data['prior32_reference']:
            first = r['observed_first'] or {}
            md.append(
                f"|{r['agent']}|{first.get('step','—')}|{r['first_step_delta'] if first else '—'}|"
                f"{number(r['step20_projection_m'])}|{number(r['projection_minus_prior_m'])}|"
            )
        md += [
            '',
            f"同步采样槽 {data['alignment']['matched_slots']} / {data['alignment']['expected_slots']}。"
            "三对艇的水平和三维间距、auv_2 有符号组间高度差见 timeseries.csv。逐艇最东位置、首次右侧采样和经过格数见 agent_metrics.json。",
            '',
            '![三艇实测轨迹](tracks.svg)',
            '',
            '![组内平均水平间距](dispersion.svg)',
            '',
            '![auv_2 实测高度](auv2_z.svg)',
            '',
            '![共同搜索阶段 auv_2 有符号高度差](auv2_delta_z.svg)',
        ]
    md += [
        '',
        '格网为任务坐标 10×10 个水平格，每格 2×2 m。内部边界归坐标较大一侧，外缘归最末格。'
        '逐对相邻 20 ms 里程计位置连线计经过格，共同窗口仅使用两组同步的相邻样本。'
        '首次进入按首个实测 x≥10 m 的时刻记录。轨迹格、原生扫描射线计数和占据图未知比例分别报告。',
        '',
        '单一场景和种子的配对仅支持本次运行的时间与几何比较。',
    ]
    (batch / 'REPORT.md').write_text('\n'.join(md) + '\n', encoding='utf-8')


if __name__ == '__main__':
    from .initial_batch import main

    main(full=True)

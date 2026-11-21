"""Batch reports and verified ZIP, using only the Ubuntu host standard library."""

import hashlib
import html
from pathlib import Path
import zipfile
from .initial_contract import CONDITIONS, CRITERIA
from .initial_records import write

LABELS = dict(
    allocation_only='只改变了分配',
    request_unresolved='请求已分叉，位移未分辨',
    directional_displacement='已观察到预期方向的位移',
    insufficient='对照不充分',
)


def file_hash(path):
    with Path(path).open('rb') as f:
        return stream_hash(f)


def stream_hash(f):
    h = hashlib.sha256()
    for block in iter(lambda: f.read(1024 * 1024), b''):
        h.update(block)
    return h.hexdigest()


def figure(agents):
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="850" height="570" viewBox="0 0 850 570">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<g font-family="sans-serif" font-size="14" fill="#183348">',
        '<text x="60" y="30" font-size="20">Same-time displacement along the expected B minus A direction</text>',
    ]
    if not agents:
        parts.append('<text x="60" y="100">No validated pair available.</text>')
    for i, a in enumerate(agents):
        y = 125 + i * 163
        vals = a['series']
        threshold = a['projection_threshold_m']
        scale = 55 / max([threshold * 1.2, 0.015] + [abs(v['projection_m']) for v in vals])
        right = 80 + 105 * (a['window_end_step'] * 0.2)

        def line(y1, dash, color):
            return f'<path d="M80 {y1} H{right:.1f}" fill="none" stroke="{color}" stroke-dasharray="{dash}"/>'

        parts.extend(
            [
                f'<text x="25" y="{y-57}">{html.escape(a["agent"])} · threshold {threshold:.3f} m</text>',
                line(y, '1,0', '#68818b'),
                line(y - threshold * scale, '6,4', '#ad6008'),
            ]
        )
        points = ' '.join(f'{80+105*v["t_s"]:.1f},{y-scale*v["projection_m"]:.2f}' for v in vals)
        parts.append(f'<polyline fill="none" stroke="#007e80" stroke-width="2" points="{points}"/>')
        parts.append(
            f'<text x="80" y="{y+65}">0 s</text><text x="{right-25:.0f}" y="{y+65}">{a["window_end_step"]*.2:.1f} s</text>'
        )
    return '\n'.join(parts + ['</g></svg>'])


def reports(batch, meta, result):
    if result is None:
        result = dict(valid=False, criteria=CRITERIA, agents=[], shared_failures=[meta['reason']])
    result.update(batch_status=meta['status'], reason=meta['reason'], git_sha=meta['git_sha'])
    write(batch / 'comparison.json', result)
    agents = result['agents']
    write(batch / 'agent_metrics.json', [{k: v for k, v in a.items() if k != 'series'} for a in agents])
    (batch / 'projection.svg').write_text(figure(agents), encoding='utf-8')
    rows = [
        '# G6 初始候选 32 步成对验证',
        '',
        f"批次 `{batch.name}`，状态 `{meta['status']}`。{meta['reason']}",
        '',
        f"Git `{meta['git_sha']}`。运行环境为 {meta['environment']}。",
        '',
        '|条件|执行状态|终止原因|初始化核验|',
        '|---|---|---|---|',
    ]
    for c in CONDITIONS:
        entry = meta['cases'].get(c, {})
        rows.append(
            f"|{c}|{entry.get('status','skipped')}|{entry.get('termination_reason','—')}|{entry.get('reference_valid','—')}|"
        )
    for c, entry in meta['cases'].items():
        if 'reference' in entry:
            ref = entry['reference']
            goals = ', '.join(str(v) for v in ref['selected_endpoints']['observed'])
            scores = entry['scores']
            rows.extend(
                [
                    '',
                    f"{c} 的三艇终点依次为 {goals}，执行艇待命点为 {entry['standby']}。"
                    f"搜索分数 {scores['objective']:.12g}，联合分数 {scores['joint_score']:.12g}。"
                    f"与离线参考的原始规范哈希相等为 {ref['canonical_hash_equal']}。",
                ]
            )
    if agents:
        rows += [
            '',
            '|艇|比较到步末|结论|首次不同跟踪点（步）|首次可分辨请求（秒）|末次有向位移（米）|',
            '|---|---:|---|---:|---:|---:|',
        ]
        for a in agents:
            v = a['series'][-1]['projection_m'] if a['series'] else None
            rows.append(
                f"|{a['agent']}|{a['window_end_step']}|{LABELS[a['classification']]}|"
                f"{(a['first_different_pre_motion_target'] or {}).get('step','—')}|"
                f"{a['events'].get('requested',{}).get('command_time_s','—')}|{v if v is None else round(v,5)}|"
            )
        rows += [
            '',
            '|艇|首次调节后分叉（秒）|首次艇位分离（秒 / 米）|有向位移首次达标 / 确认（秒）|窗口末 B−A 垂直位移（米）|',
            '|---|---:|---|---|---:|',
        ]
        for a in agents:
            events = a['events']
            sep = events.get('separation')
            effect = events.get('directional')
            separation = f"{sep['t_s']:.3f} / {sep['separation_m']:.4f}" if sep else '未分辨'
            direction = (
                f"{effect['first']['t_s']:.3f} / {effect['confirmed']['t_s']:.3f}"
                if effect
                else '未分辨'
            )
            z = f"{a['series'][-1]['delta_z_m']:+.4f}" if a['series'] else '—'
            rows.append(
                f"|{a['agent']}|{events.get('governed',{}).get('command_time_s','—')}|{separation}|{direction}|{z}|"
            )
        rows += [
            '',
            '|艇|条件|比较窗沿路净进展（米）|初始路线全段净进展（米）|首次自然换路步末|换路余程（米）|',
            '|---|---|---:|---:|---:|---:|',
        ]
        for a in agents:
            for condition, r in a['routes'].items():
                remaining = r['remaining_at_replacement_m']
                rows.append(
                    f"|{a['agent']}|{condition}|{r['common_window']['along_route_net_m']:.4f}|"
                    f"{r['own_initial_route']['along_route_net_m']:.4f}|{r['first_replacement_after_step'] or '窗内未换路'}|"
                    f"{round(remaining,4) if remaining is not None else '—'}|"
                )
        rows += [
            '',
            '![各艇同物理时刻 B−A 艇位差的有向投影](projection.svg)',
            '',
            '正向阈值取共同制导期有向差的绝对峰值与 0.01 m 中较大者。两艇组进入各自登记的第二跟踪点后，连续三个同时间戳的样本超过阈值才判定有向位移。各艇的比较截至任一组首条路线被自然替换的步末。重新安装的制导在下一步生效。',
            '',
            '各组 admission.json 保存候选排名、路径、旅行成本、待命点、原生分数、浮点差与规范哈希。agent_metrics.json 给出逐艇请求和调节分叉、三维与垂直位移、初始路线净进展及首次换路余程。comparison.json 保存完整同步序列及可比性失败项。',
        ]
    rows += [
        '',
        '正式任务时限 400 步。本诊断每个条件最多运动 32 步。任务提前终止、工程失败和批次有效性分别保留。',
    ]
    (batch / 'REPORT.md').write_text('\n'.join(rows) + '\n', encoding='utf-8')
    write(batch / 'batch.json', meta)


def archive(batch, scratch):
    paths = []
    links = {}
    for p in sorted(batch.rglob('*')):
        if p.is_symlink():
            # A ROS log alias may point outside this batch. Preserve its name and
            # link text without following it or copying unrelated host files.
            links[p.relative_to(batch).as_posix()] = str(p.readlink())
            continue
        if p.is_file() and p != scratch and p != batch / 'SHA256SUMS':
            paths.append(p)
    if links:
        index = batch / 'archive_links.json'
        write(index, links)
        paths.append(index)
    sums = {p.relative_to(batch).as_posix(): file_hash(p) for p in paths}
    manifest = batch / 'SHA256SUMS'
    manifest.write_bytes(''.join(h + '  ' + name + '\n' for name, h in sums.items()).encode())
    with zipfile.ZipFile(scratch, 'x', zipfile.ZIP_DEFLATED, compresslevel=4, allowZip64=True) as z:
        for p in [*paths, manifest]:
            z.write(p, p.relative_to(batch).as_posix())
    with zipfile.ZipFile(scratch) as z:
        if (
            z.testzip() is not None
            or len(z.namelist()) != len(sums) + 1
            or set(z.namelist()) != set(sums) | {'SHA256SUMS'}
        ):
            raise ValueError('ZIP member or CRC mismatch')
        if z.read('SHA256SUMS') != manifest.read_bytes():
            raise ValueError('Archived manifest mismatch')
        for name, h in sums.items():
            with z.open(name) as f:
                if stream_hash(f) != h:
                    raise ValueError('Archived SHA-256 differs: ' + name)
    return file_hash(scratch)

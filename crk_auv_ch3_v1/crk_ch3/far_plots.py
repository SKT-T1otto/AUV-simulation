"""Write self-contained batch figures with the Ubuntu standard library."""

from .initial_contract import FULL_CONDITIONS

COLORS = ('#00766f', '#8649a1', '#ba5913')
ARMS = ('#236095', '#cb5e13')


def chart(path, title, xlabel, ylabel, series, *, xy=False, boundaries=()):
    left, top = 88, 65
    width, height = (440, 440) if xy else (660, 300)
    points = [p for _, _, _, line in series for p in line]
    if xy:
        xmax, ymin, ymax = 20, 0, 20
    else:
        xmax = max([1] + [p[0] for p in points])
        low = min([p[1] for p in points], default=0)
        high = max([p[1] for p in points], default=1)
        pad = max(0.05, 0.08 * (high - low))
        ymin, ymax = low - pad, high + pad

    def x(v):
        return left + width * v / xmax

    def y(v):
        return top + height * (ymax - v) / (ymax - ymin)

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{left+width+50}" height="{top+height+116}">',
        '<rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" fill="#1b323b">',
        f'<text x="{left}" y="32" font-size="20">{title}</text>',
    ]
    for k in range(6):
        xx, yy = x(xmax * k / 5), y(ymin + (ymax - ymin) * k / 5)
        out.extend(
            [
                f'<path d="M{xx} {top} v{height} M{left} {yy} h{width}" fill="none" stroke="#d9e1e4"/>',
                f'<text x="{xx}" y="{top+height+19}" text-anchor="middle">{xmax*k/5:.1f}</text>',
                f'<text x="{left-8}" y="{yy+4}" text-anchor="end">{ymin+(ymax-ymin)*k/5:.1f}</text>',
            ]
        )
    for value in sorted(set(boundaries)):
        out.append(f'<path d="M{x(value)} {top} v{height}" stroke="#495a60" stroke-dasharray="3 4"/>')
    out.extend(
        [
            f'<text x="{left+width/2}" y="{top+height+40}" text-anchor="middle">{xlabel}</text>',
            f'<text transform="translate(24,{top+height/2}) rotate(-90)" text-anchor="middle">{ylabel}</text>',
        ]
    )
    for i, (label, color, style, line) in enumerate(series):
        coords = ' '.join(f'{x(h):.2f},{y(v):.2f}' for h, v in line)
        out.append(
            f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2" {style}/>'
        )
        lx, ly = left + i % 3 * 145, top + height + 62 + (i // 3) * 19
        out.append(
            f'<path d="M{lx} {ly} h26" stroke="{color}" stroke-width="3" {style}/>'
            f'<text x="{lx+31}" y="{ly+4}" font-size="13">{label}</text>'
        )
    if not points:
        out.append(f'<text x="{left+22}" y="{top+36}">No validated paired tracks available</text>')
    path.write_text('\n'.join(out + ['</g></svg>']), encoding='utf-8')


def plot_search(batch, rows, data):
    tracks, dispersion, heights = [], [], []
    for k, (label, condition) in enumerate(zip(('A', 'B'), FULL_CONDITIONS)):
        own = [r for r in rows if r['condition'] == condition and r['search_phase'] and r['t_s'] >= 0]
        style = 'stroke-dasharray="6 3"' if k else ''
        for i, col in enumerate(COLORS):
            tracks.append(
                (f'{label} auv_{i}', col, style, [(r[f'auv{i}_x_m'], r[f'auv{i}_y_m']) for r in own])
            )
        dispersion.append((label, ARMS[k], style, [(r['t_s'], r['Dxy']) for r in own]))
        heights.append((label, ARMS[k], style, [(r['t_s'], r['auv2_z_m']) for r in own]))
    edges = [data['common_end_s']] if data.get('common_end_s') is not None else []
    chart(
        batch / 'tracks.svg',
        'Measured search tracks',
        'Task x (m)',
        'Task y (m)',
        tracks,
        xy=True,
        boundaries=[10],
    )
    chart(
        batch / 'dispersion.svg',
        'Mean horizontal pair distance',
        'Task time (s)',
        'Dxy (m)',
        dispersion,
        boundaries=edges,
    )
    chart(
        batch / 'auv2_z.svg',
        'Measured height of auv_2',
        'Task time (s)',
        'Task z (m, up)',
        heights,
        boundaries=edges,
    )
    delta = [
        (r['t_s'], r['B_minus_A_auv2_z_m'])
        for r in rows
        if r['condition'] == FULL_CONDITIONS[1] and r['common_search'] and r['stamp_paired']
    ]
    chart(
        batch / 'auv2_delta_z.svg',
        'Signed height difference of auv_2',
        'Task time (s)',
        'B minus A (m, up)',
        [('B minus A', '#00766f', '', delta)],
        boundaries=edges,
    )

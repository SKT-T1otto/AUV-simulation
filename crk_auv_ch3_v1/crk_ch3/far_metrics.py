"""Spatial measurements on the three simultaneous Gazebo tracks, in native metres."""

import math

from .initial_records import TASK_NS, require

PAIRS = ((0, 1), (0, 2), (1, 2))
XY_FIELDS = ('d01_xy', 'd02_xy', 'd12_xy')
XYZ_FIELDS = ('d01_3d', 'd02_3d', 'd12_3d')
FIELDS = (*XY_FIELDS, *XYZ_FIELDS, 'Dxy', 'D3d', 'min_xy', 'min_3d')


def geometry(points):
    horizontal = [math.dist(points[i][:2], points[j][:2]) for i, j in PAIRS]
    spatial = [math.dist(points[i], points[j]) for i, j in PAIRS]
    return dict(
        zip(
            FIELDS,
            [
                *horizontal,
                *spatial,
                sum(horizontal) / 3,
                sum(spatial) / 3,
                min(horizontal),
                min(spatial),
            ],
        )
    )


def between(a, b, fraction):
    return tuple(x + (y - x) * fraction for x, y in zip(a, b))


def cell(point, shape, extent):
    return tuple(
        min(n - 1, max(0, math.floor(x * n / length))) for x, n, length in zip(point, shape, extent)
    )


def crossed_cells(a, b, shape, extent):
    # Visit the interiors on either side of a crossing. At an exactly diagonal
    # corner this avoids credit for the two cells only touched at that corner.
    cuts = {0.0, 1.0}
    for x, y, n, length in zip(a, b, shape, extent):
        if x != y:
            for k in range(1, n):
                t = (k * length / n - x) / (y - x)
                if 0 < t < 1:
                    cuts.add(t)
    times = sorted(cuts)
    result = {cell(a, shape, extent), cell(b, shape, extent)}
    result.update(cell(between(a, b, (lo + hi) / 2), shape, extent) for lo, hi in zip(times, times[1:]))
    return result


def right_fraction(x, y, mid):
    if min(x, y) >= mid:
        return 1.0
    if max(x, y) < mid:
        return 0.0
    return (max(x, y) - mid) / abs(y - x)


def spatial_window(samples, steps, shape, extent):
    """A None slot is unmatched. It splits a track and contributes no unobserved interval."""
    end_ns = steps * TASK_NS
    totals = dict.fromkeys(FIELDS, 0.0)
    integrated = 0.0
    visits = [set() for _ in range(3)]
    right = [0.0] * 3
    first = [None] * 3
    max_x = [None] * 3
    z_values = []
    min_pair = None
    previous = None
    valid = 0
    mid = extent[0] / 2
    for sample in samples[: 10 * steps + 1]:
        if sample is None:
            previous = None
            continue
        ns, points = sample
        if 0 <= ns <= end_ns:
            valid += 1
            z_values.append(points[2][2])
            now_min = geometry(points)['min_xy']
            min_pair = now_min if min_pair is None else min(min_pair, now_min)
            for i, p in enumerate(points):
                max_x[i] = p[0] if max_x[i] is None else max(max_x[i], p[0])
                visits[i].add(cell(p, shape, extent))
                if first[i] is None and p[0] >= mid:
                    first[i] = ns / 1e9
        if previous is not None:
            old, origin = previous
            require(ns > old, 'Non-increasing Gazebo odometry')
            lo, hi = max(0, old), min(ns, end_ns)
            if hi > lo:
                dt = (hi - lo) / 1e9
                a = (
                    origin
                    if lo == old
                    else [between(x, y, (lo - old) / (ns - old)) for x, y in zip(origin, points)]
                )
                b = (
                    points
                    if hi == ns
                    else [between(x, y, (hi - old) / (ns - old)) for x, y in zip(origin, points)]
                )
                left, last = geometry(a), geometry(b)
                integrated += dt
                for key in FIELDS:
                    totals[key] += dt * (left[key] + last[key]) / 2
                for i, (x, y) in enumerate(zip(a, b)):
                    right[i] += dt * right_fraction(x[0], y[0], mid)
                    visits[i].update(crossed_cells(x, y, shape, extent))
        previous = sample
    union = visits[0] | visits[1] | visits[2]
    repeat = {c for c in union if sum(c in v for v in visits) >= 2}
    return dict(
        window_end_step=steps,
        nominal_s=end_ns / 1e9,
        integrated_s=integrated,
        unobserved_duration_s=max(0.0, end_ns / 1e9 - integrated),
        samples_in_window=valid,
        mean_m={k: v / integrated if integrated else None for k, v in totals.items()},
        minimum_observed_pair_xy_m=min_pair,
        auv2_z_range_m=[min(z_values), max(z_values)] if z_values else None,
        per_agent=[
            dict(
                agent='auv_' + str(i),
                max_x_m=max_x[i],
                first_observed_right_s=first[i],
                right_boat_s=right[i],
                visited_xy_cells=len(visits[i]),
            )
            for i in range(3)
        ],
        right_boat_s=sum(right),
        right_fraction=sum(right) / (3 * integrated) if integrated else None,
        union_cells=len(union),
        overlapping_cells=len(repeat),
        overlap_fraction=len(repeat) / len(union) if union else None,
        grid_shape_xy=shape,
        cell_size_m=[s / n for s, n in zip(extent, shape)],
    )


def common_samples(a, b, steps):
    count = 10 * steps + 1
    require(len(a) >= count and len(b) >= count, 'Missing physical search rows')
    views = ([], [])
    missing = []
    for i, (sa, sb) in enumerate(zip(a[:count], b[:count])):
        if sa[0] != sb[0]:
            missing.append(dict(slot=i, A_ns=sa[0], B_ns=sb[0]))
            views[0].append(None)
            views[1].append(None)
        else:
            views[0].append(sa)
            views[1].append(sb)
    return views, dict(expected_slots=count, matched_slots=count - len(missing), unmatched=missing)

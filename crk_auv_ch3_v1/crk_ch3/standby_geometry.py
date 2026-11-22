"""Geometry shared by the diagnostic runtime and the standard-library host reader."""

import math
from itertools import product
from xml.etree import ElementTree as ET

CONDITIONS = ('quadrant_standby_original400', 'quadrant_standby_clearance400')
PREFIX = 'g6_standby_clearance_pair400_'
SCHEMA = 'crk.g6.standby_clearance_pair400.v1'
REFERENCE_NS = 20_208_000_000


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def minus(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def hulls(path):
    xml = ET.parse(path)
    result = []
    for i in range(4):
        box = xml.find(
            f".//model[@name='auv_{i}']/link[@name='base_link']/collision[@name='displacement_hull']"
        )
        origin = tuple(map(float, box.findtext('pose').split()))
        half = tuple(float(v) / 2 for v in box.findtext('geometry/box/size').split())
        if len(origin) != 6 or any(origin[3:]) or len(half) != 3 or min(half) <= 0:
            raise ValueError('Unsupported SDF displacement hull')
        result.append((origin[:3], half))
    return result


def radii(boxes):
    return [math.hypot(*(abs(a) + b for a, b in zip(origin, half))) for origin, half in boxes]


def tick_ranges(ticks):
    ranges = []
    for ns in sorted(ticks):
        if ranges and ranges[-1][1] + 2_000_000 == ns:
            ranges[-1][1] = ns
        else:
            ranges.append([ns, ns])
    return ranges


def nearest(point, route):
    if not route:
        raise ValueError('Missing remaining route')
    best = (math.inf, None)
    for idx, (a, b) in enumerate(zip(route, route[1:] or route)):
        v, w = minus(b, a), minus(point, a)
        squared = dot(v, v)
        t = min(1.0, max(0.0, dot(v, w) / squared)) if squared else 0.0
        best = min(best, (math.hypot(*(x - t * y for x, y in zip(w, v))), idx))
    return best


def world_box(body, hull):
    q = body['quaternion']
    x, y, z, w = (v / math.hypot(*q) for v in q)
    axes = (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )
    origin, half = hull
    center = tuple(p + dot(origin, col) for p, col in zip(body['position'], zip(*axes)))
    return center, axes, half


def hull_gap(a, b):
    ca, ax, ha = a
    cb, bx, hb = b
    delta = minus(ca, cb)
    axes = (*ax, *bx, *(cross(u, v) for u, v in product(ax, bx)))
    gap = 0.0
    for axis in axes:
        norm = math.hypot(*axis)
        if norm > 1e-9:
            span = sum(abs(dot(axis, v)) * h for v, h in zip(ax + bx, ha + hb))
            gap = max(gap, (abs(dot(delta, axis)) - span) / norm)
    return gap

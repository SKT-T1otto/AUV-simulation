"""G2: native scenario validation and rigid frame conversion; no policy access."""
from copy import deepcopy
from dataclasses import dataclass
import itertools
import math


def vector3(value):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError('Expected three coordinates')
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in value):
        raise ValueError('Coordinates must be finite numbers')
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class Frame:
    """Yaw rotation and translation: preserves z-up and lengths, never scales."""
    translation: tuple = (-10., -10., -9.)
    yaw: float = 0.

    def __post_init__(self):
        object.__setattr__(self, 'translation', vector3(self.translation))
        if isinstance(self.yaw, bool) or not math.isfinite(self.yaw):
            raise ValueError('Invalid frame yaw')

    def rotate(self, value, inverse=False):
        x, y, z = vector3(value)
        angle = -self.yaw if inverse else self.yaw
        c, s = math.cos(angle), math.sin(angle)
        return (c*x-s*y, s*x+c*y, z)

    def position(self, value):
        return tuple(x+t for x, t in zip(self.rotate(value), self.translation))

    def inverse_position(self, value):
        return self.rotate(tuple(x-t for x, t in zip(vector3(value), self.translation)), inverse=True)


def validate_scene(scene, space_size=(20., 20., 8.)):
    """Consume native field names; preserve all metadata without regenerating a scene."""
    result = deepcopy(scene)
    bounds = vector3(space_size)
    if any(x <= 0 for x in bounds):
        raise ValueError('Invalid world bounds')
    if result.get('protocol') != 'ch3_unknown_map_v1' or result.get('scenario_profile') != 'M20_MOVING_UNKNOWN_MULTI':
        raise ValueError('Wrong native scenario protocol/profile')
    if not isinstance(result.get('scenario_id'), str) or not result['scenario_id'].strip():
        raise ValueError('Missing scenario identity')
    if type(result.get('scenario_seed')) is not int or result['scenario_seed'] < 0:
        raise ValueError('Invalid scenario seed')
    if result.get('obstacle_knowledge_mode') != 'online_unknown':
        raise ValueError('This stage accepts unknown-map scenes only')
    if result.get('map_sharing_mode') != 'central_shared_deterministic_v1':
        raise ValueError('Changed map sharing semantics')
    if result.get('target_motion_mode') != 'constant_velocity_reflect_v1':
        raise ValueError('Unsupported native target motion')
    agents = result.get('initial_agent_positions')
    if not isinstance(agents, list) or len(agents) != 4:
        raise ValueError('Expected four initial agents')
    points = [vector3(x) for x in agents]
    points += [vector3(result['initial_executor_wait_point'])]
    target = vector3(result.get('target_initial_position', result.get('target_position')))
    if 'target_position' in result and vector3(result['target_position']) != target:
        raise ValueError('Conflicting native target positions')
    points.append(target)
    vector3(result['target_initial_velocity'])
    if any(not 0 <= x <= limit for point in points for x, limit in zip(point, bounds)):
        raise ValueError('Initial point outside native workspace')
    boxes = result.get('obstacles')
    if not isinstance(boxes, list) or result.get('obstacle_count') != len(boxes):
        raise ValueError('Obstacle count mismatch')
    if result.get('obstacle_layout_id') != ('custom_aabb_v1' if boxes else 'none'):
        raise ValueError('Obstacle layout mismatch')
    for box in boxes:
        center, size = vector3(box['center']), vector3(box['size'])
        if any(s <= 0 or c-s/2 < 0 or c+s/2 > limit for c, s, limit in zip(center, size, bounds)):
            raise ValueError('Invalid or out-of-bounds obstacle')
        if any(all(abs(x-c) <= s/2 for x, c, s in zip(point, center, size)) for point in points):
            raise ValueError('An initial point is inside an obstacle')
    return result


def project_truth_scene(scene, frame=Frame(), space_size=(20., 20., 8.)):
    """Privileged rendering data only; never use this object as map/actor input."""
    s = validate_scene(scene, space_size)
    corners = [frame.position(point) for point in itertools.product(*[(0., n) for n in space_size])]
    if max(p[2] for p in corners) >= 0:
        raise ValueError('Whole workspace must be below z=0 for this underwater preview')
    return {'schema': 'crk.ch3.truth_scene.v1', 'privileged_truth_not_policy_input': True,
            'scenario_id': s['scenario_id'], 'space_size': list(space_size),
            'frame': {'translation': list(frame.translation), 'yaw': frame.yaw},
            'workspace_corners': corners,
            'agents': [frame.position(p) for p in s['initial_agent_positions']],
            'wait_point': frame.position(s['initial_executor_wait_point']),
            'target_position': frame.position(s.get('target_initial_position', s.get('target_position'))),
            'target_velocity': frame.rotate(s['target_initial_velocity']),
            'obstacles': [{'center': frame.position(box['center']), 'size': vector3(box['size']),
                           'yaw': frame.yaw} for box in s['obstacles']]}


def select_scene(manifest, scenario_id=None):
    scenes = manifest.get('scenarios')
    if not isinstance(scenes, list) or not scenes:
        raise ValueError('Expected a native exported scenario manifest')
    ids = [scene.get('scenario_id') for scene in scenes]
    if any(not isinstance(x, str) for x in ids) or len(set(ids)) != len(ids):
        raise ValueError('Missing or duplicate scenario IDs')
    if scenario_id is None and len(scenes) != 1:
        raise ValueError('Select --scenario-id explicitly for multiple scenes')
    chosen = scenes if scenario_id is None else [s for s in scenes if s['scenario_id'] == scenario_id]
    if len(chosen) != 1:
        raise ValueError('Unknown scenario ID')
    return validate_scene(chosen[0])

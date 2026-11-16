"""G1: explicit experiment contracts, independent of ROS and the source checkout."""
import hashlib
import json
import math
from pathlib import Path

ROLES = ('search_fast', 'search_balanced', 'search_precise', 'executor')


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode('utf-8')).hexdigest()


def positive_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('Expected a finite positive number')
    return float(value)


def validate_experiment(config):
    c = json.loads(json.dumps(config, allow_nan=False))
    if c.get('schema') != 'crk.ch3.bootstrap.v1':
        raise ValueError('Unsupported experiment schema')
    if c.get('roles') != list(ROLES) or c.get('observation_dim') != 28 or c.get('action_dim') != 3:
        raise ValueError('Expected four original roles and 28D/3D interfaces')
    if c.get('method') != 'ZERO' or c.get('map_version') != 'original_d2':
        raise ValueError('Only the ZERO/original D2 baseline is enabled before G7')
    if c.get('checkpoint') is not None:
        raise ValueError('ZERO must not load a checkpoint')
    if c.get('map_sharing') != 'central_shared_deterministic_v1' or c.get('handoff') != 'fixed_reliable_one_step_v1':
        raise ValueError('Preserve the Chapter 3 shared map and reliable one-step handoff')
    if c.get('obstacle_sensor_noise_std') != 0.0 or c.get('training_enabled') is not False:
        raise ValueError('Bootstrap has zero sensing noise and no training')
    if c.get('profile') != 'M20_MOVING_UNKNOWN_MULTI' or c.get('task_protocol') != 'collision_terminal_v1':
        raise ValueError('Unsupported source task/profile')
    dt = positive_number(c['task_dt_s'])
    control = positive_number(c['control_dt_s'])
    physics = positive_number(c['physics_dt_s'])
    for numerator, denominator in ((dt, control), (control, physics)):
        ratio = numerator / denominator
        if ratio < 1 or not math.isclose(ratio, round(ratio), rel_tol=0, abs_tol=1e-9):
            raise ValueError('Task/control/physics steps must be exactly divisible')
    if type(c.get('max_steps')) is not int or c['max_steps'] <= 0:
        raise ValueError('Invalid task horizon')
    if c.get('motion_authority') != 'gazebo' or c.get('world_mode') != 'synchronous':
        raise ValueError('Gazebo must own motion under synchronous stepping')
    return c


def verify_source_lock(lock, source_root):
    """Read only: validate exactly the recorded files without importing source code."""
    root = Path(source_root).resolve()
    if lock.get('schema') != 'crk.ch3.source_lock.v1' or not lock.get('files'):
        raise ValueError('Invalid source lock')
    if canonical_hash(lock['files']) != lock.get('files_sha256'):
        raise ValueError('Source inventory digest mismatch')
    if lock.get('inventory_patterns'):
        current = {p.relative_to(root).as_posix() for pattern in lock['inventory_patterns']
                   for p in root.glob(pattern) if p.is_file()}
        if current != set(lock['files']):
            raise ValueError('Source inventory has added or removed files')
    for relative, expected in lock['files'].items():
        parts = relative.split('/')
        if relative.startswith('/') or '\\' in relative or ':' in relative or any(p in ('', '.', '..') for p in parts):
            raise ValueError('Unsafe source inventory path')
        path = root.joinpath(*parts)
        if not path.resolve().is_relative_to(root) or path.is_symlink():
            raise ValueError('Source path escapes locked root')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Changed or missing source file: {relative}')
    return len(lock['files'])

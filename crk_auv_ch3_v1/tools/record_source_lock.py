"""Explicit read-only inspection of Active; write a new source identity in this project."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from crk_ch3.contracts import canonical_hash, validate_experiment, verify_source_lock

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
source = args.source_root.resolve()
patterns = ['core/**/*.py', 'chapter3_bser/**/*.py', 'configs/*.json', 'project_paths.py']
files = {p.relative_to(source).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
         for pattern in patterns for p in sorted(source.glob(pattern)) if p.is_file()}
if not all(name in files for name in ['core/env/uav_env.py', 'configs/environment.json',
                                     'configs/b2.json', 'configs/b3.json', 'project_paths.py']):
    raise ValueError('Missing Chapter 3 source files')
tree = ast.parse((source/'core/env/uav_env.py').read_text(encoding='utf-8'))
defaults = {}
for node in ast.walk(tree):
    if isinstance(node, ast.FunctionDef) and node.name == '__init__':
        for name, value in zip(node.args.args[-len(node.args.defaults):], node.args.defaults):
            if name.arg in ('dt', 'space_size', 'n_agents', 'max_steps'):
                defaults[name.arg] = ast.literal_eval(value)
config = validate_experiment(json.loads((ROOT/'config/experiment.json').read_text(encoding='utf-8')))
environment = json.loads((source/'configs/environment.json').read_text(encoding='utf-8'))
if defaults.get('n_agents') != 4 or defaults.get('dt') != config['task_dt_s']:
    raise ValueError('Native agent count or time period differs from adapter config')
if environment['max_steps'] != config['max_steps'] or environment['map_sharing_mode'] != config['map_sharing']:
    raise ValueError('Native horizon or map sharing differs from adapter config')
lock = {'schema': 'crk.ch3.source_lock.v1', 'source_name': 'AUV-Thesis-Active',
        'identity_only_not_runtime_snapshot': True, 'inventory_patterns': patterns,
        'native_constructor_defaults': defaults, 'files': dict(sorted(files.items())),
        'files_sha256': canonical_hash(files), 'checkpoint_loaded': False}
verify_source_lock(lock, source)
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open('x', encoding='utf-8', newline='\n') as stream:
    json.dump(lock, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
print(f'Locked {len(files)} source files; source checkout was not modified.')

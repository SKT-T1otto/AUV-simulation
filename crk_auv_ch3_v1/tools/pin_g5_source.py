"""Explicitly snapshot current Active runtime bytes; never overwrite an old pin."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from crk_ch3.contracts import canonical_hash, verify_source_lock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    args = parser.parse_args()
    source = args.source_root.resolve()
    destination = ROOT/'native_g5'
    output = ROOT/'config/source_lock_g5.json'
    if destination.exists() or output.exists():
        raise FileExistsError('G5 pin exists. Review source evolution into a new version; do not overwrite.')
    patterns = ['core/**/*.py', 'chapter3_bser/**/*.py', 'configs/*.json', 'project_paths.py']
    files = {p.relative_to(source).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
             for pattern in patterns for p in sorted(source.glob(pattern)) if p.is_file()}
    old = json.loads((ROOT/'config/source_lock.json').read_text(encoding='utf-8'))
    delta = {'added': sorted(files.keys()-old['files'].keys()),
             'removed': sorted(old['files'].keys()-files.keys()),
             'changed': sorted(k for k in files.keys() & old['files'].keys() if files[k] != old['files'][k])}
    for name, digest in files.items():
        data = (source/name).read_bytes()
        if hashlib.sha256(data).hexdigest() != digest:
            raise RuntimeError('Source changed during capture: '+name)
        target = destination/name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as stream:
            stream.write(data)
    lock = {'schema': 'crk.ch3.source_lock.v1', 'source_name': 'AUV-Thesis-Active',
            'pin_reason': 'User requested current Active for G5 on 2026-11-16; later updates require review.',
            'identity_only_not_runtime_snapshot': False, 'snapshot_directory': 'native_g5',
            'inventory_patterns': patterns, 'files': files, 'files_sha256': canonical_hash(files),
            'previous_g1_files_sha256': old['files_sha256'], 'evolution_from_g1': delta,
            'enabled_runtime': 'original D2 / ZERO via configs/b2.json; research variants not selected',
            'checkpoint_loaded': False}
    verify_source_lock(lock, source)
    verify_source_lock(lock, destination)
    with output.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(lock, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'pinned_files': len(files), 'source_sha256': lock['files_sha256'], 'delta': delta}, indent=2))


if __name__ == '__main__':
    main()

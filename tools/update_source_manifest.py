"""Refresh Linux source checksums and local inventory; never create archives."""
import hashlib
import json
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.git', '__pycache__', 'build', 'install', 'log', 'results'}
count = 0
for workspace in ('crk_auv_singleboat_v1', 'crk_auv_ch3_v1'):
    source = PROJECT/workspace
    files = sorted(p for p in source.rglob('*') if p.is_file()
                   and not EXCLUDED.intersection(p.relative_to(source).parts)
                   and p.suffix != '.pyc' and p.name != 'SHA256SUMS')
    manifest = '\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(source).as_posix()
                         for p in files)+'\n'
    (source/'SHA256SUMS').write_text(manifest, encoding='utf-8', newline='\n')
    count += len(files)
inventory = [{'path': p.relative_to(PROJECT).as_posix(),
              'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
             for p in sorted(PROJECT.rglob('*')) if p.is_file()
             and '.git' not in p.relative_to(PROJECT).parts
             and p.name != 'SINGLEBOAT_CREATED_FILES.json']
inventory.append({'path': 'SINGLEBOAT_CREATED_FILES.json', 'sha256': None,
                  'note': 'Self record without recursive self hash.'})
(PROJECT/'SINGLEBOAT_CREATED_FILES.json').write_text(json.dumps(inventory, indent=2)+'\n', encoding='utf-8')
print(f'Recorded {count} source checksums in two workspaces; no archives generated.')

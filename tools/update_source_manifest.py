"""Refresh Linux source checksums and local inventory; never create archives."""
import hashlib
import json
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT/'crk_auv_singleboat_v1'
EXCLUDED = {'.git', '__pycache__', 'build', 'install', 'log', 'results'}
files = sorted(p for p in SOURCE.rglob('*') if p.is_file()
               and not EXCLUDED.intersection(p.relative_to(SOURCE).parts)
               and p.suffix != '.pyc' and p.name != 'SHA256SUMS')
manifest = '\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(SOURCE).as_posix()
                     for p in files)+'\n'
(SOURCE/'SHA256SUMS').write_text(manifest, encoding='utf-8', newline='\n')
inventory = [{'path': p.relative_to(PROJECT).as_posix(),
              'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
             for p in sorted(PROJECT.rglob('*')) if p.is_file()
             and '.git' not in p.relative_to(PROJECT).parts
             and p.name != 'SINGLEBOAT_CREATED_FILES.json']
inventory.append({'path': 'SINGLEBOAT_CREATED_FILES.json', 'sha256': None,
                  'note': 'Self record without recursive self hash.'})
(PROJECT/'SINGLEBOAT_CREATED_FILES.json').write_text(json.dumps(inventory, indent=2)+'\n', encoding='utf-8')
print(f'Recorded {len(files)} source checksums; no archives generated.')

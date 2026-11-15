"""Package this independent simulation project and record its workspace files."""
import hashlib
import json
import tarfile
import zipfile
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT/'crk_auv_singleboat_v1'
EXCLUDED_PARTS = {'__pycache__', '.git', 'build', 'install', 'log', 'results'}
files = sorted(p for p in ROOT.rglob('*') if p.is_file()
               and not EXCLUDED_PARTS.intersection(p.relative_to(ROOT).parts)
               and p.suffix != '.pyc' and p.name != 'SHA256SUMS')
manifest = '\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(ROOT).as_posix()
                     for p in files)+'\n'
(ROOT/'SHA256SUMS').write_text(manifest, encoding='utf-8', newline='\n')
files.append(ROOT/'SHA256SUMS')
archives = [OUT/'crk_auv_singleboat_v1.tar.gz', OUT/'crk_auv_singleboat_v1.zip']
with tarfile.open(archives[0], 'w:gz') as archive:
    for path in files:
        info = archive.gettarinfo(str(path), arcname='crk_auv_singleboat_v1/'+path.relative_to(ROOT).as_posix())
        info.mode = 0o755 if path.suffix == '.sh' else 0o644
        with path.open('rb') as stream:
            archive.addfile(info, stream)
with zipfile.ZipFile(archives[1], 'w', zipfile.ZIP_DEFLATED) as archive:
    for path in files:
        archive.write(path, 'crk_auv_singleboat_v1/'+path.relative_to(ROOT).as_posix())
for artifact in archives:
    with (tarfile.open(artifact) if artifact.suffix == '.gz' else zipfile.ZipFile(artifact)) as archive:
        if artifact.suffix == '.gz':
            entries = {m.name: archive.extractfile(m).read() for m in archive.getmembers() if m.isfile()}
        else:
            entries = {name: archive.read(name) for name in archive.namelist()}
        for row in manifest.splitlines():
            digest, name = row.split('  ', 1)
            assert hashlib.sha256(entries['crk_auv_singleboat_v1/'+name]).hexdigest() == digest
    print(artifact.name, artifact.stat().st_size, hashlib.sha256(artifact.read_bytes()).hexdigest())
inventory = []
for path in sorted(OUT.rglob('*')):
    if path.is_file() and '.git' not in path.relative_to(OUT).parts and path.name != 'SINGLEBOAT_CREATED_FILES.json':
        inventory.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                          'packaged': path in files})
inventory.append({'path': str(OUT/'SINGLEBOAT_CREATED_FILES.json'), 'sha256': None,
                  'note': 'This inventory records itself without a recursive self hash.'})
(OUT/'SINGLEBOAT_CREATED_FILES.json').write_text(json.dumps(inventory, indent=2), encoding='utf-8')
print('Verified both archives against their file manifests; all new files recorded.')

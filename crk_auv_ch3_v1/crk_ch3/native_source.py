"""Activate only the immutable, reviewed G5 runtime snapshot."""
import json
from pathlib import Path
import sys

from .contracts import verify_source_lock

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT/'native_g5'


def activate_native():
    lock = json.loads((ROOT/'config/source_lock_g5.json').read_text(encoding='utf-8'))
    verify_source_lock(lock, SNAPSHOT)
    # Refuse an already imported sibling checkout with the same top-level names.
    for name, module in list(sys.modules.items()):
        if name in ('core', 'chapter3_bser', 'project_paths') or name.startswith(('core.', 'chapter3_bser.')):
            path = getattr(module, '__file__', None)
            if path is None or not Path(path).resolve().is_relative_to(SNAPSHOT):
                raise RuntimeError('Foreign native module already loaded: '+name)
    sys.dont_write_bytecode = True
    if str(SNAPSHOT) not in sys.path:
        sys.path.insert(0, str(SNAPSHOT))
    return lock

"""Run the bounded G5 semantic gate; does not start Gazebo or load policies."""
from datetime import datetime
import json
import os
from pathlib import Path
import platform
import sys
import tempfile
import traceback

from .native_source import activate_native


def main():
    output_root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if Path('/.dockerenv').is_file() and not output_root.is_mount():
        raise RuntimeError('G5 requires the actual mounted Linux host output directory')
    if os.name == 'posix' and os.geteuid() == 0:
        if not all(os.environ.get(k, '').isdigit() for k in ('CRK_OUTPUT_UID', 'CRK_OUTPUT_GID')):
            raise RuntimeError('G5 requires host desktop UID/GID; use the host launcher')
    parent = output_root/'results/ch3'
    parent.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='mission_check_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_', dir=parent))
    output.chmod(0o755)
    summary = dict(stage='G5', run_kind='source_recorded_motion_parity', engineering_fixture=True,
                   semantic_parity_passed=False, gazebo_motion_executed=False,
                   native_reference_fixture_executed=False, checkpoint_loaded=False,
                   training_executed=False, performance_claim=False,
                   python_version=platform.python_version(), termination_reason='not_started', cases=[])
    def save():
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    save()
    print(f'G5 parity results: {output}', flush=True)
    try:
        lock = activate_native()
        summary['source_files_sha256'] = lock['files_sha256']
        import numpy as np
        import torch
        from .mission_parity import run_suite
        summary.update(numpy_version=np.__version__, torch_version=torch.__version__)
        summary['native_reference_fixture_executed'] = True
        def record_cases(cases):
            summary['cases'] = list(cases)
            save()
        summary['cases'] = run_suite(output, on_case=record_cases)
        summary.update(termination_reason='completed', semantic_parity_passed=True,
                       fixture_episode_limit=8, fixture_step_limit=32)
    except BaseException as error:
        summary.update(termination_reason='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed',
                       error=str(error))
        detail = traceback.format_exc()
        (output/'error_traceback.txt').write_text(detail, encoding='utf-8')
        print(detail, file=sys.stderr, flush=True)
    finally:
        save()
        if os.name == 'posix' and os.geteuid() == 0:
            uid, gid = int(os.environ['CRK_OUTPUT_UID']), int(os.environ['CRK_OUTPUT_GID'])
            for directory, folders, files in os.walk(output, followlinks=False):
                os.chown(directory, uid, gid, follow_symlinks=False)
                for name in files:
                    os.chown(Path(directory)/name, uid, gid, follow_symlinks=False)
        print(f"G5 parity={summary['semantic_parity_passed']}; inspect {output}/summary.json", flush=True)
    raise SystemExit(0 if summary['semantic_parity_passed'] else 1)


if __name__ == '__main__':
    main()

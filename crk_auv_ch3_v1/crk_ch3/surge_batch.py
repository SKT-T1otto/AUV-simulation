"""Run both levels from fresh Gazebo worlds and package exactly this batch."""
from datetime import datetime
import os
from pathlib import Path
import signal
import tempfile
import traceback

from .initial_package import archive
from .motion_run import supervise
from .surge import LEVELS, create_trial, write_json
from .surge_report import report
from .vehicle import ROOT


def run(root):
    return run_trials(root, 'surge_force', [(level, f'{level}N') for level in LEVELS],
                      create_trial, report, 'surge_velocity')


def run_trials(root, prefix, trials, prepare, analyze, figure_name):
    parent = root/'results/ch3'
    parent.mkdir(parents=True,exist_ok=True)
    batch = Path(tempfile.mkdtemp(prefix=prefix+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_',dir=parent))
    batch.chmod(0o755)
    print(f'BATCH_DIR={batch}',flush=True)
    data = dict(status='partial', git_sha=os.environ.get('CRK_SOURCE_SHA'), trials={})
    (batch/'source_SHA256SUMS').write_bytes((ROOT/'SHA256SUMS').read_bytes())
    cases = []
    status = 1
    try:
        for condition, suffix in trials:
            case = batch/f'{batch.name}_{suffix}'
            data['trials'][condition] = case.name
            print(f'RUNNING={suffix}', flush=True)
            prepare(case, condition)
            code = supervise(root, case, True, 'crk_ch3.surge_ros', True)
            if code:
                raise RuntimeError(f'{suffix} trial stopped with exit {code}; see {case.name}/summary.json')
            cases.append((condition,case))
        analyze(batch,cases)
        data['status']='completed'
        status = 0
    except (Exception, KeyboardInterrupt) as error:
        data['error']=repr(error)
        (batch/'batch_traceback.log').write_text(traceback.format_exc())
    finally:
        write_json(batch/'batch.json',data)
        scratch = parent/(batch.name+'.zip.partial')
        checksum = archive(batch,scratch)
        archive_path = parent/(batch.name+'.zip')
        scratch.rename(archive_path)
        if os.geteuid()==0:
            uid,gid = int(os.environ['CRK_OUTPUT_UID']),int(os.environ['CRK_OUTPUT_GID'])
            for path in (batch,*batch.iterdir(),archive_path):
                os.chown(path,uid,gid,follow_symlinks=False)
        print(f"BATCH_STATUS={data['status']}",flush=True)
        if status==0:
            figure = batch/(figure_name+'.png')
            print(f"FIGURE={figure if figure.exists() else figure.with_suffix('.svg')}",flush=True)
        print(f'UPLOAD_FILE={archive_path}\nZIP_SHA256={checksum}',flush=True)
    return status


def main(run_batch):
    root = Path(os.environ['CRK_OUTPUT_ROOT']).resolve()
    if not root.is_mount() or not Path('/.dockerenv').is_file():
        raise RuntimeError('Use the host launcher with the mounted output directory')
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    raise SystemExit(run_batch(root))


if __name__=='__main__':
    main(run)

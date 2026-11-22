"""One Ubuntu invocation, two isolated simulators, one verified evidence archive."""

from datetime import datetime, timezone
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import time

from .standby_geometry import CONDITIONS as STANDBY, PREFIX as STANDBY_PREFIX
from .initial_compare import pair
from .initial_contract import CONDITIONS, FULL_CONDITIONS, FULL_PREFIX, check_admission
from .initial_records import load_case, read, write
from .initial_package import archive, file_hash, reports

PROJECT = Path(__file__).resolve().parents[2]
ROOT = Path('/home/legion/AUV-simulation')
CONTAINER = 'crk-ros2-home'


def command(args):
    done = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=90)
    if done.returncode:
        raise RuntimeError(f'{args[0]} exited {done.returncode}: {done.stdout.strip()}')
    return done.stdout.strip()


def reserve(parent, prefix='g6_initial_candidates_pair_'):
    name = (
        prefix
        + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%SZ_')
        + secrets.token_hex(5)
    )
    path = parent / name
    try:
        parent.mkdir(parents=True, exist_ok=True)
        path.mkdir(mode=0o755)
    except PermissionError:
        # Older container runs can leave results/ch3 owned by root. Only take ownership of this new batch.
        command(['sudo', 'mkdir', '-p', '--', str(parent)])
        if not os.access(parent, os.X_OK):
            # Permit the caller to traverse this output directory to its own ZIP.
            # Do not change historical files or grant world permissions.
            command(['sudo', 'setfacl', '-m', f'u:{os.getuid()}:--x', '--', str(parent)])
        command(['sudo', 'mkdir', '-m', '755', '--', str(path)])
        command(['sudo', 'chown', f'{os.getuid()}:{os.getgid()}', '--', str(path)])
    return path


class DockerCases:
    environment = 'Linux/Gazebo'

    def __init__(self, full=False, standby=False):
        self.standby = standby
        self.full = full

    def prepare(self, batch, meta):
        meta['git_sha'] = command(['git', '-C', str(PROJECT), 'rev-parse', 'HEAD'])
        code = PROJECT / 'crk_auv_ch3_v1'
        manifest = code / 'SHA256SUMS'
        for line in manifest.read_text().splitlines():
            expected, name = line.split('  ', 1)
            if file_hash(code / name) != expected:
                raise ValueError('Source manifest mismatch: ' + name)
        meta['adapter_manifest_sha256'] = file_hash(manifest)
        entry = 'run_ch3_far_search_pair400.sh' if self.full else 'run_ch3_initial_candidates_pair.sh'
        if self.standby:
            entry = 'run_ch3_standby_clearance_pair400.sh'
        paths = ['crk_auv_ch3_v1', 'tools/' + entry]
        if command(['git', '-C', str(PROJECT), 'status', '--porcelain', '--', *paths]):
            raise ValueError('Source differs from the reported Git revision')
        format_string = (
            '{{range .Mounts}}{{.Type}}|{{.Source}}|{{.Destination}}|{{.RW}}{{println}}{{end}}'
        )
        mounts = command(
            ['sudo', 'docker', 'inspect', '--format', format_string, CONTAINER]
        ).splitlines()
        for mount in (f'bind|{ROOT}|{ROOT}|true', f'bind|{PROJECT}|/root/AUV-simulation|true'):
            if mount not in mounts:
                raise ValueError('Container mount mismatch: ' + mount)
        command(['sudo', 'docker', 'start', CONTAINER])
        command(
            [
                'sudo',
                'docker',
                'exec',
                '-u',
                '0',
                CONTAINER,
                'bash',
                '-c',
                'test -f /.dockerenv && mountpoint -q "$1" && test -w "$2" && test -r "$3" && test -x /opt/crk-g5-cpu-v1/bin/python',
                'preflight',
                str(ROOT),
                str(batch),
                '/root/AUV-simulation/crk_auv_ch3_v1/run_initial_candidate_linux.sh',
            ]
        )
        meta['verified_mounts'] = mounts

    def run(self, batch, condition):
        wrapper = '/root/AUV-simulation/crk_auv_ch3_v1/run_initial_candidate_linux.sh'
        args = [
            'sudo',
            'docker',
            'exec',
            '-d',
            '-u',
            '0',
            '-e',
            f'CRK_OUTPUT_ROOT={ROOT}',
            '-e',
            f'CRK_OUTPUT_UID={os.getuid()}',
            '-e',
            f'CRK_OUTPUT_GID={os.getgid()}',
            CONTAINER,
            'bash',
            wrapper,
            condition,
            str(batch),
        ]
        # The detached wrapper writes its exit file AFTER the supervisor reaps its children.
        # A terminal Ctrl-C is communicated through the shared CANCEL file.
        done = batch / (condition + '.exit')
        try:
            command(args)
            return self.wait_for_runner(batch, condition, done)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            # A client/daemon communication error can occur after Docker accepted
            # the detached exec. Tell this batch's supervisor to stop before packing.
            cancel = batch / 'CANCEL'
            if not cancel.exists():
                cancel.write_text('fault: runner communication or timeout\n')
            deadline = time.monotonic() + 120
            while not done.is_file() and time.monotonic() < deadline:
                time.sleep(0.5)
            status = done.read_text().strip() if done.is_file() else 'missing; cleanup unverified'
            raise RuntimeError(f'{error}; runner exit after cancellation: {status}') from error

    @staticmethod
    def wait_for_runner(batch, condition, done):
        started = time.monotonic()
        wall_limit = 7500 if condition in FULL_CONDITIONS + STANDBY else 2100
        next_probe = started + 20
        while not done.is_file():
            now = time.monotonic()
            if now - started > 60 and not (batch / (condition + '.runner.log')).exists():
                raise RuntimeError('Detached wrapper did not open its log; simulator startup unverified')
            if now - started > wall_limit and not (batch / 'CANCEL').exists():
                (batch / 'CANCEL').write_text('fault: batch_wall_timeout\n')
            if now - started > wall_limit + 150:
                raise TimeoutError('Container runner status missing; cleanup unverified')
            if now >= next_probe:
                if (
                    command(['sudo', 'docker', 'inspect', '--format', '{{.State.Running}}', CONTAINER])
                    != 'true'
                ):
                    raise RuntimeError('Existing container stopped during ' + condition)
                next_probe = now + 30
            time.sleep(0.5)
        return int(done.read_text().strip())


def execute(batch, engine, meta, log, compare=pair):
    loaded = []
    result = None
    try:
        engine.prepare(batch, meta)
        for condition in meta['conditions']:
            if (batch / 'CANCEL').exists():
                raise InterruptedError('Batch cancelled before ' + condition)
            entry = meta['cases'][condition] = {'status': 'started'}
            log(condition + ' started')
            rc = engine.run(batch, condition)
            directory = batch / condition
            entry.update(exit_code=rc, directory=condition)
            if (directory / 'summary.json').is_file():
                s = read(directory / 'summary.json')
                entry.update(
                    termination_reason=s.get('termination_reason'),
                    native_outcome=s.get('task_outcome'),
                    cleanup=s.get('supervisor_cleanup'),
                    error=s.get('error'),
                )
            if (directory / 'admission.json').is_file():
                admission = read(directory / 'admission.json')
                reference = check_admission(admission, condition)
                entry.update(
                    reference_valid=reference['valid'],
                    reference=reference,
                    scores=admission['scores'],
                    standby=admission['executor']['target_region'],
                )
            if (batch / 'CANCEL').exists():
                cause = (batch / 'CANCEL').read_text().strip()
                if cause.startswith('fault:'):
                    raise RuntimeError(cause)
                raise InterruptedError('Batch cancelled during ' + condition)
            if entry.get('reference_valid') is False:
                entry['status'] = 'invalid_initialization'
                raise ValueError('对照无效，初始化入围、公共输入或评分与登记基准不符。')
            standby = condition in STANDBY
            measured_contact = standby and rc == 1 and entry.get('termination_reason') == 'physical_contact'
            if rc and not measured_contact:
                raise RuntimeError(
                    f"{condition} exit {rc}: {entry.get('termination_reason')} {entry.get('error') or ''}"
                )
            try:
                if standby:
                    from .standby_records import load_standby
                    case = load_standby(directory, condition, rc)
                else:
                    case = load_case(directory, condition)
            except (ValueError, KeyError, OSError) as error:
                if condition in FULL_CONDITIONS + STANDBY:
                    entry.update(status='invalid_evidence', verification_error=str(error))
                raise
            if case['summary']['adapter_manifest_sha256'] != meta['adapter_manifest_sha256']:
                raise ValueError('Host/container adapter manifest mismatch')
            entry.update(status='observed_contact' if measured_contact else 'valid', reference_valid=True)
            if 'mission' in case:
                entry['mission'] = case['mission']
            loaded.append(case)
            log(condition + ' validated, simulator cleanup confirmed')
        result = compare(*loaded)
        if not result['valid']:
            raise ValueError('Initial-state or per-vehicle common-phase comparability insufficient')
        meta.update(status='completed', reason='两组端点、清理与共同前缀已核验。' if standby else '两组均通过工程与可比性核验。')
    except (Exception, KeyboardInterrupt) as error:
        cancel = batch / 'CANCEL'
        user_cancel = cancel.exists() and not cancel.read_text().startswith('fault:')
        if user_cancel or isinstance(error, (InterruptedError, KeyboardInterrupt)):
            meta['status'] = 'interrupted'
        else:
            invalid_reference = any(e.get('reference_valid') is False for e in meta['cases'].values())
            invalid_evidence = any(e['status'] == 'invalid_evidence' for e in meta['cases'].values())
            meta['status'] = 'insufficient' if result is not None or invalid_reference or invalid_evidence else 'failed'
        meta['reason'] = f'{type(error).__name__}: {error}'
        for name, entry in meta['cases'].items():
            if entry['status'] == 'started':
                entry['status'] = 'failed'
                summary = batch / name / 'summary.json'
                if 'termination_reason' not in entry and summary.exists():
                    try:
                        stopped = read(summary)
                        entry.update(
                            termination_reason=stopped.get('termination_reason'),
                            native_outcome=stopped.get('task_outcome'),
                            cleanup=stopped.get('supervisor_cleanup'),
                        )
                    except (OSError, ValueError):
                        entry['summary_readable'] = False
    return result


def export_zip(source, folder, digest):
    folder = Path(folder).resolve(strict=True)
    mount = command(['findmnt', '-n', '-T', str(folder), '-o', 'TARGET'])
    if not folder.is_dir() or not mount or mount == '/':
        raise ValueError('CRK_EXPORT_DIR must be a directory on a mounted share')
    destination = folder / source.name
    if destination.exists():
        if file_hash(destination) != digest:
            raise FileExistsError(str(destination))
        return destination
    with destination.open('xb') as dst:
        try:
            with source.open('rb') as src:
                shutil.copyfileobj(src, dst)
            dst.flush()
            os.fsync(dst.fileno())
        except BaseException:
            destination.unlink()
            raise
    if file_hash(destination) != digest:
        destination.unlink()
        raise ValueError('Shared ZIP hash mismatch')
    return destination


def run_batch(batch, engine, export_dir=None, *, full=False, standby=False):
    compare, report = pair, reports
    if standby:
        from .standby_compare import compare as compare_standby, report as report_standby
        compare, report = compare_standby, report_standby
    elif full:
        from .far_pair import compare_full, report_full
        compare, report = compare_full, report_full
    meta = dict(
        batch_id=batch.name,
        environment=engine.environment,
        git_sha='unavailable',
        status='started',
        cases={},
        reason='',
        conditions=list(STANDBY if standby else FULL_CONDITIONS if full else CONDITIONS),
    )

    def log(msg):
        msg = datetime.now(timezone.utc).isoformat() + ' ' + msg
        print(msg, flush=True)
        with (batch / 'batch.log').open('a', encoding='utf-8') as f:
            print(msg, file=f)

    log('batch ' + batch.name)
    result = execute(batch, engine, meta, log, compare)
    if (batch / 'CANCEL').exists() and meta['status'] == 'completed':
        meta.update(status='interrupted', reason='Batch interrupted before archive publication')
    log(meta['status'] + ' ' + meta['reason'])
    report(batch, meta, result)
    temporary = batch / '.evidence.partial'
    digest = archive(batch, temporary)
    if full and (batch / 'CANCEL').exists() and meta['status'] == 'completed':
        meta.update(status='interrupted', reason='Cancellation received during report or ZIP preparation')
        log(meta['reason'])
        report(batch, meta, result)
        temporary.unlink()
        digest = archive(batch, temporary)
    destination = batch.parent / (batch.name + '.zip')
    temporary.chmod(0o644)
    try:
        os.link(temporary, destination)
    except FileExistsError:
        raise
    except OSError:
        # Some mounted filesystems and Windows sandboxes disallow hard links.
        try:
            out = destination.open('xb')
        except PermissionError:
            if not sys.platform.startswith('linux'):
                raise
            command(['sudo', 'ln', '--', str(temporary), str(destination)])
        else:
            with out, temporary.open('rb') as src:
                shutil.copyfileobj(src, out)
    if file_hash(destination) != digest:
        raise ValueError('Published ZIP hash mismatch')
    temporary.unlink()
    status = meta['status']
    if export_dir:
        try:
            print('EXPORTED_FILE=' + str(export_zip(destination, export_dir, digest)), flush=True)
        except (OSError, ValueError, RuntimeError) as error:
            print('EXPORT_ERROR=' + str(error), flush=True)
            status += '_export_failed'
    print(f'BATCH_STATUS={status}\nUPLOAD_FILE={destination}\nZIP_SHA256={digest}', flush=True)
    return 0 if status == 'completed' else (130 if meta['status'] == 'interrupted' else 1)


def main(*, full=False, standby=False):
    if len(sys.argv) != 1 or not sys.platform.startswith('linux') or Path('/.dockerenv').exists():
        raise SystemExit(
            'Run the pair launcher without arguments on the Ubuntu host'
        )
    prefix = STANDBY_PREFIX if standby else FULL_PREFIX if full else 'g6_initial_candidates_pair_'
    batch = reserve(ROOT / 'results/ch3', prefix)

    def cancel(sig, _frame):
        if not (batch / 'CANCEL').exists():
            (batch / 'CANCEL').write_text(signal.Signals(sig).name + '\n', encoding='utf-8')
            print('Cancellation requested; waiting for evidence and simulator cleanup.', flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, cancel)
    raise SystemExit(run_batch(batch, DockerCases(full, standby), os.environ.get('CRK_EXPORT_DIR'), full=full, standby=standby))


if __name__ == '__main__':
    main()

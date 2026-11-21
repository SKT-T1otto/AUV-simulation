"""G6 Linux supervisor: source-pinned ZERO mission and real Gazebo plant."""
import argparse
import hashlib
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

from .contracts import canonical_hash
from .motion_world import WORLD
from .search_world import fixture, build_search_world, search_bridge_yaml
from .scenario import select_scene
from .native_source import activate_native
from .vehicle import ROOT, load_profile
from .reassignment_contract import CONDITIONS, SCHEMA, BUDGET, BOUNDARY, PREFIX_CRITERIA
from .progress_commit import CONDITIONS as PROGRESS_CONDITIONS, FIXTURE_CONDITIONS, SCHEMA as PROGRESS_SCHEMA, prefix_criteria
from .initial_contract import CONDITIONS as INITIAL_CONDITIONS, SCHEMA as INITIAL_SCHEMA, CRITERIA


def create_assets(output_root, mode='smoke', scene=None, *, diagnostic_condition=None, progress_condition=None,
                  initial_condition=None, pair_directory=None):
    if initial_condition or pair_directory is not None:
        if (initial_condition not in INITIAL_CONDITIONS or mode != 'search' or scene is not None
                or diagnostic_condition or progress_condition or pair_directory is None):
            raise ValueError('Initial pair requires the original search fixture and an explicit batch directory')
        parent = (output_root/'results/ch3').resolve()
        pair_directory = Path(pair_directory)
        if (pair_directory.is_symlink() or pair_directory.resolve().parent != parent
                or not pair_directory.name.startswith('g6_initial_candidates_pair_') or not pair_directory.is_dir()):
            raise ValueError('Invalid initial pair directory')
    if progress_condition is not None and (progress_condition not in PROGRESS_CONDITIONS or mode != 'search'
            or scene is not None or diagnostic_condition is not None):
        raise ValueError('Progress v1 requires explicit condition, original world, and separate entry')
    if diagnostic_condition is not None and (diagnostic_condition not in CONDITIONS or mode != 'search' or scene is not None):
        raise ValueError('Diagnostic v1 requires the original search fixture')
    lock = activate_native()
    scene = fixture(mode) if scene is None else scene
    if type(scene.get('planner_seed')) is not int:
        raise ValueError('Native scene requires explicit planner_seed')
    profile = load_profile()
    xml, truth = build_search_world(scene, profile)
    parent = output_root/'results/ch3'
    parent.mkdir(parents=True, exist_ok=True)
    prefix = 'search_'+mode if diagnostic_condition is None else 'd2_'+diagnostic_condition
    if progress_condition:
        prefix = 'progress_v1_'+progress_condition
    if initial_condition:
        output = pair_directory / initial_condition
        output.mkdir(exist_ok=False)
    else:
        output = Path(tempfile.mkdtemp(prefix=prefix+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_', dir=parent))
    output.chmod(0o755)
    summary = dict(stage='G6', mode=mode, run_kind='zero_gazebo_closed_loop',
        engineering_fixture='engineering' in scene.get('purpose', ''),
        mission_executed=False, map_executed=False, checkpoint_loaded=False, training_executed=False,
        performance_claim=False, model_profile=profile['profile_id'],
        vehicle_profile_sha256=canonical_hash(profile), source_files_sha256=lock['files_sha256'],
        scenario_sha256=canonical_hash(scene), termination_reason='not_started',
        engineering_acceptance_passed=False, g6_search_acceptance_passed=False, g6_capture_acceptance_passed=False,
        motion_started=False, linux_sdf_valid=False, stop_zero_command_sent=False,
        task_dt_s=.2, control_dt_s=.02, physics_dt_s=.002,
        contact_absence_proven=False,
        command_acknowledgement='DDS bridge receipt, not atomic Gazebo plant application')
    if diagnostic_condition is not None:
        summary.update(diagnostic_schema=SCHEMA, diagnostic_condition=diagnostic_condition,
            diagnostic_budget=BUDGET, boundary_step=BOUNDARY, native_max_steps=400,
            prefix_criteria=PREFIX_CRITERIA, diagnostic_valid=False,
            run_kind='d2_reassignment_diagnostic_gazebo',
            adapter_manifest_sha256=hashlib.sha256((ROOT/'SHA256SUMS').read_bytes()).hexdigest())
    if mode == 'capture':
        summary['adapter_manifest_sha256'] = hashlib.sha256((ROOT/'SHA256SUMS').read_bytes()).hexdigest()
    if initial_condition:
        summary.update(diagnostic_schema=INITIAL_SCHEMA, initial_candidate_condition=initial_condition,
            batch_id=pair_directory.name, diagnostic_budget=32, native_max_steps=400, diagnostic_valid=False,
            prefix_criteria=CRITERIA, run_kind='initial_candidate_pair_gazebo',
            adapter_manifest_sha256=hashlib.sha256((ROOT/'SHA256SUMS').read_bytes()).hexdigest())
    if progress_condition:
        summary.update(diagnostic_schema=PROGRESS_SCHEMA, diagnostic_condition=progress_condition,
            diagnostic_budget=BUDGET, boundary_step=5 if progress_condition in FIXTURE_CONDITIONS else 20,
            native_max_steps=400, prefix_criteria=prefix_criteria(progress_condition), diagnostic_valid=False,
            run_kind='progress_fixture_gazebo' if progress_condition in FIXTURE_CONDITIONS else 'progress_search_gazebo',
            adapter_manifest_sha256=hashlib.sha256((ROOT/'SHA256SUMS').read_bytes()).hexdigest(),
            synthetic_proposal_fixture=progress_condition in FIXTURE_CONDITIONS)
    for name, value in [('scenario.json', scene), ('truth_scene.json', truth),
                        ('vehicle_profile.json', profile), ('summary.json', summary)]:
        (output/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'world.sdf').write_text(xml+'\n', encoding='utf-8')
    (output/'bridge.yaml').write_text(search_bridge_yaml(), encoding='utf-8')
    return output


def update_summary(output, **changes):
    path = output/'summary.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    value.update(changes)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--generate-only', action='store_true', help='Only generate assets; no motion validation')
    parser.add_argument('--mode', choices=('smoke', 'capture', 'search'), default='smoke')
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--scenario-id')
    parser.add_argument('--diagnostic-condition', choices=CONDITIONS)
    parser.add_argument('--progress-condition', choices=PROGRESS_CONDITIONS)
    parser.add_argument('--initial-condition', choices=INITIAL_CONDITIONS)
    parser.add_argument('--pair-directory', type=Path)
    args = parser.parse_args()
    if args.scenario_id and not args.manifest:
        parser.error('--scenario-id requires --manifest')
    if args.manifest and args.mode != 'search':
        parser.error('--manifest requires --mode search')
    if args.diagnostic_condition and (args.mode != 'search' or args.manifest):
        parser.error('Diagnostic v1 requires --mode search and the original fixture')
    if args.progress_condition and (args.mode != 'search' or args.manifest or args.diagnostic_condition):
        parser.error('Progress v1 requires --mode search and no manifest/other diagnostic')
    output_root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if Path('/.dockerenv').is_file() and not output_root.is_mount():
        raise RuntimeError('Linux host output root is not mounted')
    scene = None if args.manifest is None else select_scene(json.loads(args.manifest.read_text(encoding='utf-8')), args.scenario_id)
    output = create_assets(output_root, args.mode, scene, diagnostic_condition=args.diagnostic_condition,
                           progress_condition=args.progress_condition, initial_condition=args.initial_condition,
                           pair_directory=args.pair_directory)
    print(f'G6 results: {output}', flush=True)
    if args.generate_only:
        update_summary(output, run_kind='g6_assets_only')
        return
    processes, streams, reader = [], [], None
    process_logs = []
    failed_processes = []
    lock_stream = None
    failure_changes = None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    result = 1
    try:
        import fcntl
        from ament_index_python.packages import get_package_prefix
        lock_path = output_root/'.container/g4_motion.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock_stream = lock_path.open('a')
        fcntl.flock(lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.initial_condition and (output.parent/'CANCEL').exists():
            raise KeyboardInterrupt
        with (output/'sdf_validation.log').open('w') as log:
            valid = subprocess.run(['gz', 'sdf', '-k', str(output/'world.sdf')], stdout=log,
                                   stderr=subprocess.STDOUT, timeout=30)
        update_summary(output, linux_sdf_valid=valid.returncode == 0)
        if valid.returncode != 0:
            raise RuntimeError('SDF validation failed; see sdf_validation.log')
        bridge = str(Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge')
        partition = output.parent.name+'_'+output.name if args.initial_condition else output.name
        env = {**os.environ, 'GZ_PARTITION': 'crk_g6_'+partition, 'ROS_DOMAIN_ID': '76',
               'ROS_LOCALHOST_ONLY': '1', 'ROS_LOG_DIR': str(output/'ros_logs'),
               'CRK_G6_REALTIME_VIEW': '0' if args.headless else '1'}
        def launch(command, filename):
            stream = (output/filename).open('w', encoding='utf-8')
            streams.append(stream)
            process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True, cwd=ROOT)
            processes.append(process)
            process_logs.append((process, filename))
            return process
        gz_args = ['gz', 'sim', '-v', '3']+(['-s'] if args.headless else [])+[str(output/'world.sdf')]
        launch(gz_args, 'gazebo.log')  # Deliberately starts paused; controller owns stepping.
        launch([bridge, '--ros-args', '-p', f'config_file:={output/"bridge.yaml"}'], 'bridge.log')
        launch([bridge, f'/world/{WORLD}/control@ros_gz_interfaces/srv/ControlWorld'], 'world_control_bridge.log')
        launch([bridge, f'/world/{WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose'], 'target_marker_bridge.log')
        controller_log = (output/'controller.log').open('w', encoding='utf-8')
        streams.append(controller_log)
        module = 'crk_ch3.reassignment_ros' if args.diagnostic_condition else 'crk_ch3.search_ros'
        if args.progress_condition:
            module = 'crk_ch3.progress_fixture_ros' if args.progress_condition in FIXTURE_CONDITIONS else 'crk_ch3.reassignment_ros'
        options = ['--initial-condition', args.initial_condition] if args.initial_condition else []
        controller = subprocess.Popen([sys.executable, '-u', '-m', module, '--output', str(output), *options],
                                      env=env, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, bufsize=1, start_new_session=True)
        processes.append(controller)
        process_logs.append((controller, 'controller.log'))
        def forward():
            for line in controller.stdout:
                controller_log.write(line)
                controller_log.flush()
                print(line, end='', flush=True)
        reader = threading.Thread(target=forward, daemon=True)
        reader.start()
        deadline = time.monotonic()+1800
        while controller.poll() is None:
            if args.initial_condition and (output.parent/'CANCEL').exists():
                raise KeyboardInterrupt
            if time.monotonic() >= deadline:
                raise TimeoutError('G6 wall-time limit exceeded (1800 seconds)')
            failed_processes = [dict(log=filename, exit_code=p.returncode)
                                for p, filename in process_logs[:-1] if p.poll() is not None]
            if failed_processes:
                detail = ', '.join(f"{p['log']} (exit {p['exit_code']})" for p in failed_processes)
                raise RuntimeError(f'Process exited before the motion controller completed: {detail}')
            time.sleep(.1)
        result = controller.returncode
        metadata = json.loads((output/'summary.json').read_text(encoding='utf-8'))
        if result == 0 and not metadata.get('engineering_acceptance_passed'):
            raise RuntimeError('Controller exited without a valid acceptance summary')
        if result != 0 and metadata.get('termination_reason') == 'not_started':
            raise RuntimeError('Controller exited before reporting; see controller.log')
    except KeyboardInterrupt:
        result = 130
        failure_changes = dict(termination_reason='interrupted', engineering_acceptance_passed=False, g6_search_acceptance_passed=False, g6_capture_acceptance_passed=False)
    except Exception as error:
        result = 1
        print(f'G6 launch failed: {error}', file=sys.stderr, flush=True)
        failure_changes = dict(termination_reason='launch_or_infrastructure_failure', error=str(error),
                               failed_processes=failed_processes,
                               engineering_acceptance_passed=False, g6_search_acceptance_passed=False, g6_capture_acceptance_passed=False)
    finally:
        # Each child owns a process group; stop controller before bridges/world.
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
                except ProcessLookupError:
                    process.wait()
        if reader is not None:
            reader.join(timeout=3)
        if args.initial_condition:
            # A group leader can exit while its simulator child is winding down.
            # Check only the session groups created in this run, under the motion lock.
            def live(group):
                for stat in Path('/proc').glob('[0-9]*/stat'):
                    try:
                        fields = stat.read_text().rsplit(')', 1)[1].split()
                        if fields[0] != 'Z' and int(fields[2]) == group:
                            return True
                    except (OSError, IndexError, ValueError):
                        continue
                return False
            groups = [p.pid for p in processes]
            for sig in (signal.SIGTERM, signal.SIGKILL):
                pending = [g for g in groups if live(g)]
                if not pending:
                    break
                for group in pending:
                    try:
                        os.killpg(group, sig)
                    except ProcessLookupError:
                        pass
                end = time.monotonic()+3
                while time.monotonic()<end and any(live(g) for g in pending):
                    time.sleep(.1)
            if reader is not None:
                reader.join(timeout=3)
            clean = (reader is None or not reader.is_alive()) and all(
                p.poll() is not None and not live(p.pid) for p in processes)
            update_summary(output, supervisor_cleanup=dict(verified=clean, groups=groups,
                exit_codes=[p.returncode for p in processes]))
            if not clean:
                result = 1
                failure_changes = dict(termination_reason='subprocess_cleanup_failure', engineering_acceptance_passed=False)
        for stream in streams:
            stream.close()
        if lock_stream is not None:
            lock_stream.close()
        if failure_changes is not None:
            if args.diagnostic_condition or args.progress_condition or args.initial_condition:
                metadata = json.loads((output/'summary.json').read_text(encoding='utf-8'))
                failure_changes.update(diagnostic_valid=False,
                    failure_tags=metadata.get('failure_tags', [])+['supervisor_'+failure_changes['termination_reason']])
            update_summary(output, **failure_changes)
        # Only report processes that had already exited before cleanup. Bridge
        # SIGINTs sent above must not be mistaken for the cause of a failure.
        for failed in failed_processes:
            lines = (output/failed['log']).read_text(encoding='utf-8', errors='replace').splitlines()
            print(f"Last 35 lines of {output/failed['log']}:", file=sys.stderr)
            print('\n'.join(lines[-35:]), file=sys.stderr, flush=True)
        if os.geteuid() == 0 and os.environ.get('CRK_OUTPUT_UID', '').isdigit() and os.environ.get('CRK_OUTPUT_GID', '').isdigit():
            uid, gid = int(os.environ['CRK_OUTPUT_UID']), int(os.environ['CRK_OUTPUT_GID'])
            for directory, folders, files in os.walk(output, followlinks=False):
                os.chown(directory, uid, gid, follow_symlinks=False)
                for name in files:
                    os.chown(Path(directory)/name, uid, gid, follow_symlinks=False)
        print(f'Inspect {output}/summary.json', flush=True)
    raise SystemExit(result)


if __name__ == '__main__':
    main()

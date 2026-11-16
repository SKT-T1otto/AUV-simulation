"""G4 Linux launch supervisor: new output dir, SDF validation, bridges and control."""
import argparse
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
from .motion_world import WORLD, bridge_yaml, build_motion_world
from .vehicle import ROOT, load_profile


def create_assets(output_root):
    scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text(encoding='utf-8'))
    profile = load_profile()
    xml, truth = build_motion_world(scene, profile)
    parent = output_root/'results/ch3'
    parent.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='motion_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_', dir=parent))
    output.chmod(0o755)
    summary = {'stage': 'G4', 'run_kind': 'four_vehicle_motion_control', 'engineering_fixture': True,
               'mission_executed': False, 'map_executed': False, 'checkpoint_loaded': False,
               'performance_claim': False, 'model_profile': profile['profile_id'],
               'added_mass_mode': profile['added_mass_mode'],
               'model_status': profile['model_status'], 'vehicle_profile_sha256': canonical_hash(profile),
               'termination_reason': 'not_started', 'engineering_acceptance_passed': False,
               'motion_started': False, 'linux_sdf_valid': False,
               'control_dt_s': .02, 'physics_dt_s': .002, 'state_tolerance_ns': 2_000_000,
               'command_acknowledgement': 'DDS bridge receipt, not atomic Gazebo plant application'}
    for name, value in [('scenario.json', scene), ('truth_scene.json', truth),
                        ('vehicle_profile.json', profile), ('summary.json', summary)]:
        (output/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'world.sdf').write_text(xml+'\n', encoding='utf-8')
    (output/'bridge.yaml').write_text(bridge_yaml(), encoding='utf-8')
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
    args = parser.parse_args()
    output_root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if Path('/.dockerenv').is_file() and not output_root.is_mount():
        raise RuntimeError('Linux host output root is not mounted')
    output = create_assets(output_root)
    print(f'G4 results: {output}', flush=True)
    if args.generate_only:
        update_summary(output, run_kind='g4_assets_only')
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
        with (output/'sdf_validation.log').open('w') as log:
            valid = subprocess.run(['gz', 'sdf', '-k', str(output/'world.sdf')], stdout=log,
                                   stderr=subprocess.STDOUT, timeout=30)
        update_summary(output, linux_sdf_valid=valid.returncode == 0)
        if valid.returncode != 0:
            raise RuntimeError('SDF validation failed; see sdf_validation.log')
        bridge = str(Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge')
        env = {**os.environ, 'GZ_PARTITION': 'crk_g4_'+output.name, 'ROS_DOMAIN_ID': '75',
               'ROS_LOCALHOST_ONLY': '1', 'ROS_LOG_DIR': str(output/'ros_logs'),
               'CRK_G4_REALTIME_VIEW': '0' if args.headless else '1'}
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
        controller_log = (output/'controller.log').open('w', encoding='utf-8')
        streams.append(controller_log)
        controller = subprocess.Popen([sys.executable, '-u', '-m', 'crk_ch3.motion_ros', '--output', str(output)],
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
        deadline = time.monotonic()+600
        while controller.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError('G4 wall-time limit exceeded (600 seconds)')
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
        failure_changes = dict(termination_reason='interrupted', engineering_acceptance_passed=False)
    except Exception as error:
        result = 1
        print(f'G4 launch failed: {error}', file=sys.stderr, flush=True)
        failure_changes = dict(termination_reason='launch_or_infrastructure_failure', error=str(error),
                               failed_processes=failed_processes,
                               engineering_acceptance_passed=False)
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
        for stream in streams:
            stream.close()
        if lock_stream is not None:
            lock_stream.close()
        if failure_changes is not None:
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

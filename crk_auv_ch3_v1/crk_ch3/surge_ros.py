"""Real paused Gazebo only. One vehicle, eight Newton commands and body odometry."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from .envelope_ros import state_data
from . import heave
from .search_ros import SearchNode
from .search_transport import SearchTransport
from .surge import DT, LEVELS, PHASES, SurgeDrive, write_json
from .vehicle import load_profile, roll_pitch


class SingleHullNode(SearchNode):
    vehicle_count = 1


def collect(node, output, summary, stream):
    plan = json.loads((output/'program.json').read_text())
    profile = load_profile()
    if profile != json.loads((output/'vehicle_profile.json').read_text()):
        raise ValueError('Physical profile differs from recorded assets')
    vertical = summary.get('run_kind') == heave.KIND
    if vertical:
        schedule = heave.phases(summary['case'], profile)
        if (plan['case'] != summary['case'] or plan['dt_s'] != DT or plan['phases'] !=
                [dict(name=p, ticks=n, requested_body_z_N=f) for p,n,f in schedule]):
            raise ValueError('Unexpected heave program')
        force_field = 'requested_heave_N'
    else:
        level = plan['level_N']
        if level not in LEVELS or level != summary['level_N']:
            raise ValueError('Unexpected surge level')
        schedule = [(p, n, level if p == 'drive' else 0.) for p,n in PHASES]
        force_field = 'requested_surge_N'
    states = node.ready()
    epoch = node.confirm_pause()
    start = np.array(plan['initial_world_position'])
    if abs(states[0].stamp_ns-epoch)>2_000_000 or np.linalg.norm(states[0].position-start)>.02:
        raise RuntimeError('Initial odometry or position mismatch')
    control = SurgeDrive(profile)
    guard = SearchTransport(node, states, profile, record=lambda _: None)
    summary.update(task_epoch_ns=epoch, initial_state=state_data(states))
    if vertical and (np.linalg.norm(states[0].velocity_body) > .02 or
                     np.max(np.abs(np.rad2deg(roll_pitch(states[0].quaternion)))) > 1.):
        raise RuntimeError('Heave initial state exceeds near-rest or level tolerance')
    for phase, count, target in schedule:
        if phase == 'drive' and np.linalg.norm(states[0].velocity_body) > .02:
            raise RuntimeError('Vehicle not at rest before excitation (>0.02 m/s)')
        for k in range(count):
            node.check()
            guard.validate(states)
            before = state_data(states)
            if vertical:
                force, diagnostic = control.apply_force([0., 0., target], states[0])
            else:
                force, diagnostic = control.command(target, states[0], start[2])
            node.publish([force])
            states, requested_ns = node.step(10)
            tick = summary['completed_control_steps']+1
            if (node.clock_ns != epoch+tick*20_000_000 or requested_ns != node.clock_ns or
                    states[0].stamp_ns <= before['stamp_ns'] or abs(states[0].stamp_ns-node.clock_ns)>2_000_000):
                raise RuntimeError('Control clock / odometry timestamp mismatch')
            row = dict(tick=tick, phase=phase, local_tick=k+1, requested_clock_ns=requested_ns,
                       clock_ns=node.clock_ns, before=before, after=state_data(states),
                       forces_N=[force.tolist()], diagnostics=[diagnostic])
            row[force_field] = target
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
            summary['completed_control_steps'] = tick
            guard.validate(states)
            node.check()
        node.confirm_pause(node.clock_ns)
    summary['max_roll_pitch_deg'] = guard.max_tilt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    root = Path(os.environ['CRK_OUTPUT_ROOT']).resolve()
    if not root.is_mount() or not output.is_relative_to(root/'results/ch3'):
        raise ValueError('Expected mounted host results')
    summary = json.loads((output/'summary.json').read_text())
    summary.update(completed_control_steps=0, stop_confirmed=False)
    node = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with (output/'control.jsonl').open('x') as stream, (output/'physical_contacts.jsonl').open('x') as contacts:
        def record(event):
            contacts.write(json.dumps(event)+'\n')
            contacts.flush()
        try:
            node = SingleHullNode(record)
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, lambda *_: setattr(node, 'stopping', True))
            collect(node, output, summary, stream)
            summary['termination_reason'] = 'completed'
        except (Exception, KeyboardInterrupt) as error:
            summary.update(termination_reason='interrupted' if isinstance(error, KeyboardInterrupt)
                           else 'engineering_failure', error=repr(error))
            (output/'traceback.log').write_text(traceback.format_exc())
        finally:
            if node is not None:
                try:
                    node.stop_commands()
                    until = time.monotonic()+.15
                    while time.monotonic()<until:
                        rclpy.spin_once(node, timeout_sec=.01)
                    node.confirm_pause()
                    summary['stop_confirmed'] = True
                except (Exception, KeyboardInterrupt) as error:
                    summary['stop_error'] = repr(error)
                summary['contact_events_observed'] = node.contact_events
                if node.contact_failure:
                    summary['termination_reason'] = 'physical_contact'
                node.destroy_node()
            rclpy.shutdown()
            expected_steps = (sum(n for _, n, _ in heave.phases(summary['case'], load_profile()))
                              if summary.get('run_kind') == heave.KIND else sum(n for _, n in PHASES))
            summary['engineering_acceptance_passed'] = bool(
                summary['termination_reason']=='completed' and summary['linux_sdf_valid']
                and summary['stop_confirmed'] and summary['completed_control_steps']==expected_steps
                and summary.get('contact_events_observed') == 0)
            write_json(output/'summary.json', summary)
    raise SystemExit(0 if summary['engineering_acceptance_passed'] else 1)


if __name__ == '__main__':
    main()

"""Linux real-Gazebo envelope runner; no task, policy, pose setting or state integration."""
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
from .search_ros import SearchNode
from .search_transport import SearchTransport
from .search_control import AccelerationControl
from .vehicle import load_profile, rotation
from .envelope_protocol import reference


def state_data(states):
    return dict(stamp_ns=states[0].stamp_ns,
        positions_world=[list(s.position) for s in states],
        velocity_world=[(rotation(s.quaternion)@s.velocity_body).tolist() for s in states],
        quaternion_xyzw=[list(s.quaternion) for s in states],
        velocity_body=[list(s.velocity_body) for s in states], rates_body=[list(s.rates_body) for s in states])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(root/'results/ch3') or not root.is_mount():
        raise ValueError('Use the mounted Linux host output root')
    summary = json.loads((output/'summary.json').read_text())
    plan = json.loads((output/'program.json').read_text())
    profile = load_profile()
    if profile != json.loads((output/'vehicle_profile.json').read_text()):
        raise ValueError('Recorded profile differs from active controller profile')
    node = guard = None
    completed = 0
    max_force = 0.
    last_states = None
    reason = 'startup_failure'
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with (output/'control.jsonl').open('x', encoding='utf-8') as log, \
            (output/'physical_contacts.jsonl').open('x', encoding='utf-8') as contacts:
        try:
            def contact_record(row):
                contacts.write(json.dumps(row)+'\n'); contacts.flush()
            node = SearchNode(contact_record)
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, lambda *_: setattr(node, 'stopping', True))
            states = node.ready()
            last_states = states
            epoch = node.confirm_pause()
            if any(abs(s.stamp_ns-epoch)>2_000_000 for s in states):
                raise RuntimeError('Startup odometry/paused clock mismatch')
            origins = np.array([s.position for s in states])
            truth = json.loads((output/'truth_scene.json').read_text())
            if np.max(np.linalg.norm(origins-np.array(truth['agents']),axis=1))>.02:
                raise RuntimeError('Warmup position differs from scene by >0.02m')
            guard = SearchTransport(node, states, profile, record=lambda _: None)
            controllers = [AccelerationControl(profile) for _ in range(4)]
            summary.update(task_epoch_ns=epoch, initial_state=state_data(states))
            last_target = origins.copy()
            for phase in plan['phases']:
                segment_start = last_target.copy()
                if phase['mode']=='waypoint':
                    last_target = origins+phase['delta']
                node.get_logger().info(f"Envelope phase={phase['name']} ticks={phase['ticks']}")
                for local_tick in range(phase['ticks']):
                    started = time.monotonic()
                    node.check(); guard.validate(states)
                    before = state_data(states)
                    forces, diagnostics, targets, desired_velocities = [], [], [], []
                    for i,s in enumerate(states):
                        velocity = rotation(s.quaternion)@s.velocity_body
                        request, target, desired_v = reference(phase,origins[i],np.array(s.position),velocity,profile)
                        force, diagnostic = controllers[i].command(request,s.quaternion,s.velocity_body,s.rates_body)
                        forces.append(force); diagnostics.append(diagnostic)
                        targets.append(None if target is None else target.tolist())
                        desired_velocities.append(None if desired_v is None else desired_v.tolist())
                    node.publish(forces)
                    states, requested_ns = node.step(10)
                    last_states = states
                    max_force = max(max_force,float(np.max(np.abs(forces))))
                    completed += 1
                    after = state_data(states)
                    dt = (after['stamp_ns']-before['stamp_ns'])/1e9
                    if dt<=0 or node.clock_ns != epoch+completed*20_000_000:
                        raise RuntimeError('Non-increasing state or control clock drift')
                    guard.validate(states)
                    estimate = (np.array(after['velocity_world'])-before['velocity_world'])/dt
                    row = dict(tick=completed, phase=phase['name'], local_tick=local_tick+1,
                        elapsed_s=completed*.02, requested_clock_ns=requested_ns, clock_ns=node.clock_ns,
                        before=before, after=after, forces_N=np.array(forces).tolist(),diagnostics=diagnostics,
                        estimated_acceleration_world=estimate.tolist(), targets_world=targets,
                        desired_velocity_world=desired_velocities,
                        segment_start_world=segment_start.tolist() if phase['mode']=='waypoint' else None)
                    log.write(json.dumps(row,allow_nan=False)+'\n')
                    if completed%50==0:
                        log.flush()
                        node.get_logger().info(f'Envelope t={completed*.02:.1f}s / {plan["total_ticks"]*.02:.1f}s')
                    if os.environ.get('CRK_G4_REALTIME_VIEW')=='1':
                        time.sleep(max(0,.02-(time.monotonic()-started)))
                node.confirm_pause(epoch+completed*20_000_000)
            reason='completed'
        except KeyboardInterrupt:
            reason='interrupted'
        except Exception as error:
            reason='engineering_failure'
            summary['error']=str(error)
            (output/'error_traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
            print(traceback.format_exc(),flush=True)
        finally:
            if node is not None:
                try:
                    until=time.monotonic()+.15
                    while time.monotonic()<until:
                        rclpy.spin_once(node,timeout_sec=.01)
                    node.publish(np.zeros((4,8)))
                    node.stop_commands()
                    node.confirm_pause()
                    summary['stop_zero_command_sent']=True
                except Exception as error:
                    summary['stop_error']=str(error)
                summary['contact_events_observed']=node.contact_events
                if node.contact_failure:
                    reason='physical_contact'
                node.destroy_node()
            rclpy.shutdown()
            summary.update(termination_reason=reason, motion_started=completed>0,
                completed_control_steps=completed,simulated_duration_s=completed*.02,
                max_abs_thrust_N=max_force,
                final_speed_m_s=[] if last_states is None else [float(np.linalg.norm(s.velocity_body)) for s in last_states],
                max_roll_pitch_deg=0 if guard is None else guard.max_tilt,
                contact_absence_proven=False,
                engineering_acceptance_passed=bool(reason=='completed' and completed==plan['total_ticks']
                    and summary.get('linux_sdf_valid') and summary.get('stop_zero_command_sent')
                    and 'stop_error' not in summary and summary.get('contact_events_observed')==0))
            (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    # Read-only analysis, separate report artifacts; preserve even partial runs.
    from .envelope_report import write_report
    try:
        write_report(output)
    except Exception:
        summary.update(engineering_acceptance_passed=False,report_error=traceback.format_exc())
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(f'Envelope {reason}; engineering={summary["engineering_acceptance_passed"]}; {output}',flush=True)
    raise SystemExit(0 if summary['engineering_acceptance_passed'] else 1)


if __name__=='__main__':
    main()

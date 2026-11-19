"""Opt-in 32-step diagnostic using the existing paused Gazebo transport."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback
import numpy as np
import torch
from . import search_ros as ros
from .mission_runtime import ExternalMissionRuntime
from .mission_parity import normalized
from .search_transport import SearchTransport, PlantFailure
from .reassignment_gate import ReassignmentGate, DiagnosticInvalid
from .reassignment_contract import SCHEMA, CONDITIONS, BUDGET, PREFIX_CRITERIA
from .scenario import Frame
from .vehicle import load_profile
from .reassignment_report import write_run_report
from .progress_commit import SCHEMA as PROGRESS_SCHEMA, SEARCH_CONDITIONS


def map_counts(native):
    module = native.map_module
    return dict(scan_count=int(native.map_sensor_scan_count), map_revision=int(module.map_revision),
                known_free_cells=int(np.count_nonzero(module.known_free_mask)),
                known_occupied_cells=int(np.count_nonzero(module.known_occupied_mask)),
                unknown_cells=int(np.count_nonzero(module.unknown_mask)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(root/'results/ch3'):
        raise ValueError('Diagnostic requires the mounted host results directory')
    summary = json.loads((output/'summary.json').read_text(encoding='utf-8'))
    progress = summary.get('diagnostic_schema') == PROGRESS_SCHEMA
    allowed = SEARCH_CONDITIONS if progress else CONDITIONS
    schema = PROGRESS_SCHEMA if progress else SCHEMA
    if (summary.get('diagnostic_schema') != schema or summary.get('diagnostic_condition') not in allowed
            or summary.get('diagnostic_budget') != BUDGET or summary.get('prefix_criteria') != PREFIX_CRITERIA):
        raise ValueError('Explicit versioned diagnostic assets are required')
    scene = json.loads((output/'scenario.json').read_text(encoding='utf-8'))
    torch.set_num_threads(1)
    ros.rclpy.init(signal_handler_options=ros.SignalHandlerOptions.NO)
    streams = {}
    def record(name, data):
        if name not in streams:
            streams[name] = (output/name).open('x', encoding='utf-8')
        streams[name].write(json.dumps(normalized(data), allow_nan=False)+'\n')
        streams[name].flush()
    node = plant = runtime = gate = None
    reason, completed, event_cursor = 'startup_failure', 0, 0
    failures = []
    native_commit_steps = []
    try:
        node = ros.SearchNode(lambda event: record('physical_contacts.jsonl', event))
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(node, 'stopping', True))
        states = node.ready()
        warmup = node.clock_ns
        expected = np.array([Frame().position(p) for p in scene['initial_agent_positions']])
        if np.max(np.linalg.norm(np.array([s.position for s in states])-expected, axis=1)) > .02:
            raise PlantFailure('Spawn/warmup differs from scene by more than 0.02 m')
        runtime = ExternalMissionRuntime(scene, lambda request: plant(request))
        if runtime.config['max_steps'] != 400:
            raise DiagnosticInvalid('Native task max_steps must remain 400')
        if progress and summary['diagnostic_condition'] == 'search_progress32':
            from .progress_gate import ProgressCommitGate
            gate = ProgressCommitGate(runtime, summary['diagnostic_condition'], record)
        else:
            condition = 'baseline32' if progress else summary['diagnostic_condition']
            gate = ReassignmentGate(runtime, condition, record)
            if progress:
                gate.condition = summary['diagnostic_condition']
        native = runtime.env.unwrapped
        for name, data in [('resolved_config.json', runtime.config),
                           ('role_specs.json', runtime.env.get_public_vehicle_specs())]:
            (output/name).write_text(json.dumps(normalized(data), indent=2)+'\n', encoding='utf-8')
        native.enable_public_event_capture('g6', capacity=2048)
        native._public_event_journal.reset()
        node.show_target(native.target_state.position)
        epoch = node.confirm_pause()
        summary.update(warmup_clock_ns=warmup, task_epoch_ns=epoch,
                       startup_clock_delta_ns=epoch-warmup, startup_world_status=node.world_status)
        if not 0 <= epoch-warmup <= 2_000_000 or any(abs(s.stamp_ns-epoch) > 2_000_000 for s in states):
            raise PlantFailure('Startup clock/odometry drift')
        plant = SearchTransport(node, states, load_profile(),
            record=lambda row: record('control.jsonl', dict(row, body_states=plant.states)),
            realtime=os.environ.get('CRK_G6_REALTIME_VIEW') == '1')
        gate.after_advance()
        record('initial.jsonl', dict(step=0, physical_time_s=0., body_states=states,
            observations=runtime.observations, task=runtime.env.get_task_state(),
            guidance=runtime.guidance, allocation=runtime.controller.inner.current_allocation,
            tracker=gate.trackers(), maps=map_counts(native)))
        for _ in range(BUDGET):
            node.check()
            before = normalized(dict(guidance=runtime.guidance, tracker=gate.trackers(),
                allocation=runtime.controller.inner.current_allocation, body_states=plant.states,
                maps=map_counts(native)))
            record('step_start.jsonl', dict(step=runtime.step+1, applied_before_motion=before))
            result = runtime.advance()
            completed += 1
            if progress:
                native_result = normalized(gate.native_result) if hasattr(gate, 'native_result') else gate.last['applied_controller_result']
                if native_result['replanned']:
                    native_commit_steps.append(runtime.step)
            record('transitions.jsonl', dict(step=runtime.step,
                physical_time_s=(node.clock_ns-epoch)/1e9,
                condition=gate.condition, boundary_active=gate.active,
                applied_before_motion=before, transition=result,
                task=runtime.env.get_task_state(), outcome=native.get_episode_result(),
                raw_reward_components=native.last_reward_components,
                reward_breakdown=runtime.env.last_reward_breakdown,
                guidance=runtime.guidance, allocation=runtime.controller.inner.current_allocation,
                tracker_after=gate.trackers(), body_states=plant.states,
                maps=map_counts(native), safety_status=runtime.bridge.last_safety_status,
                task_collisions=native.last_collision_records,
                target_truth_audit_only=native.target_state.to_payload()))
            for event in native.read_public_events(event_cursor):
                record('events.jsonl', event)
                event_cursor = event.sequence
            if completed == 1 or completed % 10 == 0 or completed == BUDGET or runtime.terminal:
                record('maps.jsonl', dict(step=runtime.step, audit_only_not_policy_truth=True,
                    fields={k: getattr(native.map_module, k) for k in ('occupancy_probability',
                        'coverage', 'belief_map', 'known_free_mask', 'known_occupied_mask', 'unknown_mask')}))
            if gate.anchor_reference is not None and completed == 20:
                (output/'retained_reference.json').write_text(
                    json.dumps(gate.anchor_reference, indent=2)+'\n', encoding='utf-8')
            # All safety logic remains enabled; a changed safety route invalidates B.
            gate.after_advance()
            node.show_target(native.target_state.position)
            node.confirm_pause(plant.epoch_ns+runtime.step*200_000_000)
            node.check()
            node.get_logger().info(f'D2 diagnostic {gate.condition} step={runtime.step}/32')
        reason = 'diagnostic_budget_completed'
    except KeyboardInterrupt:
        reason = 'interrupted'
    except ros.PhysicalContact as error:
        reason, summary['error'] = 'physical_contact', str(error)
    except DiagnosticInvalid as error:
        reason, summary['error'] = 'diagnostic_invalid', str(error)
    except Exception as error:
        reason, summary['error'] = 'engineering_failure', str(error)
        (output/'error_traceback.txt').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        if reason != 'diagnostic_budget_completed':
            failures.append(reason)
        if node is not None:
            try:
                until = time.monotonic()+.15
                while time.monotonic() < until:
                    ros.rclpy.spin_once(node, timeout_sec=.01)
                if node.contact_failure:
                    reason = 'physical_contact'
                    failures.append(reason)
            except Exception as error:
                failures.append('late_callback_failure')
                summary['late_callback_error'] = str(error)
            try:
                node.publish(np.zeros((4, 8)))
                node.stop_commands()
                summary['stop_zero_command_sent'] = True
            except Exception as error:
                failures.append('stop_failure')
                summary['stop_error'] = str(error)
            summary.update(failure_clock_ns=node.clock_ns, final_world_status=node.world_status)
        if runtime is not None:
            native = runtime.env.unwrapped
            summary.update(task_outcome=normalized(native.get_episode_result()),
                task_success=bool(native.mission_complete), final_maps=map_counts(native))
            if gate is not None:
                if progress:
                    queue_events = getattr(getattr(gate, 'queue', None), 'events', [])
                    summary.update(pending_before_stop=len(getattr(getattr(gate, 'queue', None), 'pending', {})),
                        search_intent_commits=getattr(getattr(gate, 'queue', None), 'committed', 0),
                        delayed_search_commits=sum(e['kind']=='COMMITTED' and e['committed_step']>e['proposed_step'] for e in queue_events),
                        adapter_commit_steps=getattr(gate, 'adapter_commit_steps', []),
                        native_commit_steps=native_commit_steps)
                    if hasattr(gate, 'on_stop'):
                        gate.on_stop(reason)
                record('final_state.jsonl', dict(step=runtime.step, completed_steps=completed,
                    guidance=runtime.guidance, tracker=gate.trackers(),
                    allocation=runtime.controller.inner.current_allocation,
                    task=runtime.env.get_task_state(), safety_status=runtime.bridge.last_safety_status,
                    collisions=native.last_collision_records,
                    raw_policy_decision=gate.policy_decision, raw_allocator_proposals=gate.raw_allocator,
                    stabilization=gate.proposals, reason=reason, failure_tags=failures))
            try:
                runtime.close()
            except Exception as error:
                failures.append('runtime_close_failure')
                summary['runtime_close_error'] = str(error)
        if plant is not None:
            summary.update(control_steps=plant.control_steps,
                actual_simulated_duration_s=plant.measured_duration_s,
                max_roll_pitch_deg=plant.max_tilt, max_abs_thrust_N=plant.max_force,
                max_finished_hold_error_m=plant.max_hold_error)
        summary.update(termination_reason=reason, completed_task_steps=completed,
            failure_tags=sorted(set(failures)), mission_executed=completed > 0,
            map_executed=completed > 0, motion_started=plant is not None and plant.control_steps > 0,
            contact_events_observed=0 if node is None else node.contact_events,
            contact_absence_proven=False, performance_claim=False,
            g6_search_acceptance_passed=False, g6_capture_acceptance_passed=False,
            search_commits_deferred=0 if gate is None else gate.deferred_commits,
            physical_stop='zero command with bridge receipt and world pause; no settling rollout')
        summary['diagnostic_valid'] = (reason == 'diagnostic_budget_completed' and completed == BUDGET
            and not failures and summary.get('stop_zero_command_sent', False)
            and summary.get('linux_sdf_valid', False))
        summary['engineering_acceptance_passed'] = summary['diagnostic_valid']
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        for stream in streams.values():
            stream.close()
        try:
            if progress:
                from .progress_report import write_run_report as progress_report
                progress_report(output)
            else:
                write_run_report(output)
        except Exception as error:
            summary.update(report_error=str(error), diagnostic_valid=False, engineering_acceptance_passed=False)
            summary['failure_tags'].append('report_failure')
            (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        if node is not None:
            node.destroy_node()
        ros.rclpy.shutdown()
        print(f"D2 {reason}; diagnostic_valid={summary['diagnostic_valid']}; "
              f"failures={summary['failure_tags']}; results={output}", flush=True)
    raise SystemExit(0 if summary['diagnostic_valid'] else 1)


if __name__ == '__main__':
    main()

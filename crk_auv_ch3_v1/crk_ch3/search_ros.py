"""G6 ZERO mission on the real Gazebo plant; ROS/Linux entry point."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback
import numpy as np
import torch
import rclpy
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Contacts, WorldStatistics
from ros_gz_interfaces.srv import SetEntityPose

from .motion_ros import MotionNode
from .motion_world import WORLD
from .mission_runtime import ExternalMissionRuntime
from .mission_parity import normalized
from .search_transport import SearchTransport, PlantFailure
from .search_outcome import acceptance
from .initial_contract import CONDITIONS as INITIAL_CONDITIONS, BUDGET as INITIAL_BUDGET
from .initial_contract import FULL_CONDITIONS, HORIZON
from .capture_evidence import capture_review, capture_sample
from .scenario import Frame
from .standby_geometry import CONDITIONS as STANDBY, hulls
from .vehicle import load_profile


class PhysicalContact(PlantFailure):
    pass


class SearchNode(MotionNode):
    def __init__(self, contact_record):
        super().__init__()
        self.contact_record = contact_record
        self.contact_events = 0
        self.contact_failure = None
        self.world_status = None
        self.stats_sequence = 0
        self.stats_sub = self.create_subscription(WorldStatistics, '/g6/world_stats', self.on_stats, 10)
        self.contact_subs = [self.create_subscription(Contacts, f'/auv_{i}/contacts',
            lambda msg, agent=i: self.on_contact(agent, msg), 100) for i in range(4)]
        self.marker_client = self.create_client(SetEntityPose, f'/world/{WORLD}/set_pose')

    def on_stats(self, message):
        self.stats_sequence += 1
        self.world_status = dict(sim_ns=message.sim_time.sec*1_000_000_000+message.sim_time.nanosec,
                                 paused=bool(message.paused), iterations=int(message.iterations))

    def confirm_pause(self, expected_ns=None):
        # ControlWorld replies on queue receipt; observe a fresh paused-world
        # status as well. Do not infer completed pause from the service reply.
        sequence = self.stats_sequence
        minimum_ns = self.clock_ns
        self.control_world()
        self.wait(lambda: self.stats_sequence > sequence and self.world_status['paused']
                  and self.world_status['sim_ns'] >= minimum_ns
                  and self.clock_ns == self.world_status['sim_ns'], 10., 'Paused-world status timeout')
        status_ns = self.world_status['sim_ns']
        if self.clock_ns != status_ns or (expected_ns is not None and status_ns != expected_ns):
            raise PlantFailure(f'Paused-world clock mismatch: expected_ns={expected_ns}, '
                               f'clock_ns={self.clock_ns}, world_status={self.world_status}')
        return status_ns

    def on_contact(self, agent, message):
        if not message.contacts:
            return
        self.contact_events += 1
        self.contact_record(dict(agent=agent, received_clock_ns=self.clock_ns,
            contacts=[dict(collision1=c.collision1.name, collision2=c.collision2.name,
                           entity1_id=c.collision1.id, entity2_id=c.collision2.id,
                           depths=list(c.depths)) for c in message.contacts]))
        self.contact_failure = 'Gazebo hull contact event observed; see physical_contacts.jsonl'

    def check(self):
        super().check()
        if self.contact_failure:
            raise PhysicalContact(self.contact_failure)

    def ready(self):
        states = self.start()
        self.wait(lambda: all(s.get_publisher_count() > 0 for s in self.contact_subs),
                  30., 'Missing contact ROS bridge publisher')
        self.wait(self.marker_client.service_is_ready, 30., 'Missing target visualization service bridge')
        return states

    def show_target(self, position):
        # Only this non-colliding privileged marker can be moved by this client.
        request = SetEntityPose.Request()
        request.entity.name = 'target_truth_marker'
        request.entity.type = 2  # MODEL
        request.pose.position.x, request.pose.position.y, request.pose.position.z = Frame().position(list(position))
        request.pose.orientation.w = 1.
        future = self.marker_client.call_async(request)
        self.wait(future.done, 10., 'Target marker update timeout')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Target marker service rejected update')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--initial-condition', choices=INITIAL_CONDITIONS + FULL_CONDITIONS + STANDBY)
    args = parser.parse_args()
    output = args.output.resolve()
    root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(root/'results/ch3'):
        raise ValueError('G6 requires the configured Linux host results directory')
    summary = json.loads((output/'summary.json').read_text(encoding='utf-8'))
    scene = json.loads((output/'scenario.json').read_text(encoding='utf-8'))
    initial = args.initial_condition
    full_pair = initial in FULL_CONDITIONS + STANDBY
    if initial != summary.get('initial_candidate_condition'):
        raise ValueError('Initial candidate condition differs from supervisor assets')
    torch.set_num_threads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    streams = {}
    def record(name, data):
        if name not in streams:
            streams[name] = (output/name).open('x', encoding='utf-8')
        streams[name].write(json.dumps(normalized(data), allow_nan=False)+'\n')
        streams[name].flush()
    node = plant = runtime = None
    reason, completed, last_map_step = 'startup_failure', 0, -1
    search_displacement_before_found = 0.
    capture_rows = []
    capture_mode = summary['mode'] == 'capture'
    try:
        node = SearchNode(lambda event: record('physical_contacts.jsonl', event))
        if initial in STANDBY:
            # Retain empty-message timestamps too, when the bridge actually supplies them.
            contact_ticks = [set() for _ in range(4)]
            native_contact = getattr(node, 'on_contact', None)
            if native_contact is not None:
                def observe_contact(agent, message):
                    stamp = message.header.stamp
                    ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                    contact_ticks[agent].add(ns)
                    node.contact_source_ns = ns
                    native_contact(agent, message)
                node.on_contact = observe_contact
                native_record = node.contact_record
                node.contact_record = lambda event: native_record(dict(event, source_ns=node.contact_source_ns))
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(node, 'stopping', True))
        states = node.ready()
        warmup_clock_ns = node.clock_ns
        summary['warmup_clock_ns'] = warmup_clock_ns
        expected = np.array([Frame().position(p) for p in scene['initial_agent_positions']])
        if np.max(np.linalg.norm(np.array([s.position for s in states])-expected, axis=1)) > .02:
            raise PlantFailure('Spawn/warmup state differs from scene by more than 0.02 m')
        # Native reset/planning does not call the motion callback. Bind the
        # actual plant only once ROS startup and marker initialization finish.
        if initial:
            from .initial_candidates import InitialCandidateRuntime
            def diagnostic_motion(request):
                record('motion_input.jsonl', runtime.request_evidence(request))
                return plant(request)
            if initial in STANDBY:
                from .standby_runtime import StandbyRuntime
                runtime = StandbyRuntime(scene, diagnostic_motion, initial, hulls(output/'world.sdf'))
            else:
                runtime = InitialCandidateRuntime(scene, diagnostic_motion, initial)
            (output/'admission.json').write_text(json.dumps(normalized(runtime.admission),
                indent=2, allow_nan=False)+'\n', encoding='utf-8')
            if not runtime.admission['reference_check']['valid']:
                summary['comparison_invalid_reason'] = 'initial_reference_mismatch'
                raise PlantFailure('Initial candidates, scores or public inputs disagree with the registered reference')
        else:
            runtime = ExternalMissionRuntime(scene, lambda request: plant(request))
        native = runtime.env.unwrapped
        if full_pair:
            summary['native_detect_radii_m'] = [
                float(r.item()) + float(native.detect_eps_bias) for r in native._sensor_range[:3]]
        (output/'resolved_config.json').write_text(json.dumps(normalized(runtime.config), indent=2)+'\n', encoding='utf-8')
        (output/'role_specs.json').write_text(json.dumps(normalized(runtime.env.get_public_vehicle_specs()), indent=2)+'\n', encoding='utf-8')
        native.enable_public_event_capture('g6', capacity=2048)
        native._public_event_journal.reset()
        node.show_target(native.target_state.position)
        epoch_ns = node.confirm_pause()
        startup_delta = epoch_ns-warmup_clock_ns
        summary.update(task_epoch_ns=epoch_ns, startup_clock_delta_ns=startup_delta,
                       startup_world_status=node.world_status)
        if not 0 <= startup_delta <= 2_000_000:
            raise PlantFailure(f'World advanced during startup: warmup_ns={warmup_clock_ns}, '
                               f'epoch_ns={epoch_ns}, delta_ns={startup_delta}; do not press Play')
        # The existing odometry contract admits at most one physics-substep lag.
        if any(abs(s.stamp_ns-epoch_ns) > 2_000_000 for s in states):
            raise PlantFailure('Startup odometry does not match the confirmed paused epoch')
        def contact_publishers():
            return [s.get_publisher_count() for s in node.contact_subs]
        if initial in STANDBY:
            summary['epoch_contact_publishers'] = contact_publishers()
        def record_control(row):
            if initial in STANDBY:
                row['contact_publishers'] = contact_publishers()
            if initial:
                row['body_states'] = plant.states
            record('control.jsonl', row)
        plant = SearchTransport(node, states, load_profile(), record=record_control,
                                realtime=os.environ.get('CRK_G6_REALTIME_VIEW') == '1')
        node.get_logger().info(f'G6 synchronized startup: warmup_ns={warmup_clock_ns}, epoch_ns={epoch_ns}')
        event_cursor = 0
        budget = 16 if summary['mode'] == 'smoke' else 400
        if initial:
            if native.max_steps != HORIZON:
                raise PlantFailure('Native task horizon differs from 400')
            budget = native.max_steps if full_pair else INITIAL_BUDGET
        for _ in range(budget):
            node.check()
            if initial:
                record('step_start.jsonl', dict(step=runtime.step+1, clock_ns=node.clock_ns,
                    installed=runtime.installed_state(), body_states=plant.states))
            if capture_mode or full_pair:
                executor_start = Frame().inverse_position(plant.states[native.executor_idx].position)
                target_start = native.target_state.position.tolist()
            result = runtime.advance()
            completed += 1
            task = runtime.env.get_task_state()
            if initial:
                if initial in STANDBY:
                    decision = dict(step=runtime.step, reason='native_terminal') if runtime.terminal else runtime.boundary
                    record('standby_checks.jsonl', dict(boundary=decision, terminal=runtime.terminal, clock_ns=node.clock_ns))
                record('step_end.jsonl', dict(step=runtime.step, clock_ns=node.clock_ns,
                    installed=runtime.installed_state(), body_states=plant.states))
            if capture_mode or full_pair:
                motion = result['motion']
                sample = capture_sample(runtime.step, task, native, executor_start,
                    motion['measured_positions_native'][native.executor_idx], target_start,
                    motion['feedback_stamp_ns'])
                record('capture_trace.jsonl', sample)
                capture_rows.append(sample)
            if not task.target_found or runtime.found_step == runtime.step:
                search_displacement_before_found = max(search_displacement_before_found,
                    float(np.max(np.linalg.norm(np.array([s.position for s in plant.states[:3]])
                        -plant.initial_positions[:3], axis=1))))
            record('transitions.jsonl', dict(step=runtime.step, transition=result, task=task,
                outcome=native.get_episode_result(), raw_reward_components=native.last_reward_components,
                reward_breakdown=runtime.env.last_reward_breakdown,
                guidance=runtime.guidance, target_truth_audit_only=native.target_state.to_payload()))
            events = native.read_public_events(event_cursor)
            for event in events:
                record('events.jsonl', event)
                event_cursor = event.sequence
            if completed == 1 or completed % 10 == 0 or runtime.terminal:
                record('maps.jsonl', dict(step=runtime.step, audit_only_not_policy_truth=True,
                    fields={k: getattr(native.map_module, k) for k in ('occupancy_logodds',
                    'occupancy_probability', 'coverage', 'claim_count', 'pheromone', 'belief_map',
                    'known_free_mask', 'known_occupied_mask', 'unknown_mask')}))
                last_map_step = completed
            node.show_target(native.target_state.position)
            node.confirm_pause(plant.epoch_ns+runtime.step*200_000_000)
            node.check()
            if completed == 1 or completed % 5 == 0 or runtime.terminal:
                node.get_logger().info(f'G6 step={runtime.step}/400 found={task.target_found} '
                    f'handoff={task.executor_knows_target} hold={native.capture_hold_counter_max} '
                    f'outcome={native.episode_outcome.termination_reason}')
            if runtime.terminal:
                break
        reason = 'task_terminal' if runtime.terminal else 'smoke_completed'
        if full_pair and not runtime.terminal:
            reason = 'native_terminal_missing_at_horizon'
        elif initial and not runtime.terminal:
            reason = 'diagnostic_budget_completed'
    except KeyboardInterrupt:
        reason = 'interrupted'
    except PhysicalContact as error:
        reason, summary['error'] = 'physical_contact', str(error)
    except Exception as error:
        reason, summary['error'] = 'engineering_failure', str(error)
        if node is not None:
            summary['failure_clock_ns'] = node.clock_ns
            summary['failure_world_status'] = node.world_status
        (output/'error_traceback.txt').write_text(traceback.format_exc(), encoding='utf-8')
        print(traceback.format_exc(), flush=True)
    finally:
        # Drain late event callbacks while physics stays paused. The event-only
        # sensor has no empty-message heartbeat; absence of events is not proof.
        if node is not None:
            until = time.monotonic()+.15
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            if initial in STANDBY:
                summary['reason_before_contact_drain'] = reason
            if node.contact_failure:
                reason = 'physical_contact'
            try:
                node.publish(np.zeros((4, 8)))  # Require bridge receipt before pause.
                node.stop_commands()
                summary['stop_zero_command_sent'] = True
            except Exception as error:
                summary['stop_error'] = str(error)
        if initial in STANDBY and node is not None:
            from .standby_geometry import tick_ranges
            summary['sensor_source_ranges_ns'] = [tick_ranges(ticks) for ticks in contact_ticks]
        if runtime is not None:
            native = runtime.env.unwrapped
            if last_map_step != completed:
                record('maps.jsonl', dict(step=runtime.step, fields={k: getattr(native.map_module, k)
                    for k in ('occupancy_probability', 'coverage', 'belief_map')}))
            summary.update(task_outcome=normalized(native.get_episode_result()),
                task_success=bool(native.mission_complete),
                found_step=runtime.found_step, handoff_decision_step=runtime.handoff_decision_step,
                search_displacement_before_found_m=search_displacement_before_found,
                capture_hold_max=native.capture_hold_counter_max,
                map_sensor_scan_count=native.map_sensor_scan_count,
                map_revision=int(native.map_module.map_revision))
            runtime.close()
        if plant is not None:
            summary.update(control_steps=plant.control_steps, actual_simulated_duration_s=plant.measured_duration_s,
                max_roll_pitch_deg=plant.max_tilt, max_abs_thrust_N=plant.max_force,
                max_finished_hold_error_m=plant.max_hold_error,
                acceleration_error_norm_rms_m_s2=(plant.acceleration_error_squared/max(1, plant.acceleration_error_samples))**.5,
                acceleration_estimate_method='finite difference of filtered odometry world velocity',
                governed_active_sample_fraction=plant.governed_samples/max(1, plant.active_samples),
                max_displacement_m=plant.max_displacement.tolist(),
                final_speed_m_s=[float(np.linalg.norm(s.velocity_body)) for s in plant.states])
        summary.update(termination_reason=reason, completed_task_steps=completed,
            mission_executed=completed > 0, map_executed=completed > 0,
            motion_started=plant is not None and plant.control_steps > 0,
            contact_events_observed=0 if node is None else node.contact_events,
            contact_absence_proven=False,
            contact_monitor_status='ROS subscriptions plus event-only hull sensors; asynchronous coverage',
            physical_stop='zero command with bridge receipt and world pause; no settling rollout')
        if capture_mode or (full_pair and summary.get('task_success')):
            summary['capture_evidence'] = capture_review(capture_rows, summary)
        summary.update(acceptance(summary))
        if initial in INITIAL_CONDITIONS:
            completed_budget = reason == 'diagnostic_budget_completed' and completed == INITIAL_BUDGET
            # Apply the usual engineering predicates to a completed diagnostic budget.
            healthy = acceptance(dict(summary, termination_reason='smoke_completed'))['engineering_acceptance_passed']
            summary.update(diagnostic_valid=bool(completed_budget and healthy),
                engineering_acceptance_passed=bool(completed_budget and healthy),
                g6_search_acceptance_passed=False, g6_capture_acceptance_passed=False)
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        for stream in streams.values():
            stream.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print(f"G6 {reason}; engineering={summary['engineering_acceptance_passed']}; "
              f"task_success={summary.get('task_success', False)}; results={output}", flush=True)
    if capture_mode:
        print(f"Capture acceptance={summary['g6_capture_acceptance_passed']}; "
              f"evidence={summary['capture_evidence']}", flush=True)
    passed = summary['g6_capture_acceptance_passed'] if capture_mode else summary['engineering_acceptance_passed']
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()

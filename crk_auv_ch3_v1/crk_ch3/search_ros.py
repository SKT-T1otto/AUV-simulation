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
from .scenario import Frame
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
    args = parser.parse_args()
    output = args.output.resolve()
    root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(root/'results/ch3'):
        raise ValueError('G6 requires the configured Linux host results directory')
    summary = json.loads((output/'summary.json').read_text(encoding='utf-8'))
    scene = json.loads((output/'scenario.json').read_text(encoding='utf-8'))
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
    try:
        node = SearchNode(lambda event: record('physical_contacts.jsonl', event))
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
        runtime = ExternalMissionRuntime(scene, lambda request: plant(request))
        native = runtime.env.unwrapped
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
        plant = SearchTransport(node, states, load_profile(),
                                record=lambda row: record('control.jsonl', row),
                                realtime=os.environ.get('CRK_G6_REALTIME_VIEW') == '1')
        node.get_logger().info(f'G6 synchronized startup: warmup_ns={warmup_clock_ns}, epoch_ns={epoch_ns}')
        event_cursor = 0
        budget = 16 if summary['mode'] == 'smoke' else 400
        for _ in range(budget):
            node.check()
            result = runtime.advance()
            completed += 1
            task = runtime.env.get_task_state()
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
            if node.contact_failure:
                reason = 'physical_contact'
            try:
                node.publish(np.zeros((4, 8)))  # Require bridge receipt before pause.
                node.stop_commands()
                summary['stop_zero_command_sent'] = True
            except Exception as error:
                summary['stop_error'] = str(error)
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
        summary.update(acceptance(summary))
        (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        for stream in streams.values():
            stream.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print(f"G6 {reason}; engineering={summary['engineering_acceptance_passed']}; "
              f"task_success={summary.get('task_success', False)}; results={output}", flush=True)
    raise SystemExit(0 if summary['engineering_acceptance_passed'] else 1)


if __name__ == '__main__':
    main()

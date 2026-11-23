"""G4 ROS client: Newton commands only; Gazebo is the sole state integrator."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld
from std_msgs.msg import Float64

from .motion_world import WORLD
from .motion_protocol import BodyState, TeamStateBuffer, build_program, evaluate_phase
from .vehicle import VehicleControl, load_profile, roll_pitch

CONTROL_NS = 20_000_000
PHYSICS_NS = 2_000_000


class MotionNode(Node):
    vehicle_count = 4
    retry_startup_pause_timeout = False

    def __init__(self):
        super().__init__('crk_g4_four_vehicle_motion')
        self.clock_ns = -1
        self.error = None
        self.stopping = False
        self.buffer = TeamStateBuffer(self.vehicle_count)
        self.client = self.create_client(ControlWorld, f'/world/{WORLD}/control')
        self.subscriptions_ = [self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)]
        self.publishers_ = []
        for i in range(self.vehicle_count):
            self.subscriptions_.append(self.create_subscription(Odometry, f'/auv_{i}/odom',
                                      lambda message, agent=i: self.on_state(agent, message), qos_profile_sensor_data))
            self.publishers_.append([self.create_publisher(Float64, f'/auv_{i}/thruster_{j}/cmd_thrust', 1)
                                     for j in range(8)])

    def on_clock(self, message):
        stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
        if self.clock_ns >= 0 and stamp < self.clock_ns:
            self.error = 'Simulation clock moved backwards'
        self.clock_ns = stamp

    def on_state(self, agent, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        try:
            state = BodyState(message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                              (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), (v.x, v.y, v.z), (w.x, w.y, w.z))
            self.buffer.push(agent, state, message.header.frame_id, message.child_frame_id)
        except ValueError as error:
            self.error = str(error)

    def check(self):
        if self.stopping:
            raise KeyboardInterrupt
        if self.error:
            raise RuntimeError(self.error)

    def wait(self, predicate, timeout, description):
        until = time.monotonic()+timeout
        while not predicate():
            self.check()
            if time.monotonic() >= until:
                stamps = [max(h, default=-1) for h in self.buffer.by_agent]
                raise TimeoutError(f'{description}; clock_ns={self.clock_ns}; latest_odom_ns={stamps}')
            rclpy.spin_once(self, timeout_sec=.01)

    def control_world(self, steps=0, *, timeout=10.):
        request = ControlWorld.Request()
        request.world_control.pause = True
        request.world_control.multi_step = int(steps)
        future = self.client.call_async(request)
        try:
            self.wait(future.done, timeout, 'World control service timeout')
        except TimeoutError:
            self.client.remove_pending_request(future)
            raise
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('Gazebo rejected world control request')

    def publish(self, values, acknowledge=True):
        values = np.asarray(values, dtype=float)
        if values.shape != (self.vehicle_count, 8) or not np.isfinite(values).all():
            raise ValueError('Invalid vehicle force command')
        for i in range(self.vehicle_count):
            for j in range(8):
                self.publishers_[i][j].publish(Float64(data=float(values[i, j])))
        if acknowledge:
            # DDS acknowledgement establishes bridge receipt, not atomic Gazebo application.
            for row in self.publishers_:
                for publisher in row:
                    if not publisher.wait_for_all_acked(Duration(seconds=.5)):
                        raise TimeoutError('Thrust DDS acknowledgement timeout')

    def step(self, steps):
        previous = max(0, self.clock_ns)
        target = previous+steps*PHYSICS_NS
        self.control_world(steps)
        result = None
        def ready():
            nonlocal result
            if self.clock_ns > target+PHYSICS_NS:
                raise RuntimeError('World advanced outside lockstep; do not press Play')
            # Do not consume a one-substep-old sample until the whole requested
            # step batch has completed; otherwise the next request could overlap it.
            if self.clock_ns >= target:
                result = self.buffer.take(target, PHYSICS_NS)
            return result is not None
        self.wait(ready, 10., f'Clock/four-state barrier timeout at {target}; clock={self.clock_ns}')
        return result, target

    def start(self):
        self.wait(self.client.service_is_ready, 45., 'World service not available')
        # The ROS service may be discoverable before its Gazebo counterpart is ready.
        until = time.monotonic()+45
        while True:
            try:
                self.control_world(timeout=min(10., max(0., until-time.monotonic())))
                break
            except TimeoutError:
                if not self.retry_startup_pause_timeout or time.monotonic() >= until:
                    raise
                self.get_logger().warning('Retrying the initial pause (zero physics steps)')
            except RuntimeError:
                if time.monotonic() > until:
                    raise
                time.sleep(.1)
        self.wait(lambda: all(p.get_subscription_count() for row in self.publishers_ for p in row),
                  30., 'Missing one or more ROS thrust bridge subscribers')
        self.publish(np.zeros((self.vehicle_count, 8)))
        # Flush the odometry finite-difference initialization transient before control.
        return self.step(20)[0]

    def stop_commands(self):
        self.publish(np.zeros((self.vehicle_count, 8)), acknowledge=False)
        request = ControlWorld.Request()
        request.world_control.pause = True
        future = self.client.call_async(request)
        until = time.monotonic()+2
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.01)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Could not confirm paused world during stop')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output_root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(output_root/'results/ch3'):
        raise ValueError('G4 output must be under the configured Linux host result root')
    metadata = json.loads((output/'summary.json').read_text(encoding='utf-8'))
    profile = load_profile()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    reason, results, last_states = 'startup_failure', [], None
    max_tilt = 0.
    samples = 0
    max_force = 0.
    try:
        node = MotionNode()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(node, 'stopping', True))
        states = node.start()
        initial = np.array([s.position for s in states])
        expected = np.array(json.loads((output/'truth_scene.json').read_text())['agents'])
        if np.max(np.linalg.norm(initial-expected, axis=1)) > .1:
            raise RuntimeError('Initial odometry does not match the generated world')
        program = build_program(expected)
        controllers = [VehicleControl(profile) for _ in range(4)]
        start_clock = node.clock_ns
        tick = 0
        with (output/'trajectories.csv').open('x', newline='') as stream, \
                (output/'commands.jsonl').open('x', encoding='utf-8') as commands, \
                (output/'events.jsonl').open('x', encoding='utf-8') as events:
            writer = csv.writer(stream)
            writer.writerow(['sim_elapsed_s', 'phase', 'agent_id', 'state_ns', 'clock_ns',
                             'x', 'y', 'z', 'target_x', 'target_y', 'target_z', 'error_m', 'speed_m_s',
                             'roll_deg', 'pitch_deg', 'max_abs_thrust_N'])
            for phase in program:
                starts = np.array([s.position for s in states])
                idle_drift = np.zeros(4)
                node.get_logger().info(f'Phase: {phase.name}; moving={list(phase.moving)}')
                for local_tick in range(phase.ticks):
                    wall_started = time.monotonic()
                    node.check()
                    forces, diagnostics = [], []
                    for i, state in enumerate(states):
                        force, diagnostic = controllers[i].track(phase.targets[i], state.position,
                              state.quaternion, state.velocity_body, state.rates_body, .02)
                        forces.append(force)
                        diagnostics.append(diagnostic)
                    node.publish(forces)
                    states, requested_ns = node.step(10)
                    last_states = states
                    tick += 1
                    positions = np.array([s.position for s in states])
                    idle_drift = np.maximum(idle_drift, np.linalg.norm(positions-starts, axis=1))
                    if np.any(np.abs(positions[:, :2]) > 11) or np.any(positions[:, 2] < -9.2) or np.any(positions[:, 2] > -.5):
                        raise RuntimeError('Vehicle left the G4 engineering safety bounds')
                    commands.write(json.dumps({'tick': tick, 'phase': phase.name, 'requested_clock_ns': requested_ns,
                                               'state_ns': states[0].stamp_ns, 'received_clock_ns': node.clock_ns,
                                               'forces_N': np.asarray(forces).tolist(), 'diagnostics': diagnostics}, allow_nan=False)+'\n')
                    for i, state in enumerate(states):
                        tilt = np.rad2deg(roll_pitch(state.quaternion))
                        max_tilt = max(max_tilt, float(np.max(np.abs(tilt))))
                        max_force = max(max_force, float(np.max(np.abs(forces[i]))))
                        writer.writerow([(node.clock_ns-start_clock)/1e9, phase.name, i, state.stamp_ns, node.clock_ns,
                                         *state.position, *phase.targets[i], np.linalg.norm(positions[i]-phase.targets[i]),
                                         np.linalg.norm(state.velocity_body), *tilt, np.max(np.abs(forces[i]))])
                    samples += 1
                    if tick % 50 == 0:
                        errors = np.linalg.norm(positions-phase.targets, axis=1)
                        node.get_logger().info(f't={tick*.02:.1f}s {phase.name} errors_m={errors.round(3).tolist()}')
                        stream.flush()
                        commands.flush()
                    if os.environ.get('CRK_G4_REALTIME_VIEW') == '1':
                        time.sleep(max(0., .02-(time.monotonic()-wall_started)))
                result = evaluate_phase(phase, starts, positions, idle_drift)
                results.append(result)
                events.write(json.dumps(result, allow_nan=False)+'\n')
                events.flush()
            reason = 'completed'
    except KeyboardInterrupt:
        reason = 'interrupted'
    except Exception as error:
        reason = 'engineering_failure'
        metadata['error'] = str(error)
        if node is not None:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            try:
                node.stop_commands()
            except Exception as error:
                metadata['stop_error'] = str(error)
            node.destroy_node()
        rclpy.shutdown()
        speeds = [] if last_states is None else [float(np.linalg.norm(s.velocity_body)) for s in last_states]
        passed = (reason == 'completed' and len(results) == 7 and all(x['passed'] for x in results)
                  and max_tilt < 10 and speeds and max(speeds) < .1 and 'stop_error' not in metadata)
        metadata.update(termination_reason=reason, motion_started=samples > 0,
                        engineering_acceptance_passed=bool(passed), phases=results,
                        completed_control_steps=samples, simulated_duration_s=samples*.02,
                        max_roll_pitch_deg=max_tilt, max_abs_thrust_N=max_force, final_speed_m_s=speeds,
                        stop_zero_command_sent=node is not None and 'stop_error' not in metadata)
        (output/'summary.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(f'G4 result: {reason}; acceptance={bool(passed)}; results={output}', flush=True)
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()

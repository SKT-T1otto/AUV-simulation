"""G6 callback binding one 0.2 s task request to ten Gazebo control batches."""
import time
import numpy as np
from .scenario import Frame
from .vehicle import rotation, roll_pitch, VehicleControl
from .search_control import AccelerationControl
from .mission_transport import feedback_from_gazebo


class PlantFailure(RuntimeError):
    pass


class SearchTransport:
    def __init__(self, node, states, profile, *, record, realtime=False):
        self.node, self.states, self.profile = node, states, profile
        self.frame = Frame()
        self.epoch_ns = node.clock_ns
        self.record, self.realtime = record, realtime
        self.controllers = [AccelerationControl(profile) for _ in range(4)]
        self.holders = [VehicleControl(profile) for _ in range(4)]
        self.holding = [False]*4
        self.control_steps = 0
        self.max_tilt, self.max_force, self.max_hold_error = 0., 0., 0.
        self.initial_positions = np.array([s.position for s in states])
        self.max_displacement = np.zeros(4)
        self.measured_duration_s = 0.
        self.acceleration_error_squared = 0.
        self.acceleration_error_samples = 0
        self.governed_samples = 0
        self.active_samples = 0

    def validate(self, states):
        for state in states:
            native = self.frame.inverse_position(state.position)
            if any(x < 0 or x > limit for x, limit in zip(native, (20, 20, 8))):
                raise PlantFailure('Measured vehicle left native task bounds; no pose clamping sent to Gazebo')
            tilt = float(np.max(np.abs(np.rad2deg(roll_pitch(state.quaternion)))))
            self.max_tilt = max(self.max_tilt, tilt)
            if tilt > 20 or np.linalg.norm(state.velocity_body) > 2:
                raise PlantFailure('Measured tilt/speed exceeded the G6 engineering envelope')

    def __call__(self, request):
        expected = self.epoch_ns+request.start_ns
        if self.node.clock_ns != expected:
            raise PlantFailure('Task/Gazebo clock mismatch before control request')
        for substep in range(10):
            started = time.monotonic()
            self.node.check()
            self.validate(self.states)
            forces, diagnostics = [], []
            previous_states = self.states
            for i, state in enumerate(self.states):
                if request.blocked[i]:
                    if not self.holding[i]:
                        self.holders[i].force = self.controllers[i].force.copy()
                        self.holding[i] = True
                    target = self.frame.position(request.positions[i].tolist())
                    force, diagnostic = self.holders[i].track(target, state.position, state.quaternion,
                                                              state.velocity_body, state.rates_body, .02)
                    error = float(np.linalg.norm(np.array(state.position)-target))
                    self.max_hold_error = max(self.max_hold_error, error)
                    if error > .2:
                        raise PlantFailure('Finished-agent measured drift exceeds 0.2 m; frozen task pose is not physical feedback')
                    diagnostic['mode'] = 'finished_agent_hold'
                else:
                    acceleration = self.frame.rotate(request.acceleration_world[i].tolist())
                    force, diagnostic = self.controllers[i].command(acceleration, state.quaternion,
                                                                    state.velocity_body, state.rates_body)
                    diagnostic['mode'] = 'source_acceleration'
                    self.active_samples += 1
                    self.governed_samples += int(not np.allclose(diagnostic['requested_acceleration_world'],
                                                                 diagnostic['governed_acceleration_world'], atol=1e-9))
                forces.append(force)
                diagnostics.append(diagnostic)
            self.node.publish(forces)
            self.states, target_ns = self.node.step(10)
            self.control_steps += 1
            self.measured_duration_s = (self.node.clock_ns-self.epoch_ns)/1e9
            self.validate(self.states)
            for i in range(4):
                if request.blocked[i]:
                    error = float(np.linalg.norm(np.asarray(self.states[i].position)-self.frame.position(request.positions[i].tolist())))
                    self.max_hold_error = max(self.max_hold_error, error)
                    if error > .2:
                        raise PlantFailure('Finished-agent drift exceeds 0.2 m at measured endpoint')
            self.max_force = max(self.max_force, float(np.max(np.abs(forces))))
            positions = np.array([s.position for s in self.states])
            self.max_displacement = np.maximum(self.max_displacement,
                                                np.linalg.norm(positions-self.initial_positions, axis=1))
            measured_acceleration = []
            for i, state in enumerate(self.states):
                previous = previous_states[i]
                dt = (state.stamp_ns-previous.stamp_ns)/1e9
                if dt <= 0:
                    raise PlantFailure('Non-increasing odometry timestamp')
                estimate = (rotation(state.quaternion)@np.asarray(state.velocity_body)
                            -rotation(previous.quaternion)@np.asarray(previous.velocity_body))/dt
                measured_acceleration.append(estimate.tolist())
                if not request.blocked[i]:
                    error = estimate-np.asarray(diagnostics[i]['governed_acceleration_world'])
                    self.acceleration_error_squared += float(np.dot(error, error))
                    self.acceleration_error_samples += 1
            self.record(dict(task_step=request.task_step, substep=substep+1, clock_ns=self.node.clock_ns,
                requested_clock_ns=target_ns, state_ns=self.states[0].stamp_ns,
                positions_world=positions.tolist(), velocity_world=[(rotation(s.quaternion)@np.array(s.velocity_body)).tolist()
                    for s in self.states], estimated_acceleration_world=measured_acceleration,
                forces_N=np.asarray(forces).tolist(), diagnostics=diagnostics))
            if self.realtime:
                time.sleep(max(0, .02-(time.monotonic()-started)))
        return feedback_from_gazebo(request, self.states, epoch_ns=self.epoch_ns, frame=self.frame)

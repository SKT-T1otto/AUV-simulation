"""G4C: shared-time state barrier and deterministic engineering motion program."""
from dataclasses import dataclass
import numpy as np
from .vehicle import rotation


@dataclass(frozen=True)
class BodyState:
    stamp_ns: int
    position: tuple
    quaternion: tuple
    velocity_body: tuple
    rates_body: tuple

    def __post_init__(self):
        if type(self.stamp_ns) is not int or self.stamp_ns < 0:
            raise ValueError('Invalid state timestamp')
        for name, size in [('position', 3), ('quaternion', 4), ('velocity_body', 3), ('rates_body', 3)]:
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (size,) or not np.isfinite(value).all():
                raise ValueError('Invalid state vector: '+name)
            object.__setattr__(self, name, tuple(float(x) for x in value))
        rotation(self.quaternion)


class TeamStateBuffer:
    def __init__(self, vehicle_count=4):
        self.by_agent = [dict() for _ in range(vehicle_count)]
        self.last_accepted_ns = -1

    def push(self, agent, state, frame_id='map', child_frame=None):
        if type(agent) is not int or agent not in range(len(self.by_agent)):
            raise ValueError('Unknown agent ID')
        if frame_id != 'map' or child_frame != f'auv_{agent}/base_link':
            raise ValueError('Odometry frame mismatch')
        if state.stamp_ns <= self.last_accepted_ns:
            return
        history = self.by_agent[agent]
        old = history.get(state.stamp_ns)
        if old is not None and old != state:
            raise ValueError('Conflicting state at the same agent/timestamp')
        history[state.stamp_ns] = state
        for key in sorted(history)[:-16]:
            del history[key]

    def take(self, target_ns, tolerance_ns):
        common = set.intersection(*(set(h) for h in self.by_agent))
        valid = [s for s in common if s > self.last_accepted_ns and abs(s-target_ns) <= tolerance_ns]
        if not valid:
            return None
        stamp = min(valid, key=lambda s: (abs(s-target_ns), -s))
        result = [history[stamp] for history in self.by_agent]
        self.last_accepted_ns = stamp
        return result


@dataclass
class MotionPhase:
    name: str
    ticks: int
    targets: np.ndarray
    moving: tuple
    delta: np.ndarray


def build_program(initial_positions, control_dt=.02):
    initial = np.asarray(initial_positions, dtype=float)
    if initial.shape != (4, 3) or not np.isfinite(initial).all():
        raise ValueError('Expected four initial world positions')
    if control_dt != .02:
        raise ValueError('G4 engineering program is fixed at 50 Hz')
    target = initial.copy()
    phases = [MotionPhase('initial_hold', 50, target.copy(), (), np.zeros((4, 3)))]
    displacements = [[1, 0, 0], [0, 1, 0], [0, 0, .75], [.75, -.5, 0]]
    for agent, displacement in enumerate(displacements):
        delta = np.zeros((4, 3))
        delta[agent] = displacement
        target += delta
        phases.append(MotionPhase(f'isolation_auv_{agent}', 300, target.copy(), (agent,), delta))
    delta = np.array([[.5, 0, .3], [-.4, .3, 0], [.4, -.4, -.25], [.3, .4, .2]])
    target += delta
    phases.append(MotionPhase('simultaneous_four', 400, target.copy(), (0, 1, 2, 3), delta))
    phases.append(MotionPhase('final_hold', 200, target.copy(), (), np.zeros((4, 3))))
    return phases


def evaluate_phase(phase, starts, ends, idle_drift_max):
    starts, ends = np.asarray(starts), np.asarray(ends)
    errors = np.linalg.norm(ends-phase.targets, axis=1)
    progress = {}
    for i in phase.moving:
        length = np.linalg.norm(phase.delta[i])
        progress[str(i)] = float(np.dot(ends[i]-starts[i], phase.delta[i]/length))
    inactive = [i for i in range(4) if i not in phase.moving]
    isolation_ok = not phase.name.startswith('isolation_') or all(idle_drift_max[i] <= .15 for i in inactive)
    progress_ok = all(progress[str(i)] >= .5*np.linalg.norm(phase.delta[i]) for i in phase.moving)
    return {'phase': phase.name, 'position_errors_m': errors.tolist(),
            'progress_m': progress, 'idle_drift_max_m': list(map(float, idle_drift_max)),
            'passed': bool(np.max(errors) <= .2 and isolation_ok and progress_ok)}

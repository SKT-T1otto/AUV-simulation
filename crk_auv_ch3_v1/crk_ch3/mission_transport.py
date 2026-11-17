"""G5 motion transaction: public control out, measured motion in; no integrator."""
from dataclasses import dataclass
import uuid
import numpy as np

from .scenario import Frame
from .vehicle import rotation

FRAME = 'ch3_native_xyz_up_v1'
TASK_NS = 200_000_000


def matrix(value):
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (4, 3) or not np.isfinite(result).all():
        raise ValueError('Expected finite four-vehicle 4 x 3 motion data')
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class MotionRequest:
    token: str
    task_step: int
    start_ns: int
    end_ns: int
    positions: np.ndarray
    velocities_world: np.ndarray
    acceleration_world: np.ndarray
    blocked: tuple
    frame: str = FRAME

    def __post_init__(self):
        if (type(self.task_step) is not int or self.task_step < 1 or not self.token
                or self.start_ns != (self.task_step-1)*TASK_NS or self.end_ns != self.task_step*TASK_NS
                or self.frame != FRAME or len(self.blocked) != 4 or any(type(x) is not bool for x in self.blocked)):
            raise ValueError('Invalid motion request identity, interval or frame')
        for name in ('positions', 'velocities_world', 'acceleration_world'):
            object.__setattr__(self, name, matrix(getattr(self, name)))

    @classmethod
    def create(cls, step, positions, velocities, acceleration, blocked):
        return cls(uuid.uuid4().hex, step, (step-1)*TASK_NS, step*TASK_NS,
                   positions, velocities, acceleration, tuple(bool(x) for x in blocked))


@dataclass(frozen=True)
class MotionFeedback:
    token: str
    task_step: int
    stamp_ns: int
    positions: np.ndarray
    velocities_world: np.ndarray
    frame: str = FRAME

    def __post_init__(self):
        for name in ('positions', 'velocities_world'):
            object.__setattr__(self, name, matrix(getattr(self, name)))

    @classmethod
    def for_request(cls, request, positions, velocities, *, stamp_ns=None):
        return cls(request.token, request.task_step, request.end_ns if stamp_ns is None else stamp_ns,
                   positions, velocities)


class MotionExchange:
    def __init__(self, callback):
        self.callback = callback
        self.last_request = None
        self.last_feedback = None
        self.failed = False

    def exchange(self, request):
        if self.failed:
            raise RuntimeError('Motion transaction failed; discard/reset the mission, do not retry its step')
        try:
            if self.last_request is not None and request.task_step != self.last_request.task_step+1:
                raise ValueError('Repeated or skipped motion transaction')
            if self.last_request is None and request.task_step != 1:
                raise ValueError('First motion transaction must be task step 1')
            feedback = self.callback(request)
            if not isinstance(feedback, MotionFeedback):
                raise ValueError('Motion callback must return MotionFeedback')
            if (feedback.token != request.token or feedback.task_step != request.task_step or feedback.frame != FRAME
                    or type(feedback.stamp_ns) is not int
                    or not request.end_ns-2_000_000 <= feedback.stamp_ns <= request.end_ns):
                raise ValueError('Stale, future, mismatched or wrong-frame motion feedback')
            self.last_request, self.last_feedback = request, feedback
            return feedback
        except Exception:
            self.failed = True
            raise


def feedback_from_gazebo(request, states, *, epoch_ns, frame=Frame()):
    """Convert FLU body velocity to native world velocity, never treat it as ENU."""
    if len(states) != 4 or len({s.stamp_ns for s in states}) != 1:
        raise ValueError('All four Gazebo states must have one shared timestamp in agent order')
    positions = [frame.inverse_position(s.position) for s in states]
    velocities = [frame.rotate(tuple(rotation(s.quaternion) @ np.asarray(s.velocity_body)), inverse=True)
                  for s in states]
    return MotionFeedback.for_request(request, positions, velocities, stamp_ns=states[0].stamp_ns-int(epoch_ns))

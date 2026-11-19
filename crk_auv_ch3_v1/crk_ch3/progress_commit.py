"""Task-free pending-intent protocol; no physics, planner refresh or policy calls."""
from dataclasses import dataclass
import numpy as np

SCHEMA = 'crk.progress_commit.v1'
SEARCH_CONDITIONS = ('search_baseline32', 'search_progress32')
FIXTURE_CONDITIONS = ('fixture_baseline32', 'fixture_progress32')
CONDITIONS = FIXTURE_CONDITIONS + SEARCH_CONDITIONS
BUDGET = 32


def prefix_criteria(condition):
    from .reassignment_contract import PREFIX_CRITERIA
    result = dict(PREFIX_CRITERIA)
    if condition in FIXTURE_CONDITIONS:
        result.update(motion_prefix_steps=5, guidance_prefix_steps=4,
            explanation='Same absolute four-vehicle tolerances; fixture guidance0..4 and motion1..5 before post-motion5 synthetic proposal.')
    return result


class RouteUnavailable(RuntimeError):
    """A stale intent cannot be rebuilt safely; discard, never install its old path."""


def signature(assignment):
    return (assignment.agent_id, assignment.candidate_id, assignment.waypoint,
            assignment.path, assignment.failure_reason)


@dataclass(frozen=True)
class Intent:
    token: int
    assignment: object
    proposed_step: int
    proposal_hash: str
    installed_signature: tuple
    arrival_point: tuple


@dataclass(frozen=True)
class Prepared:
    intent: Intent
    assignment: object
    route_evidence: dict


class PendingSearchCommits:
    def __init__(self, threshold=.75):
        if not np.isfinite(threshold) or threshold <= 0:
            raise ValueError('Positive unchanged tracker threshold required')
        self.threshold = float(threshold)
        self.pending = {}
        self.events = []
        self.serial = 0
        self.committed = 0
        self.deferred = 0

    def emit(self, step, agent, kind, **details):
        self.events.append(dict(step=int(step), agent=int(agent), kind=kind, **details))

    def drop(self, agent, step, reason):
        item = self.pending.pop(agent, None)
        if item is not None:
            self.emit(step, agent, 'DISCARDED', reason=reason, token=item.token,
                      proposed_step=item.proposed_step)

    def clear(self, step, reason):
        for agent in tuple(self.pending):
            self.drop(agent, step, reason)

    def offer(self, assignment, installed, tracking_point, *, step, proposal_hash):
        """Only call for searchers affected by an accepted native proposal."""
        agent = assignment.agent_id
        if agent not in (0, 1, 2) or installed.agent_id != agent:
            raise ValueError('Only the three searchers may enter this queue')
        if signature(assignment) == signature(installed):
            self.drop(agent, step, 'native_reaffirmed_installed')
            return
        self.drop(agent, step, 'superseded_by_accepted_native_proposal')
        point = np.asarray(tracking_point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Finite pre-motion installed tracking point required')
        self.serial += 1
        item = Intent(self.serial, assignment, int(step), str(proposal_hash),
                      signature(installed), tuple(float(x) for x in point))
        self.pending[agent] = item
        self.emit(step, agent, 'PENDING', token=item.token, proposed_step=item.proposed_step,
                  proposal_hash=item.proposal_hash, arrival_point=item.arrival_point,
                  desired_candidate=assignment.candidate_id, desired_waypoint=assignment.waypoint)

    def prepare(self, installed, positions, safe_by_agent, rebuild, *, step):
        """Prepare all safe replacements before touching controller/bookkeeping."""
        ready = []
        for agent, item in tuple(self.pending.items()):
            current = installed.get(agent)
            if current is None or signature(current) != item.installed_signature:
                self.drop(agent, step, 'installed_route_changed')
                continue
            if not safe_by_agent.get(agent, False):
                self.drop(agent, step, 'old_route_invalid_native_safety_takes_priority')
                continue
            position = np.asarray(positions[agent], dtype=float)
            distance = float(np.linalg.norm(position-np.asarray(item.arrival_point)))
            if distance >= self.threshold:  # Exactly the native strict < threshold.
                self.emit(step, agent, 'WAITING', token=item.token, distance_m=distance)
                self.deferred += 1
                continue
            try:
                rebuilt, evidence = rebuild(item.assignment, tuple(position))
                points = np.asarray(rebuilt.path, dtype=float)
                if (rebuilt.agent_id != agent or rebuilt.candidate_id != item.assignment.candidate_id
                        or rebuilt.waypoint != item.assignment.waypoint or rebuilt.failure_reason
                        or points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 1
                        or not np.isfinite(points).all()
                        or not np.allclose(points[0], position, atol=1e-9, rtol=0)
                        or not np.allclose(points[-1], item.assignment.waypoint, atol=1e-9, rtol=0)
                        or not np.isfinite([rebuilt.planning_cost, rebuilt.physical_travel_time]).all()
                        or min(rebuilt.planning_cost, rebuilt.physical_travel_time) < 0):
                    raise RouteUnavailable('rebuilt_route_identity_or_endpoints_invalid')
            except RouteUnavailable as error:
                self.drop(agent, step, 'rebuild_failed:'+str(error))
                continue
            ready.append(Prepared(item, rebuilt, evidence))
            self.emit(step, agent, 'READY', token=item.token, proposed_step=item.proposed_step,
                      distance_m=distance, route_evidence=evidence)
        return tuple(ready)

    def acknowledge(self, prepared, *, step):
        # Validate the whole acknowledgement before removing any intent.
        if any(self.pending.get(x.assignment.agent_id) != x.intent for x in prepared):
            raise RuntimeError('Stale prepared transaction')
        for item in prepared:
            agent = item.assignment.agent_id
            self.pending.pop(agent)
            self.committed += 1
            self.emit(step, agent, 'COMMITTED', token=item.intent.token,
                      proposed_step=item.intent.proposed_step, committed_step=int(step),
                      proposal_hash=item.intent.proposal_hash, route_evidence=item.route_evidence)

"""Synthetic waypoint stimulus for commit mechanics, NOT the D2 search task."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from .progress_commit import PendingSearchCommits, RouteUnavailable, signature
from .native_source import activate_native
activate_native()
from chapter3_bser.online.types import SearchAssignment, ExecutorAssignment, OnlineAllocation
from chapter3_bser.integration.rmaddpg_bridge import RMADDPGGuidanceBridge


class FixtureProgram:
    def __init__(self, origins_native, condition, positions_native=None):
        if condition not in ('fixture_baseline32','fixture_progress32'):
            raise ValueError('Explicit fixture condition required')
        self.condition = condition
        self.origins = np.asarray(origins_native, dtype=float)
        self.positions = self.origins.copy() if positions_native is None else np.asarray(positions_native,dtype=float)
        self.step = 0
        self.bridge = RMADDPGGuidanceBridge()
        self.queue = PendingSearchCommits(self.bridge.path_tracker.threshold)
        self.commit_steps = []
        self.raw_proposal = None
        old = tuple(replace(self.assignment(i, self.origins[i]+[0,1.5,0], 'fixture_old'),
                    path=(tuple(self.origins[i]),tuple(self.origins[i]+[0,1.5,0]))) for i in range(3))
        executor = ExecutorAssignment(3,tuple(self.origins[3]),(tuple(self.origins[3]),),0.,'fixture_hold',True,(),0.,None)
        self.allocation = OnlineAllocation(old,executor,0.,0.,0.,'synthetic_fixture')
        self.compile()

    def assignment(self, agent, goal, label):
        distance = float(np.linalg.norm(self.positions[agent]-goal))
        return SearchAssignment(agent,f'{label}_{agent}',tuple(goal),
            (tuple(self.positions[agent]),tuple(goal)),distance,(),distance,None)

    def safe(self, agent, points):
        # Declared fixture corridors lie entirely left of the original obstacles.
        # Synthetic public geometry only, not hidden task geometry or map sensing.
        lo = self.origins[agent]+[-1.,-.5,-.5]
        hi = self.origins[agent]+[2.5,3.5,1.]
        data = np.asarray(points)
        return bool(np.all(data>=lo) and np.all(data<=hi))

    def rebuild(self, intent, position):
        if not self.safe(intent.agent_id,[position,intent.waypoint]):
            raise RouteUnavailable('synthetic_corridor_rejected')
        fresh = self.assignment(intent.agent_id,intent.waypoint,'fixture_new')
        return fresh,dict(scope='synthetic_straight_segment; not D2 route evidence',
            start=position,original_proposal_start=intent.path[0])

    def compile(self):
        roles=('search_fast','search_balanced','search_precise','executor')
        agents = tuple(SimpleNamespace(agent_id=i,position=tuple(p),role=roles[i])
                       for i,p in enumerate(self.positions))
        state = SimpleNamespace(step=self.step,agents=agents,executor_id=3,searcher_ids=(0,1,2))
        context = SimpleNamespace(step=self.step,target_found=False,executor_knows_target=False,
            mission_complete=False,searcher_finished_flags=(False,False,False))
        self.guidance = self.bridge.compile_guidance(self.allocation,state,context)

    def advance(self, cycle, positions_native):
        if cycle != self.step+1 or cycle>32:
            raise ValueError('No repeats, continuation or >32 cycles')
        self.step = cycle
        self.positions = np.asarray(positions_native, dtype=float)
        safe = {i:self.safe(i,(self.positions[i],*self.bridge.path_tracker.snapshot(i).remaining_path_points)) for i in range(3)}
        if not all(safe.values()):
            self.queue.clear(cycle,'fixture_corridor_guard')
            raise RuntimeError('Fixture corridor guard; no unsafe old-route retention')
        if cycle==5:
            proposal = replace(self.allocation,search_assignments=tuple(
                self.assignment(i,self.origins[i]+[1.,2.5,.5],'fixture_new') for i in range(3)))
            self.raw_proposal = proposal
            if self.condition=='fixture_baseline32':
                self.allocation = proposal
                self.commit_steps.append(cycle)
            else:
                old = {s.agent_id:s for s in self.allocation.search_assignments}
                for item in proposal.search_assignments:
                    self.queue.offer(item,old[item.agent_id],self.guidance.agent_assignments[item.agent_id].tracking_waypoint,
                        step=cycle,proposal_hash=proposal.allocation_sha256)
        ready = self.queue.prepare({s.agent_id:s for s in self.allocation.search_assignments},self.positions,
                                   safe,self.rebuild,step=cycle)
        if ready:
            selected = {x.assignment.agent_id:x.assignment for x in ready}
            self.allocation = replace(self.allocation,search_assignments=tuple(selected.get(s.agent_id,s) for s in self.allocation.search_assignments))
            self.queue.acknowledge(ready,step=cycle)
            self.commit_steps.append(cycle)
        self.compile()  # Native tracker advances, even when every intent waits.

    def snapshot(self):
        return dict(cycle=self.step,allocation=self.allocation,guidance=self.guidance,
            tracker={str(i):self.bridge.path_tracker.snapshot(i) for i in range(3)},
            pending=self.queue.pending,adapter_commits=self.queue.committed,commit_steps=self.commit_steps)

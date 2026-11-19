"""Component facades only. Never construct a native mission/environment."""
from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import torch
from crk_ch3.native_source import activate_native
activate_native()
from chapter3_bser.online.types import SearchAssignment, ExecutorAssignment, OnlineAllocation, BSERActionAssignment
from chapter3_bser.online.waypoint_manager import WaypointManager
from chapter3_bser.integration.rmaddpg_bridge import RMADDPGGuidanceBridge
from chapter3_bser.events.trigger_policy import TriggerDecision
from core.mapping.planning_state import (PlanningAgentView, GridGeometryView, TargetBeliefView,
    OccupancyBeliefView, PlanningStateView)
from core.mapping.planning_graph import PlanningGraphView, PlanningEdgeView, PlanningConnectorView, EndpointConnectorSet


def search(i=0, goal=(3.,1.,.5), start=(1.1,1.,.5), name='old'):
    return SearchAssignment(i,name,goal,(start,goal),2.,(),2.,None)


def state(step=0, position=(1.1,1.,.5)):
    centers=np.array([[1.,1.,.5],[3.,1.,.5],[5.,1.,.5]])
    grid=GridGeometryView((3,1,1),(0.,0.,0.),(2.,2.,1.),centers)
    occ=OccupancyBeliefView(np.full(3,.5),np.zeros(3,bool),np.zeros(3,bool),np.zeros(3,bool),np.ones(3,bool),step,'online_unknown')
    belief=TargetBeliefView(np.full(3,1/3),1.,1/3,0,step,True)
    agents=tuple(PlanningAgentView(i,'executor' if i==3 else 'search_fast',position,(0.,0.,0.),1.,.55,2. if i<3 else None,(3.,1.,.5)) for i in range(4))
    adjacency=((PlanningEdgeView(1,2.,2.),),(PlanningEdgeView(0,2.,2.),PlanningEdgeView(2,2.,2.)),(PlanningEdgeView(1,2.,2.),))
    endpoint=EndpointConnectorSet('current','searcher',position,(PlanningConnectorView(1,0,abs(3-position[0]),abs(3-position[0])),))
    graph=PlanningGraphView((3,1,1),centers,np.ones(3,bool),np.zeros(3,int),adjacency,adjacency,(endpoint,),1.,1.,1.,1.,step,'online_unknown','component_graph')
    return PlanningStateView(step,False,False,grid,belief,occ,agents,3,(0,1,2),step,'online_unknown',graph)


class Facade:
    def __init__(self):
        self.step=0;self.terminal=False;self.state=state();self.records=[]
        self.flags=NS(target_found=False,executor_knows_target=False,mission_complete=False)
        self.nav=torch.zeros((4,3))
        self.env=NS(get_task_state=lambda:self.flags,get_agent_state=self.public)
        self.env.unwrapped=NS(_nav_targets=self.nav,last_collision_records=[],map_module=NS(map_revision=0))
        self.provider=NS(last_snapshot_was_full_refresh=False,_last_refresh_reason='INCREMENTAL')
        self.planning_views=NS(clearance=0.,live=lambda:self.state)
        self.bridge=RMADDPGGuidanceBridge();self.bridge._apply_overrides=lambda a:a
        self.bridge.last_safety_status={i:'SAFE' for i in range(3)}
        executor=ExecutorAssignment(3,(3.,1.,.5),((1.1,1.,.5),(3.,1.,.5)),2.,'native_old',True,(),2.,None)
        initial=OnlineAllocation(tuple(search(i) for i in range(3)),executor,.1,.1,2.,'initial')
        manager=WaypointManager(agent_cooldown_steps=5);manager.commit(initial)
        self.policy_calls=[];self.marked=[];self.score_calls=[]
        def decide(events,new,old,step):
            self.policy_calls.append(step);return TriggerDecision(True,'NATIVE_ACCEPT',new-old,0)
        policy=NS(decide=decide,primary_event=lambda events:next(iter(events),None),mark_replan=self.marked.append)
        def rescore(state,allocation,finalize=False):
            self.score_calls.append(finalize)
            return replace(allocation,objective_value=sum(s.waypoint[0] for s in allocation.search_assignments)/100)
        allocator=NS(rescore=rescore,allocate=lambda *a,**k:None,allocate_partial=lambda *a,**k:None)
        self.inner=NS(waypoints=manager,current_allocation=initial,policy=policy,allocator=allocator,replan_count=0,replan_steps=[])
        self.controller=NS(inner=self.inner,step=self.native_step)
        self.compile(initial)

    def public(self):
        return NS(positions=tuple(a.position for a in self.state.agents),velocities=tuple(a.velocity for a in self.state.agents))

    def compile(self,allocation):
        context=NS(step=self.step,mission_complete=self.flags.mission_complete,target_found=self.flags.target_found,
            executor_knows_target=self.flags.executor_knows_target,searcher_finished_flags=(False,)*3)
        self.guidance=self.bridge.compile_guidance(allocation,self.state,context)
        self.nav.copy_(torch.tensor([g.tracking_waypoint for g in self.guidance.agent_assignments]))

    def position(self,step,point):
        self.step=step;self.state=state(step,point)
        self.env.unwrapped.map_module.map_revision=step

    def native_step(self,state,context=None):
        old=self.inner.current_allocation
        if self.step==20:
            proposed=replace(old,search_assignments=tuple(search(i,(5.,1.,.5),self.state.agents[i].position,'new') for i in range(3)),
                executor_assignment=replace(old.executor_assignment,target_region=(5.,1.,.5),source='native_new'),objective_value=.9)
            self.inner.policy.decide(('BELIEF_SHIFT',),.9,.1,self.step)
            applied=self.inner.waypoints.stabilize(old,proposed,affected_agent_ids=(0,1,2),step=self.step)
            updates=self.inner.waypoints.updates(old,applied,reason='NATIVE_ACCEPT',step=self.step)
            self.inner.current_allocation=applied;self.inner.waypoints.commit(applied)
            self.inner.policy.mark_replan(self.step);self.inner.replan_count+=1;self.inner.replan_steps.append(self.step)
            return BSERActionAssignment(self.step,True,(),applied,updates,'NATIVE_ACCEPT',None,None)
        return BSERActionAssignment(self.step,False,(),old,(),'NO_REPLAN_EVENT',None,None)


def rebuild(intent,position):
    return replace(intent,path=(tuple(position),intent.waypoint),planning_cost=1.,physical_travel_time=1.),dict(start=position)

"""32-cycle real-plant commit fixture; synthetic proposals, no native task instance."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback
import numpy as np
from . import search_ros as ros
from .mission_parity import normalized
from .mission_transport import MotionRequest
from .search_transport import SearchTransport, PlantFailure
from .scenario import Frame
from .vehicle import load_profile, rotation
from .envelope_protocol import reference
from .progress_fixture import FixtureProgram
from .progress_commit import SCHEMA, FIXTURE_CONDITIONS, BUDGET, prefix_criteria
from .progress_report import write_run_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output = parser.parse_args().output.resolve()
    root = Path(os.environ.get('CRK_OUTPUT_ROOT','/home/legion/AUV-simulation')).resolve()
    if not output.is_relative_to(root/'results/ch3'):
        raise ValueError('Mounted host results required')
    summary = json.loads((output/'summary.json').read_text(encoding='utf-8'))
    if (summary.get('diagnostic_schema')!=SCHEMA or summary.get('diagnostic_condition') not in FIXTURE_CONDITIONS
            or summary.get('diagnostic_budget')!=32 or summary.get('boundary_step')!=5
            or summary.get('prefix_criteria')!=prefix_criteria(summary.get('diagnostic_condition'))):
        raise ValueError('Explicit progress fixture assets required')
    scene = json.loads((output/'scenario.json').read_text(encoding='utf-8'))
    streams = {}
    def record(name,value):
        if name not in streams:streams[name]=(output/name).open('x',encoding='utf-8')
        streams[name].write(json.dumps(normalized(value),allow_nan=False)+'\n');streams[name].flush()
    node = plant = program = None
    completed = event_cursor = 0
    failures=[];reason='startup_failure'
    profile=load_profile();frame=Frame()
    ros.rclpy.init(signal_handler_options=ros.SignalHandlerOptions.NO)
    try:
        node=ros.SearchNode(lambda event:record('physical_contacts.jsonl',event))
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:setattr(node,'stopping',True))
        states=node.ready();warmup=node.clock_ns
        expected=np.asarray([frame.position(p) for p in scene['initial_agent_positions']])
        if np.max(np.linalg.norm(np.asarray([s.position for s in states])-expected,axis=1))>.02:
            raise PlantFailure('Fixture warmup differs from original spawn')
        measured_origins=np.asarray([frame.inverse_position(s.position) for s in states])
        origins=np.asarray(scene['initial_agent_positions'],dtype=float)
        program=FixtureProgram(origins,summary['diagnostic_condition'],positions_native=measured_origins)
        epoch=node.confirm_pause()
        if not 0<=epoch-warmup<=2_000_000 or any(abs(s.stamp_ns-epoch)>2_000_000 for s in states):
            raise PlantFailure('Fixture startup clock drift')
        summary.update(task_epoch_ns=epoch,warmup_clock_ns=warmup,startup_clock_delta_ns=epoch-warmup)
        plant=SearchTransport(node,states,profile,record=lambda row:record('control.jsonl',dict(row,body_states=plant.states)),
                              realtime=os.environ.get('CRK_G6_REALTIME_VIEW')=='1')
        record('initial.jsonl',dict(cycle=0,body_states=states,program=program.snapshot()))
        (output/'program.json').write_text(json.dumps(normalized(dict(schema=SCHEMA,proposal_cycle=5,
            old_delta=[0,1.5,0],new_delta=[1,2.5,.5],origin_native=origins,threshold=.75,
            reference='existing envelope waypoint reference; not native D2 base navigation',
            routing='synthetic fresh straight segments within declared obstacle-free corridors',
            task_instances=0)),indent=2)+'\n',encoding='utf-8')
        for cycle in range(1,BUDGET+1):
            node.check()
            before=normalized(program.snapshot())
            positions=np.asarray([frame.inverse_position(s.position) for s in plant.states])
            velocities=np.asarray([rotation(s.quaternion)@s.velocity_body for s in plant.states])
            requests=[]
            for i,s in enumerate(plant.states):
                guide=program.guidance.agent_assignments[i]
                target=frame.position(guide.tracking_waypoint)
                origin=frame.position(program.origins[i].tolist())
                request,_,_=reference(dict(mode='waypoint',delta=np.asarray(target)-origin),
                    np.asarray(origin),np.asarray(s.position),velocities[i],profile)
                requests.append(request)
            positions[3]=program.origins[3]  # Explicit fixed executor fixture hold.
            record('step_start.jsonl',dict(cycle=cycle,applied_before_motion=before,body_states=plant.states,requested_acceleration=requests))
            plant(MotionRequest.create(cycle,positions,velocities,requests,(False,False,False,True)))
            completed+=1
            program.advance(cycle,[frame.inverse_position(s.position) for s in plant.states])
            record('cycles.jsonl',dict(cycle=cycle,physical_time_s=(node.clock_ns-epoch)/1e9,
                condition=program.condition,applied_before_motion=before,body_states=plant.states,
                program=program.snapshot(),raw_synthetic_proposal=program.raw_proposal,
                queue_events=program.queue.events[event_cursor:]))
            event_cursor=len(program.queue.events)
            node.confirm_pause(epoch+cycle*200_000_000);node.check()
            node.get_logger().info(f'Progress fixture {program.condition} cycle={cycle}/32')
        reason='diagnostic_budget_completed'
    except KeyboardInterrupt:reason='interrupted'
    except ros.PhysicalContact as error:reason='physical_contact';summary['error']=str(error)
    except Exception as error:
        reason='engineering_failure';summary['error']=str(error)
        (output/'error_traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        if reason!='diagnostic_budget_completed':failures.append(reason)
        if node is not None:
            try:
                until=time.monotonic()+.15
                while time.monotonic()<until:ros.rclpy.spin_once(node,timeout_sec=.01)
                if node.contact_failure:reason='physical_contact';failures.append(reason)
                node.publish(np.zeros((4,8)));node.stop_commands();summary['stop_zero_command_sent']=True
            except Exception as error:failures.append('stop_failure');summary['stop_error']=str(error)
            summary.update(final_clock_ns=node.clock_ns,final_world_status=node.world_status)
        if program is not None:
            summary.update(delayed_search_commits=program.queue.committed,pending_before_stop=len(program.queue.pending),
                adapter_commit_steps=program.commit_steps,deferred_wait_samples=program.queue.deferred)
            program.queue.clear(program.step,'run_stopped:'+reason)
            record('final_state.jsonl',dict(reason=reason,program=program.snapshot(),queue_events=program.queue.events[event_cursor:]))
            if program.condition=='fixture_progress32' and reason=='diagnostic_budget_completed' and program.queue.committed!=3:
                failures.append('release_not_observed_within_budget')
            if program.condition=='fixture_progress32' and reason=='diagnostic_budget_completed' and (
                    program.queue.deferred==0 or not program.commit_steps or min(program.commit_steps)<=5):
                failures.append('wait_then_release_not_exercised')
        if plant is not None:
            summary.update(control_steps=plant.control_steps,actual_simulated_duration_s=plant.measured_duration_s,
                max_roll_pitch_deg=plant.max_tilt,max_abs_thrust_N=plant.max_force,max_finished_hold_error_m=plant.max_hold_error)
        summary.update(termination_reason=reason,completed_fixture_cycles=completed,completed_task_steps=0,
            native_task_instances=0,mission_executed=False,map_executed=False,task_success=False,
            g6_search_acceptance_passed=False,g6_capture_acceptance_passed=False,
            motion_started=plant is not None and plant.control_steps>0,failure_tags=sorted(set(failures)),
            contact_events_observed=0 if node is None else node.contact_events,contact_absence_proven=False,
            physical_stop='zero command with bridge receipt and world pause; no settling rollout')
        summary['diagnostic_valid']=(reason=='diagnostic_budget_completed' and completed==32 and not failures
            and summary.get('stop_zero_command_sent',False) and summary.get('linux_sdf_valid',False))
        summary['engineering_acceptance_passed']=summary['diagnostic_valid']
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        for stream in streams.values():stream.close()
        try:write_run_report(output)
        except Exception as error:
            summary.update(diagnostic_valid=False,engineering_acceptance_passed=False,report_error=str(error))
            summary['failure_tags'].append('report_failure')
            (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        if node is not None:node.destroy_node()
        ros.rclpy.shutdown()
        print(f"Progress fixture {reason}; valid={summary['diagnostic_valid']}; failures={summary['failure_tags']}; output={output}",flush=True)
    raise SystemExit(0 if summary['diagnostic_valid'] else 1)


if __name__=='__main__':main()

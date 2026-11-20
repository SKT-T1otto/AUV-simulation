"""Read-only evidence for the G6 near-target fixture, in native metres."""
import math


def capture_sample(step, task, native, start, end, target_start, stamp_ns):
    target_end = native.target_state.position.tolist()
    return dict(step=step, feedback_elapsed_ns=stamp_ns, finder_id=task.finder_id,
                target_found=task.target_found, executor_received=task.executor_knows_target,
                executor_start=start, executor_end=end, target_start=target_start,
                target_end=target_end, distance_start_m=math.dist(start, target_start),
                distance_end_m=math.dist(end, target_end), radius_m=float(native.target_capture_radius),
                required_steps=int(native.target_capture_hold_steps),
                native_hold=int(native.hold_counters[native.executor_idx]),
                mission_complete=task.mission_complete)


def capture_review(rows, summary):
    """Cross-check the native terminal label against consecutive measured endpoints."""
    failures = []
    found = received = None
    streak = 0
    for index, row in enumerate(rows, 1):
        if row['step'] != index or not 0 <= index*200_000_000-row['feedback_elapsed_ns'] <= 2_000_000:
            failures.append('capture_clock_or_step_gap')
        if row['target_found'] and found is None and row['finder_id'] in (0, 1, 2):
            found = index
        if row['executor_received'] and received is None:
            received = index
        inside = (row['distance_start_m'] <= row['radius_m'] and
                  row['distance_end_m'] <= row['radius_m'])
        streak = streak+1 if row['executor_received'] and row['target_found'] and inside else 0
        if streak != row['native_hold']:
            failures.append('measured_and_native_hold_disagree')
    if not rows or len(rows) != summary['completed_task_steps']:
        failures.append('capture_trace_incomplete')
    if found is None or found != summary.get('found_step'):
        failures.append('discovery_unverified')
    if received is None or found is None or received != found+1 or received != summary.get('handoff_decision_step'):
        failures.append('one_step_handoff_unverified')
    if not rows or streak < rows[-1]['required_steps'] or not rows[-1]['mission_complete']:
        failures.append('terminal_hold_unverified')
    if summary.get('task_outcome', {}).get('termination_reason') != 'success':
        failures.append('native_task_not_successful')
    return dict(schema='crk.g6.capture_evidence.v1', found_step=found, received_step=received,
                final_consecutive_steps=streak, verified=not failures, failure_tags=sorted(set(failures)))

"""Keep a healthy control run distinct from capture and search task acceptance."""
def acceptance(summary):
    healthy = (summary.get('termination_reason') in ('task_terminal', 'smoke_completed')
        and summary.get('linux_sdf_valid') is True and summary.get('motion_started') is True
        and summary.get('map_executed') is True and summary.get('map_sensor_scan_count', 0) > 0
        and summary.get('completed_task_steps', 0) > 0
        and summary.get('control_steps') == summary.get('completed_task_steps', 0)*10
        and summary.get('stop_zero_command_sent') is True and 'stop_error' not in summary
        and summary.get('contact_events_observed', 0) == 0)
    if summary.get('mode') == 'smoke':
        moved = summary.get('max_displacement_m', [])
        healthy = healthy and summary.get('completed_task_steps') == 16 and len(moved) == 4 and min(moved[:3]) >= .02
    captured = (healthy and summary.get('task_success') is True
        and summary.get('task_outcome', {}).get('termination_reason') == 'success'
        and summary.get('found_step') is not None and summary.get('handoff_decision_step') is not None
        and summary.get('capture_hold_max', 0) >= 5)
    return dict(engineering_acceptance_passed=bool(healthy),
        g6_capture_acceptance_passed=bool(captured and summary.get('mode') == 'capture'
            and summary.get('capture_evidence', {}).get('verified') is True),
        g6_search_acceptance_passed=bool(captured and summary.get('mode') == 'search'
                                       and summary.get('search_displacement_before_found_m', 0) >= .02))

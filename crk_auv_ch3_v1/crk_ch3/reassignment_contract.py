"""Preregistered v1 diagnostic constants, independent of ROS and task imports."""
CONDITIONS = ('baseline32', 'defer_search_reassignment32')
SCHEMA = 'crk.d2.search_commit_ablation.v1'
BUDGET = 32
BOUNDARY = 20
PREFIX_CRITERIA = dict(
    initial_position_max_m=.002, initial_velocity_max_m_s=.01,
    prefix_position_max_m=.01, prefix_velocity_max_m_s=.02,
    quaternion_angle_max_deg=1., requested_acceleration_max_m_s2=.05,
    guidance_routes_exact=True, task_state_exact=True,
    motion_prefix_steps=20, guidance_prefix_steps=19,
    explanation='Absolute four-vehicle maxima; initial plus every 20 ms endpoint through motion20; '
                'guidance0..19 route/hold/reachability equality. Allocation hashes reported separately, never rounded.')

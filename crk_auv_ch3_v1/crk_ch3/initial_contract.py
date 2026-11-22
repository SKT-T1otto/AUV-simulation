"""Pre-registered checking criteria for the original far-search fixture. Never used to select candidates."""

import math
from .standby_geometry import CONDITIONS as STANDBY
from .contracts import canonical_hash

SCHEMA = 'crk.g6.initial_candidate_pair.v1'
CONDITIONS = ('initial_original32', 'initial_quadrant32')
BUDGET = 32
FULL_CONDITIONS = ('initial_original400', 'initial_quadrant400')
FULL_SCHEMA = 'crk.g6.far_search_pair400.v1'
FULL_PREFIX = 'g6_far_search_pair400_'
HORIZON = 400


def admission_rule(condition):
    if condition in STANDBY:
        return CONDITIONS[1]
    if condition in FULL_CONDITIONS:
        return CONDITIONS[FULL_CONDITIONS.index(condition)]
    if condition in CONDITIONS:
        return condition
    raise ValueError('Unknown initial candidate condition: ' + str(condition))


CRITERIA = dict(
    initial_position_m=0.002,
    initial_velocity_m_s=0.01,
    attitude_deg=1.0,
    common_position_m=0.01,
    common_velocity_m_s=0.02,
    acceleration_m_s2=0.05,
    minimum_projection_m=0.01,
    consecutive_samples=3,
    score_abs_tolerance=1e-10,
)
COMMON = ((1, 1, 1.5), (1, 5, 2.5), (1, 9, 3.5))
NEXT = (((1, 3, 2.5), (1, 3, 3.5), (1, 9, 4.5)), ((3, 3, 1.5), (3, 7, 1.5), (1, 9, 2.5)))
FINGERPRINTS = dict(
    graph='fd69b6492d7e3ceeac2c6c1cbb70841959c82610146423e7a77c13e4c22070a3',
    safe_graph='0ce359cb317ae8c10322de135c52d4a121a09300d573b64dd8caf45c278b8823',
    public_state='c68a9d6b9e6f1b2c2e0918ab46854b4f894023196fd817e90c5aa8335ca1dea9',
)
FIXTURE = dict(
    source_files_sha256='1da086f7a7418dcca6914168d71843c4cf93d86c0f8ae95c1a4d6260137299d1',
    scenario_sha256='72c6a0237325f055c7257b631482a66af5b467724f9ba722843995a32d6a2799',
    vehicle_profile_sha256='798ace94c09dce1cd1bcc9405a5488b676bc9813413adea7ea1dbe12fa45458c',
)
# Compact acceptance fingerprints from the preregistered offline comparison.
GOLD = {
    CONDITIONS[0]: dict(
        admitted='bb251e48edb173966df34ddb115751ce11d357a17742ad224dbb43b623010a69',
        selected='b831697d84c3ea085e352163dde3f1dbe8ee928680036ee3377a1498bc37b00a',
        allocation='b9b3de7f94b84346eaed9a2c6aab3d76d30200c45036c9fe16f80fd867f84e49',
        objective=0.10278100894180675,
        joint_score=0.06484763978250008,
        response_time=8.721455947968272,
    ),
    CONDITIONS[1]: dict(
        admitted='0ce5afef1739bacadca5925edd5648b9699f86686a9d3bef8eeef8920be69ee6',
        selected='67ac9b465e7dbc9d594368262bb43e44d8e9af65ac653f605456a78d8b0cdee8',
        allocation='1f21138af48c45ec0640d1deb1113a25c79e77d1b7ccd3fbba6875aecdbb38fb',
        objective=0.1553227568597067,
        joint_score=0.10351016059104536,
        response_time=7.472414935149104,
    ),
}
STANDBYS = 'fd6783756d76a6e1a480caac7c4ba1a8fdb000079bb1be5277647de5046e0538'
CONFIG_SHA = 'b410ccd614b8072c8db624b8a290313797edcb2ac411b8a31aca789f4b77956d'
ROLES_SHA = 'fa0ab678efe234c04f8695e3dcc70638c3290d74e4783d08d7814406b51c63cf'
# Read-only acceptance values in native slot order (metres, planning cost, seconds).
ORIGINAL_COSTS = (
    (2.5, 3.7671035439270337, 3.0505771987367316),
    (5.5, 9.084483616665711, 6.323304471464004),
    (1.5, 1.9596681078276406, 1.9596681078276406),
    (6.73606797749979, 11.082201263856323, 8.323304471464004),
    (5.97213595499958, 9.90565581014411, 8.141486289645822),
    (6.97213595499958, 11.881407337138052, 9.232395380554912),
    (5.73606797749979, 8.323735609924384, 7.050577198736732),
    (5.73606797749979, 9.314034672328825, 7.232395380554913),
    (10.20820393249937, 16.269723312240853, 13.232395380554914),
    (10.73606797749979, 16.425141497514826, 12.141486289645822),
    (9.97213595499958, 14.687803112021127, 12.141486289645822),
    (8.73606797749979, 13.286739886272647, 10.141486289645822),
)
QUADRANT_COSTS = (
    (2.5, 3.7671035439270337, 3.0505771987367316),
    (11.73606797749979, 17.936151067925323, 13.05057719873673),
    (11.73606797749979, 19.574722553087632, 13.05057719873673),
    (16.64213562373095, 25.2109521074731, 17.19271282246768),
    (5.97213595499958, 9.90565581014411, 8.141486289645822),
    (7.97213595499958, 13.244721469936854, 10.141486289645822),
    (13.56449510224598, 21.577449855326766, 15.798340539138202),
    (14.32842712474619, 23.21921163063727, 16.626767663884394),
    (10.20820393249937, 16.269723312240853, 13.232395380554914),
    (5.73606797749979, 10.176943601279424, 7.232395380554913),
    (16.15685424949238, 26.484538996764776, 18.54610387953967),
    (13.80056307974577, 23.235679815352018, 16.0608225053011),
)
STANDBY_COSTS = (
    (0.0, 0.0, 0.0),
    (1.5, 1.7297509238026916, 1.7297509238026916),
    (13.73606797749979, 18.90391282944827, 13.164533532498346),
    (12.39292222699217, 16.915272366863615, 12.127015488578676),
)


def check_admission(data, condition):
    rule = admission_rule(condition)
    gold = GOLD[rule]
    candidates = data['candidates']
    admitted = [[c['id'], c['pool_rank'], c['source'], c['waypoint']] for c in candidates]
    chosen = [[c['id'], c['waypoint'], c['path']] for c in candidates if c['selected']]
    standby = data['executor']
    chosen.append([standby['source'], standby['target_region'], standby['path']])
    ys = [[y['id'], y['source'], y['waypoint'], y['path']] for y in data['standbys']]
    original = rule == CONDITIONS[0]
    expected_cells = [0, 5, 1, 13] if original else [0, 40, 400, 440]
    expected_ends = (
        [[1, 3, 5.5], [1, 1, 5.5], [1, 1, 5.5]]
        if original
        else [[11, 11, 0.5], [11, 11, 0.5], [11, 1, 0.5]]
    )
    observed_cells = [[c['cell'] for c in candidates if c['agent'] == i] for i in range(3)]
    observed_ends = [c['waypoint'] for c in candidates if c['selected']]
    checks = dict(
        cell_order=observed_cells == [expected_cells] * 3,
        final_search_points=observed_ends == expected_ends,
        admission=canonical_hash(admitted) == gold['admitted'],
        chosen_routes=canonical_hash(chosen) == gold['selected'],
        standby_pool=canonical_hash(ys) == STANDBYS,
        public_inputs=data['fingerprints'] == FINGERPRINTS,
        condition=data['condition'] == condition,
        once=data['initialize_calls'] == 1
        and data['special_pool_calls'] == int(not original),
        scope_released=data['native_pool_restored'],
        public_pool_count=data['pool_count'] == 800,
    )
    delta = {
        key: data['scores'][key] - gold[key] for key in ('objective', 'joint_score', 'response_time')
    }
    costs = ORIGINAL_COSTS if original else QUADRANT_COSTS
    fields = ('length_m', 'planning_cost', 'native_travel_s')
    travel_deltas = {
        c['id']: {k: c[k] - v for k, v in zip(fields, expected)}
        for c, expected in zip([*candidates, *data['standbys']], (*costs, *STANDBY_COSTS))
    }
    numerical = [*delta.values(), *(v for entry in travel_deltas.values() for v in entry.values())]
    return dict(
        discrete_and_input_checks=checks,
        admitted_cells=dict(expected=[expected_cells] * 3, observed=observed_cells),
        selected_endpoints=dict(expected=expected_ends, observed=observed_ends),
        float_deltas=delta,
        observed_allocation_sha256=data['allocation_sha256'],
        reference_allocation_sha256=gold['allocation'],
        canonical_hash_equal=data['allocation_sha256'] == gold['allocation'],
        candidate_travel_deltas=travel_deltas,
        valid=all(checks.values())
        and all(math.isfinite(n) and abs(n) <= CRITERIA['score_abs_tolerance'] for n in numerical),
    )

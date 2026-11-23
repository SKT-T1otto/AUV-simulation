"""Audience-facing visuals and measured milestones for the stationary survey buoy."""
from dataclasses import replace
import math
import xml.etree.ElementTree as ET
import numpy as np

from .motion_world import visual
from .preview import child
from .capture_evidence import capture_sample


def inspection_sample(step, task, native, *motion):
    row = capture_sample(step, task, native, *motion)
    # The native per-agent counter also records station-keeping at the waiting point.
    row['standby_or_capture_hold'] = row['native_hold']
    row['native_hold'] = int(native._capture_hold_counter)
    return row


class SurveyTransects:
    """Presentation-only waypoint reference for three survey lines and one stationary instrument."""
    def __init__(self, starts):
        self.references = np.array(starts, dtype=float)
        self.references[:3, 0] += 4.
        self.delivered_station = None

    def apply(self, request, env):
        references = self.references.copy()
        if env.get_task_state().executor_knows_target:
            if self.delivered_station is None:
                delivery = env.get_public_delivered_target()
                payload = (delivery or {}).get('payload', {})
                if payload.get('velocity') != [0., 0., 0.]:
                    raise ValueError('Inspection demo requires a delivered stationary instrument')
                station = payload.get('position')
                if station is None or np.shape(station) != (3,) or not np.isfinite(station).all():
                    raise ValueError('Missing public inspection-station position at handoff')
                self.delivered_station = np.array(station, dtype=float)
            references[3] = self.delivered_station
        # Original G6 hold controller takes over for agents marked finished by the native task.
        errors = references - request.positions
        speeds = np.array([.16, .16, .16, .26])
        distances = np.linalg.norm(errors, axis=1)
        reference_velocity = errors * np.minimum(.8, speeds / np.maximum(distances, 1e-12))[:, None]
        acceleration = 1.2 * (reference_velocity - request.velocities_world)
        acceleration[list(request.blocked)] = 0.
        applied = replace(request, acceleration_world=acceleration)
        return applied, dict(step=request.task_step, blocked=request.blocked, waypoint_native_m=references.tolist(),
                             stationary_site_received=self.delivered_station is not None,
                             original_d2_request_m_s2=request.acceleration_world.tolist(),
                             presentation_request_m_s2=acceleration.tolist())


def presentation_world(xml):
    sdf = ET.fromstring(xml)
    world = sdf.find('world')
    station = world.find("model[@name='target_truth_marker']/link")
    visual(station, 'instrument_mast', 'cylinder', {'radius':.035,'length':.75},
           '1 .85 .08 1', '0 0 -.20 0 0 0')
    for i,color in enumerate(['.1 .7 1 1','.2 1 .3 1','.8 .3 1 1','1 .5 .05 1']):
        base = world.find(f"model[@name='auv_{i}']/link[@name='base_link']")
        visual(base, 'role_light', 'sphere', {'radius':.095}, color, '0 0 .19 0 0 0')
        child(base.find("visual[@name='role_light']/material"),'emissive',color)
    # Metre references on the floor help viewers see motion, with no new collisions.
    floor = world.find("model[@name='workspace_floor']/link")
    for k in range(-8,9,2):
        visual(floor,f'grid_x_{k}','box',{'size':'0.018 19 .002'},'.28 .44 .44 1',f'{k} 0 .028 0 0 0')
        visual(floor,f'grid_y_{k}','box',{'size':'19 0.018 .002'},'.28 .44 .44 1',f'0 {k} .028 0 0 0')
    ET.indent(sdf)
    return ET.tostring(sdf,encoding='unicode')


def phase(row):
    if row['mission_complete']:
        return '驻留完成'
    if row['native_hold'] > 0:
        return '进入观测范围，开始驻留'
    if row['executor_received']:
        return '已交接，橙色执行艇前往观测站'
    if row['target_found']:
        return f"auv_{row['finder_id']} 发现观测站"
    return '三艘搜索艇正在搜索'


def outcome(rows, summary):
    handoff = next((r for r in rows if r['executor_received']),None)
    travel = math.dist(handoff['executor_end'],rows[-1]['executor_end']) if handoff else 0.
    visible = bool(summary['engineering_acceptance_passed'] and summary['capture_evidence']['verified']
                   and summary['found_step'] >= 6 and min(summary['demo_search_displacements_m']) >= .2
                   and handoff and handoff['distance_end_m'] > handoff['radius_m'] and travel >= .5)
    return dict(verified=visible, distance_at_handoff_m=handoff['distance_end_m'] if handoff else None,
                executor_displacement_after_handoff_m=travel,
                final_distance_m=rows[-1]['distance_end_m'] if rows else None)

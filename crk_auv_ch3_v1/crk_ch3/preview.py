"""G3: static, privileged scene preview. No motion, perception or actor simulation."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from .contracts import ROLES, canonical_hash, validate_experiment
from .scenario import Frame, project_truth_scene, select_scene

ROOT = Path(__file__).resolve().parents[1]


def child(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node


def shape_model(world, name, point, shape, dimensions, color, yaw=0, collision=False):
    model = child(world, 'model', name=name)
    child(model, 'static', 'true')
    child(model, 'pose', ' '.join(map(str, (*point, 0, 0, yaw))))
    link = child(model, 'link', name='marker_link')
    visual = child(link, 'visual', name='visual')
    geometry = child(child(visual, 'geometry'), shape)
    for key, value in dimensions.items():
        child(geometry, key, value)
    material = child(visual, 'material')
    child(material, 'ambient', color)
    child(material, 'diffuse', color)
    if collision:
        geometry = child(child(child(link, 'collision', name='collision'), 'geometry'), shape)
        for key, value in dimensions.items():
            child(geometry, key, value)


def build_preview(scene, frame=Frame()):
    view = project_truth_scene(scene, frame)
    sdf = ET.Element('sdf', version='1.9')
    world = child(sdf, 'world', name='crk_ch3_preview')
    child(world, 'gravity', '0 0 -9.81')
    physics = child(world, 'physics', name='physics', type='ignored')
    child(physics, 'max_step_size', '0.002')
    for short, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                        ('scene-broadcaster', 'SceneBroadcaster')]:
        child(world, 'plugin', filename=f'gz-sim-{short}-system', name=f'gz::sim::systems::{name}')
    setting = child(world, 'scene')
    child(setting, 'ambient', '.65 .65 .65 1')
    child(setting, 'background', '.05 .15 .25 1')
    light = child(world, 'light', name='sun', type='directional')
    child(light, 'diffuse', '.9 .9 .9 1')
    child(light, 'direction', '-.5 .2 -1')
    child(light, 'cast_shadows', 'true')
    floor = frame.position((10, 10, -.5))
    shape_model(world, 'workspace_floor', floor, 'box', {'size': '20 20 .05'},
                '.2 .3 .3 1', yaw=frame.yaw, collision=True)
    colors = ['.1 .7 1 1', '.2 1 .3 1', '.8 .3 1 1', '1 .5 .05 1']
    for index, (position, role, color) in enumerate(zip(view['agents'], ROLES, colors)):
        shape_model(world, f'auv_{index}_preview_{role}', position, 'box', {'size': '.6 .3 .1'},
                    color, yaw=frame.yaw)
    for index, box in enumerate(view['obstacles']):
        shape_model(world, f'obstacle_{index}', box['center'], 'box',
                    {'size': ' '.join(map(str, box['size']))}, '.7 .35 .2 1', box['yaw'], collision=True)
    shape_model(world, 'target_truth_marker', view['target_position'], 'sphere', {'radius': '.15'}, '1 1 0 1')
    shape_model(world, 'executor_wait_marker', view['wait_point'], 'sphere', {'radius': '.12'}, '0 1 1 1')
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode'), view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--fixture', action='store_true', help='Engineering layout only, not a thesis scenario')
    choice.add_argument('--manifest', type=Path, help='Native exported scenario manifest')
    parser.add_argument('--scenario-id')
    args = parser.parse_args()
    config = validate_experiment(json.loads((ROOT/'config/experiment.json').read_text(encoding='utf-8')))
    lock = json.loads((ROOT/'config/source_lock.json').read_text(encoding='utf-8'))
    if canonical_hash(lock['files']) != lock['files_sha256']:
        raise ValueError('Source lock inventory was modified')
    if args.fixture:
        if args.scenario_id:
            parser.error('--scenario-id requires --manifest')
        scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text(encoding='utf-8'))
    else:
        scene = select_scene(json.loads(args.manifest.read_text(encoding='utf-8')), args.scenario_id)
    xml, truth = build_preview(scene)
    output_root = Path(os.environ.get('CRK_OUTPUT_ROOT', '/home/legion/AUV-simulation')).resolve()
    if Path('/.dockerenv').is_file() and not output_root.is_mount():
        raise RuntimeError('Host output root is not mounted')
    parent = output_root/'results/ch3'
    parent.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='scenecheck_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_', dir=parent))
    output.chmod(0o755)
    for name, value in [('scenario.json', scene), ('truth_scene.json', truth), ('resolved_config.json', config),
                        ('summary.json', {'stage': 'G3', 'run_kind': 'static_scene_preview',
                                          'scenario_sha256': canonical_hash(scene),
                                          'source_files_sha256': lock['files_sha256'],
                                          'engineering_fixture': args.fixture or scene.get('purpose') == 'engineering_fixture_not_thesis_scenario',
                                          'algorithm_runtime_imported': False,
                                          'mission_executed': False, 'checkpoint_loaded': False,
                                          'performance_claim': False, 'linux_sdf_checked': False,
                                          'gui_confirmed': False})]:
        (output/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'world.sdf').write_text(xml+'\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()

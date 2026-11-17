"""G6 keeps the G4 plant; adds contact-event sensors and privileged target display."""
import copy
import json
import xml.etree.ElementTree as ET
from .vehicle import ROOT
from .motion_world import WORLD, build_motion_world, bridge_yaml, plugin
from .preview import child
from .scenario import Frame, validate_scene


def fixture(mode):
    if mode not in ('smoke', 'capture', 'search'):
        raise ValueError('Unknown G6 mode')
    scene = json.loads((ROOT/'tests/fixtures/scene.json').read_text(encoding='utf-8'))
    scene = copy.deepcopy(scene)
    scene.update(planner_seed=2729, scenario_id='g6_'+mode+'_engineering',
                 purpose='g6_engineering_fixture_not_performance_scenario')
    if mode == 'capture':
        scene['initial_agent_positions'][3] = [4.8, 2, 2]
        scene.update(initial_executor_wait_point=[4.8, 2, 2], target_position=[4.1, 2, 2],
                     target_initial_position=[4.1, 2, 2], target_initial_velocity=[.1, 0, 0])
    return validate_scene(scene)


def build_search_world(scene, profile, frame=Frame()):
    text, truth = build_motion_world(scene, profile, frame)
    sdf = ET.fromstring(text)
    world = sdf.find('world')
    plugin(world, 'contact', 'Contact')
    for i in range(4):
        base = world.find(f"model[@name='auv_{i}']/link[@name='base_link']")
        sensor = child(base, 'sensor', name='hull_contact', type='contact')
        contact = child(sensor, 'contact')
        child(contact, 'collision', 'displacement_hull')
        child(contact, 'topic', f'/auv_{i}/contacts')
        child(sensor, 'always_on', 'true')
        child(sensor, 'update_rate', '500')
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode'), truth


def search_bridge_yaml():
    value = bridge_yaml()
    value += ('- ros_topic_name: "/g6/world_stats"\n'
              f'  gz_topic_name: "/world/{WORLD}/stats"\n'
              '  ros_type_name: "ros_gz_interfaces/msg/WorldStatistics"\n'
              '  gz_type_name: "gz.msgs.WorldStatistics"\n'
              '  direction: GZ_TO_ROS\n  lazy: false\n')
    for i in range(4):
        value += (f'- ros_topic_name: "/auv_{i}/contacts"\n'
                  f'  gz_topic_name: "/auv_{i}/contacts"\n'
                  '  ros_type_name: "ros_gz_interfaces/msg/Contacts"\n'
                  '  gz_type_name: "gz.msgs.Contacts"\n'
                  '  direction: GZ_TO_ROS\n  lazy: false\n  publisher_queue: 100\n')
    return value

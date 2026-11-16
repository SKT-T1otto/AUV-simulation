"""G4B: four dynamic literature-parameter vehicles and isolated ROS/Gazebo topics."""
import json
import xml.etree.ElementTree as ET
import numpy as np

from .preview import build_preview, child
from .scenario import Frame, project_truth_scene
from .vehicle import allocation, base_inertial, thrusters

WORLD = 'crk_ch3_motion'


def plugin(parent, short, name):
    return child(parent, 'plugin', filename=f'gz-sim-{short}-system', name=f'gz::sim::systems::{name}')


def geometry(parent, shape, fields):
    node = child(child(parent, 'geometry'), shape)
    for key, value in fields.items():
        child(node, key, value)


def visual(link, name, shape, fields, color, pose='0 0 0 0 0 0'):
    node = child(link, 'visual', name=name)
    child(node, 'pose', pose)
    geometry(node, shape, fields)
    material = child(node, 'material')
    child(material, 'ambient', color)
    child(material, 'diffuse', color)


def inertial(link, mass, center, tensor):
    node = child(link, 'inertial')
    child(node, 'pose', ' '.join(map(str, (*center, 0, 0, 0))))
    child(node, 'mass', mass)
    inertia = child(node, 'inertia')
    for name, i, j in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                        ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        child(inertia, name, float(tensor[i, j]))


def build_motion_world(scene, profile, frame=Frame()):
    allocation(profile)
    xml, truth = build_preview(scene, frame)
    sdf = ET.fromstring(xml)
    # Native fluid_added_mass is part of SDFormat 1.11 (Harmonic / sdformat14).
    sdf.set('version', '1.11')
    world = sdf.find('world')
    world.set('name', WORLD)
    physics = world.find("plugin[@name='gz::sim::systems::Physics']")
    child(child(physics, 'engine'), 'filename', 'gz-physics-dartsim-plugin')
    world.find('gravity').text = f"0 0 -{profile['gravity_m_s2']}"
    for model in list(world.findall('model')):
        if model.get('name').startswith('auv_'):
            world.remove(model)
    buoyancy = plugin(world, 'buoyancy', 'Buoyancy')
    child(buoyancy, 'uniform_fluid_density', profile['density_kg_m3'])
    geometry_data = thrusters(profile)
    mb, cg, ib = base_inertial(profile)
    colors = ['.1 .7 1 1', '.2 1 .3 1', '.8 .3 1 1', '1 .5 .05 1']
    for agent, (position, color) in enumerate(zip(truth['agents'], colors)):
        name = f'auv_{agent}'
        child(buoyancy, 'enable', name)
        model = child(world, 'model', name=name, canonical_link='base_link')
        child(model, 'static', 'false')
        child(model, 'self_collide', 'false')
        child(model, 'pose', ' '.join(map(str, (*position, 0, 0, 0))))
        base = child(model, 'link', name='base_link')
        inertial(base, mb, cg, ib)
        # Positive physical added inertia, integrated by DART with rigid inertia.
        # Legacy hydrodynamics *Dot* terms instead use lagged acceleration and
        # diverge for this profile's heave added-mass / rigid-mass ratio > 1.
        added = child(base.find('inertial'), 'fluid_added_mass')
        for axis, mass in zip(('xx', 'yy', 'zz', 'pp', 'qq', 'rr'), profile['added_mass']):
            child(added, axis, mass)
        collision = child(base, 'collision', name='displacement_hull')
        cb = profile['center_of_buoyancy_flu_m']
        child(collision, 'pose', ' '.join(map(str, (*cb, 0, 0, 0))))
        length, width = .457, .338
        height = profile['volume_m3']/(length*width)
        geometry(collision, 'box', {'size': f'{length} {width} {height}'})
        visual(base, 'pressure_hull', 'cylinder', {'radius': .07, 'length': .38}, color,
               '0 0 .02 0 1.5707963267948966 0')
        visual(base, 'buoyancy_visual', 'box', {'size': '.4 .23 .06'}, '.8 .85 .8 1', '0 0 .1 0 0 0')
        visual(base, 'nose', 'box', {'size': '.08 .025 .025'}, '1 .1 .1 1', '.24 0 .02 0 0 0')
        for index, (point, direction) in enumerate(geometry_data):
            rotor_name = f'thruster_{index}'
            rotor = child(model, 'link', name=rotor_name)
            child(rotor, 'pose', ' '.join(map(str, (*point, 0, 0, 0))))
            inertial(rotor, profile['rotor_link_mass_kg'], [0, 0, 0],
                     np.eye(3)*profile['rotor_link_inertia_kg_m2'])
            pitch = 0 if direction[2] > .5 else np.pi/2
            yaw = 0 if direction[2] > .5 else np.arctan2(direction[1], direction[0])
            visual(rotor, 'propeller', 'cylinder', {'radius': profile['propeller_diameter_m']/2, 'length': .02},
                   '.1 .1 .1 1', f'0 0 0 0 {pitch} {yaw}')
            joint = child(model, 'joint', name=rotor_name+'_joint', type='revolute')
            child(joint, 'parent', 'base_link')
            child(joint, 'child', rotor_name)
            child(child(joint, 'axis'), 'xyz', ' '.join(map(str, direction)))
            motor = plugin(model, 'thruster', 'Thruster')
            for key, value in {'namespace': name, 'joint_name': rotor_name+'_joint',
                               'thrust_coefficient': .01, 'fluid_density': profile['density_kg_m3'],
                               'propeller_diameter': profile['propeller_diameter_m'], 'velocity_control': 'true',
                               'max_thrust_cmd': profile['test_thrust_limit_N'],
                               'min_thrust_cmd': -profile['test_thrust_limit_N']}.items():
                child(motor, key, value)
        hydro = plugin(model, 'hydrodynamics', 'Hydrodynamics')
        child(hydro, 'link_name', 'base_link')
        child(hydro, 'default_current', '0 0 0')
        child(hydro, 'disable_added_mass', 'true')
        # Native DART inertia supplies added-mass Coriolis as well. Do not apply
        # the legacy plugin's Coriolis a second time (zero current in G4).
        child(hydro, 'disable_coriolis', 'true')
        for axis, velocity, linear, quadratic in zip('xyzkmn', 'UVWPQR', profile['linear_drag'],
                                                    profile['quadratic_drag']):
            child(hydro, axis+velocity, -linear)
            child(hydro, axis+velocity+'abs'+velocity, -quadratic)
        odom = plugin(model, 'odometry-publisher', 'OdometryPublisher')
        for key, value in {'dimensions': 3, 'odom_frame': 'map', 'robot_base_frame': name+'/base_link',
                           'odom_publish_frequency': 50, 'odom_topic': '/model/'+name+'/odometry'}.items():
            child(odom, key, value)
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode'), truth


def bridge_entries():
    entries = [('/clock', f'/world/{WORLD}/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS')]
    for agent in range(4):
        entries.append((f'/auv_{agent}/odom', f'/model/auv_{agent}/odometry',
                        'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'))
        for index in range(8):
            entries.append((f'/auv_{agent}/thruster_{index}/cmd_thrust',
                            f'/model/auv_{agent}/joint/thruster_{index}_joint/cmd_thrust',
                            'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
    return entries


def bridge_yaml():
    lines = []
    for ros, gz, rt, gt, direction in bridge_entries():
        lines += [f'- ros_topic_name: "{ros}"', f'  gz_topic_name: "{gz}"',
                  f'  ros_type_name: "{rt}"', f'  gz_type_name: "{gt}"',
                  f'  direction: {direction}', '  lazy: false']
    return '\n'.join(lines)+'\n'

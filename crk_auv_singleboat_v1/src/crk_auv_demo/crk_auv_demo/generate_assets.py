"""Generate SDF, bridge YAML and allocation geometry from one JSON source."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def element(parent, tag, value=None, **attributes):
    result = ET.SubElement(parent, tag, attributes)
    if value is not None:
        result.text = str(value)
    return result


def plugin(parent, short, class_name):
    return element(parent, 'plugin', filename='gz-sim-'+short+'-system',
                   name='gz::sim::systems::'+class_name)


def geometry(parent, kind, size):
    shape = element(element(parent, 'geometry'), kind)
    for tag, value in size.items():
        element(shape, tag, value)


def visual(parent, name, kind, size, color, pose='0 0 0 0 0 0'):
    v = element(parent, 'visual', name=name)
    element(v, 'pose', pose)
    geometry(v, kind, size)
    mat = element(v, 'material')
    element(mat, 'ambient', color)
    element(mat, 'diffuse', color)


def generate(package):
    package = Path(package)
    c = json.loads((package/'config/vehicle.json').read_text())
    points = json.loads((package/'config/waypoints.json').read_text())['points']
    sdf = ET.Element('sdf', {'version': '1.9'})
    world = element(sdf, 'world', name='crk_pool')
    element(world, 'gravity', '0 0 -9.81')
    physics = element(world, 'physics', name='physics', type='ignored')
    element(physics, 'max_step_size', 0.002)
    element(physics, 'real_time_factor', 1.)
    plugin(world, 'physics', 'Physics')
    plugin(world, 'user-commands', 'UserCommands')
    plugin(world, 'scene-broadcaster', 'SceneBroadcaster')
    buoy = plugin(world, 'buoyancy', 'Buoyancy')
    element(buoy, 'uniform_fluid_density', c['fluid_density_kg_m3'])
    element(buoy, 'enable', 'auv_0')
    scene = element(world, 'scene')
    element(scene, 'ambient', '0.55 0.65 0.75 1')
    element(scene, 'background', '0.08 0.2 0.3 1')
    light = element(world, 'light', name='sun', type='directional')
    element(light, 'pose', '0 0 8 0 0 0')
    element(light, 'diffuse', '0.9 0.9 0.9 1')
    element(light, 'specular', '0.1 0.1 0.1 1')
    element(light, 'direction', '-0.5 0.2 -1')
    floor = element(world, 'model', name='seafloor')
    element(floor, 'static', 'true')
    element(floor, 'pose', '0 0 -6 0 0 0')
    floor_link = element(floor, 'link', name='floor')
    geometry(element(floor_link, 'collision', name='floor_collision'),
             'plane', {'normal': '0 0 1', 'size': '24 24'})
    visual(floor_link, 'floor_visual', 'plane',
           {'normal': '0 0 1', 'size': '24 24'}, '0.25 0.3 0.3 1')
    for i, point in enumerate(points):
        marker = element(world, 'model', name='waypoint_'+str(i))
        element(marker, 'static', 'true')
        element(marker, 'pose', ' '.join(map(str, point))+' 0 0 0')
        visual(element(marker, 'link', name='marker'), 'sphere', 'sphere',
               {'radius': 0.08}, '1 0.65 0.05 1')
    model = element(world, 'model', name='auv_0', canonical_link='base_link')
    element(model, 'pose', '0 0 -2 0 0 0')
    element(model, 'self_collide', 'false')
    base = element(model, 'link', name='base_link')
    inertia = element(base, 'inertial')
    hull_mass = c['mass_kg']-len(c['thrusters'])*c['propeller_mass_kg']
    element(inertia, 'mass', hull_mass)
    x, y, z = c['hull_size_m']
    I = element(inertia, 'inertia')
    for tag, value in zip(('ixx', 'iyy', 'izz'),
                          (hull_mass*(y*y+z*z)/12,
                           hull_mass*(x*x+z*z)/12,
                           hull_mass*(x*x+y*y)/12)):
        element(I, tag, value)
    for tag in ('ixy', 'ixz', 'iyz'):
        element(I, tag, 0)
    box = {'size': ' '.join(map(str, c['hull_size_m']))}
    geometry(element(base, 'collision', name='displacement_hull'), 'box', box)
    visual(base, 'hull', 'box', box, '0.08 0.35 0.95 1')
    visual(base, 'heading', 'box', {'size': '0.15 0.04 0.03'},
           '1 0.15 0.1 1', '0.3 0 0.08 0 0 0')
    for i, t in enumerate(c['thrusters']):
        name = 'thruster_'+str(i)
        propeller = element(model, 'link', name=name)
        element(propeller, 'pose', ' '.join(map(str, t['position']))+' 0 0 0')
        inertial = element(propeller, 'inertial')
        element(inertial, 'mass', c['propeller_mass_kg'])
        I = element(inertial, 'inertia')
        for tag in ('ixx', 'iyy', 'izz'):
            element(I, tag, 0.00002)
        for tag in ('ixy', 'ixz', 'iyz'):
            element(I, tag, 0)
        dx, dy, dz = t['direction']
        pitch = 0 if dz else math.pi/2
        yaw = 0 if dz else math.atan2(dy, dx)
        visual(propeller, 'rotor', 'cylinder', {'radius': 0.045, 'length': 0.025},
               '0.12 0.12 0.12 1', f'0 0 0 0 {pitch} {yaw}')
        joint = element(model, 'joint', name=name+'_joint', type='revolute')
        element(joint, 'parent', 'base_link')
        element(joint, 'child', name)
        axis = element(joint, 'axis')
        element(axis, 'xyz', ' '.join(map(str, t['direction'])))
        dynamics = element(axis, 'dynamics')
        element(dynamics, 'damping', 0)
        thruster = plugin(model, 'thruster', 'Thruster')
        for tag, value in {
            'namespace': 'auv_0', 'joint_name': name+'_joint',
            'thrust_coefficient': 0.01, 'fluid_density': c['fluid_density_kg_m3'],
            'propeller_diameter': 0.1, 'velocity_control': 'true',
            'max_thrust_cmd': c['thrust_limit_N'],
            'min_thrust_cmd': -c['thrust_limit_N']}.items():
            element(thruster, tag, value)
    hydro = plugin(model, 'hydrodynamics', 'Hydrodynamics')
    element(hydro, 'link_name', 'base_link')
    element(hydro, 'default_current', '0 0 0')
    # Conservative legacy plugin added mass for compatibility across Sim 8 minors.
    # No <fluid_added_mass> is present, so there is no double-counting.
    for axis, vel, lin, quad, added in zip(
            'xyzkmn', 'UVWPQR', c['linear_drag'], c['quadratic_drag'], c['added_mass']):
        element(hydro, axis+vel, -lin)
        element(hydro, axis+vel+'abs'+vel, -quad)
        element(hydro, axis+'Dot'+vel, -added)
    odom = plugin(model, 'odometry-publisher', 'OdometryPublisher')
    for tag, value in {'dimensions': 3, 'odom_frame': 'map',
                       'robot_base_frame': 'auv_0/base_link',
                       'odom_publish_frequency': 50,
                       'odom_topic': '/model/auv_0/odometry'}.items():
        element(odom, tag, value)
    (package/'worlds').mkdir(exist_ok=True)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(package/'worlds/single_auv.sdf', encoding='utf-8', xml_declaration=True)
    bridge = [('/clock', '/world/crk_pool/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
              ('/auv_0/odom', '/model/auv_0/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS')]
    for i in range(8):
        bridge.append((f'/auv_0/thruster_{i}/cmd_thrust',
                       f'/model/auv_0/joint/thruster_{i}_joint/cmd_thrust',
                       'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
    lines = []
    for ros, gz, ros_type, gz_type, direction in bridge:
        lines += [f'- ros_topic_name: "{ros}"', f'  gz_topic_name: "{gz}"',
                  f'  ros_type_name: "{ros_type}"', f'  gz_type_name: "{gz_type}"',
                  f'  direction: {direction}', '  lazy: false']
    (package/'config/bridge.yaml').write_text('\n'.join(lines)+'\n', encoding='utf-8', newline='\n')


if __name__ == '__main__':
    import sys
    generate(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1])

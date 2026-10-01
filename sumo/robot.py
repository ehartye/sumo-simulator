"""Build MuJoCo collision bodies from Agent Meshes' physical export, in SI units."""
import json
import math
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def numbers(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def wxyz(quat):
    return [quat[3], *quat[:3]]


def multiply(a, b):
    result = np.zeros(4)
    mujoco.mju_mulQuat(result, np.array(a, dtype=float), np.array(b, dtype=float))
    return result


def rotate(quat, value):
    result = np.zeros(3)
    mujoco.mju_rotVecQuat(result, np.array(value, dtype=float), np.array(quat, dtype=float))
    return result


def load_robot(path=None):
    path = Path(path or ROOT / 'artifacts/robot/robot.json')
    if not path.exists():
        raise FileNotFoundError('Build the Agent Meshes robot first: npm run build:robot')
    robot = json.loads(path.read_text(encoding='utf-8'))
    source = json.loads((ROOT / 'hardware/minimal-spike.json').read_text())
    # The minimal build is a fixed stock assembly. No hidden extra hardware/ballast.
    expected = {p['name']: p for p in source['parts']}
    if len(robot['parts']) != len(expected) or {p['name'] for p in robot['parts']} != set(expected):
        raise ValueError('Robot violates the minimal-build component inventory')
    for part in robot['parts']:
        allowed = expected[part['name']]
        for key in ('ldraw', 'massKg', 'body', 'position', 'rotation', 'colliders'):
            if part[key] != allowed[key]:
                raise ValueError(f'Minimal-build mismatch: {part["name"]}.{key}')
    if robot['bodies'] != source['bodies']:
        # Exporters may enrich bodies with aggregate fields, but not move joints.
        for original in source['bodies']:
            body = next((b for b in robot['bodies'] if b['name'] == original['name']), None)
            if body is None or any(body.get(k) != v for k, v in original.items()):
                raise ValueError('Minimal-build body or joint mismatch')
            for key, default in (('rotation', [0, 0, 0, 1]), ('hingeAxis', None), ('ballJoint', None)):
                if body.get(key, default) != original.get(key, default):
                    raise ValueError(f'Minimal-build joint mismatch: {original["name"]}.{key}')
        if len(robot['bodies']) != len(source['bodies']):
            raise ValueError('Unexpected robot body')
    return robot


def make_model(robot, arena, step, friction, outward_start=False):
    root = Element('mujoco', model='SPIKE sumo')
    SubElement(root, 'compiler', angle='radian', autolimits='true')
    SubElement(root, 'option', timestep=str(step), gravity='0 -9.81 0', integrator='implicitfast',
               cone='elliptic', iterations='80', tolerance='1e-10', impratio='10')
    default = SubElement(root, 'default')
    SubElement(default, 'geom', solref='0.004 1', solimp='0.95 0.99 0.001', margin='0.0001')
    world = SubElement(root, 'worldbody')
    ground_rotation = numbers([math.sqrt(.5), -math.sqrt(.5), 0, 0])
    SubElement(world, 'geom', name='outside_floor', type='plane', size='3 3 0.05',
               quat=ground_rotation, contype='1', conaffinity='6', friction=f'{friction} 0.001 0.0001')
    SubElement(world, 'geom', name='ring', type='cylinder', size=numbers([arena.radius, arena.height/2]),
               pos=numbers([0, arena.height/2, 0]), quat=ground_rotation, contype='1', conaffinity='6',
               friction=f'{friction} 0.001 0.0001')
    for bot in range(2):
        prefix = f'b{bot}_'
        elements = {}
        body_map = {body['name']: body for body in robot['bodies']}
        def depth(body):
            return 0 if body['parent'] is None else 1 + depth(body_map[body['parent']])
        for body in sorted(robot['bodies'], key=depth):
            parent = elements.get(body['parent'], world)
            position = body['position']
            quat = wxyz(body.get('rotation', [0, 0, 0, 1]))
            if body['parent'] is None:
                offset = .1524 if outward_start else .23
                position = [0, arena.height + .001, -offset if bot == 0 else offset]
                quat = [1, 0, 0, 0] if (bot == 0) != outward_start else [0, 0, 1, 0]
            element = SubElement(parent, 'body', name=prefix+body['name'], pos=numbers(position), quat=numbers(quat))
            elements[body['name']] = element
            if body['parent'] is None:
                SubElement(element, 'freejoint', name=prefix+'root')
            elif body.get('hingeAxis'):
                SubElement(element, 'joint', name=prefix+body['name'], type='hinge', axis=numbers(body['hingeAxis']),
                           damping='0.00002', armature='0.000002')
            elif body.get('ballJoint'):
                SubElement(element, 'joint', name=prefix+body['name'], type='ball', damping='0.000001')
        for part in robot['parts']:
            # Uniform-density proxy inertia is an explicit approximation. Catalog mass is exact input.
            for index, collider in enumerate(part['colliders']):
                part_q = wxyz(part['rotation'])
                pos = np.array(part['position']) + rotate(part_q, collider['position'])
                quat = multiply(part_q, wxyz(collider['rotation']))
                shape = collider['shape']
                if shape == 'box':
                    size = np.array(collider['size'])/2
                elif shape == 'sphere':
                    size = [collider['radius']]
                elif shape == 'cylinder':
                    size = [collider['radius'], collider['height']/2]
                    # Manifest cylinders point along Y; MuJoCo cylinders point along Z.
                    quat = multiply(quat, [math.sqrt(.5), -math.sqrt(.5), 0, 0])
                else:
                    raise ValueError(f'Unsupported collider: {shape}')
                is_tire = part['name'].endswith('_tire')
                coefficient = friction if is_tire else .3
                if part['name'] == 'caster_ball':
                    coefficient = .25
                SubElement(elements[part['body']], 'geom', name=f'{prefix}{part["name"]}_{index}',
                           type=shape, pos=numbers(pos), quat=numbers(quat), size=numbers(size),
                           mass=str(part['massKg']/len(part['colliders'])),
                           priority='1',
                           contype=str(2 if bot == 0 else 4), conaffinity=str(5 if bot == 0 else 3),
                           friction=f'{coefficient} 0.0005 0.00001', condim='6')
    return tostring(root, encoding='unicode')

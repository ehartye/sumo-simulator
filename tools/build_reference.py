"""Author the reference chassis from actual LDraw hole coordinates, in millimeters.

Run before npm run build:robot. No placements are inferred from bounding-box contact.
Each pin joins two measured primitive hole centers one stud (8 mm) apart.
"""
from collections import Counter
from functools import lru_cache
import json
from pathlib import Path
import math

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT/'assets/ldraw'
C = np.diag([1., -1., -1.])
I = np.eye(3)


def rx(degrees):
    a = math.radians(degrees)
    return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])


def ry(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])


def rz(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])


def rounded(value):
    return np.round(value, 10).tolist()


def quaternion(matrix):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, matrix.reshape(9))
    return rounded([*q[1:], q[0]])


def locate(ref):
    ref = ref.lower().replace('\\','/')
    return next(p for p in (LIB/'parts'/ref, LIB/'p'/ref, LIB/ref) if p.is_file())


@lru_cache(None)
def geometry(ref):
    vertices, holes = [], []
    for line in locate(ref).read_text(errors='replace').splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] in ('3','4'):
            vertices.extend(np.array(list(map(float, f[2:]))).reshape(-1,3))
        elif f[0] == '1':
            values = np.array(list(map(float, f[2:14])))
            offset, matrix, child = values[:3], values[3:].reshape(3,3), ' '.join(f[14:]).lower().replace('\\','/')
            subvertices, subholes = geometry(child)
            vertices.extend(subvertices@matrix.T+offset)
            if child in ('connhol3.dat','connhole.dat','beamhole.dat'):
                holes.append((offset, matrix@np.array([0,1,0]), f'{ref} â†’ {child}'))
            elif child == 'peghole.dat':
                holes.append((offset+matrix@np.array([0,10,0]), matrix@np.array([0,1,0]), f'{ref} â†’ peghole.dat face plus 10 LDU depth to engagement center'))
            else:
                holes.extend((matrix@p+offset, matrix@a, source) for p,a,source in subholes)
    return np.array(vertices).reshape(-1,3), holes


def source(part):
    return 'https://www.bricklink.com/v2/catalog/catalogitem.page?P='+part


def build():
    parts, placements, connections = [], {}, []
    bodies = [{'name':'chassis','parent':None,'position':[0,0,0]},
              *[{'name':side+'_wheel','parent':'chassis','position':[sign*.040,.028,.032],'hingeAxis':[1,0,0]} for side,sign in [('left',-1),('right',1)]],
              {'name':'caster','parent':'chassis','position':[0,.0104,-.056],'ballJoint':True}]
    body_origins = {b['name']:np.array(b['position'])*1000 for b in bodies}

    def add(name, part, mass, color, origin, matrix=C, body='chassis', catalog=None):
        vertices, _ = geometry(part+'.dat')
        minimum, maximum = vertices.min(0)*.4, vertices.max(0)*.4
        center = (minimum+maximum)/2
        local_center = C@center
        rotation = matrix@C
        world_center = np.array(origin)+matrix@center
        item = {'name':name,'ldraw':part+'.dat','color':color,'body':body,
                'position':rounded((world_center-body_origins[body])/1000), 'rotation':quaternion(rotation),
                'massKg':mass,'massSource':source(catalog or part), 'connectors':[],
                'colliders':[{'shape':'box','size':rounded((maximum-minimum)/1000),'position':[0,0,0],'rotation':[0,0,0,1]}]}
        parts.append(item)
        placements[name] = (item, np.array(origin,dtype=float), matrix, center)
        return item

    def anchor(name, raw, axis, kind='pin-hole', label=None, note=None):
        item, origin, matrix, center = placements[name]
        raw, axis = np.array(raw,dtype=float), np.array(axis,dtype=float)
        if kind == 'pin-hole':
            matches = [(p,a,s) for p,a,s in geometry(item['ldraw'])[1] if np.linalg.norm(p*.4-raw)<.001 and abs(np.dot(a/np.linalg.norm(a),axis/np.linalg.norm(axis)))>1-1e-7]
            if not matches:
                raise ValueError(f'No actual LDraw hole at {name} {raw} / {axis}')
            note = matches[0][2]
        key = label or f'attachment_{len(item["connectors"])}'
        point = rounded(C@(raw-center)/1000)
        if any(np.linalg.norm(np.array(c['position'])-point) < 1e-7 for c in item['connectors']):
            raise ValueError(f'Physical connector reused: {name} {raw}')
        item['connectors'].append({'name':key,'kind':kind,'position':point,
                                   'axis':rounded(C@(axis/np.linalg.norm(axis))),
                                   'source':note or f'LDraw {item["ldraw"]}: nominal shaft engagement geometry'})
        return {'part':name,'connector':key}, origin+matrix@raw, matrix@(axis/np.linalg.norm(axis))

    def hole(name, raw, axis):
        return anchor(name, raw, axis)

    def pin(name, first, second, body='chassis'):
        ea, a, axis = first
        eb, b, other = second
        if not np.isclose(np.linalg.norm(b-a),8,atol=.001) or abs(np.dot(axis,other)) < 1-1e-7:
            raise ValueError(f'Pin {name} must bridge aligned holes 8mm apart: {a} â†’ {b}')
        direction=(b-a)/8
        if np.linalg.norm(np.cross(direction,axis)) > 1e-7:
            raise ValueError(f'Pin {name} cannot join offset hole axes')
        helper=np.array([0,1,0]) if abs(direction[1])<.9 else np.array([0,0,1])
        y=np.cross(helper,direction);y/=np.linalg.norm(y)
        matrix=np.column_stack([direction,y,np.cross(direction,y)])
        item = add(name,'2780',.00015,0,(a+b)/2,matrix,body)
        item['massNote']='Catalog approximate 0.15 g for the modeled slotted mold; unslotted variants are 0.20 g.'
        pa=anchor(name,[-4,0,0],[1,0,0],'pin','end_a')[0]
        pb=anchor(name,[4,0,0],[1,0,0],'pin','end_b')[0]
        connections.extend([{'a':ea,'b':pa},{'a':eb,'b':pb}])

    hub=add('hub','45601c01',.14982,15,[0,40,0],catalog='53444c01')
    hub['massComponents']=[{'name':'hub_without_battery','massKg':.06312,'massSource':source('53444c01')}, {'name':'battery','massKg':.08670,'massSource':source('55422c01')}]
    hub['massNote']='Battery counted once; white hub face and molded yellow battery/body retained.'
    for side,sign in [('left',-1),('right',1)]:
        motor=side+'_motor'
        add(motor,'54696p01',.04953,15,[sign*16,28,32],rz(-sign*90)@C,catalog='54696c01')
        for i,(raw_z,world_z) in enumerate([(-8,40),(8,24),(56,-24)]):
            pin(f'{side}_hub_pin_{i}',hole(motor,[-sign*8,-8,raw_z],[1,0,0]),hole('hub',[sign*24,-4,-world_z],[0,1,0]))
        wheel=add(side+'_tire','39367p01',.01430,322,[sign*40,28,32],ry(-sign*90)@C,side+'_wheel','39367pb01')
        wheel['colliders']=[{'shape':'cylinder','radius':.028,'height':.014,'position':[0,0,0],'rotation':quaternion(rx(90))}]
        # Two pins index and retain each stock wheel on the motor output disc.
        for i,raw_x in enumerate([-8,8]):
            y=-sign*raw_x
            pin(f'{side}_wheel_pin_{i}',hole(motor,[raw_x,-16,0],[0,1,0]),hole(side+'_tire',[0,-y,0],[0,0,1]),side+'_wheel')
        shaft=side+'_axle'
        add(shaft,'4519',.00043,14,[sign*40,28,32],I,side+'_wheel')
        for target,raw,axis in [(motor,[0,-16,0],[0,1,0]),(side+'_tire',[0,0,0],[0,0,1])]:
            endpoint, world, _=anchor(target,raw,axis,'axle-hole',note='LDraw axl3hole.dat: anchor within the actual coaxial engagement interval')
            _,o,m,c=placements[shaft]
            shaftpoint=m.T@(world-o)
            mate=anchor(shaft,shaftpoint,[1,0,0],'axle')[0]
            connections.append({'a':endpoint,'b':mate})

    # Two real 5x7 frames brace the sensor bracket and rear caster to hub end holes.
    frame_matrix=rx(90)
    for end,sign in [('front',1),('rear',-1)]:
        frame_y = 36 if end == 'front' else 52
        frame=add(end+'_frame','64179',.0053,0,[0,frame_y,sign*48],frame_matrix)
        # Local converted frame is XZ, with a real open center, not a solid box.
        frame['colliders']=[{'shape':'box','size':size,'position':pos,'rotation':[0,0,0,1]} for size,pos in [([.008,.008,.056],[-.016,0,0]),([.008,.008,.056],[.016,0,0]),([.024,.008,.008],[0,0,-.024]),([.024,.008,.008],[0,0,.024])]]
        for x in [-16,16]:
            for y in ([44] if end == 'front' else [44,60]):
                pin(f'{end}_frame_pin_{x}_{y}',hole('hub',[x,40-y,-sign*40],[0,0,1]),hole(end+'_frame',[x,0,frame_y-y],[0,1,0]))

    # Eye bridge: two pins into the frame and two into the sensor's rear sockets.
    beam_matrix=np.array([[0,0,1],[1,0,0],[0,1,0]],dtype=float)
    add('eyes_bridge','32524',.00174,322,[0,60,56],beam_matrix)
    for x in [-16,16]:
        pin_name=f'eyes_bridge_pin_{x}'
        add(pin_name,'6558',.00037,1,[x,60,48],ry(-90))
        for raw_x,target in [(-8,hole('hub',[x,-20,-40],[0,0,1])),
                             (0,hole('front_frame',[x,0,-24],[0,1,0])),
                             (8,hole('eyes_bridge',[0,0,x],[0,1,0]))]:
            mate=anchor(pin_name,[raw_x,0,0],[1,0,0],'pin')[0]
            connections.append({'a':target[0],'b':mate})
    add('eyes','37316',.03438,15,[0,60,64],C,catalog='37316c01')
    for x in [-24,24]:
        pin(f'eyes_pin_{x}',hole('eyes_bridge',[0,0,x],[0,1,0]),hole('eyes',[x,0,0],[0,0,1]))

    # Perpendicular connector blocks turn the front frame's Z-axis joints into
    # X-axis joints that hold the downward color sensor by real side holes.
    add('color_bridge','32524',.00174,322,[0,28,56],beam_matrix)
    add('color_sensor','37308',.01285,15,[0,36,72],rx(-90),catalog='37308c01')
    for side,sign in [('left',-1),('right',1)]:
        block=side+'_sensor_block'
        add(block,'39793',.002,0,[sign*24,28,64],rx(90))
        # A 3L pin fills all three aligned holes: frame, spacer beam, block.
        pin_name=side+'_sensor_block_pin'
        add(pin_name,'6558',.00037,1,[sign*16,28,56],ry(-90))
        for raw_x,target in [(-8,hole('front_frame',[sign*16,0,8],[0,1,0])),
                             (0,hole('color_bridge',[0,0,sign*16],[0,1,0])),
                             (8,hole(block,[-sign*8,0,0],[0,1,0]))]:
            mate=anchor(pin_name,[raw_x,0,0],[1,0,0],'pin')[0]
            connections.append({'a':target[0],'b':mate})
        pin(side+'_color_pin',hole(block,[-sign*8,0,-8],[1,0,0]),hole('color_sensor',[sign*8,8,0],[1,0,0]))

    add('caster_holder','39370',.00411,322,[0,28,-56],ry(90)@C)
    for x in [-16,16]:
        pin(f'caster_pin_{x}',hole('rear_frame',[x,0,24],[0,1,0]),hole('caster_holder',[0,0,-x],[1,0,0]))
    ball=add('caster_ball','52629',.00499,15,[0,10.4,-56],C,'caster')
    ball['colliders']=[{'shape':'sphere','radius':.0096,'position':[0,0,0],'rotation':[0,0,0,1]}]
    socket=anchor('caster_holder',[0,17.6,0],[0,1,0],'socket',note='39370 socket circular surfaces centered at raw Y=44 LDU')[0]
    ball_anchor=anchor('caster_ball',[0,0,0],[0,1,0],'ball',note='52629 sphere center at raw origin')[0]
    connections.append({'a':socket,'b':ball_anchor})

    manifest={'version':1,'name':'SPIKE connected minimal sumo','libraryPath':'../assets/ldraw','bodies':bodies,'parts':parts,'connections':connections,'requireConnected':True}
    # Map aliases to one selected kit row, never sum replacement alternatives.
    stock = json.loads((ROOT/'hardware/spike-prime-inventory.json').read_text())
    elements = {'45601c01.dat':['6464165','6482346'], '54696p01.dat':['6359210'],
                '2780.dat':['6279875'], '39367p01.dat':['6253244'], '4519.dat':['6130007'],
                '64179.dat':['6016154'], '32524.dat':['6261638'], '37316.dat':['6302968'],
                '37308.dat':['6217705'], '39793.dat':['6265644'], '6558.dat':['4514553'],
                '39370.dat':['6261590'], '52629.dat':['6261647']}
    counts = Counter(p['ldraw'] for p in parts)
    usage = []
    for part, count in counts.items():
        for element in elements[part]:
            row = next(r for r in stock['sourceRows'] if r['ElementID'] == element)
            if count > int(row['Qty']):
                raise ValueError(f'Kit inventory exceeded: {part}')
            usage.append({'ldraw':part,'elementId':element,'name':row['ElementName'],
                          'color':row['Colour'],'used':count,'available':int(row['Qty'])})
    report = {'setNumber':stock['setNumber'],'sourceUrl':stock['provenance']['sourcePage'],
              'notes':['One selected row per mold family; hub and battery are accounted separately.',
                       'Electronics are catalog equivalents of later kit replacement IDs.',
                       '2780 is the slotted pin geometry; its approximate 0.15 g mass differs from unslotted molds.'],
              'usage':usage}
    (ROOT/'hardware/build-inventory.json').write_text(json.dumps(report,indent=2)+'\n')
    (ROOT/'hardware/minimal-spike.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'parts':len(parts),'connections':len(connections),'massKg':sum(p['massKg'] for p in parts),'inventory':dict(Counter(p['ldraw'] for p in parts))},indent=2))


if __name__ == '__main__':
    build()

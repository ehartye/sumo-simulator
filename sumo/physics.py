"""Fixed-step rigid body simulation. Parameters have explicit, inspectable provenance."""
from dataclasses import dataclass
import math

import mujoco
import numpy as np

from .robot import load_robot, make_model, rotate, wxyz


@dataclass(frozen=True)
class Arena:
    # House arena defaults, not an asserted external competition standard.
    radius: float = .385
    height: float = .025
    border: float = .025
    timeout: float = 60


@dataclass(frozen=True)
class MotorModel:
    # LEGO 45603 technical sheet at 7.2V; linear interpolation is uncalibrated.
    stall_torque: float = .18
    free_speed: float = 185 * 2 * math.pi / 60
    nominal_voltage: float = 7.2
    stall_current: float = .8

    def torque(self, duty, speed, voltage):
        duty = float(np.clip(duty, -1, 1))
        torque = self.stall_torque * (duty*voltage/self.nominal_voltage - speed/self.free_speed)
        return float(np.clip(torque, -self.stall_torque, self.stall_torque))


class World:
    def __init__(self, seed=0, arena=None, timestep=.001, friction=.8, robot_path=None, outward_start=False):
        if not 0 < timestep <= .002 or not .05 <= friction <= 2:
            raise ValueError('Invalid timestep or friction')
        self.arena = arena or Arena()
        if self.arena.radius < .25 or self.arena.height <= 0:
            raise ValueError('Arena must accommodate two robots and be raised')
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.robot = load_robot(robot_path)
        self.outward_start = outward_start
        self.xml = make_model(self.robot, self.arena, timestep, friction, outward_start)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.motor = MotorModel()
        self.modes = [[{'mode': 'coast', 'value': 0, 'angle': 0} for _ in range(2)] for _ in range(2)]
        self.joints = [[self.model.joint(f'b{i}_{side}_wheel').id for side in ('left', 'right')] for i in range(2)]
        self.roots = [self.model.body(f'b{i}_chassis').id for i in range(2)]
        self.offsets = np.zeros((2, 2))
        self.voltage = [7.2, 7.2]
        self.currents = [[0., 0.], [0., 0.]]
        self.result = None
        self.sensor_cache = [None, None]
        self.sensor_tick = [-1, -1]
        self.steps = 0
        self.contacts = 0
        mujoco.mj_forward(self.model, self.data)

    def _wheel(self, bot, port):
        if bot not in (0, 1) or port not in ('A', 'B'):
            raise ValueError('Invalid motor port; minimal build motors use A and B')
        index = 0 if port == 'A' else 1
        joint = self.joints[bot][index]
        return index, self.model.jnt_qposadr[joint], self.model.jnt_dofadr[joint]

    def command(self, bot, port, mode, value=0):
        index, q, _ = self._wheel(bot, port)
        if mode not in ('run', 'dc', 'coast', 'brake', 'hold') or not math.isfinite(value):
            raise ValueError('Unsupported or nonfinite motor command')
        self.modes[bot][index] = {'mode': mode, 'value': float(value), 'angle': self.data.qpos[q]}

    def wheel_speed(self, bot, port):
        _, _, dof = self._wheel(bot, port)
        return math.degrees(self.data.qvel[dof])

    def wheel_angle(self, bot, port):
        index, q, _ = self._wheel(bot, port)
        return math.degrees(self.data.qpos[q]) - self.offsets[bot, index]

    def reset_angle(self, bot, port, angle):
        index, q, _ = self._wheel(bot, port)
        self.offsets[bot, index] = math.degrees(self.data.qpos[q])-angle

    def pose(self, bot):
        root = self.roots[bot]
        return self.data.xpos[root].copy(), self.data.xquat[root].copy()

    def teleport(self, bot, position):
        joint = self.model.joint(f'b{bot}_root').id
        address = self.model.jnt_qposadr[joint]
        self.data.qpos[address:address+3] = position
        mujoco.mj_forward(self.model, self.data)

    def _apply_motors(self):
        self.data.qfrc_applied[:] = 0
        for bot in range(2):
            # Shared battery sag approximation; measured resistance can replace this assumption.
            self.voltage[bot] = max(5, 7.2 - .35*sum(self.currents[bot]))
            for i, port in enumerate(('A', 'B')):
                _, q, dof = self._wheel(bot, port)
                speed = self.data.qvel[dof]
                state = self.modes[bot][i]
                mode, value = state['mode'], state['value']
                if mode == 'coast':
                    torque = -0.00002*speed
                else:
                    if mode == 'dc':
                        duty = value/100
                    elif mode == 'brake':
                        duty = 0
                    elif mode == 'hold':
                        duty = np.clip(2*(state['angle']-self.data.qpos[q]) - .02*speed, -1, 1)
                    else:
                        target = np.clip(math.radians(value), -self.motor.free_speed, self.motor.free_speed)
                        duty = np.clip(target/self.motor.free_speed + .02*(target-speed), -1, 1)
                    torque = self.motor.torque(duty, speed, self.voltage[bot])
                self.data.qfrc_applied[dof] = torque
                self.currents[bot][i] = abs(torque)/self.motor.stall_torque*self.motor.stall_current

    def advance(self, seconds):
        count = round(seconds/self.model.opt.timestep)
        if count < 0 or not math.isclose(count*self.model.opt.timestep, seconds, abs_tol=1e-9):
            raise ValueError('Advance duration must be a nonnegative multiple of timestep')
        for _ in range(count):
            if self.result:
                break
            self._apply_motors()
            mujoco.mj_step(self.model, self.data)
            self.steps += 1
            self._judge()

    def _judge(self):
        floor = self.model.geom('outside_floor').id
        losers = set()
        self.contacts = 0
        for contact in self.data.contact:
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in contact.geom]
            if floor in contact.geom:
                losers.update(int(name[1]) for name in names if name.startswith(('b0_', 'b1_')))
            if any(n.startswith('b0_') for n in names) and any(n.startswith('b1_') for n in names):
                self.contacts += 1
        if losers:
            self.result = {'winner': None if len(losers) == 2 else 1-next(iter(losers)),
                           'reason': 'outside_floor_contact', 'time': self.data.time}
        elif self.data.time + 1e-9 >= self.arena.timeout:
            self.result = {'winner': None, 'reason': 'timeout', 'time': self.data.time}

    def sensor_pose(self, bot, sensor):
        # Optical faces in original LDraw coordinates, millimeters. The part
        # export centers its geometry after the LDraw -> Y-up axis conversion.
        name, raw = {'color': ('color_sensor', [0, 0, -19.6]),
                     'distance': ('eyes', [0, 0, -27.6])}[sensor]
        part = next(p for p in self.robot['parts'] if p['name'] == name)
        local = np.array(raw) * [1, -1, -1] / 1000 - np.array(part['sourceCenter'])
        point = np.array(part['position']) + rotate(wxyz(part['rotation']), local)
        body = self.model.body(f'b{bot}_{part["body"]}').id
        return self.data.xpos[body] + rotate(self.data.xquat[body], point)

    def sensors(self, bot):
        tick = int((self.data.time+1e-9)*100)
        if self.sensor_tick[bot] == tick:
            return dict(self.sensor_cache[bot])
        # Noise depends on simulated sample identity, never viewer polling frequency.
        rng = np.random.default_rng(np.random.SeedSequence([self.seed, bot, tick]))
        position, quaternion = self.pose(bot)
        # Nine sampled ultrasonic rays span the nominal cone; multi-path is not modeled.
        origin = self.sensor_pose(bot, 'distance')
        distances = []
        geomgroup = np.ones(6, dtype=np.uint8)
        for yaw in np.linspace(-math.radians(35), math.radians(35), 9):
            direction = rotate(quaternion, [math.sin(yaw), 0, math.cos(yaw)])
            geomid = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(self.model, self.data, origin, direction, geomgroup, 1, self.roots[bot], geomid)
            if .05 <= distance <= 2:
                distances.append(distance)
        distance = round(np.clip(min(distances)*1000 + rng.normal(0, 5), 50, 2000)) if distances else 2000
        sensor = self.sensor_pose(bot, 'color')
        radius = math.hypot(sensor[0], sensor[2])
        # Approximate 6 mm footprint smooths the border; height/tipping gates the reading.
        white = np.clip((radius-(self.arena.radius-self.arena.border)+.003)/.006, 0, 1)
        reflection = 8 + 80*white
        if radius > self.arena.radius or sensor[1]-self.arena.height > .035:
            reflection = 5
        forward = rotate(quaternion, [0, 0, 1])
        values = {'distance': int(distance), 'reflection': int(np.clip(round(reflection+rng.normal(0, .8)), 0, 100)),
                  # Assumed gyro bias/drift, not an exact world heading exposed to programs.
                  'heading': math.degrees(math.atan2(forward[0], forward[2])) + .05*self.data.time + rng.normal(0, .08)}
        self.sensor_tick[bot] = tick
        self.sensor_cache[bot] = values
        return dict(values)

    def snapshot(self):
        bodies = []
        for bot in range(2):
            body_poses = {}
            for body in self.robot['bodies']:
                idx = self.model.body(f'b{bot}_{body["name"]}').id
                q = self.data.xquat[idx]
                body_poses[body['name']] = {'position': self.data.xpos[idx].tolist(), 'rotation': [*q[1:].tolist(), float(q[0])]}
            bodies.append({'bodies': body_poses, 'sensors': self.sensors(bot),
                           'motors': [{'port': p, 'speed': self.wheel_speed(bot, p), 'angle': self.wheel_angle(bot, p),
                                       'current': self.currents[bot][i], 'command': self.modes[bot][i]} for i,p in enumerate(('A','B'))],
                           'battery': self.voltage[bot]})
        return {'time': self.data.time, 'bots': bodies, 'contacts': self.contacts, 'result': self.result, 'seed': self.seed}

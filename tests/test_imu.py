import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from sumo.physics import Arena, World
from sumo.robot import multiply, rotate
from sumo.runtime import Runner


def orient(world, quaternion, bot=0):
    joint = world.model.joint(f'b{bot}_root').id
    address = world.model.jnt_qposadr[joint]
    world.data.qpos[address+3:address+7] = quaternion
    mujoco.mj_forward(world.model, world.data)
    world.sensor_tick[bot] = -1


def axis_rotation(axis, degrees):
    half = math.radians(degrees)/2
    return [math.cos(half), *(math.sin(half)*np.array(axis))]


@pytest.mark.parametrize('pitch, roll', [(0, 0), (30, 0), (-45, 0), (90, 0), (-90, 0), (0, 35), (0, -60), (30, 40), (0, 180)])
def test_tilt_matches_pybricks_pitch_then_roll_in_robot_neutral_frame(pitch, roll):
    world = World(seed=19)
    # Positive pitch lowers the +Z nose; positive roll raises the +X side.
    # Compose local pitch then roll using the right-handed robot frame.
    rotation = multiply(axis_rotation([1, 0, 0], pitch), axis_rotation([0, 0, 1], roll))
    orient(world, rotation)
    measured_pitch, measured_roll = world.sensors(0)['tilt']
    assert measured_pitch == pytest.approx(pitch, abs=.5)
    if abs(pitch) == 90:
        # Roll is undefined with the forward axis vertical (Euler singularity).
        assert math.isfinite(measured_roll) and -180 <= measured_roll < 180
    else:
        assert (measured_roll-roll+180) % 360-180 == pytest.approx(0, abs=.5)


def test_tilt_neutral_is_independent_of_start_heading_and_has_small_noise():
    world = World(seed=19)
    first, second = [world.sensors(bot)['tilt'] for bot in (0, 1)]
    assert all(abs(angle) < .5 for sample in (first, second) for angle in sample)
    assert first != (0., 0.)
    assert first != second


def test_tilt_is_cached_at_100_hz_and_read_frequency_does_not_change_samples():
    first, second = World(seed=31), World(seed=31)
    initial = first.sensors(0)['tilt']
    # An orientation change between samples must not leak through a device read.
    joint = first.model.joint('b0_root').id
    address = first.model.jnt_qposadr[joint]
    first.data.qpos[address+3:address+7] = axis_rotation([0, 0, 1], 25)
    mujoco.mj_forward(first.model, first.data)
    assert first.sensors(0)['tilt'] == initial
    # Move the next sample clock without integrating the deliberately tilted
    # assembly into the floor; this test isolates sampling from collision forces.
    first.data.time = .01
    assert abs(first.sensors(0)['tilt'][1]) > 20
    first, second = World(seed=31), World(seed=31)
    for _ in range(20):
        first.advance(.001)
        first.snapshot()
        first.sensors(0)
    second.advance(.02)
    assert first.sensors(0)['tilt'] == second.sensors(0)['tilt']
    assert first.sensors(0)['tilt'] != initial
    np.testing.assert_array_equal(first.data.qpos, second.data.qpos)


def legacy_digest(world_class, seed):
    world = world_class(seed=seed, arena=Arena(radius=2))
    observations = []
    for tick in range(100):
        if tick % 20 == 0:
            for bot in (0, 1):
                world.command(bot, 'A', 'dc', [65, 45, -30, 85, 0][tick//20])
                world.command(bot, 'B', 'dc', [40, 70, -30, 85, 0][tick//20])
        world.advance(.01)
        observations.append({'qpos': world.data.qpos.tolist(), 'qvel': world.data.qvel.tolist(),
                             'sensors': [{key: world.sensors(bot)[key] for key in ('distance', 'reflection', 'heading')}
                                         for bot in (0, 1)],
                             'voltage': world.voltage, 'currents': [list(row) for row in world.currents],
                             'contacts': world.contacts, 'result': world.result})
    payload = {'observations': observations, 'events': world.events, 'ultrasonic': world.ultrasonic_log}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def test_tilt_addition_preserves_frozen_existing_channels_and_physics():
    golden = json.loads((Path(__file__).parent/'fixtures/imu-existing-channels.json').read_text())
    assert mujoco.__version__ == golden['engine']
    for seed, digest in golden['digests'].items():
        assert legacy_digest(World, int(seed)) == digest


def test_real_worker_receives_cached_pitch_roll_tuple_and_supports_default_calibrated_flag():
    source = '''
from pybricks.hubs import PrimeHub
hub = PrimeHub()
sample = hub.imu.tilt()
assert isinstance(sample, tuple) and len(sample) == 2
pitch, roll = sample
assert abs(pitch) < 1 and abs(roll) < 1
assert hub.imu.tilt(calibrated=True) == sample
print('tilt tuple checked')
'''
    with Runner(World(seed=19), [source, source]) as runner:
        final = runner.run_until_done(max_seconds=.03)
        assert [program['status'] for program in final['programs']] == ['done', 'done'], final['programs']
        assert all(program['logs'] == ['tilt tuple checked'] for program in final['programs'])


def test_raw_accelerometer_tilt_is_explicitly_unsupported():
    source = 'from pybricks.hubs import PrimeHub\nPrimeHub().imu.tilt(calibrated=False)'
    with Runner(World(), [source, 'pass']) as runner:
        final = runner.step(.01)
        assert final['programs'][0]['status'] == 'error'
        assert 'Unsupported' in final['programs'][0]['error']


def test_runtime_and_saved_trial_metadata_declare_uncalibrated_tilt(tmp_path):
    from sumo.physics import IMU_TILT_PROFILE
    from sumo.runtime import RUNTIME_PROFILE, SUPPORTED_API
    from sumo.server import info
    from tools.strategy_tournament import run_bout

    assert RUNTIME_PROFILE == 'sync-subset-v4'
    assert 'imu.tilt(calibrated=True)' in SUPPORTED_API['pybricks.hubs'][0]
    assert info()['imuTiltModel'] == IMU_TILT_PROFILE
    path = tmp_path/'tilt-bout.json'
    run_bout({'programs': ['pass', 'raise ValueError("fixture fault")'], 'names': ['one', 'two'],
              'seed': 31, 'swapped': False, 'timeout': 1, 'friction': .8, 'path': str(path)})
    metadata = json.loads(path.read_text())['metadata']
    assert metadata['runtime'] == RUNTIME_PROFILE
    assert metadata['imuTiltModel']['fidelity'] == 'uncalibrated'
    assert metadata['imuTiltModel'] == IMU_TILT_PROFILE


@pytest.mark.parametrize('driven_port, clockwise_sign', [('A', -1), ('B', 1)])
def test_heading_matches_actual_left_and_right_turns(driven_port, clockwise_sign):
    # Historical A is on -X (physical right); driving A turns the +Z nose
    # toward +X (physical left). Pybricks must report that turn as negative.
    source = f'''
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
hub = PrimeHub()
a = Motor(Port.A)
b = Motor(Port.B, Direction.COUNTERCLOCKWISE)
assert hub.imu.heading() == 0
{'a' if driven_port == 'A' else 'b'}.dc(60)
{'b' if driven_port == 'A' else 'a'}.hold()
wait(300)
assert hub.imu.heading()*{clockwise_sign} > 25
'''
    world = World(seed=19, arena=Arena(radius=2))
    with Runner(world, [source, 'pass']) as runner:
        final = runner.run_until_done(max_seconds=.4)
        assert final['programs'][0]['status'] == 'done', final['programs']
        forward = rotate(world.pose(0)[1], [0, 0, 1])
        assert forward[0]*clockwise_sign < -.2


def test_clockwise_heading_is_relative_and_unwrapped_through_a_full_turn():
    source = '''
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
hub = PrimeHub()
a = Motor(Port.A)
b = Motor(Port.B, Direction.COUNTERCLOCKWISE)
assert hub.imu.heading() == 0
a.dc(60)
b.dc(-60)
wait(2000)
assert hub.imu.heading() < -360
'''
    # Both initial headings (0 and 180 degrees) must yield startup-relative 0.
    with Runner(World(seed=19, arena=Arena(radius=2)), [source, source]) as runner:
        final = runner.run_until_done(max_seconds=2.1)
        assert [program['status'] for program in final['programs']] == ['done', 'done'], final['programs']


class LegacyHeadingRunner(Runner):
    """Frozen v2 RPC sign, with the same World and scheduler as v3."""
    def _dispatch(self, bot, operation, args):
        if operation == 'heading':
            self._reply(bot, self.headings[bot])
        else:
            super()._dispatch(bot, operation, args)


def test_internal_yaw_conversion_preserves_exact_values_rpc_timing_and_mechanics():
    source = '''
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import StopWatch, wait
hub = PrimeHub()
a = Motor(Port.A)
b = Motor(Port.B, Direction.COUNTERCLOCKWISE)
watch = StopWatch()
for duty in (50, -30, 70):
    a.dc(duty)
    b.dc(-duty)
    for sample in range(5):
        print(hub.imu.heading(), watch.time())
        wait(20)
'''
    converted = source.replace('hub.imu.heading()', '-hub.imu.heading()')
    records = []
    for runner_class, program in ((LegacyHeadingRunner, source), (Runner, converted)):
        world = World(seed=31, arena=Arena(radius=2))
        with runner_class(world, [program, program]) as runner:
            frames = []
            for _ in range(20):
                frames.append(runner.step(.02))
                frames[-1]['qpos'] = world.data.qpos.tolist()
                frames[-1]['qvel'] = world.data.qvel.tolist()
            records.append({'frames': frames, 'events': world.events,
                            'ultrasonic': world.ultrasonic_log})
    assert records[0] == records[1]

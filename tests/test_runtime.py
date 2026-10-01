import numpy as np
import pytest
from pathlib import Path

from sumo.physics import Arena, World
from sumo.runtime import Runner


DRIVE = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
left.run(600)
right.run(600)
wait(1000)
'''


def test_real_programs_drive_both_mounted_motors_forward_and_wait_independently():
    world = World(seed=2, arena=Arena(radius=2))
    initial = [world.pose(bot)[0].copy() for bot in range(2)]
    with Runner(world, ['from pybricks.tools import wait\nwait(500)', DRIVE]) as runner:
        result = runner.step(.2)
        assert result['programs'][0]['status'] == 'waiting'
        assert result['programs'][1]['status'] == 'waiting'
        assert abs(world.pose(0)[0][2]-initial[0][2]) < .002
        assert world.pose(1)[0][2] < initial[1][2]-.01
        assert world.wheel_speed(1, 'A') > 0
        assert world.wheel_speed(1, 'B') > 0
        assert world.data.time == pytest.approx(.2)


def test_clock_sensors_battery_and_encoder_reads_run_inside_real_python():
    program = '''
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import StopWatch, wait
hub = PrimeHub()
watch = StopWatch()
motor = Motor(Port.A)
motor.reset_angle(42)
assert abs(motor.angle()-42) < 1
assert abs(motor.speed()) < 10
assert 0 <= ColorSensor(Port.C).reflection() <= 100
assert 50 <= UltrasonicSensor(Port.D).distance() <= 2000
assert 5000 <= hub.battery.voltage() <= 7200
assert abs(hub.imu.heading()) < 5
wait(40)
assert 40 <= watch.time() < 100
watch.reset()
assert watch.time() < 5
print('sensor program done')
'''
    with Runner(World(seed=4), [program, program]) as runner:
        result = runner.run_until_done(max_seconds=.2)
        assert [program['status'] for program in result['programs']] == ['done', 'done']
        assert all('sensor program done' in program['logs'] for program in result['programs'])


def test_timed_motion_nonblocking_and_completion_modes():
    program = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
left = Motor(Port.A)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
left.run_time(400, 80, then=Stop.BRAKE, wait=False)
right.dc(40)
wait(150)
assert left.angle() > 5
left.hold()
right.brake()
wait(100)
left.stop()
right.stop()
'''
    with Runner(World(arena=Arena(radius=2)), [program, 'pass']) as runner:
        result = runner.run_until_done(max_seconds=.5)
        assert result['programs'][0]['status'] == 'done', result['programs']
        assert all(mode['mode'] == 'coast' for mode in runner.world.modes[0])


def test_blocking_angle_and_target_operations_use_encoder_feedback():
    program = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
motor = Motor(Port.A)
motor.run_angle(300, 30)
# A 100 Hz encoder can lag by one sample (3 degrees at this speed).
angle = motor.angle()
assert 26 <= angle <= 34, angle
motor.run_target(250, 0, then=Stop.BRAKE)
angle = motor.angle()
assert -4 <= angle <= 4, angle
motor.run_time(-200, 40)
assert motor.angle() < -1
'''
    with Runner(World(arena=Arena(radius=2)), [program, 'pass']) as runner:
        result = runner.run_until_done(max_seconds=1.5)
        assert result['programs'][0]['status'] == 'done', result['programs']


@pytest.mark.parametrize('program, message', [
    ('import os', 'Unsupported import'),
    ('from pybricks.robotics import DriveBase', 'Unsupported import'),
    ('from pybricks.tools import multitask', 'Unsupported API'),
    ('async def main():\n    pass', 'async'),
    ('from pybricks.pupdevices import Motor\nfrom pybricks.parameters import Port\nMotor(Port.F)', 'port'),
    ('raise ValueError("robot failed")', 'robot failed'),
])
def test_errors_are_explicit_and_isolated_to_one_program(program, message):
    with Runner(World(), [program, 'from pybricks.tools import wait\nwait(100)']) as runner:
        result = runner.step(.03)
        assert result['programs'][0]['status'] == 'error'
        assert message.lower() in result['programs'][0]['error'].lower()
        assert result['programs'][1]['status'] == 'waiting'


def test_cpu_loop_and_rpc_operation_caps_terminate_workers():
    with Runner(World(), ['while True: pass', 'pass'], cpu_timeout=.1) as runner:
        result = runner.step(.01)
        assert 'CPU' in result['programs'][0]['error']
    source = 'from pybricks.tools import wait\nwhile True: wait(0)'
    with Runner(World(), [source, 'pass'], operation_limit=5) as runner:
        result = runner.step(.02)
        assert 'operation limit' in result['programs'][0]['error']


def test_same_seed_reproduces_program_randomness_and_physics_across_snapshot_frequency():
    source = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from random import randint
left, right = Motor(Port.A), Motor(Port.B, Direction.COUNTERCLOCKWISE)
for i in range(3):
    left.dc(randint(10, 70))
    right.dc(randint(10, 70))
    wait(25)
wait(100)
'''
    first, second = World(seed=15), World(seed=15)
    with Runner(first, [source, source]) as runner:
        runner.step(.15)
        first_snapshot = runner.snapshot()
    with Runner(second, [source, source]) as runner:
        for _ in range(15):
            runner.step(.01)
            runner.snapshot()
        second_snapshot = runner.snapshot()
    np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
    assert first_snapshot['programs'] == second_snapshot['programs']
    assert first_snapshot['bots'][0]['sensors'] == second_snapshot['bots'][0]['sensors']


def test_close_reaps_workers_and_rejects_future_steps():
    runner = Runner(World(), ['from pybricks.tools import wait\nwait(10000)']*2)
    runner.step(.005)
    runner.close()
    assert all(not worker.is_alive() for worker in runner.processes)
    runner.close()
    with pytest.raises(RuntimeError, match='closed'):
        runner.step(.01)


def test_bundled_strategies_execute_as_two_independent_programs():
    root = Path(__file__).resolve().parents[1]
    programs = [(root / 'examples' / name).read_text() for name in ('aggressive.py', 'patient.py')]
    with Runner(World(seed=8), programs) as runner:
        result = runner.step(.5)
        assert all(program['error'] is None for program in result['programs'])
        assert all(program['status'] == 'waiting' for program in result['programs'])
        assert all(program['operations'] >= 1 for program in result['programs'])


def test_random_seed_without_argument_stays_deterministic_and_closed_run_rejects():
    source = 'from random import seed, randint\nseed()\nprint(randint(0, 2**50))'
    logs = []
    for _ in range(2):
        with Runner(World(seed=17), [source, 'pass']) as runner:
            logs.append(runner.run_until_done(max_seconds=.01)['programs'][0]['logs'])
        with pytest.raises(RuntimeError, match='closed'):
            runner.run_until_done()
    assert logs[0] == logs[1]


def test_program_fault_loses_and_simultaneous_faults_draw_at_same_clock():
    for programs, winner in [(['raise ValueError("bad")', 'pass'], 1), (['raise ValueError("bad")']*2, None)]:
        with Runner(World(), programs) as runner:
            result = runner.step(.01)
            assert result['result'] == {'winner': winner, 'reason': 'program_error', 'time': 0.0}


def test_completed_programs_coast_until_match_timeout():
    with Runner(World(arena=Arena(timeout=.03)), ['pass', 'pass']) as runner:
        result = runner.run_until_done()
        assert result['result']['reason'] == 'timeout'
        assert result['time'] == pytest.approx(.03)
        assert all(program['status'] == 'done' for program in result['programs'])


def test_encoder_is_integer_degrees_at_100hz_and_reset_does_not_spike_speed():
    source = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
motor = Motor(Port.A)
motor.dc(100)
wait(30)
for i in range(6):
    print(motor.angle())
motor.reset_angle(4000)
assert 4000 <= motor.angle() <= 4010
wait(15)
assert abs(motor.speed()) < 1300
'''
    with Runner(World(arena=Arena(radius=2)), [source, 'pass']) as runner:
        result = runner.run_until_done(max_seconds=.08)
        assert result['programs'][0]['error'] is None, result['programs']
        readings = [float(line) for line in result['programs'][0]['logs']]
        assert len(readings) == 6
        assert all(value.is_integer() for value in readings)
        assert len(set(readings[:4])) == 1

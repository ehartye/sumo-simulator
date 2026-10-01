import math

import numpy as np
import pytest

from sumo.physics import Arena, MotorModel, World


def test_motor_torque_has_back_emf_and_voltage_limits():
    motor = MotorModel()
    free_speed = 185 * 2 * math.pi / 60
    assert motor.torque(1, 0, 7.2) == pytest.approx(0.18)
    assert motor.torque(1, free_speed, 7.2) == pytest.approx(0, abs=1e-10)
    assert motor.torque(1, 0, 6) < motor.torque(1, 0, 7.2)
    assert motor.torque(3, 0, 7.2) == motor.torque(1, 0, 7.2)
    assert motor.torque(-1, 0, 7.2) == pytest.approx(-0.18)


def test_rest_is_stable_and_gravity_is_real():
    world = World(seed=3)
    initial = world.pose(0)[0].copy()
    world.advance(1)
    position, _ = world.pose(0)
    assert abs(position[0]-initial[0]) < 0.002
    assert abs(position[2]-initial[2]) < 0.002
    assert np.isfinite(world.data.qpos).all()
    assert world.data.time == pytest.approx(1)


def test_motor_acceleration_braking_and_slip():
    world = World(seed=9, arena=Arena(radius=2))
    start = world.pose(0)[0][2]
    world.command(0, 'A', 'run', 700)
    world.command(0, 'B', 'run', 700)
    world.advance(0.4)
    assert world.pose(0)[0][2] > start + 0.03
    assert 0 < world.wheel_speed(0, 'A') <= 1200
    world.command(0, 'A', 'brake', 0)
    world.command(0, 'B', 'brake', 0)
    world.advance(0.6)
    assert abs(world.wheel_speed(0, 'A')) < 30


def test_same_seed_and_commands_reproduce_state():
    first, second = World(seed=10), World(seed=10)
    for world in (first, second):
        world.command(0, 'A', 'dc', 70)
        world.command(0, 'B', 'dc', 30)
        world.advance(0.2)
    np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
    assert first.sensors(0) == second.sensors(0)


def test_sensor_values_are_not_omniscient_and_ports_are_validated():
    world = World(seed=0)
    sensors = world.sensors(0)
    assert 50 <= sensors['distance'] <= 2000
    assert 0 <= sensors['reflection'] <= 100
    with pytest.raises(ValueError, match='port'):
        world.command(0, 'F', 'run', 500)
    assert 'opponent_pose' not in sensors


def test_raised_arena_outside_floor_contact_loses():
    world = World()
    world.teleport(0, [1, 0.01, 0])
    world.advance(0.5)
    assert world.result is not None
    assert world.result['winner'] == 1
    assert world.result['reason'] == 'outside_floor_contact'


def test_viewer_observation_does_not_change_sensor_noise():
    watched, headless = World(seed=33), World(seed=33)
    for _ in range(20):
        watched.advance(.01)
        watched.snapshot()
    headless.advance(.2)
    assert watched.sensors(0) == headless.sensors(0)
    assert watched.sensors(1) == headless.sensors(1)


def test_timestep_convergence_for_straight_acceleration():
    positions = []
    for timestep in (.001, .0005, .00025):
        world = World(timestep=timestep, arena=Arena(radius=2))
        for port in ('A', 'B'):
            world.command(0, port, 'run', 700)
        world.advance(.4)
        positions.append(world.pose(0)[0])
    assert np.linalg.norm(positions[0]-positions[-1]) < .002


def test_opponent_collision_transfers_momentum():
    world = World(arena=Arena(radius=2))
    world.teleport(0, [0, .026, -.1])
    world.teleport(1, [0, .026, .1])
    for port in ('A', 'B'):
        world.command(0, port, 'run', 700)
    had_contact = False
    for _ in range(50):
        world.advance(.01)
        had_contact |= world.contacts > 0
    assert had_contact
    assert world.pose(1)[0][2] > .12


def test_material_friction_reaches_actual_contact_solver():
    world = World()
    world.advance(.05)
    caster = world.model.geom('b0_caster_ball_0').id
    contacts = [c for c in world.data.contact if caster in c.geom]
    assert contacts
    assert all(c.friction[0] == pytest.approx(.25) for c in contacts)


def test_timeout_is_robust_to_floating_point_clock_accumulation():
    world = World(arena=Arena(timeout=10))
    world.advance(10)
    assert world.result and world.result['reason'] == 'timeout'


def test_outward_start_and_sensor_faces_follow_real_part_mounts():
    world = World(outward_start=True)
    for bot, sign in [(0, -1), (1, 1)]:
        position, _ = world.pose(bot)
        assert position[2] == pytest.approx(sign*.1524)
        color = world.sensor_pose(bot, 'color')
        eyes = world.sensor_pose(bot, 'distance')
        assert color[2]-position[2] == pytest.approx(sign*.072)
        assert color[1]-position[1] == pytest.approx(.0164)
        assert eyes[2]-position[2] == pytest.approx(sign*.0916)

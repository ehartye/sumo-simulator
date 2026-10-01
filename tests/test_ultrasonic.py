import math

import mujoco
import numpy as np
import pytest

from sumo.physics import Arena, World


def target_world(distance=1, angle=0, height=.026):
    world = World(arena=Arena(radius=3))
    world.teleport(0, [0, .026, 0])
    a = math.radians(angle)
    world.teleport(1, [distance*math.sin(a), height, distance*math.cos(a)])
    return world


@pytest.mark.parametrize('angle', [0, 4, 12, 20, 28])
def test_robot_does_not_disappear_between_horizontal_samples(angle):
    world = target_world(angle=angle)
    assert 750 < world.sensors(0)['distance'] < 1000


def test_vertical_cone_detects_robot_above_the_center_ray():
    world = target_world(height=.15)
    assert 750 < world.sensors(0)['distance'] < 1000


@pytest.mark.parametrize('angle', [50, 90, 180])
def test_outside_cone_and_grazing_floor_do_not_become_opponents(angle):
    world = target_world(angle=angle)
    assert world.sensors(0)['distance'] == 2000


def test_distance_tracks_actual_sensor_rotation_not_just_chassis():
    world = target_world()
    eyes = next(p for p in world.robot['parts'] if p['name'] == 'eyes')
    eyes['rotation'] = [0, math.sqrt(.5), 0, math.sqrt(.5)]
    world.sensor_tick = [-1, -1]
    assert world.sensors(0)['distance'] == 2000


def test_articulated_parts_of_emitting_robot_cannot_become_echoes():
    world = target_world(distance=2.5)
    wheel = world.model.body('b0_left_wheel').id
    world.model.body_pos[wheel] = [0, .06, .4]
    mujoco.mj_forward(world.model, world.data)
    world.sensor_tick = [-1, -1]
    assert world.sensors(0)['distance'] == 2000
    assert world.ultrasonic_latest[0]['hitObject'] is None


def test_occlusion_and_sensor_log_explain_the_reported_distance():
    world = target_world()
    # Move the ring into the beam as an opaque obstacle, with a surface normal
    # toward the sensor. This tests real scene occlusion, not opponent lookup.
    ring = world.model.geom('ring').id
    world.model.geom_size[ring] = [.3, .03, 0]
    world.model.geom_pos[ring] = [0, .1, .4]
    world.model.geom_quat[ring] = [1, 0, 0, 0]
    mujoco.mj_forward(world.model, world.data)
    world.sensor_tick = [-1, -1]
    measured = world.sensors(0)['distance']
    assert 250 < measured < 310
    event = world.ultrasonic_log[-1]
    assert event['bot'] == 0 and event['hitObject'] == 'ring'
    assert event['reason'] == 'echo' and event['distanceMm'] == measured
    assert np.linalg.norm(np.array(event['hitPoint'])-event['origin']) == pytest.approx(event['idealDistanceMm']/1000)
    count = len(world.ultrasonic_log)
    world.sensors(0)
    assert len(world.ultrasonic_log) == count, 'Cached reads must not duplicate samples'


def test_no_return_and_below_minimum_are_distinguishable_in_log():
    far = target_world(distance=2.5)
    assert far.sensors(0)['distance'] == 2000
    assert far.ultrasonic_log[-1]['reason'] == 'no_echo'
    near = target_world(distance=.20)
    assert near.sensors(0)['distance'] == 2000
    assert near.ultrasonic_log[-1]['reason'] == 'below_minimum'


def test_live_diagnostics_are_automatically_logged_and_saved_in_replay(tmp_path, monkeypatch):
    import json
    from pathlib import Path
    from fastapi.testclient import TestClient
    from sumo import server
    monkeypatch.setattr(server, 'LOG_ROOT', tmp_path)
    with TestClient(server.app) as client:
        first = client.post('/api/reset', json={'programs': ['pass']*2}).json()
        assert all('ultrasonic' in bot for bot in first['bots'])
        client.post('/api/step', json={'seconds': .05})
        diagnostic = client.get('/api/diagnostics').json()
        replay = client.get('/api/replay').json()
        disk = [json.loads(line) for line in Path(diagnostic['logFile']).read_text().splitlines()]
        assert disk == diagnostic['samples'] == replay['ultrasonicSamples']
        assert len(disk) >= 4
        assert replay['metadata']['ultrasonicModel']['calibrated'] is False
        saved_metadata = json.loads(Path(diagnostic['metadataFile']).read_text())
        assert saved_metadata == replay['metadata']
        prior = diagnostic['logFile']
        client.post('/api/reset', json={'programs': ['pass']*2})
        assert client.get('/api/diagnostics').json()['logFile'] != prior
        assert Path(prior).exists(), 'Reset must preserve the previous run log'


def test_sensor_samples_and_logs_at_100_hz_without_program_or_viewer_reads():
    world = World()
    world.advance(.035)
    for bot in (0, 1):
        samples = [event for event in world.ultrasonic_log if event['bot'] == bot]
        assert [event['sampleTick'] for event in samples] == [0, 1, 2, 3]
        assert [event['sampleTime'] for event in samples] == pytest.approx([0, .01, .02, .03])
    count = len(world.ultrasonic_log)
    world.snapshot()
    assert len(world.ultrasonic_log) == count


def test_batch_bouts_preserve_sensor_logs_and_model_provenance(tmp_path, monkeypatch):
    import json
    from pathlib import Path
    from sumo.cli import main
    program = tmp_path/'bot.py'
    program.write_text('pass')
    output = tmp_path/'bouts.json'
    monkeypatch.setattr('sys.argv', ['sumo', 'batch', str(program), str(program), '--timeout', '1', '--out', str(output)])
    main()
    report = json.loads(output.read_text())
    assert report['ultrasonicModel']['model'] == 'geometric-cone-v2'
    assert len(report['bouts']) == 2
    for bout in report['bouts']:
        samples = [json.loads(line) for line in Path(bout['sensorLogFile']).read_text().splitlines()]
        assert len(samples) == 202
        assert samples[-1]['sampleTime'] == pytest.approx(1)

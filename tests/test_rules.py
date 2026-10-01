import math
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from sumo.physics import World
from sumo.rules import ATOMIC_EAGLES_2022, StartProtocol
from sumo.runtime import Runner


class RefereeWorld:
    def __init__(self):
        self.arena = ATOMIC_EAGLES_2022.arena()
        self.data = SimpleNamespace(time=0.)
        self.outward_start = True
        self.turns = [0., 0.]
        self.angles = [[0., 0.], [0., 0.]]
        self.offsets = [[0., 0.], [0., 0.]]
        self.white = [False, False]

    def pose(self, bot):
        yaw = math.radians((180 if bot == 0 else 0) + self.turns[bot])
        return [0, self.arena.height, -.1524 if bot == 0 else .1524], [math.cos(yaw/2), 0, math.sin(yaw/2), 0]

    def wheel_angle(self, bot, port):
        index = 0 if port == 'A' else 1
        return self.angles[bot][index]-self.offsets[bot][index]

    def sensor_pose(self, bot, kind):
        assert kind == 'color'
        radius = self.arena.radius-.02 if self.white[bot] else .2
        return [0, self.arena.height+.01, radius*(-1 if bot == 0 else 1)]


def test_profile_dimensions_provenance_and_explicit_limits():
    info = ATOMIC_EAGLES_2022.info()
    assert vars(ATOMIC_EAGLES_2022.arena()) == {'radius': .6096, 'height': .0254, 'border': .0508, 'timeout': 120}
    assert info['waitSeconds'] == 3
    assert info['startSeparationMeters'] == .3048
    assert info['pivotDegrees'] == 180
    assert any('2022' in source['title'] for source in info['sources'])
    assert any('height' in assumption.lower() for assumption in info['assumptions'])
    assert info['notEnforced']


def test_wait_rejects_early_nonzero_drive_but_allows_brake_and_hold():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world)
    assert protocol.command(world, 0, 'A', 'hold', 0)
    assert protocol.command(world, 0, 'B', 'run', 0)
    world.data.time = 2.999
    assert not protocol.command(world, 0, 'A', 'run', 1)
    assert 'three-second' in protocol.snapshot()['bots'][0]['violation']
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world)
    world.data.time = 3
    assert protocol.command(world, 0, 'A', 'dc', 20)
    assert protocol.snapshot()['bots'][0]['phase'] == 'opening'


def test_pivot_uses_one_forward_wheel_and_175_degree_completion_tolerance():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert protocol.command(world, 0, 'B', 'run', 500)
    assert protocol.command(world, 0, 'A', 'hold', 0)
    world.turns[0] = 174; world.angles[0][0] = 4.9; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['phase'] == 'opening'
    world.turns[0] = 175; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['phase'] == 'engaged'
    assert protocol.command(world, 0, 'A', 'run', -500)


def test_pivot_stationary_wheel_cannot_be_faked_by_resetting_encoder():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert protocol.command(world, 0, 'A', 'run', 500)
    world.angles[0][1] = 6; world.offsets[0][1] = 6; protocol.observe(world)
    assert 'stationary' in protocol.snapshot()['bots'][0]['violation']


def test_first_reverse_and_counterrotation_are_rejected():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert not protocol.command(world, 0, 'A', 'run', -500)
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert protocol.command(world, 0, 'A', 'run', 500)
    assert not protocol.command(world, 0, 'B', 'run', -500)


def test_edge_opening_needs_physical_white_then_about_face():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert protocol.command(world, 0, 'A', 'run', 500)
    assert protocol.command(world, 0, 'B', 'run', 500)
    assert protocol.snapshot()['bots'][0]['opening'] == 'edge'
    world.white[0] = True; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['whiteTouched']
    assert protocol.snapshot()['bots'][0]['phase'] == 'opening'
    assert protocol.command(world, 0, 'A', 'run', -300)
    world.turns[0] = 180; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['phase'] == 'engaged'


def test_edge_contact_cannot_credit_turning_done_before_reaching_white():
    world = RefereeWorld(); protocol = StartProtocol(ATOMIC_EAGLES_2022, world); world.data.time = 3
    assert protocol.command(world, 0, 'A', 'run', 500)
    assert protocol.command(world, 0, 'B', 'run', 500)
    world.turns[0] = 160; protocol.observe(world)
    world.white[0] = True; protocol.observe(world)
    world.turns[0] = 180; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['phase'] == 'opening'
    assert abs(protocol.snapshot()['bots'][0]['turnAfterWhite']) == pytest.approx(20)
    world.turns[0] = 340; protocol.observe(world)
    assert protocol.snapshot()['bots'][0]['phase'] == 'opening'  # Still facing away from the opponent.


def test_runtime_rule_faults_and_profile_are_exposed_without_changing_default_runner():
    source = 'from pybricks.pupdevices import Motor\nfrom pybricks.parameters import Port\nMotor(Port.A).run(100)'
    with Runner(World(arena=ATOMIC_EAGLES_2022.arena(), outward_start=True), [source]*2, rules=ATOMIC_EAGLES_2022) as runner:
        result = runner.step(.01)
        assert result['result']['reason'] == 'rule_violation'
        assert result['result']['winner'] is None
        assert all(bot['violation'] for bot in result['rules']['bots'])
    with Runner(World(), [source, 'pass']) as runner:
        assert runner.step(.01)['result'] is None


@pytest.mark.parametrize('call', ['run(100)', 'dc(20)', 'run_time(100, 10)', 'run_angle(100, 10)', 'run_target(100, 10)'])
def test_all_motor_drive_apis_observe_the_start_delay(call):
    source = f'from pybricks.pupdevices import Motor\nfrom pybricks.parameters import Port\nMotor(Port.A).{call}'
    with Runner(World(arena=ATOMIC_EAGLES_2022.arena(), outward_start=True), [source, 'pass'], rules=ATOMIC_EAGLES_2022) as runner:
        result = runner.step(.01)
        assert result['result']['reason'] == 'rule_violation'
        assert result['result']['winner'] == 1


def test_snapshot_phase_changes_at_the_three_second_boundary():
    source = 'from pybricks.tools import wait\nwait(4000)'
    with Runner(World(arena=ATOMIC_EAGLES_2022.arena(), outward_start=True), [source]*2, rules=ATOMIC_EAGLES_2022) as runner:
        result = runner.step(3)
        assert all(bot['phase'] == 'opening' for bot in result['rules']['bots'])


def test_examples_wait_then_finish_their_distinct_openings():
    root = Path(__file__).resolve().parents[1]
    programs = [(root/'examples'/name).read_text() for name in ('aggressive.py', 'patient.py')]
    with Runner(World(seed=2, arena=ATOMIC_EAGLES_2022.arena(), outward_start=True), programs, rules=ATOMIC_EAGLES_2022) as runner:
        first = runner.step(2.9)
        assert all(bot['phase'] == 'waiting' for bot in first['rules']['bots'])
        assert all(motor['mode'] not in ('run', 'dc') for modes in runner.world.modes for motor in modes)
        last = runner.step(9)
        assert all(bot['violation'] is None for bot in last['rules']['bots']), last
        assert [bot['opening'] for bot in last['rules']['bots']] == ['edge', 'pivot']
        assert all(bot['phase'] == 'engaged' for bot in last['rules']['bots']), last['rules']


def test_server_enables_profile_and_preserves_it_in_replay():
    from fastapi.testclient import TestClient
    from sumo.server import app
    with TestClient(app) as client:
        info = client.get('/api/info').json()
        assert info['arena']['radius'] == .6096
        assert info['rules']['id'] == ATOMIC_EAGLES_2022.id
        snapshot = client.post('/api/reset', json={'programs': ['pass', 'pass']}).json()
        assert snapshot['rules']['profile'] == ATOMIC_EAGLES_2022.id
        replay = client.get('/api/replay').json()
        assert replay['metadata']['arena']['timeout'] == 120
        assert replay['metadata']['rules']['id'] == ATOMIC_EAGLES_2022.id


def test_batch_cli_enables_profile_and_records_an_explicit_timeout_override(tmp_path, monkeypatch, capsys):
    from sumo.cli import main
    blue, red, output = tmp_path/'blue.py', tmp_path/'red.py', tmp_path/'batch.json'
    blue.write_text('pass'); red.write_text('pass')
    monkeypatch.setattr('sys.argv', ['sumo', 'batch', str(blue), str(red), '--timeout', '1', '--out', str(output)])
    main()
    report = json.loads(output.read_text())
    assert report['rules']['id'] == ATOMIC_EAGLES_2022.id
    assert report['arena']['timeout'] == 1
    assert report['arena']['radius'] == .6096
    assert len(report['bouts']) == 2
    assert all(bout['result']['reason'] == 'timeout' for bout in report['bouts'])
    assert len(capsys.readouterr().out.strip().splitlines()) == 2

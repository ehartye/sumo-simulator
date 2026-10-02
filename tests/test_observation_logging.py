import json
import threading
from pathlib import Path

import numpy as np
import pytest

from sumo.physics import Arena, World
from sumo.runtime import Runner


PROGRAM = '''
from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Port
from pybricks.tools import wait
a = Motor(Port.A, Direction.CLOCKWISE)
b = Motor(Port.B, Direction.COUNTERCLOCKWISE)
for speed in (300, 600, 200, 500):
    TRACE
    a.run(speed)
    b.run(speed)
    wait(20)
wait(1000)
'''


def run_trace(trace, **options):
    world = World(seed=17, arena=Arena(radius=2))
    commands = []
    original = world.command
    def record(bot, port, mode, value=0):
        commands.append((world.data.time, bot, port, mode, value))
        return original(bot, port, mode, value)
    world.command = record
    with Runner(world, [PROGRAM.replace('TRACE', trace), 'from pybricks.tools import wait\nwait(1000)'], **options) as runner:
        runner.step(.15)
        assert runner.programs[0]['error'] is None
        return commands[:], world.data.qpos.copy(), world.data.qvel.copy(), list(world.events)


def test_diagnostic_prints_do_not_change_command_times_or_motion():
    quiet = run_trace('pass')
    logged = run_trace("print('speed', speed)")
    assert quiet[0] == logged[0]
    assert np.array_equal(quiet[1], logged[1])
    assert np.array_equal(quiet[2], logged[2])
    assert len([e for e in logged[3] if e['type']=='program_log']) == 4


def test_legacy_profile_preserves_one_tick_print_delay():
    quiet = run_trace('pass', runtime_profile='sync-subset-v3')
    logged = run_trace("print('speed', speed)", runtime_profile='sync-subset-v3')
    assert logged[0][0][0]-quiet[0][0][0] == pytest.approx(.001)
    assert not np.array_equal(quiet[1], logged[1])


def test_zero_time_print_flood_retains_operation_limit():
    with Runner(World(), ['while True:\n    print("spam")', 'pass'], operation_limit=20) as runner:
        result = runner.step(.01)
        assert result['programs'][0]['status']=='error'
        assert 'operation limit' in result['programs'][0]['error']
        assert len(result['programs'][0]['logs'])==20
        assert result['result']['time']==0


def test_unlimited_diagnostic_loop_is_bounded_even_with_default_operation_budget():
    from sumo.runtime import PRINT_BURST_LIMIT
    with Runner(World(), ['while True:\n    print("spam")', 'pass']) as runner:
        result = runner.step(.001)
        assert 'print burst limit' in result['programs'][0]['error']
        assert len([e for e in runner.world.events if e['type']=='program_log'])==PRINT_BURST_LIMIT
        assert result['result']['time']==0


def test_zero_time_prints_cannot_reset_the_wall_budget_before_a_hardware_yield():
    import time
    source = 'print("one")\nprint("two")\nprint("three")\nfrom pybricks.tools import wait\nwait(1000)'
    with Runner(World(), [source, 'pass'], cpu_timeout=.05) as runner:
        connection = runner.connections[0]
        class DelayedConnection:
            # Controlled transport fixture: each reply arrives after40ms.
            def poll(self, timeout):
                time.sleep(min(.04, timeout))
                return timeout >= .04 and connection.poll(0)
            def __getattr__(self, name):
                return getattr(connection, name)
        runner.connections[0] = DelayedConnection()
        result = runner.step(.001)
        assert 'CPU time limit' in result['programs'][0]['error']
        assert result['result']['time']==0


def test_live_controller_prints_do_not_change_a_real_refereed_match():
    import ast
    from sumo.rules import ATOMIC_EAGLES_2022
    root = Path(__file__).resolve().parents[1]
    source = (root/'examples/weave_hook.py').read_text(encoding='utf-8')
    opponent = (root/'examples/pressure_memory.py').read_text(encoding='utf-8')
    class RemovePrints(ast.NodeTransformer):
        def visit_Expr(self, node):
            if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id=='print':
                return ast.copy_location(ast.Pass(), node)
            return node
    quiet = ast.unparse(ast.fix_missing_locations(RemovePrints().visit(ast.parse(source))))
    traces = []
    for program in (source, quiet):
        world = World(seed=607, arena=ATOMIC_EAGLES_2022.arena(12), outward_start=True)
        trace = []
        with Runner(world, [program, opponent], rules=ATOMIC_EAGLES_2022) as runner:
            for _ in range(240):
                runner.step(.05)
                trace.append((world.data.qpos.copy(), world.data.qvel.copy(), world.sensors(0), world.sensors(1)))
            assert all(p['error'] is None for p in runner.programs)
            assert all(b['violation'] is None for b in runner.snapshot()['rules']['bots'])
            events = [{k:v for k,v in e.items() if k!='id'} for e in world.events if e['type']!='program_log']
            assert any(e['type']=='contact_started' for e in events)
        traces.append((trace, events))
    for logged_frame, quiet_frame in zip(traces[0][0], traces[1][0]):
        assert np.array_equal(logged_frame[0], quiet_frame[0])
        assert np.array_equal(logged_frame[1], quiet_frame[1])
        assert logged_frame[2:]==quiet_frame[2:]
    assert traces[0][1]==traces[1][1]


def test_writer_never_blocks_submit_and_copies_queued_records(tmp_path, monkeypatch):
    from sumo.observation_log import ObservationLogWriter
    entered, release = threading.Event(), threading.Event()
    original = ObservationLogWriter._write
    def slow_write(self, path, records):
        entered.set()
        assert release.wait(3)
        original(self, path, records)
    monkeypatch.setattr(ObservationLogWriter, '_write', slow_write)
    writer = ObservationLogWriter()
    path = tmp_path/'events.jsonl'
    try:
        record = {'id': 1, 'nested': {'value': 'original'}}
        assert writer.submit(path, [record])
        assert entered.wait(1)
        record['nested']['value'] = 'mutated'
        # Submission completes while disk work remains deliberately blocked.
        assert writer.submit(path, [{'id': 2}])
        assert not path.exists()
        assert writer.status()['pendingBatches']==2
        release.set()
        assert writer.flush(timeout=2)
        assert [json.loads(line) for line in path.read_text().splitlines()] == [
            {'id': 1, 'nested': {'value': 'original'}}, {'id': 2}]
    finally:
        release.set()
        assert writer.close(timeout=2)


def test_writer_keeps_reset_paths_separate_and_drains_shutdown(tmp_path):
    from sumo.observation_log import ObservationLogWriter
    writer = ObservationLogWriter()
    paths = [tmp_path/'old.jsonl', tmp_path/'new.jsonl']
    for index in range(6):
        assert writer.submit(paths[index % 2], [{'id': index}])
    assert writer.close(timeout=2)
    assert [json.loads(line)['id'] for line in paths[0].read_text().splitlines()] == [0, 2, 4]
    assert [json.loads(line)['id'] for line in paths[1].read_text().splitlines()] == [1, 3, 5]
    assert writer.status()['pendingBatches']==0


def test_writer_reports_disk_failure_and_queue_overflow(tmp_path, monkeypatch):
    from sumo.observation_log import ObservationLogWriter
    entered, release = threading.Event(), threading.Event()
    def broken_write(self, path, records):
        entered.set()
        assert release.wait(3)
        raise OSError('fixture disk failure')
    monkeypatch.setattr(ObservationLogWriter, '_write', broken_write)
    writer = ObservationLogWriter(capacity=1)
    try:
        assert writer.submit(tmp_path/'one.jsonl', [{'id': 1}])
        assert entered.wait(1)
        assert writer.submit(tmp_path/'two.jsonl', [{'id': 2}])
        assert not writer.submit(tmp_path/'three.jsonl', [{'id': 3}])
        assert writer.status()['droppedBatches']==1
        release.set()
        assert writer.flush(timeout=2)
        status = writer.status()
        assert status['failedBatches']==2
        assert 'fixture disk failure' in status['error']
    finally:
        release.set()
        assert writer.close(timeout=2)


def test_timed_out_shutdown_finishes_after_the_disk_recovers(tmp_path, monkeypatch):
    from sumo.observation_log import ObservationLogWriter
    entered, release = threading.Event(), threading.Event()
    original = ObservationLogWriter._write
    def slow_write(self, path, records):
        entered.set()
        assert release.wait(3)
        original(self, path, records)
    monkeypatch.setattr(ObservationLogWriter, '_write', slow_write)
    writer = ObservationLogWriter()
    assert writer.submit(tmp_path/'events.jsonl', [{'id': 1}])
    try:
        assert entered.wait(1)
        assert not writer.close(timeout=.01)
        assert 'timed out' in writer.status()['error']
    finally:
        release.set()
        assert writer.close(timeout=2)


def test_live_step_and_replay_return_while_file_writer_is_blocked(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from sumo import server
    from sumo.observation_log import ObservationLogWriter
    entered, release = threading.Event(), threading.Event()
    original = ObservationLogWriter._write
    def slow_write(self, path, records):
        entered.set()
        assert release.wait(5)
        original(self, path, records)
    monkeypatch.setattr(ObservationLogWriter, '_write', slow_write)
    monkeypatch.setattr(server, 'LOG_ROOT', tmp_path)
    with TestClient(server.app) as client:
        try:
            client.post('/api/reset', json={'programs':['print("ready")\nfrom pybricks.tools import wait\nwait(100)', 'pass']})
            assert entered.wait(1)
            step = client.post('/api/step', json={'seconds':.02}).json()
            assert step['time']==pytest.approx(.02)
            assert step['observationLog']['pendingBatches']>0
            replay = client.get('/api/replay').json()
            assert any(e.get('message')=='ready' for e in replay['events'])
            assert replay['metadata']['runtime']=='sync-subset-v4'
            assert replay['ultrasonicSamples']
        finally:
            release.set()
    saved = [json.loads(line) for line in Path(replay['metadata']['eventLogFile']).read_text().splitlines()]
    assert saved==replay['events']

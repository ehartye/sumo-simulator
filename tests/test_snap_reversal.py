import ast
from pathlib import Path


def helpers():
    path = Path(__file__).resolve().parents[1]/'examples/snap_reverse.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    namespace = {}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def gate():
    functions = helpers()
    assert 'crossing_alarm' in functions, 'Missing crossing watchdog'
    return functions['crossing_alarm']


def test_crossing_suspicion_ignores_a_valid_close_echo():
    samples = [(t, 110, 180+t*.001, t) for t in range(0, 3601, 100)]
    assert gate()(samples)
    assert gate()([(t, 2000, h, effort) for t, _, h, effort in samples])


def test_crossing_suspicion_rejects_circling_and_insufficient_forward_effort():
    samples = [(t, 110, 180+t*.04, t) for t in range(0, 4201, 100)]
    assert not gate()(samples)
    assert not gate()([(t, d, 180, t*.5) for t, d, _, _ in samples])
    assert not gate()(samples[:2])
    assert not gate()([])


def test_crossing_uses_unwrapped_heading_without_false_wrap_or_old_command_credit():
    assert gate()([(t, 110, 350+t*.008, t) for t in range(0, 3601, 100)])
    assert not gate()([(t, 110, 350+t*.2, t) for t in range(0, 3601, 100)])
    assert not gate()([(t, 110, 180, 9000+t*.1) for t in range(0, 3601, 100)])


def test_snap_keeps_one_motor_forward_and_fully_reverses_the_other():
    functions = helpers()
    assert 'snap_duties' in functions, 'Missing snap maneuver'
    assert functions['snap_duties'](1) == (100, -100)
    assert functions['snap_duties'](-1) == (-100, 100)


def test_crossing_gate_reaches_real_worker_with_close_echo_and_white_preempts(monkeypatch):
    from sumo.physics import Arena, World
    from sumo.runtime import Runner
    source = (Path(__file__).resolve().parents[1]/'examples/snap_reverse.py').read_text(encoding='utf-8')
    for white_at in (None, 6.4, 6.7):
        # A controlled API fixture, not a tournament or physical lockup claim.
        # Wide mat avoids ring-out; fixed sensors exercise the complete controller.
        world = World(arena=Arena(radius=5, timeout=10))
        def sensors(bot):
            return {'distance': 110, 'reflection': 80 if white_at is not None and world.data.time >= white_at else 8,
                    'heading': 180 if world.data.time >= 3.05 else 0, 'tilt': (0., 0.)}
        monkeypatch.setattr(world, 'sensors', sensors)
        with Runner(world, [source, 'from pybricks.tools import wait\nwait(9000)']) as runner:
            snapshot = runner.step(6.75)
            assert snapshot['programs'][0]['error'] is None
            logs = [e['message'] for e in world.events if e['type']=='program_log' and e.get('bot')==0]
            crossed = [message for message in logs if message.startswith('watchdog crossing contradiction')]
            if white_at is None:
                assert len(crossed) == 1
                assert 'range mm 110' in crossed[0]
                assert [m['mode'] for m in world.modes[0]] == ['dc', 'dc']
                assert [m['value'] for m in world.modes[0]] == [-100, 100]
            elif white_at == 6.4:
                assert not crossed
                assert any(message.startswith('white landmark') for message in logs)
                assert 'border retreat' in logs
                assert [m['mode'] for m in world.modes[0]] == ['run', 'run']
                assert [m['value'] for m in world.modes[0]] == [400, -400]
            else:
                assert len(crossed) == 1
                assert 'border retreat' in logs
                assert [m['mode'] for m in world.modes[0]] == ['run', 'run']
                assert [m['value'] for m in world.modes[0]] == [-550, -550]

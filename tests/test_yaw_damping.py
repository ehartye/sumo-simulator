import ast
from pathlib import Path


def helpers():
    path = Path(__file__).resolve().parents[1]/'examples/snap_damped.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    namespace = {}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def test_yaw_rate_requires_time_coverage_and_preserves_unwrapped_sign():
    rate = helpers()['estimated_yaw_rate']
    assert rate([]) is None
    assert rate([(20, 350), (40, 360)]) is None
    assert rate([(20, 350), (60, 370), (120, 400)]) == 500
    assert rate([(20, -350), (120, -400)]) == -500


def test_damping_opposes_physical_yaw_with_bounded_motor_commands():
    duties = helpers()['damping_duties']
    assert duties(500, 0) == (-60, 60)
    assert duties(-500, 0) == (60, -60)
    assert duties(500, 40) == (-20, 100)
    assert duties(-500, 40) == (100, -20)
    assert duties(None, 40) == (40, 40)
    assert duties(0, 0) == (0, 0)


def test_settling_requires_continuous_observed_low_rate():
    settled = helpers()['settled_since']
    assert settled(None, 100, None) is None
    assert settled(24, 100, None) == 100
    assert settled(-24, 140, 100) == 100
    assert settled(26, 150, 100) is None
    assert settled(0, 180, None) == 180


def test_complete_worker_damps_positive_yaw_and_white_interrupts(monkeypatch):
    from sumo.physics import Arena, World
    from sumo.runtime import Runner
    source = (Path(__file__).resolve().parents[1]/'examples/snap_damped.py').read_text(encoding='utf-8')
    for white_at in (None, 6.77):
        # Controlled sensor fixture verifies dispatch/sign/priority, not performance.
        world = World(arena=Arena(radius=5, timeout=10))
        def sensors(bot):
            return {'distance': 110, 'reflection': 80 if white_at is not None and world.data.time >= white_at else 8,
                    'heading': 180+max(0, world.data.time-6.5)*300 if world.data.time >= 3.05 else 0,
                    'tilt': (0., 0.)}
        monkeypatch.setattr(world, 'sensors', sensors)
        with Runner(world, [source, 'from pybricks.tools import wait\nwait(9000)']) as runner:
            snapshot = runner.step(6.85)
            assert snapshot['programs'][0]['error'] is None
            logs = [e['message'] for e in world.events if e['type']=='program_log' and e.get('bot')==0]
            assert 'damp yaw' in logs
            if white_at is None:
                assert [m['mode'] for m in world.modes[0]] == ['dc', 'dc']
                assert world.modes[0][0]['value'] < 0 < world.modes[0][1]['value']
            else:
                assert 'border retreat' in logs
                assert [m['value'] for m in world.modes[0]] == [-550, -550]

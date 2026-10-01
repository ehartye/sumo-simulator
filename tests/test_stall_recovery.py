"""Controller decisions tested with sensor traces, without observer pose inputs."""
import ast
import math
from pathlib import Path

import pytest


def policy(name):
    path = Path(__file__).resolve().parents[1]/'examples'/f'{name}.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    functions = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[])
    namespace = {'math': math}
    exec(compile(functions, str(path), 'exec'), namespace)
    return namespace


@pytest.mark.parametrize('name', ['stall_hook', 'stall_geometry', 'stall_pressure'])
def test_lockup_requires_persistent_close_stable_range_and_heading(name):
    detect = policy(name)['suspect_lockup']
    trace = [(t, 110 + t % 3, 180 + t % 2, t*.6, t*.6) for t in range(0, 1501, 100)]
    assert detect(trace) == 'spinning-wheel duel'
    assert detect([s[:3] + (0, 0) for s in trace]) == 'low motor rotation'
    assert detect([(t, 450 - t*.2, h, a, b) for t, _, h, a, b in trace]) is None
    assert detect([(t, d, 180 + t*.04, a, b) for t, d, _, a, b in trace]) is None
    assert detect([(t, 2000, h, a, b) for t, _, h, a, b in trace]) is None
    assert detect(trace[:5]) is None
    # Brief near-field echo loss must not continually reset the observation window.
    assert detect([s if s[0] % 400 else (s[0], 2000, *s[2:]) for s in trace])


def test_geometry_prefers_ring_clearance_and_falls_back_when_uncertain():
    choose = policy('stall_geometry')['choose_turn']
    # East side, facing north: a left hook heads inward; right heads outward.
    assert choose(400, 0, 0, 50, None, 9999, 1)[0] == -1
    assert choose(-400, 0, 0, 50, None, 9999, -1)[0] == 1
    assert choose(400, 0, 0, 500, -40, 200, 1) == (-1, 'recent echo heading')
    assert choose(400, 0, 0, 500, -40, 3000, 1) == (1, 'alternating fallback')


def test_repeated_encounter_watchdog_ignores_progress_and_full_sweeps():
    detect = policy('stall_hook')['encounter_lockup']
    trace = [(t, 115, 180+t % 40, t, t) for t in range(0, 5501, 500)]
    assert detect(trace) == 'repeated close encounters'
    assert detect([(t, 190-t*.02, h, a, b) for t, _, h, a, b in trace]) is None
    assert detect([(t, d, t*.1, a, b) for t, d, _, a, b in trace]) is None


@pytest.mark.parametrize('name', ['stall_hook', 'stall_geometry', 'stall_pressure'])
def test_recovery_controller_opens_legally_and_contacts(name):
    from sumo.physics import World
    from sumo.rules import ATOMIC_EAGLES_2022
    from sumo.runtime import Runner
    root = Path(__file__).resolve().parents[1]
    programs = [(root/'examples'/f'{n}.py').read_text(encoding='utf-8')
                for n in (name, 'pressure_full_duty')]
    with Runner(World(seed=11, arena=ATOMIC_EAGLES_2022.arena(18), outward_start=True),
                programs, rules=ATOMIC_EAGLES_2022) as runner:
        for _ in range(180):
            frame = runner.step(.1)
            if frame['result']:
                break
        assert all(not b['violation'] for b in frame['rules']['bots'])
        assert all(p['error'] is None for p in frame['programs'])
        assert any(e['type'] == 'contact_started' for e in runner.world.events)


@pytest.mark.parametrize('name', ['stall_hook', 'stall_geometry', 'stall_pressure'])
def test_blocked_escape_turn_has_a_timeout_and_returns_to_control(name):
    from sumo.physics import Arena, World
    from sumo.runtime import Runner
    root = Path(__file__).resolve().parents[1]
    source = (root/'examples'/f'{name}.py').read_text(encoding='utf-8')
    world = World(seed=11, arena=Arena(radius=2, timeout=12), outward_start=True)
    original_sensors = world.sensors
    fixed_heading = None

    def blocked_trace(bot):
        nonlocal fixed_heading
        values = original_sensors(bot)
        if world.data.time >= 6 and bot == 0:
            if fixed_heading is None:
                fixed_heading = values['heading']
            # Explicit synthetic fixture: close return but no achievable yaw.
            values.update(distance=110, heading=fixed_heading, reflection=10)
        return values

    world.sensors = blocked_trace
    with Runner(world, [source, 'pass']) as runner:
        runner.step(12)
        messages = [e['message'] for e in world.events if e['type'] == 'program_log' and e['bot'] == 0]
        assert any(m.startswith('suspected lockup') for m in messages)
        assert any(m.startswith('escape turn end') and 'timed out True' in m for m in messages)
        assert any(m.startswith('recovery complete') for m in messages)
        assert runner.programs[0]['error'] is None


@pytest.mark.parametrize('name', ['stall_hook', 'stall_geometry', 'stall_pressure'])
def test_persistent_white_reading_brakes_after_bounded_retreat(name):
    from sumo.physics import Arena, World
    from sumo.runtime import Runner
    root = Path(__file__).resolve().parents[1]
    source = (root/'examples'/f'{name}.py').read_text(encoding='utf-8')
    world = World(arena=Arena(radius=2, timeout=9), outward_start=True)
    original_sensors = world.sensors

    def white_trace(bot):
        values = original_sensors(bot)
        if world.data.time >= 6 and bot == 0:
            values.update(reflection=90, distance=2000)
        return values

    world.sensors = white_trace
    with Runner(world, [source, 'pass']) as runner:
        frame = runner.step(8)
        assert all(m['command']['mode'] == 'brake' for m in frame['bots'][0]['motors'])
        assert any(e['type'] == 'program_log' and e.get('message') == 'border blocked hold'
                   for e in world.events)


@pytest.mark.parametrize('name', ['stall_hook', 'stall_geometry', 'stall_pressure'])
def test_blocked_opening_brakes_and_faults_instead_of_skipping_rule(name):
    from sumo.physics import Arena, World
    from sumo.runtime import Runner
    root = Path(__file__).resolve().parents[1]
    source = (root/'examples'/f'{name}.py').read_text(encoding='utf-8')
    world = World(arena=Arena(radius=2, timeout=15), outward_start=True)
    original_sensors = world.sensors

    def no_yaw_trace(bot):
        values = original_sensors(bot)
        if bot == 0:
            values['heading'] = 0
        return values

    world.sensors = no_yaw_trace
    with Runner(world, [source, 'pass']) as runner:
        frame = runner.step(12)
        assert frame['programs'][0]['status'] == 'error'
        assert 'Opening pivot could not complete' in frame['programs'][0]['error']

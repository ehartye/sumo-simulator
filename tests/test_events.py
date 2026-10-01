from pathlib import Path

from sumo.physics import Arena, World
from sumo.rules import ATOMIC_EAGLES_2022
from sumo.runtime import Runner


def test_sensor_events_are_transitions_and_do_not_depend_on_viewer():
    world = World()
    spotted = [e for e in world.events if e['type'] == 'opponent_spotted']
    assert {e['bot'] for e in spotted} == {0, 1}
    assert all(e['time'] == 0 and e['distanceMm'] < 2000 for e in spotted)
    world.advance(.1)
    count = len(world.events)
    world.snapshot(); world.snapshot()
    assert len(world.events) == count
    world.teleport(1, [1, .026, 1])
    world.advance(.02)
    assert any(e['type'] == 'opponent_lost' and e['bot'] == 0 for e in world.events)


def test_contact_events_survive_between_viewer_frames_and_exclude_floor():
    world = World(arena=Arena(radius=2))
    world.advance(.1)
    assert not any(e['type'] == 'contact_started' for e in world.events)
    world.teleport(0, [0, .026, -.1])
    world.teleport(1, [0, .026, .1])
    for port in ('A', 'B'):
        world.command(0, port, 'run', 700)
    world.advance(.5)
    contacts = [e for e in world.events if e['type'] == 'contact_started']
    assert contacts and contacts[0]['time'] < world.data.time
    assert contacts[0]['parts'][0].startswith('b0_')
    assert contacts[0]['parts'][1].startswith('b1_')
    world.teleport(1, [1, .026, 1])
    world.advance(.001)
    assert world.events[-1]['type'] == 'contact_ended'


def test_program_messages_and_result_have_simulation_times():
    source = 'from pybricks.tools import wait\nwait(20)\nprint("charge")\nwait(200)'
    with Runner(World(arena=Arena(timeout=.1)), [source, 'pass']) as runner:
        runner.step(.1)
        events = runner.world.events
        message = next(e for e in events if e['type'] == 'program_log')
        assert .02 <= message['time'] < .03
        assert message['bot'] == 0 and message['message'] == 'charge'
        assert events[-1]['type'] == 'match_ended'
        assert events[-1]['result']['reason'] == 'timeout'
        assert runner.snapshot()['eventCount'] == len(events)


def test_default_programs_detect_and_make_contact():
    root = Path(__file__).resolve().parents[1]
    programs = [(root/'examples'/name).read_text() for name in ('aggressive.py', 'patient.py')]
    with Runner(World(seed=7, arena=ATOMIC_EAGLES_2022.arena(20), outward_start=True), programs, rules=ATOMIC_EAGLES_2022) as runner:
        # Check physical engagement, not merely successful startup.
        contact = False
        for _ in range(200):
            frame = runner.step(.1)
            contact |= frame['contacts'] > 0
            if frame['result']:
                break
        assert all(not b['violation'] for b in frame['rules']['bots'])
        assert contact, 'Default strategies never made robot-to-robot contact'

import ast
from pathlib import Path


def helpers(filename='14_forward_weave.py'):
    path = Path(__file__).resolve().parents[1]/'experiments/strategies'/filename
    tree = ast.parse(path.read_text(encoding='utf-8'))
    namespace = {}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)],
                            type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def test_duel_alarm_needs_fresh_close_return_and_stable_sensor_history():
    detect = helpers()['duel_alarm']
    samples = [(t, 110+t % 3, 180+t % 2) for t in range(0, 1801, 100)]
    assert detect(samples, 110)
    assert not detect(samples, 2000)
    assert not detect(samples[:4], 110)
    assert not detect([(t, 190-t*.08, h) for t, _, h in samples], 60)
    assert not detect([(t, d, t*.1) for t, d, _ in samples], 110)


def test_blind_watchdog_requires_sustained_forward_command_without_observation():
    alarm = helpers()['blind_alarm']
    assert alarm(4000, 0, 0, 0, 3000)
    assert not alarm(4000, 3500, 0, 0, 3000)
    assert not alarm(4000, 0, 3500, 0, 3000)
    assert not alarm(4000, 0, 0, 3500, 3000)
    assert not alarm(4000, 0, 0, 0, 0)


def test_tip_guard_checks_roll_as_well_as_pitch_and_stops_power_when_inverted():
    guard = helpers('16_tilt_release.py')['tip_action']
    assert guard(0, 14) == 'retreat'
    assert guard(-14, 0) == 'retreat'
    assert guard(0, -176) == 'brake'
    assert guard(80, 0) == 'brake'
    assert guard(4, -5) is None


def test_rotating_engagement_budget_needs_duration_coverage_and_fresh_close_reading():
    alarm = helpers('17_circling_release.py')['duel_alarm']
    rotating = [(t, 105+t % 17, t*.08) for t in range(0, 4201, 100)]
    assert alarm(rotating, 110)
    assert not alarm(rotating[:20], 110)
    assert not alarm(rotating, 2000)
    assert not alarm([(t, 2000 if t<3000 else d, h) for t,d,h in rotating], 110)


def test_hook_target_requires_achieved_rotation_in_requested_direction():
    done = helpers('20_feedback_hook.py')['hook_complete']
    assert done(216, 180, 1, 35)
    assert done(144, 180, -1, 35)
    assert not done(130, 180, 1, 35)
    assert not done(178, 180, 1, 35)


def test_border_axis_hint_expires_and_never_excludes_an_opponent_sector():
    choose = helpers('21_border_bias.py')['choose_hook']
    assert choose(210, 180, 1000, 80, 1) == (-1, 'recent border retreat axis')
    assert choose(150, 180, 1000, 80, -1) == (1, 'recent border retreat axis')
    assert choose(210, 180, 5000, 80, 1) == (1, 'alternating fallback')
    assert choose(210, 180, 1000, 250, 1) == (1, 'alternating fallback')
    assert choose(181, 180, 1000, 80, 1) == (1, 'alternating fallback')
    assert choose(210, 180, -1, 80, 1) == (1, 'alternating fallback')
    alarm = helpers('21_border_bias.py')['blind_alarm']
    assert alarm(4000, None, None, 0, 3000)
    assert not alarm(4000, 3500, None, 0, 3000)
    assert alarm(4000, 0, None, 0, 3000)
    assert not alarm(4000, 0, None, 3500, 3000)


def test_engagement_steering_uses_unwrapped_gyro_trend_and_falls_back_when_uncertain():
    choose = helpers('23_rotation_steering.py')['engagement_side']
    assert choose([(0,110,350),(1000,110,450)], -1) == (1, 'follow observed rotation')
    assert choose([(0,110,450),(1000,110,350)], 1) == (-1, 'follow observed rotation')
    assert choose([(0,110,180),(1000,110,183)], -1) == (-1, 'alternating fallback')
    assert choose([], 1) == (1, 'alternating fallback')

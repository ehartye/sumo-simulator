import math

from tools.lockup_lab import analyse


def rows(gaps, dt=.02, contact=False):
    out = []
    for i, gap in enumerate(gaps):
        bots = {}
        for bot, x in enumerate((0., gap)):
            bots[f'b{bot}'] = {'x': x, 'z': 0., 'fx': 0., 'fz': 1., 'wa': 900., 'wb': 900., 'ia': .1, 'ib': .1, 'tilt': 0.,
                               'load': {'tireA': 0. if i % 2 else 1., 'tireB': 0. if i % 2 else 1., 'caster': 1.}}
        out.append({'t': round(i*dt, 4), 'contact': contact, **bots})
    return out


def test_pressed_nose_to_nose_draw_is_a_lockup_with_unloaded_tires():
    result = analyse(rows([.15]*400), {'winner': None, 'reason': 'timeout', 'time': 8.})
    assert result['lockSeconds'] > 7.5 and result['cause']['class'] == 'timeout'
    assert result['episodes'][0]['bots'][0]['tireUnloadedFrac'] == .5


def test_separated_bots_are_not_locked():
    result = analyse(rows([.6]*400), {'winner': None, 'reason': 'timeout', 'time': 8.})
    assert result['lockSeconds'] == 0 and result['episodes'] == []


def test_exit_long_after_engagement_is_unforced_and_prompt_exit_is_forced():
    early = [.15]*50+[.6]*400  # engaged for 1 s, then apart for 8 s
    assert analyse(rows(early), {'winner': 0, 'reason': 'outside_floor_contact', 'time': 9.})['cause']['class'] == 'unforced'
    prompt = [.6]*100+[.15]*100
    assert analyse(rows(prompt), {'winner': 0, 'reason': 'outside_floor_contact', 'time': 4.})['cause']['class'] == 'forced'


def test_slow_creep_still_counts_but_a_shove_across_the_ring_does_not():
    shove = [.15]*200
    moving = rows(shove)
    for i, row in enumerate(moving):
        row['b0']['x'] += i*.004
        row['b1']['x'] += i*.004   # pair travels 0.8 m in 4 s: a decisive push, not a lockup
    assert analyse(moving, {'winner': None, 'reason': 'timeout', 'time': 4.})['lockSeconds'] == 0

"""Physics-only lockup-escape search. No Python controllers, no sensors: fast branching from a lockup.

A fixture drives two stock robots head-on until they lock (noses pressed, both pushing), snapshots the
MuJoCo state, then replays many open-loop wheel maneuvers for bot 0 against a scripted bot 1 from that same
state. Scores are observer measurements (ring position, floor exit); a maneuver is only a *candidate*
until a Pybricks-subset controller reproduces it from its own sensors in full matches.
All results are uncalibrated simulator observations; sweep friction rather than trust one value.
"""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import random

import mujoco
import numpy as np

from sumo.physics import World
from sumo.rules import ATOMIC_EAGLES_2022

FIXTURES = [  # (lateral offset mm of bot0, yaw deg of bot0, lateral offset mm of bot1, yaw deg of bot1)
    (0, 0, 0, 0), (12, 3, -12, -3), (-20, 5, 8, -2), (25, -4, 0, 4),
    (-12, -3, 12, 3), (20, -5, -8, 2), (-25, 4, 0, -4), (6, 6, 6, -6),
]
_rng = random.Random(2026)
# 40 more randomized contact geometries (offset mm, yaw deg per bot) for outcome statistics.
FIXTURES += [(_rng.uniform(-30, 30), _rng.uniform(-6, 6), _rng.uniform(-30, 30), _rng.uniform(-6, 6)) for _ in range(40)]
LOCK_S = 2.2
POST_S = 4.0
CHARGE = 100


def _make_world(friction, fixture):
    w = World(seed=1, arena=ATOMIC_EAGLES_2022.arena(120), friction=friction)
    w.sensors = lambda bot: None  # observer-only lab; skip 3D ultrasonic casts
    off0, yaw0, off1, yaw1 = fixture
    for bot, off, yaw in ((0, off0, yaw0), (1, off1, yaw1)):
        addr = w.model.jnt_qposadr[w.model.joint(f'b{bot}_root').id]
        w.data.qpos[addr] += off/1000
        q = w.data.qpos[addr+3:addr+7].copy()
        h = math.radians(yaw)/2
        dq = np.array([math.cos(h), 0, math.sin(h), 0])
        w1, x1, y1, z1 = dq
        w2, x2, y2, z2 = q
        w.data.qpos[addr+3:addr+7] = [w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                                      w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2]
    mujoco.mj_forward(w.model, w.data)
    return w


def _drive(w, bot, left, right):
    """(mode, value) pairs; forward is raw positive on both ports (programs mount B counterclockwise)."""
    for port, (mode, value) in zip('AB', (left, right)):
        w.command(bot, port, mode, value)


def _heading_error(w, me, other):
    """Bearing of `other` in `me`'s frame, radians (positive = to the left, +X). Observer truth."""
    pm, qm = w.pose(me)
    po, _ = w.pose(other)
    w_, x, y, z = qm
    fx, fz = 2*(x*z+w_*y), 1-2*(x*x+y*y)
    dx, dz = po[0]-pm[0], po[2]-pm[2]
    return math.atan2(fz*dx-fx*dz, fx*dx+fz*dz)


def _opponent(w, kind, t, bot=1):
    """Scripted policy for `bot`. 'duty'/'run850' charge blindly. 'trk[:thr_deg[:pivot[:fwd[:run]]]]' is an
    observer-truth tracker (perfect bearing; stronger than the real +/-35 degree sonar): charge when within
    thr_deg of the target bearing, else pivot toward it, backing off at the rim. Re-decides every 10 ms."""
    me, other = bot, 1-bot
    if kind == 'duty':
        _drive(w, me, ('dc', CHARGE), ('dc', CHARGE))
    elif kind == 'run850':
        _drive(w, me, ('run', 850), ('run', 850))
    elif kind.startswith('pwm'):
        # pwm:hi:lo:period_ms:duty_cycle:phase_frac - square-wave drive; phase shifts the right wheel (fraction of period).
        _, hi, lo, period, cycle, phase = kind.split(':')
        hi, lo, period, cycle, phase = float(hi), float(lo), float(period), float(cycle), float(phase)
        ms = t*1000
        def level(offset):
            return ('dc', hi) if ((ms/period)-offset) % 1 < cycle else ('dc', lo)
        _drive(w, me, level(0), level(phase))
    elif kind.startswith('trk') or kind in ('tracker', 'tracker_run'):
        if kind == 'tracker':
            kind = 'trk:12:60:100'
        elif kind == 'tracker_run':
            kind = 'trk:12:60:850:run'
        parts = kind.split(':')
        thr = float(parts[1]) if len(parts) > 1 else 12.
        pivot = float(parts[2]) if len(parts) > 2 else 60.
        fwd_value = float(parts[3]) if len(parts) > 3 else CHARGE
        mode = parts[4] if len(parts) > 4 else 'dc'
        if int(round(t*1000)) % 10:
            return
        p, _ = w.pose(me)
        if math.hypot(p[0], p[2]) > .55:
            _drive(w, me, ('dc', -60), ('dc', -60))
            return
        err = _heading_error(w, me, other)
        fwd = (mode, fwd_value)
        if abs(err) < math.radians(thr):
            _drive(w, me, fwd, fwd)
        else:
            turn = pivot if err > 0 else -pivot  # target to the left: right wheel faster
            _drive(w, me, ('dc', -turn), ('dc', turn))
    else:
        raise ValueError(kind)


def lock_state(friction, fixture, opponent):
    w = _make_world(friction, fixture)
    w.advance(.3)
    steps = int(LOCK_S/.001)
    for i in range(steps):
        _drive(w, 0, ('dc', CHARGE), ('dc', CHARGE))
        _opponent(w, opponent, i*.001)
        w.advance(.001)
    return w


def _radius(w, bot):
    p, _ = w.pose(bot)
    return math.hypot(p[0], p[2])


def _save(w):
    return {'data': copy.copy(w.data), 'modes': [[dict(m) for m in row] for row in w.modes], 'voltage': list(w.voltage),
            'currents': [list(c) for c in w.currents], 'time': float(w.data.time)}


def _restore(w, s):
    w.data = copy.copy(s['data'])
    w.modes = [[dict(m) for m in row] for row in s['modes']]
    w.voltage = list(s['voltage'])
    w.currents = [list(c) for c in s['currents']]
    w.result = None
    w._result_logged = False


def run_maneuver(w, saved, segments, opponent, post_s=POST_S):
    """segments: [(ms, (modeL, valL), (modeR, valR)), ...] then full charge. Returns observer metrics."""
    _restore(w, saved)
    r0_us, r0_opp = _radius(w, 0), _radius(w, 1)
    p0 = np.array([w.pose(b)[0][[0, 2]] for b in (0, 1)])
    t = 0
    plan = [(ms, l, r) for ms, l, r in segments] + [(int(post_s*1000), ('dc', CHARGE), ('dc', CHARGE))]
    max_tilt = [0., 0.]
    closest_sep = 9.
    for ms, left, right in plan:
        _drive(w, 0, left, right)
        for _ in range(ms):
            _opponent(w, opponent, t*.001)
            w.advance(.001)
            t += 1
            if w.result:
                break
            if t % 20 == 0:
                for b in (0, 1):
                    _, q = w.pose(b)
                    max_tilt[b] = max(max_tilt[b], math.degrees(math.acos(max(-1., min(1., 1-2*(q[1]**2+q[3]**2))))))
        if w.result:
            break
    winner = w.result['winner'] if w.result else None
    # Positive advantage: opponent ended closer to the edge relative to us than when the maneuver began.
    adv = (_radius(w, 1)-_radius(w, 0))-(r0_opp-r0_us)
    pe = np.array([w.pose(b)[0][[0, 2]] for b in (0, 1)])
    gap = float(np.linalg.norm(pe[0]-pe[1]))
    return {'winner': winner, 'adv': float(adv), 'gap': gap, 'forced': bool(winner == 0 and gap < .25), 'endT': t/1000, 'tilt': max_tilt,
            'moved': [float(np.linalg.norm(pe[b]-p0[b])) for b in (0, 1)]}


def run_policy(w, saved, challenger, opponent, secs=8.0):
    """Both bots run scripted policies from the lock state; observer metrics only."""
    _restore(w, saved)
    t = 0
    for _ in range(int(secs*1000)):
        _opponent(w, challenger, t*.001, bot=0)
        _opponent(w, opponent, t*.001, bot=1)
        w.advance(.001)
        t += 1
        if w.result:
            break
    winner = w.result['winner'] if w.result else None
    pe = np.array([w.pose(b)[0][[0, 2]] for b in (0, 1)])
    gap = float(np.linalg.norm(pe[0]-pe[1]))
    return {'winner': winner, 'gap': gap, 'endT': t/1000, 'forced': bool(winner == 0 and gap < .25),
            'adv': float((_radius(w, 1)-_radius(w, 0)))}


def evaluate_policy(task):
    friction, opponent = task['friction'], task['opponent']
    outs = []
    for fi in task['fixtures']:
        key = (friction, fi, opponent)
        if key not in _CACHE:
            w = lock_state(friction, FIXTURES[fi], opponent)
            _CACHE[key] = (w, _save(w))
        w, saved = _CACHE[key]
        outs.append(run_policy(w, saved, task['challenger'], opponent, task.get('secs', 8.)))
    return {**{k: task[k] for k in ('friction', 'opponent', 'challenger')}, 'outcomes': outs}


def policy_matrix(args):
    fixtures = [int(f) for f in args.fixtures.split(',')]
    challengers = args.challengers.split(',')
    tasks = [{'friction': float(f), 'opponent': o, 'challenger': c, 'fixtures': fixtures, 'secs': args.secs}
             for c in challengers for f in args.frictions.split(',') for o in args.opponents.split(',')]
    from collections import defaultdict
    agg = defaultdict(lambda: {'w': 0, 'l': 0, 'f': 0, 'n': 0, 'cells': {}})
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(evaluate_policy, tasks, chunksize=1):
            o = r['outcomes']
            a = agg[r['challenger']]
            w_, l_ = sum(x['winner'] == 0 for x in o), sum(x['winner'] == 1 for x in o)
            a['w'] += w_; a['l'] += l_; a['f'] += sum(x['forced'] for x in o); a['n'] += len(o)
            a['cells'][f"{r['friction']}/{r['opponent']}"] = f'{w_}-{l_}/{len(o)}'
    for c, a in sorted(agg.items(), key=lambda kv: kv[1]['w']-kv[1]['l'], reverse=True):
        print(f"{c:28s} W{a['w']:3d} L{a['l']:3d} forcedW{a['f']:3d} of {a['n']}")
        if args.cells:
            print('   ', a['cells'])


LEVELS = [-100, -60, -30, 0, 30, 60, 100]
MODES = ['dc', 'brake', 'hold']
DURATIONS = [30, 60, 100, 160, 250, 400, 600, 900]


def random_maneuver(rng, nseg):
    segs = []
    for _ in range(nseg):
        side = []
        for _ in range(2):
            if rng.random() < .12:
                side.append((rng.choice(['brake', 'hold']), 0))
            else:
                side.append(('dc', rng.choice(LEVELS)))
        segs.append((rng.choice(DURATIONS), side[0], side[1]))
    return segs


def evaluate(task):
    """One candidate across fixtures and a friction value. Worker-local world cache keeps this fast."""
    friction, opponent, segments = task['friction'], task['opponent'], task['segments']
    cache = _CACHE
    outcomes = []
    for fi in task['fixtures']:
        key = (friction, fi, opponent)
        if key not in cache:
            w = lock_state(friction, FIXTURES[fi], opponent)
            cache[key] = (w, _save(w))
        w, saved = cache[key]
        outcomes.append(run_maneuver(w, saved, segments, opponent))
    return {'friction': friction, 'opponent': opponent, 'segments': segments, 'outcomes': outcomes}


_CACHE = {}


def summarize(res):
    outs = res['outcomes']
    wins = sum(o['winner'] == 0 for o in outs)
    losses = sum(o['winner'] == 1 for o in outs)
    return {'wins': wins, 'losses': losses, 'meanAdv': float(np.mean([o['adv'] for o in outs])),
            'minAdv': float(min(o['adv'] for o in outs))}


def validate(args):
    prior = json.loads(args.validate.read_text(encoding='utf-8'))
    ranked = sorted(prior[1:], key=lambda r: (r['summary']['wins']-r['summary']['losses'], r['summary']['meanAdv']), reverse=True)
    top = [r['segments'] for r in ranked[:args.top]]
    fixtures = [int(f) for f in args.fixtures.split(',')]
    tasks = [{'friction': float(f), 'opponent': o, 'fixtures': fixtures, 'segments': seg}
             for seg in top for f in args.frictions.split(',') for o in args.opponents.split(',')]
    tasks += [{'friction': float(f), 'opponent': o, 'fixtures': fixtures, 'segments': []}
              for f in args.frictions.split(',') for o in args.opponents.split(',')]
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(evaluate, tasks, chunksize=1):
            outs = r['outcomes']
            r['summary'] = {**summarize(r), 'forcedWins': sum(o['forced'] for o in outs), 'n': len(outs)}
            results.append(r)
    args.out.write_text(json.dumps(results), encoding='utf-8')
    # Aggregate per maneuver across every (friction, opponent) cell.
    from collections import defaultdict
    agg = defaultdict(lambda: {'w': 0, 'l': 0, 'f': 0, 'n': 0, 'cells': {}})
    for r in results:
        key = json.dumps(r['segments'])
        a = agg[key]
        a['w'] += r['summary']['wins']; a['l'] += r['summary']['losses']; a['f'] += r['summary']['forcedWins']; a['n'] += r['summary']['n']
        a['cells'][f"{r['friction']}/{r['opponent']}"] = f"{r['summary']['wins']}-{r['summary']['losses']}/{r['summary']['n']}"
    print('NULL', dict(agg['[]']))
    for key, a in sorted(agg.items(), key=lambda kv: (kv[1]['w']-kv[1]['l'], kv[1]['f']), reverse=True)[:10]:
        print(f"W{a['w']} L{a['l']} forcedW{a['f']} of {a['n']}", key); print('   ', a['cells'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validate', type=Path, help='re-run the top candidates of a search file across --frictions/--opponents')
    parser.add_argument('--challengers', help='comma list of policy names: runs a policy-vs-policy matrix from the lock')
    parser.add_argument('--secs', type=float, default=8.)
    parser.add_argument('--lock-s', type=float, default=None, help='pre-lock seconds; 0 = maneuver starts at the opening gap')
    parser.add_argument('--cells', action='store_true')
    parser.add_argument('--top', type=int, default=25)
    parser.add_argument('--frictions', default='.5,.65,.8,1.0')
    parser.add_argument('--opponents', default='duty,tracker,tracker_run')
    parser.add_argument('--candidates', type=int, default=400)
    parser.add_argument('--nseg', type=int, default=2)
    parser.add_argument('--opponent', default='duty')
    parser.add_argument('--friction', type=float, default=.8)
    parser.add_argument('--fixtures', default='0,1,2,3')
    parser.add_argument('--jobs', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if args.challengers:
        return policy_matrix(args)
    if args.validate:
        return validate(args)
    rng = random.Random(args.seed)
    fixtures = [int(f) for f in args.fixtures.split(',')]
    tasks = [{'friction': args.friction, 'opponent': args.opponent, 'fixtures': fixtures,
              'segments': random_maneuver(rng, args.nseg)} for _ in range(args.candidates)]
    # Baselines: hold the charge (no maneuver) is the null hypothesis.
    tasks.insert(0, {'friction': args.friction, 'opponent': args.opponent, 'fixtures': fixtures, 'lockS': args.lock_s, 'segments': []})
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(evaluate, tasks, chunksize=2):
            r['summary'] = summarize(r)
            results.append(r)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(results), encoding='utf-8')
    base = results[0]['summary']
    print('null (keep charging):', base)
    ranked = sorted(results[1:], key=lambda r: (r['summary']['wins']-r['summary']['losses'], r['summary']['meanAdv']), reverse=True)
    for r in ranked[:12]:
        print(r['summary'], r['segments'])


if __name__ == '__main__':
    main()

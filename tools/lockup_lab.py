"""Observer-side lockup telemetry. Never feeds anything back to a controller.

Samples the live World every SAMPLE_S and records, per robot: chassis position,
ground velocity, wheel surface speed (slip proxy), tilt, and motor current, plus
pair contact and which parts touch. Lockup episodes are contiguous intervals of
sustained contact in which the pair's centre of mass barely translates.
All values are uncalibrated simulator observations, not hardware measurements.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from sumo.physics import World
from sumo.rules import ATOMIC_EAGLES_2022
from sumo.runtime import RUNTIME_PROFILE, Runner

SAMPLE_S = .02
WHEEL_RADIUS_M = .028


def _tilt_deg(world, bot):
    _, q = world.pose(bot)
    w, x, y, z = q
    return math.degrees(math.acos(max(-1., min(1., 1-2*(x*x+z*z)))))


def _forward(world, bot):
    _, q = world.pose(bot)
    w, x, y, z = q
    return np.array([2*(x*z+w*y), 2*(y*z-w*x)*0, 1-2*(x*x+y*y)])


def contact_loads(world):
    """Normal force (N) per support and between robots, from MuJoCo's contact solver."""
    model, data = world.model, world.data
    ring_ids = {model.geom('outside_floor').id, model.geom('ring').id}
    loads = [{'tireA': 0., 'tireB': 0., 'caster': 0., 'other': 0., 'otherParts': {}} for _ in range(2)]
    between = 0.
    parts = {}
    wrench = np.zeros(6)
    for i, c in enumerate(data.contact):
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        mujoco.mj_contactForce(model, data, i, wrench)
        fn = float(wrench[0])
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or ''
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or ''
        if g1 in ring_ids or g2 in ring_ids:
            name = n2 if g1 in ring_ids else n1
            if not name.startswith(('b0_', 'b1_')):
                continue
            bot = int(name[1])
            if 'caster_ball' in name:
                loads[bot]['caster'] += fn
            elif 'left_tire' in name:
                loads[bot]['tireA'] += fn
            elif 'right_tire' in name:
                loads[bot]['tireB'] += fn
            else:
                loads[bot]['other'] += fn
                loads[bot]['otherParts'][name] = loads[bot]['otherParts'].get(name, 0.)+fn
        elif n1.startswith(('b0_', 'b1_')) and n2.startswith(('b0_', 'b1_')) and n1[1] != n2[1]:
            between += fn
            key = ' | '.join(sorted([n1, n2]))
            parts[key] = parts.get(key, 0.)+fn
    return loads, between, parts


def sample(world):
    loads, between, pair_parts = contact_loads(world)
    row = {'t': round(float(world.data.time), 4), 'contact': bool(world._touching),
           'pairForce': between, 'pairParts': pair_parts}
    for bot in range(2):
        pos, _ = world.pose(bot)
        fwd = _forward(world, bot)
        planar = np.array([fwd[0], fwd[2]])
        norm = np.linalg.norm(planar)
        planar = planar/norm if norm > 1e-6 else np.array([0., 1.])
        speeds = [world.wheel_speed(bot, p) for p in ('A', 'B')]
        row[f'b{bot}'] = {
            'x': float(pos[0]), 'z': float(pos[2]), 'fx': float(planar[0]), 'fz': float(planar[1]),
            # Wheel surface speed along chassis forward; sign follows the wheel joint axes. Left
            # wheel A, right wheel B; programs mount B counterclockwise, so rolling forward is
            # A positive / B negative in joint space. Signs are normalised in analyse().
            'wa': speeds[0], 'wb': speeds[1],
            'tilt': _tilt_deg(world, bot), 'load': loads[bot],
            'ia': world.currents[bot][0], 'ib': world.currents[bot][1],
            'cmd': [world.modes[bot][0]['mode'], world.modes[bot][0]['value'],
                    world.modes[bot][1]['mode'], world.modes[bot][1]['value']],
        }
    return row


ENGAGED_M = .19           # chassis-centre distance at which the noses are pressed together (eyes-to-eyes ~.182)
LOCK_MIN_S = 2.0          # engaged this long with the pair barely translating is a lockup
LOCK_PAIR_SPEED = .03     # m/s of pair-centre path length
FORCED_WINDOW_S = 1.5     # a loss within this long of engagement is credited to the opponent's push


def _runs(mask, t, max_gap):
    """Contiguous True spans in mask, merging gaps shorter than max_gap seconds."""
    spans, start, last = [], None, None
    for i, on in enumerate(mask):
        if on:
            if start is None:
                start = i
            last = i
        elif start is not None and t[i]-t[last] > max_gap:
            spans.append((start, last))
            start = None
    if start is not None:
        spans.append((start, last))
    return spans


def analyse(rows, result):
    t = np.array([r['t'] for r in rows])
    p = np.array([[[r[f'b{b}']['x'], r[f'b{b}']['z']] for b in range(2)] for r in rows])  # n,2,2
    centre = p.mean(axis=1)
    gap = np.linalg.norm(p[:, 0]-p[:, 1], axis=1)
    engaged = gap < ENGAGED_M
    out = []
    for a, b in _runs(engaged, t, .25):
        dur = float(t[b]-t[a])
        if dur < .25:
            continue
        path = float(np.linalg.norm(np.diff(centre[a:b+1:25], axis=0), axis=1).sum())  # 0.5 s stride: sample jitter would inflate it
        hop = []
        for bot in range(2):
            k = f'b{bot}'
            tires = np.array([rows[i][k]['load']['tireA']+rows[i][k]['load']['tireB'] for i in range(a, b+1)])
            rim = np.array([(abs(rows[i][k]['wa'])+abs(rows[i][k]['wb']))/2*math.pi/180*WHEEL_RADIUS_M for i in range(a, b+1)])
            cur = np.array([rows[i][k]['ia']+rows[i][k]['ib'] for i in range(a, b+1)])
            yaw = np.unwrap(np.arctan2([rows[i][k]['fx'] for i in range(a, b+1)], [rows[i][k]['fz'] for i in range(a, b+1)]))
            hop.append({'tireUnloadedFrac': float((tires == 0).mean()), 'tireLoadMean': float(tires.mean()),
                        'rimSpeedMean': float(rim.mean()), 'currentMean': float(cur.mean()),
                        'yawSpanDeg': float(np.degrees(yaw.max()-yaw.min())),
                        'maxTilt': float(max(rows[i][k]['tilt'] for i in range(a, b+1)))})
        pair_speed = path/dur
        out.append({'start': float(t[a]), 'end': float(t[b]), 'duration': dur, 'pathM': path, 'pairSpeed': pair_speed,
                    'lockup': bool(dur >= LOCK_MIN_S and pair_speed < LOCK_PAIR_SPEED), 'bots': hop})
    total = float(t[-1]-t[0]) if len(t) > 1 else 0.
    lock = sum(e['duration'] for e in out if e['lockup'])
    # Win-cause: who lost, how long after the last engagement, and whether a push preceded it.
    cause = None
    if result and result.get('reason') == 'outside_floor_contact':
        end = float(result['time'])
        idx = np.where(engaged)[0]
        since = float(end-t[idx[-1]]) if len(idx) else None
        cause = 'forced' if since is not None and since <= FORCED_WINDOW_S else 'unforced'
        cause = {'class': cause, 'secondsSinceEngaged': since}
    elif result and result.get('reason') == 'timeout':
        cause = {'class': 'timeout', 'lockedFraction': lock/total if total else 0.}
    return {'episodes': out, 'matchSeconds': total, 'lockSeconds': lock, 'lockFraction': lock/total if total else 0.,
            'engagedFraction': float(engaged.mean()), 'longestEngaged': max((e['duration'] for e in out), default=0.),
            'longestLock': max((e['duration'] for e in out if e['lockup']), default=0.),
            'firstEngaged': float(t[np.argmax(engaged)]) if engaged.any() else None,
            'cause': cause, 'result': result}


def run_bout(task):
    programs = [Path(p).read_text(encoding='utf-8') for p in task['programs']]
    swapped = task['swapped']
    order = programs[::-1] if swapped else programs
    world = World(seed=task['seed'], arena=ATOMIC_EAGLES_2022.arena(task['timeout']), outward_start=True,
                  friction=task.get('friction', .8))
    jitter = task.get('jitter')
    if jitter:
        # Research-only placement spread (mm along ring tangent/radius, degrees yaw) so repeated seeds
        # are not exact physical repeats. Sensor noise alone leaves trajectories identical.
        rng = np.random.default_rng(task['seed']*7919+17)
        for bot in range(2):
            addr = world.model.jnt_qposadr[world.model.joint(f'b{bot}_root').id]
            world.data.qpos[addr] += rng.uniform(-1, 1)*jitter['mm']/1000
            world.data.qpos[addr+2] += rng.uniform(-1, 1)*jitter['mm']/1000
            yaw = math.radians(rng.uniform(-1, 1)*jitter['deg'])
            q = world.data.qpos[addr+3:addr+7].copy()
            dq = np.array([math.cos(yaw/2), 0, math.sin(yaw/2), 0])
            w1, x1, y1, z1 = dq; w2, x2, y2, z2 = q
            world.data.qpos[addr+3:addr+7] = [w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                                              w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2]
        mujoco.mj_forward(world.model, world.data)
        world.sensor_tick = [-1, -1]
    rows = []
    with Runner(world, order, rules=ATOMIC_EAGLES_2022, runtime_profile=task.get('runtime', RUNTIME_PROFILE)) as runner:
        rows.append(sample(world))
        while not world.result:
            runner.step(SAMPLE_S)
            rows.append(sample(world))
        final = runner.snapshot()
        events = list(world.events)
        logs = [p['logs'] for p in final['programs']]
        errors = [p['error'] for p in final['programs']]
    result = final['result']
    winner = result['winner']
    first = None if winner is None else (1-winner if swapped else winner)
    summary = analyse(rows, result)
    summary.update({'seed': task['seed'], 'swapped': swapped, 'friction': task.get('friction', .8), 'jitter': jitter, 'winningProgram': first,
                    'programs': task['programs'], 'errors': errors,
                    'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in programs]})
    if task.get('dump'):
        Path(task['dump']).parent.mkdir(parents=True, exist_ok=True)
        Path(task['dump']).write_text(json.dumps({'rows': rows, 'events': events, 'logs': logs}), encoding='utf-8')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('challenger', type=Path)
    parser.add_argument('incumbent', type=Path)
    parser.add_argument('--seeds', default='11,23')
    parser.add_argument('--frictions', default='.8', help='comma list; results are only robust if they hold across it')
    parser.add_argument('--jitter-mm', type=float, default=0)
    parser.add_argument('--jitter-deg', type=float, default=0)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--dump-dir', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    jitter = {'mm': args.jitter_mm, 'deg': args.jitter_deg} if args.jitter_mm or args.jitter_deg else None
    tasks = []
    for friction in (float(f) for f in args.frictions.split(',')):
        for seed in (int(s) for s in args.seeds.split(',')):
            for swapped in (False, True):
                dump = (args.dump_dir/f'{args.challenger.stem}-vs-{args.incumbent.stem}-mu{friction}-s{seed}-{int(swapped)}.json'
                        if args.dump_dir else None)
                tasks.append({'programs': [str(args.challenger), str(args.incumbent)], 'seed': seed, 'swapped': swapped,
                              'timeout': args.timeout, 'friction': friction, 'jitter': jitter, 'dump': str(dump) if dump else None})
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(run_bout, tasks):
            results.append(r)
            cause = r['cause'] or {}
            print(json.dumps({'mu': r['friction'], 'seed': r['seed'], 'swapped': r['swapped'], 'win': r['winningProgram'],
                              'end': round(r['result']['time'], 2), 'cause': cause.get('class'),
                              'lockSec': round(r['lockSeconds'], 1), 'longestLock': round(r['longestLock'], 1),
                              'engaged': round(r['engagedFraction'], 3), 'episodes': len(r['episodes'])}), flush=True)
    wins = sum(r['winningProgram'] == 0 for r in results)
    losses = sum(r['winningProgram'] == 1 for r in results)
    print(json.dumps({'challengerW-D-L': [wins, len(results)-wins-losses, losses], 'meanLockFraction': round(float(np.mean([r['lockFraction'] for r in results])), 3),
                      'forcedWins': sum(r['winningProgram'] == 0 and (r['cause'] or {}).get('class') == 'forced' for r in results),
                      'unforcedWins': sum(r['winningProgram'] == 0 and (r['cause'] or {}).get('class') == 'unforced' for r in results)}))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(results, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()

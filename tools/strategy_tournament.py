"""Paired, reproducible strategy trials with replayable evidence. No physics tuning."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import time

import mujoco

from sumo.physics import IMU_TILT_PROFILE, World
from sumo.robot import ROOT
from sumo.rules import ATOMIC_EAGLES_2022
from sumo.runtime import RUNTIME_PROFILE, Runner
from sumo.ultrasonic import PROFILE


def score_bouts(bouts):
    wins = sum(b['winningProgram'] == 0 for b in bouts)
    draws = sum(b['winningProgram'] is None for b in bouts)
    faults = sum(bool(b['challengerFault'] or any(b.get('programErrors', [])) or
                      b.get('result', {}).get('reason') in ('program_error', 'rule_violation'))
                 for b in bouts)
    score = (wins + .5*draws)/len(bouts) if bouts else 0
    return {'wins': wins, 'draws': draws, 'losses': len(bouts)-wins-draws,
            'score': score, 'faults': faults, 'promote': len(bouts) >= 6 and score >= .75 and faults == 0}


def run_bout(task):
    started = time.monotonic()
    swapped = task['swapped']
    programs = task['programs'][::-1] if swapped else task['programs']
    names = task['names'][::-1] if swapped else task['names']
    world = World(seed=task['seed'], arena=ATOMIC_EAGLES_2022.arena(task['timeout']),
                  outward_start=True, friction=task['friction'])
    metadata = {'seed': task['seed'], 'programs': programs, 'names': names,
                'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in programs],
                'robotHash': hashlib.sha256((ROOT/'artifacts/robot/robot.json').read_bytes()).hexdigest(),
                'engine': f'MuJoCo {mujoco.__version__}', 'timestep': world.model.opt.timestep,
                'arena': vars(world.arena), 'rules': ATOMIC_EAGLES_2022.info(),
                'friction': task['friction'], 'ultrasonicModel': PROFILE,
                'fidelity': 'uncalibrated', 'runtime': RUNTIME_PROFILE, 'imuTiltModel': IMU_TILT_PROFILE}
    with Runner(world, programs, rules=ATOMIC_EAGLES_2022) as runner:
        frames = [runner.snapshot()]
        while not world.result:
            frames.append(runner.step(.05))
        final = frames[-1]
        events = list(world.events)
    replay = {'version': 1, 'metadata': metadata, 'frames': frames, 'events': events}
    path = Path(task['path'])
    path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation protects the trial's original evidence.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(replay, stream, separators=(',', ':'))
    winner = final['result']['winner']
    challenger = 1 if swapped else 0
    return {'seed': task['seed'], 'swapped': swapped, 'friction': task['friction'],
            'result': final['result'], 'winningProgram': None if winner is None else (1-winner if swapped else winner),
            'challengerFault': final['programs'][challenger]['status'] == 'error',
            'programErrors': [p['error'] for p in final['programs']],
            'firstContact': next((e['time'] for e in events if e['type'] == 'contact_started'), None),
            'events': events, 'replay': str(path), 'wallSeconds': round(time.monotonic()-started, 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('challenger', type=Path)
    parser.add_argument('incumbent', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--seeds', default='11,23,37')
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--friction', type=float, default=.8)
    parser.add_argument('--jobs', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(',')]
    if args.out.exists() or not seeds or min(seeds) < 0 or not 1 <= args.timeout <= 120:
        parser.error('Use a new output directory, nonnegative seeds and a 1..120 s timeout')
    args.out.mkdir(parents=True)
    paths = [args.challenger, args.incumbent]
    programs = [p.read_text(encoding='utf-8') for p in paths]
    tasks = [{'programs': programs, 'names': [p.stem for p in paths], 'seed': seed, 'swapped': swapped,
              'timeout': args.timeout, 'friction': args.friction,
              'path': str(args.out/f'seed-{seed}-swap-{int(swapped)}.json')}
             for seed in seeds for swapped in (False, True)]
    bouts = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for bout in pool.map(run_bout, tasks):
            bouts.append(bout)
            print(json.dumps({k: v for k, v in bout.items() if k != 'events'}), flush=True)
    report = {'challenger': str(paths[0]), 'incumbent': str(paths[1]), 'programs': programs,
              'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in programs],
              'score': score_bouts(bouts), 'bouts': bouts}
    (args.out/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report['score']), flush=True)


if __name__ == '__main__':
    main()

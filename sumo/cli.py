import argparse
import hashlib
import json
from pathlib import Path
from uuid import uuid4

from .physics import IMU_TILT_PROFILE, World
from .robot import ROOT
from .runtime import RUNTIME_PROFILE, RUNTIME_PROFILES, Runner
from .rules import ATOMIC_EAGLES_2022
from .ultrasonic import PROFILE as ULTRASONIC_PROFILE


def main():
    parser = argparse.ArgumentParser(description='SPIKE Prime sumo laboratory')
    commands = parser.add_subparsers(dest='command', required=True)
    serve = commands.add_parser('serve')
    serve.add_argument('--port', type=int, default=8765)
    batch = commands.add_parser('batch', help='Run paired bouts with starting sides swapped')
    batch.add_argument('blue', type=Path)
    batch.add_argument('red', type=Path)
    batch.add_argument('--pairs', type=int, default=1)
    batch.add_argument('--seed', type=int, default=0)
    batch.add_argument('--timeout', type=float, default=120, help='Simulation limit in seconds (profile default: 120)')
    batch.add_argument('--out', type=Path, default=Path('artifacts/batch.json'))
    batch.add_argument('--runtime-profile', choices=RUNTIME_PROFILES, default=RUNTIME_PROFILE)
    args = parser.parse_args()
    if args.command == 'serve':
        import uvicorn
        uvicorn.run('sumo.server:app', host='127.0.0.1', port=args.port)
        return
    if not 1 <= args.pairs <= 100 or not 1 <= args.timeout <= 120 or args.seed < 0:
        parser.error('pairs must be 1..100, timeout 1..120, and seed nonnegative')
    programs = [args.blue.read_text(), args.red.read_text()]
    log_dir = args.out.parent/'sensor-logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    run_id = uuid4().hex[:12]
    bouts = []
    for pair in range(args.pairs):
        for swapped in (False, True):
            order = programs[::-1] if swapped else programs
            with Runner(World(seed=args.seed+pair, arena=ATOMIC_EAGLES_2022.arena(args.timeout), outward_start=True), order, rules=ATOMIC_EAGLES_2022, runtime_profile=args.runtime_profile) as runner:
                final = runner.run_until_done()
                result = final['result']
                winner = result['winner'] if result else None
                bouts.append({'seed': args.seed+pair, 'swapped': swapped, 'result': result,
                              'winningProgram': None if winner is None else (1-winner if swapped else winner),
                              'programs': final['programs'], 'events': list(runner.world.events)})
                log_path = log_dir/f'{run_id}-{pair}-{int(swapped)}-ultrasonic.jsonl'
                with log_path.open('w', encoding='utf-8') as stream:
                    for sample in runner.world.ultrasonic_log:
                        stream.write(json.dumps(sample)+'\n')
                bouts[-1]['sensorLogFile'] = str(log_path.resolve())
                print(json.dumps(bouts[-1]))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({'version': 1, 'fidelity': 'uncalibrated',
        'programs': programs, 'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in programs],
        'robotHash': hashlib.sha256((ROOT/'artifacts/robot/robot.json').read_bytes()).hexdigest(),
        'arena': vars(ATOMIC_EAGLES_2022.arena(args.timeout)), 'rules': ATOMIC_EAGLES_2022.info(),
        'ultrasonicModel': ULTRASONIC_PROFILE, 'imuTiltModel': IMU_TILT_PROFILE, 'runtime': args.runtime_profile,
        'bouts': bouts}, indent=2))


if __name__ == '__main__':
    main()

"""Loopback-only development UI. Programs are trusted local Python code."""
from contextlib import asynccontextmanager
import hashlib
import json
import threading
from pathlib import Path
from datetime import datetime, timezone
from uuid import uuid4

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import mujoco

from .physics import World
from .robot import ROOT, load_robot
from .runtime import Runner
from .rules import ATOMIC_EAGLES_2022
from .ultrasonic import PROFILE as ULTRASONIC_PROFILE

lock = threading.RLock()
runner = None
recording = []
metadata = {}
LOG_ROOT = ROOT/'.cache/runs'
sensor_log_path = None
sensor_log_count = 0
event_log_count = 0


def flush_sensor_log():
    global sensor_log_count, event_log_count
    samples = runner.world.ultrasonic_log[sensor_log_count:]
    if samples:
        with sensor_log_path.open('a', encoding='utf-8') as stream:
            for sample in samples:
                stream.write(json.dumps(sample)+'\n')
        sensor_log_count += len(samples)
    events = runner.world.events[event_log_count:]
    if events:
        with Path(metadata['eventLogFile']).open('a', encoding='utf-8') as stream:
            for event in events:
                stream.write(json.dumps(event)+'\n')
        event_log_count += len(events)


@asynccontextmanager
async def lifespan(app):
    yield
    if runner:
        runner.close()


app = FastAPI(lifespan=lifespan)


class Start(BaseModel):
    programs: list[str] = Field(min_length=2, max_length=2)
    seed: int = Field(default=0, ge=0, le=2147483647)
    timeout: float = Field(default=120, ge=1, le=120)


class Step(BaseModel):
    seconds: float = Field(default=.05, ge=.001, le=.25)


@app.get('/api/info')
def info():
    robot = load_robot()
    return {'robot': robot, 'arena': vars(ATOMIC_EAGLES_2022.arena()), 'rules': ATOMIC_EAGLES_2022.info(), 'ultrasonicModel': ULTRASONIC_PROFILE, 'engine': f'MuJoCo {mujoco.__version__}',
            'examples': {p.stem: p.read_text() for p in sorted((ROOT/'examples').glob('*.py'))},
            'fidelity': 'Catalog geometry and mass; dynamics uncalibrated; synchronous Pybricks subset.'}


@app.post('/api/reset')
def reset(request: Start):
    global runner, recording, metadata, sensor_log_path, sensor_log_count, event_log_count
    if any(len(program) > 50000 for program in request.programs):
        raise HTTPException(400, 'Each program must be under 50,000 characters')
    with lock:
        if runner:
            runner.close()
        world = World(seed=request.seed, arena=ATOMIC_EAGLES_2022.arena(request.timeout), outward_start=True)
        runner = Runner(world, request.programs, rules=ATOMIC_EAGLES_2022)
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        sensor_log_path = LOG_ROOT/f'{stamp}-{uuid4().hex[:8]}-ultrasonic.jsonl'
        sensor_log_count = 0
        event_log_count = 0
        metadata = {'seed': request.seed, 'programs': request.programs,
                    'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in request.programs],
                    'robotHash': hashlib.sha256((ROOT/'artifacts/robot/robot.json').read_bytes()).hexdigest(),
                    'engine': f'MuJoCo {mujoco.__version__}', 'timestep': world.model.opt.timestep,
                    'arena': vars(world.arena), 'rules': ATOMIC_EAGLES_2022.info(), 'fidelity': 'uncalibrated', 'runtime': 'sync-subset-v1',
                    'ultrasonicModel': ULTRASONIC_PROFILE, 'sensorLogFile': str(sensor_log_path)}
        metadata['sensorMetadataFile'] = str(sensor_log_path.with_suffix('.metadata.json'))
        metadata['eventLogFile'] = sensor_log_path.with_name(sensor_log_path.name.replace('-ultrasonic', '-events')).as_posix()
        sensor_log_path.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        snapshot = runner.snapshot()
        recording = [snapshot]
        flush_sensor_log()
        return {**snapshot, 'events': list(world.events)}


@app.post('/api/step')
def step(request: Step):
    with lock:
        if not runner:
            raise HTTPException(409, 'Reset the match first')
        previous_count = len(runner.world.events)
        try:
            snapshot = runner.step(request.seconds)
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from exc
        if not recording or snapshot['time'] != recording[-1]['time']:
            recording.append(snapshot)
        else:
            recording[-1] = snapshot
        flush_sensor_log()
        return {**snapshot, 'events': runner.world.events[previous_count:]}


@app.get('/api/replay')
def replay():
    with lock:
        return {'version': 1, 'metadata': metadata, 'frames': recording,
                'events': list(runner.world.events) if runner else [],
                'ultrasonicSamples': list(runner.world.ultrasonic_log) if runner else []}


@app.get('/api/events')
def events():
    with lock:
        return {'logFile': metadata.get('eventLogFile'),
                'events': list(runner.world.events) if runner else []}


@app.get('/api/diagnostics')
def diagnostics():
    with lock:
        return {'model': ULTRASONIC_PROFILE, 'logFile': str(sensor_log_path) if sensor_log_path else None,
                'metadataFile': metadata.get('sensorMetadataFile'),
                'samples': list(runner.world.ultrasonic_log) if runner else []}


app.mount('/assets', StaticFiles(directory=ROOT/'artifacts/robot'), name='assets')
app.mount('/vendor', StaticFiles(directory=ROOT/'node_modules/three'), name='vendor')
app.mount('/ui', StaticFiles(directory=ROOT/'web'), name='ui')


@app.get('/')
def index():
    return FileResponse(ROOT/'web/index.html')

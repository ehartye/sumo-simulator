"""Loopback-only development UI. Programs are trusted local Python code."""
from contextlib import asynccontextmanager
import hashlib
import json
import threading

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import mujoco

from .physics import World
from .robot import ROOT, load_robot
from .runtime import Runner
from .rules import ATOMIC_EAGLES_2022

lock = threading.RLock()
runner = None
recording = []
metadata = {}


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
    return {'robot': robot, 'arena': vars(ATOMIC_EAGLES_2022.arena()), 'rules': ATOMIC_EAGLES_2022.info(), 'engine': f'MuJoCo {mujoco.__version__}',
            'examples': {p.stem: p.read_text() for p in sorted((ROOT/'examples').glob('*.py'))},
            'fidelity': 'Catalog geometry and mass; dynamics uncalibrated; synchronous Pybricks subset.'}


@app.post('/api/reset')
def reset(request: Start):
    global runner, recording, metadata
    if any(len(program) > 50000 for program in request.programs):
        raise HTTPException(400, 'Each program must be under 50,000 characters')
    with lock:
        if runner:
            runner.close()
        world = World(seed=request.seed, arena=ATOMIC_EAGLES_2022.arena(request.timeout), outward_start=True)
        runner = Runner(world, request.programs, rules=ATOMIC_EAGLES_2022)
        metadata = {'seed': request.seed, 'programs': request.programs,
                    'programHashes': [hashlib.sha256(p.encode()).hexdigest() for p in request.programs],
                    'robotHash': hashlib.sha256((ROOT/'artifacts/robot/robot.json').read_bytes()).hexdigest(),
                    'engine': f'MuJoCo {mujoco.__version__}', 'timestep': world.model.opt.timestep,
                    'arena': vars(world.arena), 'rules': ATOMIC_EAGLES_2022.info(), 'fidelity': 'uncalibrated', 'runtime': 'sync-subset-v1'}
        snapshot = runner.snapshot()
        recording = [snapshot]
        return snapshot


@app.post('/api/step')
def step(request: Step):
    with lock:
        if not runner:
            raise HTTPException(409, 'Reset the match first')
        try:
            snapshot = runner.step(request.seconds)
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from exc
        if not recording or snapshot['time'] != recording[-1]['time']:
            recording.append(snapshot)
        else:
            recording[-1] = snapshot
        return snapshot


@app.get('/api/replay')
def replay():
    with lock:
        return {'version': 1, 'metadata': metadata, 'frames': recording}


app.mount('/assets', StaticFiles(directory=ROOT/'artifacts/robot'), name='assets')
app.mount('/vendor', StaticFiles(directory=ROOT/'node_modules/three'), name='vendor')
app.mount('/ui', StaticFiles(directory=ROOT/'web'), name='ui')


@app.get('/')
def index():
    return FileResponse(ROOT/'web/index.html')

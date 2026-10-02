"""Synchronous Pybricks subset, executed in two deterministic RPC workers.

Workers are process-isolated for lifecycle control, NOT a security sandbox. Run only
trusted programs. Hardware calls take one 1 ms scheduler slot; wait uses simulated
time. Diagnostic prints consume no simulated time in v4. Python CPU-only work
must reach its next hardware call within cpu_timeout.
"""
import ast
import builtins
from enum import Enum, IntEnum
import math
import multiprocessing
import random
import time
import traceback
from types import ModuleType


RUNTIME_PROFILE = 'sync-subset-v4'
RUNTIME_PROFILES = ('sync-subset-v3', RUNTIME_PROFILE)
PRINT_BURST_LIMIT = 1024


SUPPORTED_API = {
    'pybricks.hubs': ['PrimeHub: imu.heading(), imu.tilt(calibrated=True), battery.voltage()'],
    'pybricks.pupdevices': [
        'Motor(A/B, positive_direction=CLOCKWISE): run, dc, stop, brake, hold, angle, speed, reset_angle, run_time, run_angle, run_target',
        'ColorSensor(C).reflection()', 'UltrasonicSensor(D).distance()',
    ],
    'pybricks.parameters': ['Port.A..F (only A/B motors, C color, D ultrasonic connected)',
                            'Direction.CLOCKWISE/COUNTERCLOCKWISE', 'Stop.COAST/BRAKE/HOLD'],
    'pybricks.tools': ['wait(milliseconds)', 'StopWatch.time(), StopWatch.reset()'],
    'standard_library': ['math', 'random: random, randint, randrange, uniform, choice, seed'],
    'limitations': ['Synchronous Python only; no async/await, multitask or DriveBase.',
                    '1 ms hardware-call scheduling; sensors update at 100 Hz.',
                    'v4 diagnostic prints take no simulated time; they are not a model of real hub print latency. v3 scheduling remains selectable for historical trials.',
                    'Prints count toward the operation limit; a burst over 1024 without a hardware yield is rejected.',
                    'Encoders: integer degrees at 100 Hz; speed is the finite difference of these samples.',
                    'Motor run_* supports then=Stop.HOLD and wait=True/False.',
                    'Motor angle/speed use degrees and degrees/second; battery voltage uses millivolts.',
                    'IMU heading is startup-relative, unwrapped and clockwise-positive (corrected in sync-subset-v3).',
                    'IMU tilt returns (pitch, roll) in degrees in the fixed robot neutral frame; calibrated=False is unsupported. Tilt error model is uncalibrated.',
                    'A program error loses the match; errors on the same scheduler tick draw. Completed programs coast.',
                    'No security sandbox; trusted local programs only.'],
}


class UnsupportedAPIError(RuntimeError):
    pass


class Port(Enum):
    A = 'A'
    B = 'B'
    C = 'C'
    D = 'D'
    E = 'E'
    F = 'F'


class Direction(IntEnum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Stop(Enum):
    COAST = 'coast'
    BRAKE = 'brake'
    HOLD = 'hold'


def _finite(value, label):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _worker(connection, source, seed):
    """Only this child executes user code; MuJoCo remains in the parent process."""
    def rpc(operation, *arguments):
        connection.send(('rpc', operation, arguments))
        kind, value = connection.recv()
        if kind != 'ok':
            raise RuntimeError(value)
        return value

    def wait(milliseconds):
        milliseconds = _finite(milliseconds, 'wait duration')
        if milliseconds < 0:
            raise ValueError('wait duration must be nonnegative')
        rpc('wait', milliseconds)

    class StopWatch:
        def __init__(self):
            self._start = rpc('clock')

        def time(self):
            return rpc('clock') - self._start

        def reset(self):
            self._start = rpc('clock')

    class Motor:
        def __init__(self, port, positive_direction=Direction.CLOCKWISE):
            if port not in (Port.A, Port.B):
                raise ValueError('Motor port must be Port.A or Port.B')
            if not isinstance(positive_direction, Direction):
                raise ValueError('positive_direction must be a Direction')
            self._port = port.value
            # Hardware mounting: clockwise A and counterclockwise B roll forward.
            self._sign = int(positive_direction) * (1 if port == Port.A else -1)

        def run(self, speed):
            rpc('command', self._port, 'run', self._sign*_finite(speed, 'speed'))

        def dc(self, duty):
            duty = _finite(duty, 'duty')
            if abs(duty) > 100:
                raise ValueError('Motor dc duty must be in -100..100')
            rpc('command', self._port, 'dc', self._sign*duty)

        def stop(self):
            rpc('command', self._port, 'coast', 0)

        def brake(self):
            rpc('command', self._port, 'brake', 0)

        def hold(self):
            rpc('command', self._port, 'hold', 0)

        def angle(self):
            return self._sign*rpc('angle', self._port)

        def speed(self):
            return self._sign*rpc('speed', self._port)

        def reset_angle(self, angle=0):
            rpc('reset_angle', self._port, self._sign*_finite(angle, 'angle'))

        def _motion(self, kind, speed, value, then, should_wait):
            if not isinstance(then, Stop) or not isinstance(should_wait, bool):
                raise ValueError('then must be Stop.COAST/BRAKE/HOLD and wait must be bool')
            rpc('motion', self._port, kind, speed, value, then.value, should_wait)

        def run_time(self, speed, time, then=Stop.HOLD, wait=True):
            duration = _finite(time, 'duration')
            if duration < 0:
                raise ValueError('Motor duration must be nonnegative')
            self._motion('time', self._sign*_finite(speed, 'speed'), duration, then, wait)

        def run_angle(self, speed, rotation_angle, then=Stop.HOLD, wait=True):
            speed = _finite(speed, 'speed')
            if speed == 0:
                raise ValueError('run_angle speed must be nonzero')
            delta = self._sign*_finite(rotation_angle, 'angle')*(1 if speed > 0 else -1)
            self._motion('relative', abs(speed), delta, then, wait)

        def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
            speed = abs(_finite(speed, 'speed'))
            if speed == 0:
                raise ValueError('run_target speed must be nonzero')
            self._motion('target', speed, self._sign*_finite(target_angle, 'angle'), then, wait)

    class ColorSensor:
        def __init__(self, port):
            if port != Port.C:
                raise ValueError('ColorSensor port must be Port.C')

        def reflection(self):
            return rpc('sensor', 'reflection')

    class UltrasonicSensor:
        def __init__(self, port):
            if port != Port.D:
                raise ValueError('UltrasonicSensor port must be Port.D')

        def distance(self):
            return rpc('sensor', 'distance')

    class IMU:
        def heading(self):
            return rpc('heading')

        def tilt(self, calibrated=True):
            if calibrated is not True:
                raise UnsupportedAPIError('Unsupported API: imu.tilt(calibrated=False); only calibrated tilt is modeled')
            return rpc('sensor', 'tilt')

    class Battery:
        def voltage(self):
            return rpc('voltage')

    class PrimeHub:
        def __init__(self):
            self.imu, self.battery = IMU(), Battery()

    class SubsetModule(ModuleType):
        def __getattr__(self, name):
            if name.startswith('__'):
                raise AttributeError(name)
            raise UnsupportedAPIError(f'Unsupported API: {self.__name__}.{name}')

    def module(name, values):
        result = SubsetModule(name)
        result.__dict__.update(values)
        return result

    generator = random.Random(seed)
    def deterministic_seed(value=None):
        generator.seed(seed if value is None else value)

    modules = {
        'pybricks.hubs': module('pybricks.hubs', {'PrimeHub': PrimeHub}),
        'pybricks.pupdevices': module('pybricks.pupdevices', {'Motor': Motor, 'ColorSensor': ColorSensor, 'UltrasonicSensor': UltrasonicSensor}),
        'pybricks.parameters': module('pybricks.parameters', {'Port': Port, 'Direction': Direction, 'Stop': Stop}),
        'pybricks.tools': module('pybricks.tools', {'wait': wait, 'StopWatch': StopWatch}),
        'math': math,
        'random': module('random', {**{name: getattr(generator, name) for name in ('random', 'randint', 'randrange', 'uniform', 'choice')}, 'seed': deterministic_seed}),
    }
    modules['pybricks'] = module('pybricks', {key.split('.')[1]: value for key, value in modules.items() if key.startswith('pybricks.')})

    def import_subset(name, globals=None, locals=None, fromlist=(), level=0):
        if level or name not in modules:
            raise UnsupportedAPIError(f'Unsupported import: {name}; see SUPPORTED_API')
        if fromlist:
            for attribute in fromlist:
                if attribute != '*':
                    getattr(modules[name], attribute)
            return modules[name]
        return modules[name.split('.')[0]]

    def print_program(*values, sep=' ', end='\n', file=None, flush=False):
        if file is not None:
            raise UnsupportedAPIError('Unsupported API: print(file=...)')
        rpc('print', (sep.join(map(str, values))+end)[:2048].rstrip('\n'))

    def unsupported(*args, **kwargs):
        raise UnsupportedAPIError('Unsupported API: file access or interactive input')

    try:
        connection.send(('ready',))
        connection.recv()
        tree = ast.parse(source, filename='<robot-program>')
        if any(isinstance(node, (ast.AsyncFunctionDef, ast.AsyncFor, ast.AsyncWith, ast.Await)) for node in ast.walk(tree)):
            raise UnsupportedAPIError('Unsupported async Python; this runner supports synchronous Pybricks only')
        namespace = {'__name__': '__main__', '__builtins__': {**vars(builtins), '__import__': import_subset, 'print': print_program, 'open': unsupported, 'input': unsupported}}
        exec(compile(tree, '<robot-program>', 'exec'), namespace)
        connection.send(('done',))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as error:
        frames = traceback.extract_tb(error.__traceback__)
        line = next((frame.lineno for frame in reversed(frames) if frame.filename == '<robot-program>'), None)
        message = f'{type(error).__name__}: {error}'
        if line is not None:
            message += f' (line {line})'
        try:
            connection.send(('error', message[:4096]))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class Runner:
    """Run exactly two source strings against World using a spawn/Pipe barrier.

    step(seconds) advances a multiple of 1 ms. snapshot() extends World.snapshot()
    with programs[{status,error,operations,logs}]. run_until_done(max_seconds)
    stops at a match result or the simulation limit. Completed programs coast.
    Always close(), or use this object as a context manager, to reap workers.
    """
    def __init__(self, world, programs, *, cpu_timeout=1.0, operation_limit=100000, rules=None,
                 runtime_profile=RUNTIME_PROFILE):
        if len(programs) != 2 or any(not isinstance(program, str) for program in programs):
            raise ValueError('Runner needs exactly two Python source strings')
        if not math.isclose(world.model.opt.timestep, .001, abs_tol=1e-12):
            raise ValueError('Program runner requires a 1 ms physics timestep')
        if not 0 < cpu_timeout <= 30 or not isinstance(operation_limit, int) or operation_limit < 1:
            raise ValueError('Invalid CPU timeout or operation limit')
        if runtime_profile not in RUNTIME_PROFILES:
            raise ValueError('Unsupported runtime profile')
        self.runtime_profile = runtime_profile
        self.world, self.cpu_timeout, self.operation_limit = world, cpu_timeout, operation_limit
        from .rules import StartProtocol
        self.rules = StartProtocol(rules, world) if rules is not None else None
        self._phases = [None, None]
        self.processes, self.connections = [], []
        self.programs = [{'status': 'running', 'error': None, 'operations': 0, 'logs': []} for _ in programs]
        self.waiting = [None, None]
        self.motions = [{}, {}]
        self.closed = False
        self.sensor_values = [world.sensors(bot) for bot in range(2)]
        self.previous_headings = [value['heading'] for value in self.sensor_values]
        self.headings = [0., 0.]
        self.encoder_tick = -1
        self.encoders = [{port: {'angle': round(world.wheel_angle(bot, port)), 'speed': 0, 'time': world.data.time}
                          for port in ('A', 'B')} for bot in range(2)]
        context = multiprocessing.get_context('spawn')
        try:
            for bot, program in enumerate(programs):
                parent, child = context.Pipe()
                process = context.Process(target=_worker, args=(child, program, int(world.seed)*2+bot), daemon=True)
                process.start()
                child.close()
                self.connections.append(parent)
                self.processes.append(process)
            for connection in self.connections:
                if not connection.poll(10) or connection.recv() != ('ready',):
                    raise RuntimeError('Python worker failed to start')
                connection.send(('start',))
        except BaseException:
            self.close()
            raise

    def _stop(self, bot, status, error=None):
        self.programs[bot].update(status=status, error=error)
        self.waiting[bot] = None
        self.motions[bot].clear()
        for port in ('A', 'B'):
            self.world.command(bot, port, 'coast')
        process = self.processes[bot]
        if process.is_alive():
            process.terminate()
        process.join(timeout=1)

    def _reply(self, bot, value=None):
        self.connections[bot].send(('ok', value))
        self.programs[bot]['status'] = 'running'

    def _sample(self):
        # Sampling both robots in fixed order makes UI snapshot frequency irrelevant to RNG.
        self.sensor_values = [self.world.sensors(bot) for bot in range(2)]
        for bot, value in enumerate(self.sensor_values):
            current = value['heading']
            self.headings[bot] += (current-self.previous_headings[bot]+180) % 360-180
            self.previous_headings[bot] = current
        tick = int((self.world.data.time+1e-9)*100)
        if tick != self.encoder_tick:
            self.encoder_tick = tick
            for bot in range(2):
                for port, sample in self.encoders[bot].items():
                    angle = round(self.world.wheel_angle(bot, port))
                    elapsed = self.world.data.time-sample['time']
                    speed = round((angle-sample['angle'])/elapsed) if elapsed > 1e-9 else sample['speed']
                    sample.update(angle=angle, speed=speed, time=self.world.data.time)

    def _update_motion(self, bot):
        for port, motion in list(self.motions[bot].items()):
            if motion['kind'] == 'time':
                done = self.world.data.time + 1e-9 >= motion['end']
            else:
                remaining = motion['target']-self.world.wheel_angle(bot, port)
                done = remaining*motion['direction'] <= 0
            if done:
                self.world.command(bot, port, motion['then'])
                del self.motions[bot][port]
        waiting = self.waiting[bot]
        if waiting is not None:
            ready = (waiting[0] == 'time' and self.world.data.time+1e-9 >= waiting[1]) or (waiting[0] == 'motion' and waiting[1] not in self.motions[bot])
            if ready:
                self.waiting[bot] = None
                self._reply(bot)

    def _service(self, bot):
        if self.programs[bot]['status'] in ('done', 'error', 'stopped'):
            return
        self._update_motion(bot)
        if self.waiting[bot] is not None:
            return
        connection = self.connections[bot]
        prints = 0
        deadline = time.monotonic() + self.cpu_timeout
        while True:
            remaining = deadline-time.monotonic() if self.runtime_profile == RUNTIME_PROFILE else self.cpu_timeout
            if remaining <= 0 or not connection.poll(remaining):
                self._stop(bot, 'error', f'CPU time limit: program did not yield within {self.cpu_timeout:g} wall seconds')
                return
            try:
                message = connection.recv()
            except (EOFError, OSError):
                self._stop(bot, 'error', 'Python worker exited without a completion message')
                return
            if message[0] == 'done':
                self._stop(bot, 'done')
                return
            if message[0] == 'error':
                self._stop(bot, 'error', message[1])
                return
            self.programs[bot]['operations'] += 1
            if self.programs[bot]['operations'] > self.operation_limit:
                self._stop(bot, 'error', f'Program operation limit exceeded ({self.operation_limit})')
                return
            _, operation, args = message
            if operation == 'print' and self.runtime_profile == RUNTIME_PROFILE:
                prints += 1
                if prints > PRINT_BURST_LIMIT:
                    self._stop(bot, 'error', f'Diagnostic print burst limit exceeded ({PRINT_BURST_LIMIT})')
                    return
            try:
                self._dispatch(bot, operation, args)
            except (ValueError, TypeError, KeyError) as error:
                connection.send(('error', str(error)))
            if operation != 'print' or self.runtime_profile != RUNTIME_PROFILE:
                return

    def _dispatch(self, bot, operation, args):
        value = None
        if operation == 'wait':
            self.waiting[bot] = ('time', self.world.data.time+args[0]/1000)
        elif operation == 'clock':
            value = round(self.world.data.time*1000)
        elif operation == 'command':
            port, mode, speed = args
            if self.rules is not None and not self.rules.command(self.world, bot, port, mode, speed):
                self._reply(bot)
                return
            self.motions[bot].pop(port, None)
            self.world.command(bot, port, mode, speed)
        elif operation == 'angle':
            value = self.encoders[bot][args[0]]['angle']
        elif operation == 'speed':
            value = self.encoders[bot][args[0]]['speed']
        elif operation == 'reset_angle':
            self.world.reset_angle(bot, *args)
            self.encoders[bot][args[0]].update(angle=round(args[1]), time=self.world.data.time)
        elif operation == 'sensor':
            value = self.sensor_values[bot][args[0]]
        elif operation == 'heading':
            # World retains its legacy absolute CCW yaw for observer/replay
            # parity; the hardware API uses Pybricks' clockwise-positive sign.
            value = -self.headings[bot]
        elif operation == 'voltage':
            value = round(self.world.voltage[bot]*1000)
        elif operation == 'print':
            self.programs[bot]['logs'].append(args[0])
            self.programs[bot]['logs'] = self.programs[bot]['logs'][-200:]
            self.world.record_event('program_log', bot=bot, message=args[0])
        elif operation == 'motion':
            port, kind, speed, target, then, should_wait = args
            motion = {'kind': kind, 'then': then}
            if kind == 'time':
                motion['end'] = self.world.data.time+target/1000
            else:
                current = self.world.wheel_angle(bot, port)
                motion['target'] = current+target if kind == 'relative' else target
                motion['direction'] = 1 if motion['target'] >= current else -1
                speed = abs(speed)*motion['direction']
            if (kind == 'time' and target == 0) or (kind != 'time' and motion['target'] == current):
                self.motions[bot].pop(port, None)
                self.world.command(bot, port, then)
                self._reply(bot)
                return
            if self.rules is not None and not self.rules.command(self.world, bot, port, 'run', speed):
                self._reply(bot)
                return
            self.world.command(bot, port, 'run', speed)
            self.motions[bot][port] = motion
            if should_wait:
                self.waiting[bot] = ('motion', port)
        else:
            raise ValueError(f'Unsupported API operation: {operation}')
        if self.waiting[bot] is None:
            self._reply(bot, value)
        else:
            self.programs[bot]['status'] = 'waiting'

    def _finish_faults(self):
        if self.rules is not None:
            for bot, state in enumerate(self.rules.bots):
                if state['phase'] != self._phases[bot]:
                    self.world.record_event('opening_phase', bot=bot, phase=state['phase'], opening=state['opening'])
                    self._phases[bot] = state['phase']
        if self.world.result:
            return True
        violations = [bot for bot in range(2) if self.rules is not None and self.rules.bots[bot]['violation']]
        for bot in violations:
            self._stop(bot, 'error', f'Rule violation: {self.rules.bots[bot]["violation"]}')
        errors = [bot for bot in range(2) if self.programs[bot]['status'] == 'error']
        if errors:
            self.world.result = {'winner': None if len(errors) == 2 else 1-errors[0],
                                 'reason': 'rule_violation' if violations else 'program_error', 'time': self.world.data.time}
            self.world.record_result()
        return bool(errors)

    def step(self, seconds):
        if self.closed:
            raise RuntimeError('Runner is closed')
        seconds = _finite(seconds, 'step duration')
        count = round(seconds/.001)
        if count < 0 or not math.isclose(count*.001, seconds, abs_tol=1e-9):
            raise ValueError('Step duration must be a nonnegative multiple of 1 ms')
        for _ in range(count):
            if self.world.result:
                break
            self._sample()
            if self.rules is not None:
                self.rules.observe(self.world)
            for bot in range(2):
                self._service(bot)
            if self._finish_faults():
                break
            self.world.advance(.001)
            if self.rules is not None:
                self.rules.observe(self.world)
                if self._finish_faults():
                    break
        self._sample()
        return self.snapshot()

    def snapshot(self):
        value = self.world.snapshot()
        value['programs'] = [{**program, 'logs': list(program['logs'])} for program in self.programs]
        if self.rules is not None:
            value['rules'] = self.rules.snapshot()
        return value

    def run_until_done(self, max_seconds=None):
        if self.closed:
            raise RuntimeError('Runner is closed')
        duration = self.world.arena.timeout if max_seconds is None else _finite(max_seconds, 'max_seconds')
        if duration < 0:
            raise ValueError('max_seconds must be nonnegative')
        for _ in range(math.ceil(duration/.001)):
            if self.world.result:
                break
            self.step(.001)
        return self.snapshot()

    def close(self):
        if self.closed:
            return
        self.closed = True
        for bot in range(len(self.processes)):
            if self.programs[bot]['status'] not in ('done', 'error'):
                self._stop(bot, 'stopped')
            self.connections[bot].close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

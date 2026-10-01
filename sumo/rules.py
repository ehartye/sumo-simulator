"""Sourced ring/start profile and a deliberately bounded opening referee.

The user's stationary-wheel 180 degree turn resolves the source's ambiguous arc
wording. This is not a claim of complete Atomic Eagles tournament compliance.
"""
from dataclasses import dataclass
import math

from .physics import Arena


@dataclass(frozen=True)
class RuleProfile:
    id: str = 'atomic-eagles-2022-user-pivot-v1'
    wait_seconds: float = 3
    pivot_degrees: float = 180
    completion_tolerance_degrees: float = 5
    stationary_tolerance_degrees: float = 5

    def arena(self, timeout=None):
        return Arena(radius=.6096, height=.0254, border=.0508, timeout=120 if timeout is None else timeout)

    def info(self):
        return {
            'id': self.id, 'name': 'Atomic Eagles 2022 · user clarified 180° pivot',
            'arena': vars(self.arena()), 'waitSeconds': self.wait_seconds,
            'startSeparationMeters': .3048, 'outwardStart': True,
            'pivotDegrees': self.pivot_degrees,
            'completionToleranceDegrees': self.completion_tolerance_degrees,
            'stationaryWheelToleranceDegrees': self.stationary_tolerance_degrees,
            'sources': [
                {'title': 'Atomic Eagles Sumo Bot Rules 2022', 'url': 'https://www.atomiceagles.com/rules'},
                {'title': 'MINDSTORMS Sumo Bot Competition Rules, August 18, 2022',
                 'url': 'https://docs.google.com/document/d/1gM1sZfyIzWYAL7DgNhXlfSiJHiPrDX5llAFiOCDmdSY/edit'},
            ],
            'assumptions': [
                'Height is 1 inch (0.0254 m): the source Arena section says about 1 inch, while its introduction says 1.5 inches.',
                'Start positions refer to chassis origins 6 inches from center, giving 12 inch center separation; source also calls this a gap.',
                'User clarification: pivot about one stationary wheel until an approximately 180 degree about-face; this replaces ambiguous source arc-radius wording.',
                'Opening completes within 5 degrees of 180; the stationary shaft may drift at most 5 degrees.',
                'White contact uses the physical color sensor position above the border on the starting half; no noisy reflection threshold controls adjudication.',
            ],
            'enforced': ['No nonzero drive command before 3 simulated seconds.',
                         'First drive is forward; no reverse or counterrotation before white contact or a completed stationary-wheel pivot.',
                         'Pivot keeps one physical wheel shaft stationary; the edge path must turn after white contact, and both paths must face opposite the initial heading before phase engaged.',
                         'Opening rule violations lose; simultaneous faults draw. Physics supplies floor-contact and timeout results.'],
            'notEnforced': ['Complete tournament compliance, hardware legality, inspection, or judge discretion.',
                            'The source 10 second motion deadline and 15 second deadlock reset.',
                            'Disabled-robot judgments, tournament points or match scheduling.',
                            'Continuous arc-radius geometry or complete strategy-intent adjudication.'],
        }


ATOMIC_EAGLES_2022 = RuleProfile()


def _heading(world, bot):
    _, quaternion = world.pose(bot)
    w, x, y, z = quaternion
    # The robot's local forward axis is +Z in the Y-up world.
    return math.degrees(math.atan2(2*(x*z+w*y), 1-2*(x*x+y*y)))


def _shaft(world, bot, port):
    index = 0 if port == 'A' else 1
    # User reset_angle changes reported zero, not the referee's physical shaft position.
    return world.wheel_angle(bot, port)+float(world.offsets[bot][index])


class StartProtocol:
    def __init__(self, profile, world):
        if not world.outward_start:
            raise ValueError('This rule profile requires outward_start=True')
        self.profile = profile
        self.bots = [{'phase': 'waiting', 'opening': None, 'whiteTouched': False,
                      'headingChange': 0., 'violation': None, 'movingWheel': None,
                      'stationaryWheelDrift': 0., '_previous': _heading(world, bot),
                      'turnAfterWhite': 0., '_whiteHeading': None,
                      '_shaft': 0., '_half': list(world.pose(bot)[0])} for bot in range(2)]

    def _violate(self, bot, message):
        self.bots[bot]['violation'] = self.bots[bot]['violation'] or message
        return False

    def observe(self, world):
        for bot, state in enumerate(self.bots):
            current = _heading(world, bot)
            state['headingChange'] += (current-state['_previous']+180) % 360-180
            state['_previous'] = current
            if state['violation'] or state['phase'] == 'engaged':
                continue
            if world.data.time+1e-9 < self.profile.wait_seconds:
                continue
            state['phase'] = 'opening'
            if state['opening'] is None:
                continue
            position = world.sensor_pose(bot, 'color')
            radius = math.hypot(position[0], position[2])
            starting_half = position[0]*state['_half'][0]+position[2]*state['_half'][2] >= 0
            if (not state['whiteTouched'] and world.arena.radius-world.arena.border <= radius <= world.arena.radius
                    and abs(position[1]-world.arena.height) <= .035 and starting_half):
                state['whiteTouched'] = True
                state['_whiteHeading'] = state['headingChange']
            if state['whiteTouched']:
                state['turnAfterWhite'] = state['headingChange']-state['_whiteHeading']
            if state['opening'] == 'pivot':
                stationary = 'B' if state['movingWheel'] == 'A' else 'A'
                state['stationaryWheelDrift'] = abs(_shaft(world, bot, stationary)-state['_shaft'])
                if state['stationaryWheelDrift'] > self.profile.stationary_tolerance_degrees+1e-9:
                    self._violate(bot, 'Pivot opening moved its stationary wheel more than 5 degrees')
                    continue
            threshold = self.profile.pivot_degrees-self.profile.completion_tolerance_degrees
            complete = abs((state['headingChange']+180) % 360-180) >= threshold
            edge_turn_complete = state['whiteTouched'] and abs(state['turnAfterWhite']) >= threshold
            if complete and (state['opening'] == 'pivot' or edge_turn_complete):
                state['phase'] = 'engaged'

    def command(self, world, bot, port, mode, value):
        self.observe(world)
        state = self.bots[bot]
        if state['violation']:
            return False
        if mode not in ('run', 'dc') or abs(value) <= 1e-12:
            return True
        if world.data.time+1e-9 < self.profile.wait_seconds:
            return self._violate(bot, 'Nonzero drive before the three-second start wait completed')
        if state['phase'] == 'engaged':
            return True
        if value < 0 and not state['whiteTouched']:
            return self._violate(bot, 'Opening must drive forward; reverse/counterrotation requires white contact or a completed pivot')
        if state['opening'] is None:
            state['opening'] = 'pivot'
            state['movingWheel'] = port
            state['_shaft'] = _shaft(world, bot, 'B' if port == 'A' else 'A')
        elif state['opening'] == 'pivot' and port != state['movingWheel']:
            # Sequential left/right calls must be allowed to begin an edge opening.
            if abs(state['headingChange']) <= self.profile.completion_tolerance_degrees:
                state['opening'] = 'edge'
            elif state['whiteTouched']:
                state['opening'] = 'edge'
            else:
                return self._violate(bot, 'Pivot opening must keep the other wheel stationary until the about-face')
        return True

    def snapshot(self):
        return {'profile': self.profile.id, 'waitSeconds': self.profile.wait_seconds,
                'pivotDegrees': self.profile.pivot_degrees,
                'bots': [{key: value for key, value in state.items() if not key.startswith('_')} for state in self.bots]}

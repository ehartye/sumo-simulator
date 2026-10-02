"""Fresh close-return lockup detection and a forward-powered escape hook.

Uses real Pybricks device APIs only. Stable range is NOT proof of ground stall.
On hardware, verify hub mounting and imu.ready() before the match countdown.
"""
import math
# Hardware heading is clockwise-positive; this policy uses internal CCW yaw.
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.tools import StopWatch, wait


def suspect_lockup(samples):
    if len(samples) < 10 or samples[-1][0] - samples[0][0] < 1300:
        return None
    echoes = [s[1] for s in samples if s[1] < 200]
    if len(echoes) < 3 or max(echoes)-min(echoes) > 30:
        return None
    headings = [s[2] for s in samples]
    if abs(headings[-1]-headings[0]) > 8 or max(headings)-min(headings) > 45:
        return None
    rotation = (abs(samples[-1][3]-samples[0][3]) +
                abs(samples[-1][4]-samples[0][4]))/2
    return 'low motor rotation' if rotation < 12 else 'spinning-wheel duel'


def encounter_lockup(samples):
    # A second, weaker watchdog catches repeated reacquisition/search dithering.
    # It is deliberately labeled separately from the stable-range detector.
    if len(samples) < 6 or samples[-1][0]-samples[0][0] < 5000:
        return None
    if samples[0][1]-samples[-1][1] > 30:
        return None
    headings = [s[2] for s in samples]
    if max(headings)-min(headings) > 120 or abs(headings[-1]-headings[0]) > 45:
        return None
    return 'repeated close encounters'


def choose_turn(x, z, heading, uncertainty, echo_heading, echo_age, fallback):
    if uncertainty < 180:
        # Compare a short forward hook on either side, not an exact target map.
        costs = []
        for direction in (-1, 1):
            angle = math.radians(heading + direction*28)
            costs.append((x + 80*math.sin(angle))**2 +
                         (z + 80*math.cos(angle))**2)
        if abs(costs[0]-costs[1]) > 100:
            return (-1 if costs[0] < costs[1] else 1), 'estimated ring clearance'
    if echo_heading is not None and echo_age < 1200:
        error = (echo_heading-heading+180) % 360-180
        if abs(error) > 5:
            return (1 if error > 0 else -1), 'recent echo heading'
    return fallback, 'alternating fallback'


GEOMETRIC = True
left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
edge = ColorSensor(Port.C)
opponent = UltrasonicSensor(Port.D)
hub = PrimeHub()
clock = StopWatch()

# Known rule placement in a local frame: +Z is the initial outward direction.
# These estimates never read the simulator's actual chassis position.
x, z, uncertainty = 0., 152.4, 40.
previous_left, previous_right = left.angle(), right.angle()
previous_heading, previous_time = -hub.imu.heading(), clock.time()
last_white, travel_since_white = None, 0.
last_close, last_echo = -2000, -2000
echo_heading = None
samples = []
encounters = []
phase, phase_end, turn_target = 'pressure', 0, 0
pressure_turn = False
cooldown, fallback, turn = 0, 1, 1
decision = None
scan_target = None
was_white = False

wait(3000)
right.hold()
left.run(500)
opening_deadline = clock.time()+8000
while abs(hub.imu.heading()) < 180:
    if clock.time() >= opening_deadline:
        left.brake()
        right.brake()
        raise RuntimeError('Opening pivot could not complete')
    wait(5)
left.brake()
right.brake()

while True:
    now = clock.time()
    heading = -hub.imu.heading()
    a, b = left.angle(), right.angle()
    distance = opponent.distance()
    white = edge.reflection() > 50
    # 56 mm wheels: shaft rotation is travel only when there is grip.
    ds = ((a-previous_left)+(b-previous_right))/2*math.pi*56/360
    angle = math.radians((heading+previous_heading)/2)
    x += ds*math.sin(angle)
    z += ds*math.cos(angle)
    dt = max(0, now-previous_time)/1000
    close_recently = distance < 200 or now-last_close < 2000
    uncertainty += abs(ds)*(.8 if close_recently else .15) + dt*(60 if close_recently else 2)
    if last_white is not None:
        travel_since_white += abs(ds)
    previous_left, previous_right, previous_heading, previous_time = a, b, heading, now
    if distance < 2000:
        last_echo, echo_heading = now, heading
    if distance < 200:
        last_close = now
        encounters.append((now, distance, heading, a, b))
    while encounters and now-encounters[0][0] > 6000:
        encounters.pop(0)

    # Highest priority, including during release/turn/arc. A single touch only
    # constrains radial position; retain tangential/slip uncertainty.
    if white:
        if not was_white:
            sx, sz = x+72*math.sin(math.radians(heading)), z+72*math.cos(math.radians(heading))
            radius = math.sqrt(sx*sx+sz*sz)
            if GEOMETRIC and radius > 1:
                x += sx*(584.2/radius-1)
                z += sz*(584.2/radius-1)
                uncertainty = max(80, uncertainty)
            last_white, travel_since_white = now, 0.
            print('white landmark', 'heading', round(heading), 'uncertainty mm', round(uncertainty))
            phase, phase_end = 'border release', now+160
            samples = []
            encounters = []
            turn = fallback
            fallback = -fallback
            last_close = -2000
        if now < phase_end:
            left.run(-500)
            right.run(-500)
            label = 'border retreat'
        else:
            left.brake()
            right.brake()
            label = 'border blocked hold'
    elif phase in ('release', 'border release') and now < phase_end:
        left.run(-450)
        right.run(-450)
        label = phase
    elif phase in ('release', 'border release'):
        pressure_turn = phase == 'release'
        turn_target = heading + turn*(100 if phase == 'border release' else 28)
        phase, phase_end = 'turn', now+800
        if pressure_turn:
            left.dc(100 if turn > 0 else 0)
            right.dc(0 if turn > 0 else 100)
        else:
            left.run(360*turn)
            right.run(-360*turn)
        label = 'escape turn'
    elif phase == 'turn' and (turn_target-heading)*turn > 3 and now < phase_end:
        if pressure_turn:
            left.dc(100 if turn > 0 else 0)
            right.dc(0 if turn > 0 else 100)
        else:
            left.run(360*turn)
            right.run(-360*turn)
        label = 'escape turn'
    elif phase == 'turn':
        print('escape turn end', 'error deg', round(turn_target-heading, 1), 'timed out', now >= phase_end)
        phase, phase_end = 'arc', now+450
        left.dc(100 if turn > 0 else 55)
        right.dc(55 if turn > 0 else 100)
        label = 'escape arc'
    elif phase == 'arc' and now < phase_end:
        left.dc(100 if turn > 0 else 55)
        right.dc(55 if turn > 0 else 100)
        label = 'escape arc'
    else:
        if phase == 'arc':
            print('recovery complete', 'range mm', distance, 'heading', round(heading))
            phase, cooldown, scan_target = 'pressure', now+2000, None
            samples = []
            encounters = []
        attack = distance < 2000 or now-last_close < 400
        if attack:
            scan_target = None
            samples.append((now, distance, heading, a, b))
            while samples and now-samples[0][0] > 1500:
                samples.pop(0)
            reason = suspect_lockup(samples)
            if not reason and now-last_close < 600:
                reason = encounter_lockup(encounters)
            if reason and distance < 140 and now >= cooldown:
                turn, basis = (choose_turn(x, z, heading, uncertainty, echo_heading,
                                          now-last_echo, fallback) if GEOMETRIC else
                               (fallback, 'alternating fallback'))
                fallback = -turn
                # The front sensor cannot guard a backward escape near the rear rim.
                reverse_ms = 0
                phase, phase_end = 'release', now+reverse_ms
                evidence = encounters if reason == 'repeated close encounters' else samples
                print('suspected lockup', reason, 'range mm', distance,
                      'heading span', round(max(s[2] for s in evidence)-min(s[2] for s in evidence), 1),
                      'turn', turn, basis, 'reverse ms', reverse_ms,
                      'since white ms', None if last_white is None else now-last_white,
                      'wheel travel mm', round(travel_since_white), 'uncertainty mm', round(uncertainty))
                left.brake()
                right.brake()
                samples = []
                encounters = []
                last_close = -2000
                label = 'lockup release'
            else:
                left.dc(100)
                right.dc(100)
                label = 'charge' if distance < 2000 else 'push memory'
        else:
            if now-last_close > 600:
                samples = []
            if GEOMETRIC:
                if scan_target is None or (scan_target-heading)*turn <= 3:
                    turn, basis = choose_turn(x, z, heading, uncertainty, echo_heading, now-last_echo, fallback)
                    scan_target = heading+turn*360
                    print('full scan', 'turn', turn, basis)
                speed = 240*turn
            else:
                speed = 240 if (now//1000) % 2 == 0 else -240
            left.run(speed)
            right.run(-speed)
            label = 'search'
    if label != decision:
        print(label)
        decision = label
    was_white = white
    wait(10)

"""Loop 23: steer with measured engagement rotation rather than fighting it.

Pybricks-only policy. Range/gyro histories indicate suspicion, not ground speed.
Hardware needs correct hub mounting and stationary calibration before countdown.
"""
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.tools import StopWatch, wait


def duel_alarm(samples, distance):
    # Sustained engagement is suspicious even when yaw is changing rapidly.
    # It can also interrupt useful pushing; no hardware ground progress is known.
    if distance >= 140 or len(samples) < 20 or samples[-1][0]-samples[0][0] < 4000:
        return False
    return sum(s[1] < 140 for s in samples)/len(samples) >= .8


def blind_alarm(now, last_echo, last_white, forward_since, duration):
    white_age = now-last_white if last_white is not None else now-forward_since
    echo_age = now-last_echo if last_echo is not None else now-forward_since
    return duration > 0 and min(echo_age, white_age, now-forward_since) >= duration


def tip_action(pitch, roll):
    if max(abs(pitch), abs(roll)) >= 75:
        return 'brake'
    if max(abs(pitch), abs(roll)) >= 12:
        return 'retreat'
    return None


def hook_complete(heading, start, side, angle):
    return (heading-start)*side >= angle


def choose_hook(heading, prior, white_age, travel_mm, fallback):
    # This ages a direction hint, not a certified position or empty sector.
    if 0 <= white_age <= 4000 and travel_mm <= 200:
        error = (prior-heading+180) % 360-180
        if abs(error) > 8:
            return (1 if error > 0 else -1), 'recent border retreat axis'
    return fallback, 'alternating fallback'


def engagement_side(samples, fallback):
    if len(samples) >= 2:
        rotation = samples[-1][2]-samples[0][2]
        if abs(rotation) > 20:
            return (1 if rotation > 0 else -1), 'follow observed rotation'
    return fallback, 'alternating fallback'


SEARCH_SPEED = 450
WEAVE_ANGLE = 25
CHARGE_SPEED = 850
BLIND_MS = 3000
RECOVERY = 'forward hook'
RELEASE_MS = 450
TURN_ANGLE = 35

left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
edge = ColorSensor(Port.C)
opponent = UltrasonicSensor(Port.D)
hub = PrimeHub()
clock = StopWatch()
wait(3000)
right.hold()
left.run(500)
deadline = clock.time()+8000
while abs(hub.imu.heading()) < 180:
    if clock.time() >= deadline:
        left.brake()
        right.brake()
        raise RuntimeError('Opening pivot could not complete')
    wait(5)
left.brake()
right.brake()

axis = hub.imu.heading()
retreat_axis = axis
travel_since_white = 0.
last_close = -2000
last_echo = None
last_white = None
forward_since = clock.time()
history = []
phase, until, target = 'normal', 0, axis
side, weave_side = 1, 1
cooldown = 0
state = None
attempt_heading = axis
next_encoder = 0
encoder_a, encoder_b = left.angle(), right.angle()
line_encoder_a, line_encoder_b = encoder_a, encoder_b

while True:
    now = clock.time()
    distance = opponent.distance()
    white = edge.reflection() > 50
    heading = hub.imu.heading()
    pitch, roll = hub.imu.tilt()
    white_age = now-last_white if last_white is not None else -1
    if now >= next_encoder:
        new_a, new_b = left.angle(), right.angle()
        travel_since_white += (abs(new_a-encoder_a)+abs(new_b-encoder_b))/2*3.14159*56/360
        encoder_a, encoder_b = new_a, new_b
        next_encoder = now+100
    if distance < 2000:
        last_echo = now
    if distance < 200:
        last_close = now
    history.append((now, distance, heading))
    while history and now-history[0][0] > 4200:
        history.pop(0)
    if white:
        last_white = now
        travel_since_white = 0.
        if not phase.startswith('border'):
            phase, until = 'border reverse', now+200
            line_encoder_a, line_encoder_b = encoder_a, encoder_b
            history = []
            print('white landmark', 'heading', round(heading))
    # White keeps priority; no observer contact/pose is available here.
    tip = tip_action(pitch, roll)
    if not phase.startswith('border') and not phase.startswith('tip') and tip:
        phase, until = 'tip '+tip, now+200
        history = []
        print('tip guard', 'pitch deg', round(pitch, 1), 'roll deg', round(roll, 1), 'action', tip)
    if phase == 'tip brake' or (phase == 'tip retreat' and tip == 'brake'):
        phase = 'tip brake'
        left.brake()
        right.brake()
        decision = 'tip brake'
        if max(abs(pitch), abs(roll)) < 6:
            phase, forward_since, axis = 'normal', now, heading
    elif phase == 'tip retreat' and now < until:
        left.run(-550)
        right.run(-550)
        decision = 'tip retreat'
    elif phase == 'tip retreat':
        phase, forward_since, axis = 'normal', now, heading
        history = []
        left.brake()
        right.brake()
        decision = 'tip release end'
    elif phase == 'border reverse' and now < until:
        left.run(-550)
        right.run(-550)
        decision = 'border retreat'
    elif phase == 'border reverse':
        phase, until, target = 'border turn', now+1100, heading+180
        left.run(400)
        right.run(-400)
        decision = 'border turn'
    elif phase == 'border turn' and target-heading > 4 and now < until:
        left.run(400)
        right.run(-400)
        decision = 'border turn'
    elif phase == 'border turn':
        print('border turn end', 'error deg', round(target-heading, 1), 'timed out', now >= until)
        axis = heading
        if target-heading <= 4:
            retreat_axis = heading
        phase, cooldown, forward_since = 'normal', now+1500, now
        left.brake()
        right.brake()
        decision = 'border facing inward estimate'
    elif phase == 'release' and now < until and not hook_complete(heading, attempt_heading, side, TURN_ANGLE):
        if RECOVERY == 'forward hook':
            if side > 0:
                left.run(CHARGE_SPEED)
                right.hold()
            else:
                left.hold()
                right.run(CHARGE_SPEED)
        elif RECOVERY == 'reverse':
            left.run(-550)
            right.run(-550)
        else:
            left.brake()
            right.brake()
        decision = 'recovery release'
    elif phase == 'release':
        print('forward hook end', 'yaw deg', round(heading-attempt_heading, 1), 'timed out', now >= until)
        if RECOVERY == 'reverse':
            phase, until, target = 'escape turn', now+700, heading+side*TURN_ANGLE
        else:
            phase, until = 'recharge', now+300
        left.brake()
        right.brake()
        decision = phase
    elif phase == 'escape turn' and (target-heading)*side > 4 and now < until:
        left.run(450*side)
        right.run(-450*side)
        decision = 'escape turn'
    elif phase == 'escape turn':
        print('escape turn end', 'error deg', round(target-heading, 1), 'timed out', now >= until)
        phase, until = 'recharge', now+300
        left.run(CHARGE_SPEED)
        right.run(CHARGE_SPEED)
        decision = 'recharge'
    elif phase == 'recharge' and now < until:
        left.run(CHARGE_SPEED)
        right.run(CHARGE_SPEED)
        decision = 'recharge'
    else:
        if phase == 'recharge':
            print('recovery complete', 'range mm', distance, 'yaw deg', round(heading-attempt_heading, 1))
            phase, cooldown, forward_since = 'normal', now+2500, now
            axis = heading
            history = []
        close_alarm = RECOVERY != 'none' and duel_alarm(history, distance)
        blind = blind_alarm(now, last_echo, last_white, forward_since, BLIND_MS)
        if now >= cooldown and (close_alarm or blind):
            fallback, basis = engagement_side(history, -side) if close_alarm else (-side, 'alternating fallback')
            side, border_basis = choose_hook(heading, retreat_axis, white_age, travel_since_white, fallback)
            if border_basis != 'alternating fallback':
                basis = border_basis
            print('hook direction', side, basis, 'since white ms (-1 unknown)', white_age, 'wheel travel mm', round(travel_since_white))
            attempt_heading = heading
            phase, until = 'release', now+RELEASE_MS
            travel = (abs(encoder_a-line_encoder_a)+abs(encoder_b-line_encoder_b))/2*3.14159*56/360
            print('suspected lockup' if close_alarm else 'watchdog blind drive',
                  'range mm', distance, 'echo age ms (-1 unknown)', now-last_echo if last_echo is not None else -1,
                  'since white ms (-1 unknown)', white_age, 'wheel travel mm', round(travel),
                  'turn', side, 'action', RECOVERY, 'yaw span deg', round(max(s[2] for s in history)-min(s[2] for s in history)))
            left.brake()
            right.brake()
            history = []
            decision = 'recovery start'
        elif distance < 2000 or now-last_close < 400:
            left.run(CHARGE_SPEED)
            right.run(CHARGE_SPEED)
            decision = 'charge' if distance < 2000 else 'push memory'
        else:
            error = heading-axis
            if error >= WEAVE_ANGLE:
                weave_side = -1
            elif error <= -WEAVE_ANGLE:
                weave_side = 1
            if weave_side > 0:
                left.run(SEARCH_SPEED)
                right.hold()
            else:
                left.hold()
                right.run(SEARCH_SPEED)
            decision = 'forward weave'
    if decision != state:
        print(decision)
        state = decision
    wait(10)

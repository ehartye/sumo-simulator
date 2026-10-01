"""Loop 04: break a sustained straight push with a brief differential hook.

This is a separate Python program with its own devices, variables, and clock.
"""
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.tools import StopWatch, wait


left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
edge = ColorSensor(Port.C)
opponent = UltrasonicSensor(Port.D)
clock = StopWatch()
hub = PrimeHub()

# Turn about the held right wheel, with only the left wheel driving forward.
wait(3000)
right.hold()
left.run(500)
while abs(hub.imu.heading()) < 180:
    wait(5)
left.brake()
right.brake()

state = None
last_close = -1000
push_started = 0
hook_until = 0
while True:
    distance = opponent.distance()
    now = clock.time()
    if distance < 200:
        last_close = now
    if edge.reflection() > 50:
        decision = 'border retreat'
        if decision != state:
            print(decision)
        left.run(-550)
        right.run(-550)
        wait(200)
        left.run(400)
        right.run(-400)
        wait(220)
    # 2000 is Pybricks' no-valid-echo sentinel, not an opponent at 2 m.
    elif distance < 2000 or now - last_close < 400:
        if state not in ('charge', 'push memory', 'hook'):
            push_started = now
        if now - push_started > 1800:
            hook_until = now + 220
            push_started = now
        decision = 'hook' if now < hook_until else 'charge' if distance < 2000 else 'push memory'
        if decision != state:
            print(decision, distance, 'mm')
        left.dc(45 if now < hook_until else 100)
        right.dc(100)
    else:
        decision = 'search'
        if decision != state:
            print(decision)
        speed = 240 if (clock.time() // 1000) % 2 == 0 else -240
        left.run(speed)
        right.run(-speed)
    state = decision
    wait(10)

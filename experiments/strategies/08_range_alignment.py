"""Loop 08: sweep across an echo and return to the shortest observed range before charging.

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
while True:
    distance = opponent.distance()
    now = clock.time()
    if state in (None, 'search') and distance < 2000 and edge.reflection() < 50:
        print('range sweep')
        best_distance = distance
        best_heading = hub.imu.heading()
        until = clock.time() + 160
        left.run(240)
        right.run(-240)
        while clock.time() < until and edge.reflection() < 50:
            reading = opponent.distance()
            if reading < best_distance:
                best_distance = reading
                best_heading = hub.imu.heading()
            wait(10)
        left.brake()
        right.brake()
        print('align shortest echo', best_distance, 'mm')
        until = clock.time() + 400
        error = best_heading - hub.imu.heading()
        while abs(error) > 4 and clock.time() < until and edge.reflection() < 50:
            speed = max(-220, min(220, error * 7))
            left.run(speed)
            right.run(-speed)
            wait(10)
            error = best_heading - hub.imu.heading()
        left.brake()
        right.brake()
        state = 'aligned'
        continue
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
        decision = 'charge' if distance < 2000 else 'push memory'
        if decision != state:
            print(decision, distance, 'mm')
        left.dc(100)
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

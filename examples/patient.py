"""Start patiently, scan in short arcs, and countercharge at closer range.

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

while True:
    if edge.reflection() > 50:
        left.run(-550)
        right.run(-550)
        wait(200)
        left.run(400)
        right.run(-400)
        wait(220)
    elif opponent.distance() < 400:
        left.run(850)
        right.run(850)
    else:
        speed = 240 if (clock.time() // 1000) % 2 == 0 else -240
        left.run(speed)
        right.run(-speed)
    wait(10)

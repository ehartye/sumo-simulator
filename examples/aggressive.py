"""Search, charge a detected opponent, and reverse away from the white border.

Uses the simulator's synchronous Pybricks subset and the fixed minimal stock build.
Ports A/B are the motors, C is the downward color sensor, D is ultrasonic.
"""
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.tools import wait


left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.COUNTERCLOCKWISE)
edge = ColorSensor(Port.C)
opponent = UltrasonicSensor(Port.D)
hub = PrimeHub()

# Both robots start facing away. Take the white-line opening after the 3 s wait.
wait(3000)
left.run(400)
right.run(400)
while edge.reflection() <= 50:
    wait(5)
left.run(-350)
right.run(-350)
wait(160)
left.run(-400)
right.run(400)
while abs(hub.imu.heading()) < 180:
    wait(5)
left.brake()
right.brake()

while True:
    if edge.reflection() > 55:
        left.run(-650)
        right.run(-650)
        wait(180)
        left.run(-450)
        right.run(450)
        wait(170)
    elif opponent.distance() < 550:
        left.run(950)
        right.run(950)
    else:
        left.run(350)
        right.run(-350)
    wait(10)

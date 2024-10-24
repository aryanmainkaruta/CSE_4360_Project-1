from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
LeftMotor=Motor(Port.A,Direction.CLOCKWISE)
RightMotor=Motor(Port.B,Direction.COUNTERCLOCKWISE)
Ultrasonic=UltrasonicSensor(Port.E)

while True:
    dis=Ultrasonic.distance()
    print(dis)
    if dis<50:
        print("ON TABLE")
        LeftMotor.run(100)
        RightMotor.run(100)
    else:
        print("OFF TABLE")
        LeftMotor.run(-100)
        RightMotor.run(-100)
        wait(3000)
        LeftMotor.run(100)
        RightMotor.run(-100)
        wait(3000)


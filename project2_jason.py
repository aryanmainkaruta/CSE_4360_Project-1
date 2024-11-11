from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ultra=UltrasonicSensor(Port.D)
hub = PrimeHub()
LeftMotor=Motor(Port.A,Direction.CLOCKWISE)
RightMotor=Motor(Port.B,Direction.COUNTERCLOCKWISE)
SweepMotor=Motor(Port.F,Direction.CLOCKWISE)
button = ForceSensor(Port.E)


multiplier=1.5
wallHuggingDistance=120
basePower=100

def sweepUP():
    SweepMotor.run(-1000)

def sweepDown():
    SweepMotor.run(1000)

def ceiling(commandedValue,maxedValue,minValue):
    if commandedValue > maxedValue:
        return maxedValue
    if commandedValue < minValue:
        return minValue
    return commandedValue

def TurnForAngle(desired_angle,threshold=1,Kp=3):
    if hub.imu.ready():
        hub.imu.reset_heading(0)
        while (abs(desired_angle-hub.imu.heading()) > threshold) and (abs(desired_angle-hub.imu.heading()) > threshold):
            LeftMotor.run(-Kp*(desired_angle-hub.imu.heading()))
            RightMotor.run(Kp*(desired_angle-hub.imu.heading()))
            print(desired_angle-hub.imu.heading())
        print("Finished Move")
        LeftMotor.stop()
        RightMotor.stop()
    else:
        print("Hub Not Ready")

def DriveTrain(correctionValue):
    offset=ceiling(correctionValue,25,-25)

    RightMotor.run(basePower-offset)
    LeftMotor.run(basePower+offset)

sweepUP()
wait(1000)
SweepMotor.brake()
while True:
    DriveTrain(ultra.distance()-wallHuggingDistance)
    if(button.pressed()):
        # Back up and Turn other Way
        print("BACKUPPP")
        RightMotor.run(-50)
        LeftMotor.run(-50)
        wait(1000)
        TurnForAngle(90)

     if timer.time() > spiral_interval:
        timer.reset()  # Reset the timer for the next interval
        TurnForAngle(15, relative=True) 






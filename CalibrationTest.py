from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

def MoveStraightForAngleLegacy(desired_angle):
    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    while desired_angle > LeftMotor.angle() and desired_angle > RightMotor.angle():
        LeftMotor.run(100)
        RightMotor.run(100)
    
    LeftMotor.stop()
    RightMotor.stop()

def MoveStraightForDistance(desired_distance,threshold=1,Kp=2):
    Wheel_Radius_in=2.2/2
    pi=umath.pi
    desired_angle=(desired_distance/(2*pi*Wheel_Radius_in))*360

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    
    while (abs(desired_angle-LeftMotor.angle()) > threshold) and (abs(desired_angle-RightMotor.angle()) > threshold):
        LeftMotor.run(Kp*(desired_angle-LeftMotor.angle()))
        RightMotor.run(Kp*(desired_angle-RightMotor.angle()))
    print("Finished Move")
    LeftMotor.stop()
    RightMotor.stop()

def TurnForAngleLegacy(desired_angle):
    wheel_spacing=3.125
    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    resultant_distance=wheel_spacing*umath.pi*(desired_angle/360)
    ANGLE=(resultant_distance/(2*pi*Wheel_Radius_in))*360

    while ANGLE > LeftMotor.angle() and ANGLE > RightMotor.angle():
        LeftMotor.run(100)
        RightMotor.run(-100)
    
    LeftMotor.stop()
    RightMotor.stop()

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


hub = PrimeHub()
LeftMotor=Motor(Port.A,Direction.CLOCKWISE)
RightMotor=Motor(Port.B,Direction.COUNTERCLOCKWISE)
#MoveStraightForDistance(4)
TurnForAngle(90)
#Ultrasonic=UltrasonicSensor(Port.E)


#TurnForAngle(90)
# notes=["D4/16", "D4/8", "C#4/4", "C#4/16","C#4/8",
#  "C#4/4", "C#4/16","C#4/8","F#4/4"]
# hub.speaker.play_notes(notes,tempo=120)
#MoveStraightForAngle(Desired_Angle)





# LeftMotor.run_target(90,Desired_Angle,then=Stop.HOLD,wait=False)
# RightMotor.run_target(90,Desired_Angle,then=Stop.HOLD,wait=False)

# while True:
#     dis=Ultrasonic.distance()
#     print(dis)
#     if dis<50:
#         print("ON TABLE")
#         LeftMotor.run(100)
#         print("ANGLE: "+str(LeftMotor.angle()))
#         print("SPEED: "+str(LeftMotor.speed()))
#         RightMotor.run(100)
#     else:
#         print("OFF TABLE")
#         LeftMotor.run(-100)
#         RightMotor.run(-100)
#         wait(3000)
#         LeftMotor.run(100)
#         RightMotor.run(-100)
#         wait(3000)

